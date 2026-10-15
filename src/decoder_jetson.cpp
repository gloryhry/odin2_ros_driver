#include <unistd.h>

#include <algorithm>
#include <fstream>
#include <mutex>

#include "odin2_ros_driver/decoder_backend.hpp"

#ifdef ODIN_DRIVER_GSTREAMER
#include <gst/app/gstappsink.h>
#include <gst/app/gstappsrc.h>
#include <gst/video/video.h>
#endif

namespace odin_driver::decoding {
namespace {
bool isJetson() {
  if (access("/run/host-nv_tegra_release", F_OK) == 0 ||
      access("/etc/nv_tegra_release", F_OK) == 0 || access("/dev/nvhost-nvdec", F_OK) == 0 ||
      access("/dev/v4l2-nvdec", F_OK) == 0)
    return true;
  std::ifstream input("/proc/device-tree/compatible", std::ios::binary);
  const std::string compatible{std::istreambuf_iterator<char>(input), {}};
  return compatible.find("tegra") != std::string::npos;
}
#ifdef ODIN_DRIVER_GSTREAMER
struct GstObjectDeleter {
  void operator()(GstElement* p) const {
    if (p) {
      gst_element_set_state(p, GST_STATE_NULL);
      gst_object_unref(p);
    }
  }
};
class JetsonBackend final : public Backend {
 public:
  explicit JetsonBackend(sdk::OdinDataFormat format) {
    if (access("/dev/v4l2-nvdec", R_OK | W_OK) != 0 &&
        access("/dev/nvhost-nvdec", R_OK | W_OK) != 0)
      throw Error(Failure::backend, "Jetson NVDEC device missing or inaccessible");
    static std::once_flag init;
    std::call_once(init, [] {
      GError* error = nullptr;
      if (!gst_init_check(nullptr, nullptr, &error)) {
        std::string message = error ? error->message : "unknown initialization error";
        if (error)
          g_error_free(error);
        throw Error(Failure::backend, "GStreamer initialization: " + message);
      }
    });
    pipeline_.reset(gst_pipeline_new(nullptr));
    if (!pipeline_)
      throw std::bad_alloc();
    auto add = [this](const char* name) {
      GstElement* element = gst_element_factory_make(name, nullptr);
      if (!element)
        throw Error(Failure::backend, std::string("missing GStreamer plugin: ") + name);
      if (!gst_bin_add(GST_BIN(pipeline_.get()), element)) {
        gst_object_unref(element);
        throw Error(Failure::backend, "adding GStreamer element failed");
      }
      return element;
    };
    source_ = add("appsrc");
    auto parser = add(format == sdk::OdinDataFormat::kImageH264   ? "h264parse"
                      : format == sdk::OdinDataFormat::kImageH265 ? "h265parse"
                                                                  : "jpegparse");
    auto decoder = add("nvv4l2decoder");
    auto converter = add("nvvidconv");
    auto filter = add("capsfilter");
    sink_ = add("appsink");
    const char* input_caps = format == sdk::OdinDataFormat::kImageH264
                                 ? "video/x-h264,stream-format=byte-stream,alignment=au"
                             : format == sdk::OdinDataFormat::kImageH265
                                 ? "video/x-h265,stream-format=byte-stream,alignment=au"
                                 : "image/jpeg";
    GstCaps* caps = gst_caps_from_string(input_caps);
    gst_app_src_set_caps(GST_APP_SRC(source_), caps);
    gst_caps_unref(caps);
    g_object_set(source_, "is-live", TRUE, "format", GST_FORMAT_TIME, "block", FALSE, "max-bytes",
                 guint64(64 * 1024 * 1024), nullptr);
    g_object_set(sink_, "sync", FALSE, "max-buffers", guint(8), "drop", FALSE, "wait-on-eos", FALSE,
                 nullptr);
    // Never disable DPB: firmware streams may contain B-frames.
    if (format == sdk::OdinDataFormat::kImageMjpeg)
      g_object_set(decoder, "mjpeg", TRUE, nullptr);
    caps = gst_caps_from_string("video/x-raw,format=BGRx,width=[1,8192],height=[1,8192]");
    g_object_set(filter, "caps", caps, nullptr);
    gst_caps_unref(caps);
    if (!gst_element_link_many(source_, parser, decoder, converter, filter, sink_, nullptr))
      throw Error(Failure::backend, "linking Jetson hardware pipeline failed");
    if (gst_element_set_state(pipeline_.get(), GST_STATE_PLAYING) == GST_STATE_CHANGE_FAILURE) {
      checkBus();
      throw Error(Failure::backend, "starting Jetson hardware pipeline failed");
    }
    checkBus();
  }
  ~JetsonBackend() override {
    if (pipeline_)
      gst_element_set_state(pipeline_.get(), GST_STATE_NULL);
  }
  bool asynchronous() const override {
    return true;
  }
  void send(const std::vector<uint8_t>& bytes, int64_t token) override {
    checkBus();
    guint64 queued = 0;
    g_object_get(source_, "current-level-bytes", &queued, nullptr);
    if (queued + bytes.size() > 64 * 1024 * 1024)
      throw Error(Failure::backend, "Jetson input queue exceeded 64 MiB");
    GstBuffer* buffer = gst_buffer_new_allocate(nullptr, bytes.size(), nullptr);
    if (!buffer)
      throw std::bad_alloc();
    gst_buffer_fill(buffer, 0, bytes.data(), bytes.size());
    GST_BUFFER_PTS(buffer) = static_cast<GstClockTime>(token);
    GST_BUFFER_DTS(buffer) = GST_CLOCK_TIME_NONE;
    auto result = gst_app_src_push_buffer(GST_APP_SRC(source_), buffer);  // Takes ownership.
    if (result != GST_FLOW_OK)
      throw Error(Failure::backend, "Jetson appsrc rejected packet: " + std::to_string(result));
  }
  std::vector<Picture> receive() override {
    checkBus();
    std::vector<Picture> out;
    for (size_t n = 0; n < 8; ++n) {
      GstSample* raw = gst_app_sink_try_pull_sample(GST_APP_SINK(sink_), 0);
      if (!raw)
        break;
      std::unique_ptr<GstSample, decltype(&gst_sample_unref)> sample(raw, gst_sample_unref);
      GstVideoInfo info;
      gst_video_info_init(&info);
      if (!gst_video_info_from_caps(&info, gst_sample_get_caps(raw)) ||
          GST_VIDEO_INFO_FORMAT(&info) != GST_VIDEO_FORMAT_BGRx || info.width <= 0 ||
          info.height <= 0 || info.width > 8192 || info.height > 8192)
        throw Error(Failure::backend, "invalid Jetson output caps");
      GstBuffer* buffer = gst_sample_get_buffer(raw);
      GstVideoFrame frame;
      if (!gst_video_frame_map(&frame, &info, buffer, GST_MAP_READ))
        throw Error(Failure::backend, "mapping Jetson host frame failed");
      Picture picture;
      picture.token = static_cast<int64_t>(GST_BUFFER_PTS(buffer));
      auto& image = picture.image;
      image.width = info.width;
      image.height = info.height;
      image.encoding = "bgr8";
      image.step = image.width * 3;
      // Unmap on allocation or validation failure as well.
      try {
        image.data.resize(size_t(image.step) * image.height);
        const auto* data = static_cast<const uint8_t*>(GST_VIDEO_FRAME_PLANE_DATA(&frame, 0));
        const int stride = GST_VIDEO_FRAME_PLANE_STRIDE(&frame, 0);
        if (!data || stride < static_cast<int>(image.width * 4))
          throw Error(Failure::backend, "invalid Jetson host frame stride");
        for (size_t y = 0; y < image.height; ++y)
          for (size_t x = 0; x < image.width; ++x)
            std::copy_n(data + y * stride + 4 * x, 3, image.data.data() + y * image.step + 3 * x);
      } catch (...) {
        gst_video_frame_unmap(&frame);
        throw;
      }
      gst_video_frame_unmap(&frame);
      out.push_back(std::move(picture));
    }
    checkBus();
    return out;
  }

 private:
  void checkBus() {
    GstBus* bus = gst_element_get_bus(pipeline_.get());
    GstMessage* message = gst_bus_pop_filtered(bus, GST_MESSAGE_ERROR);
    gst_object_unref(bus);
    if (!message)
      return;
    GError* error = nullptr;
    gchar* debug = nullptr;
    gst_message_parse_error(message, &error, &debug);
    const std::string text = error ? error->message : "unknown pipeline error";
    const auto kind =
        error && error->domain == GST_STREAM_ERROR && error->code == GST_STREAM_ERROR_DECODE
            ? Failure::bitstream
            : Failure::backend;
    if (error)
      g_error_free(error);
    g_free(debug);
    gst_message_unref(message);
    throw Error(kind, "Jetson hardware pipeline: " + text);
  }
  std::unique_ptr<GstElement, GstObjectDeleter> pipeline_;
  GstElement* source_ = nullptr;
  GstElement* sink_ = nullptr;
};
#endif
}  // namespace
std::vector<Candidate> jetsonCandidates(sdk::OdinDataFormat format) {
  if (!isJetson())
    return {};
  Candidate candidate;
  candidate.name = "jetson/nvv4l2decoder";
  candidate.device = access("/dev/v4l2-nvdec", F_OK) == 0 ? "/dev/v4l2-nvdec" : "/dev/nvhost-nvdec";
  candidate.hardware = true;
#ifdef ODIN_DRIVER_GSTREAMER
  candidate.create = [format] { return std::make_unique<JetsonBackend>(format); };
#else
  (void)format;
  candidate.create = []() -> std::unique_ptr<Backend> {
    throw Error(Failure::backend, "driver was built without GStreamer support");
  };
#endif
  return {std::move(candidate)};
}
}  // namespace odin_driver::decoding
