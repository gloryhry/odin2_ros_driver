#include "odin2_ros_driver/decoder_backend.hpp"

extern "C" {
#include <libavcodec/avcodec.h>
#include <libavutil/error.h>
#include <libavutil/hwcontext.h>
#include <libavutil/imgutils.h>
#include <libavutil/opt.h>
#include <libavutil/pixdesc.h>
#include <libswscale/swscale.h>
}
#include <dlfcn.h>
#include <glob.h>

#include <algorithm>
#include <cstring>
#include <tuple>

namespace odin_driver::decoding {
namespace {
std::string errorText(int code) {
  char text[AV_ERROR_MAX_STRING_SIZE]{};
  av_strerror(code, text, sizeof(text));
  return text;
}
class Converter {
 public:
  ~Converter() {
    sws_freeContext(sws_);
  }
  sensor_msgs::Image convert(AVFrame* frame) {
    if (frame->width <= 0 || frame->height <= 0 || frame->width > 8192 || frame->height > 8192)
      throw Error(Failure::bitstream, "invalid decoded dimensions");
    sws_ = sws_getCachedContext(
        sws_, frame->width, frame->height, static_cast<AVPixelFormat>(frame->format), frame->width,
        frame->height, AV_PIX_FMT_BGR24, SWS_BILINEAR, nullptr, nullptr, nullptr);
    if (!sws_)
      throw std::runtime_error("sws_getCachedContext failed");
    int space = SWS_CS_DEFAULT;
    switch (frame->colorspace) {
      case AVCOL_SPC_BT709:
        space = SWS_CS_ITU709;
        break;
      case AVCOL_SPC_FCC:
        space = SWS_CS_FCC;
        break;
      case AVCOL_SPC_SMPTE240M:
        space = SWS_CS_SMPTE240M;
        break;
      case AVCOL_SPC_BT2020_NCL:
      case AVCOL_SPC_BT2020_CL:
        space = SWS_CS_BT2020;
        break;
      default:
        break;
    }
    const bool full_range =
        frame->color_range == AVCOL_RANGE_JPEG || frame->format == AV_PIX_FMT_YUVJ420P ||
        frame->format == AV_PIX_FMT_YUVJ422P || frame->format == AV_PIX_FMT_YUVJ444P ||
        frame->format == AV_PIX_FMT_YUVJ440P;
    const int* coefficients = sws_getCoefficients(space);
    if (sws_setColorspaceDetails(sws_, coefficients, full_range, coefficients, 1, 0, 1 << 16,
                                 1 << 16) < 0)
      throw std::runtime_error("setting decoded image color space failed");
    sensor_msgs::Image out;
    out.width = frame->width;
    out.height = frame->height;
    out.encoding = "bgr8";
    out.step = out.width * 3;
    out.data.resize(size_t(out.step) * out.height);
    uint8_t* dst[] = {out.data.data(), nullptr, nullptr, nullptr};
    int lines[] = {static_cast<int>(out.step), 0, 0, 0};
    if (sws_scale(sws_, frame->data, frame->linesize, 0, frame->height, dst, lines) !=
        frame->height)
      throw std::runtime_error("incomplete color conversion");
    return out;
  }

 private:
  SwsContext* sws_ = nullptr;
};
struct ContextDeleter {
  void operator()(AVCodecContext* p) const {
    avcodec_free_context(&p);
  }
};
struct FrameDeleter {
  void operator()(AVFrame* p) const {
    av_frame_free(&p);
  }
};
struct PacketDeleter {
  void operator()(AVPacket* p) const {
    av_packet_free(&p);
  }
};
using Frame = std::unique_ptr<AVFrame, FrameDeleter>;
class FfmpegBackend final : public Backend {
 public:
  FfmpegBackend(const AVCodec* codec, AVHWDeviceType type, AVPixelFormat pixel,
                const std::string& device, bool hardware)
      : context_(avcodec_alloc_context3(codec)), pixel_(pixel), codec_(codec) {
    (void)hardware;
    if (!context_)
      throw std::bad_alloc();
    auto* ctx = context_.get();
    ctx->thread_count = 1;
    ctx->flags |= AV_CODEC_FLAG_LOW_DELAY;
    ctx->pkt_timebase = AVRational{1, 1000000000};
    ctx->opaque = this;
    ctx->get_format = chooseFormat;
    if (type != AV_HWDEVICE_TYPE_NONE) {
      AVDictionary* options = nullptr;
      const char* selector = device == "auto" ? nullptr : device.c_str();
      if (type == AV_HWDEVICE_TYPE_QSV) {
        // QSV's device argument selects the MFX implementation; its DRM node
        // must be passed as child_device, otherwise every candidate uses the
        // same default GPU (and an unrecognized selector can allow software).
        av_dict_set(&options, "child_device", device.c_str(), 0);
        selector = "hw_any";
      }
      int code = av_hwdevice_ctx_create(&ctx->hw_device_ctx, type, selector, options, 0);
      av_dict_free(&options);
      if (code < 0)
        throw Error(Failure::backend, "hardware device creation: " + errorText(code));
    }
    if (!std::strstr(codec->name, "_cuvid") && !std::strstr(codec->name, "_v4l2m2m"))
      open();
  }
  void send(const std::vector<uint8_t>& bytes, int64_t token) override {
    if (!opened_) {
      // CUVID otherwise probes a default 1280x720; V4L2 requires coded
      // dimensions before initializing its queues. Probe the actual stream.
      probeDimensions(bytes);
      open();
    }
    std::unique_ptr<AVPacket, PacketDeleter> packet(av_packet_alloc());
    if (!packet || av_new_packet(packet.get(), static_cast<int>(bytes.size())) < 0)
      throw std::bad_alloc();
    std::copy(bytes.begin(), bytes.end(), packet->data);
    packet->pts = token;
    packet->dts = AV_NOPTS_VALUE;
    int code = avcodec_send_packet(context_.get(), packet.get());
    if (code == AVERROR(EAGAIN)) {
      auto frames = drain();
      ready_.insert(ready_.end(), std::make_move_iterator(frames.begin()),
                    std::make_move_iterator(frames.end()));
      code = avcodec_send_packet(context_.get(), packet.get());
    }
    if (code < 0)
      fail("sending packet", code);
    // HEVC can conceal a failed get_format() and return success/EAGAIN instead of INVALIDDATA.
    if (negotiation_failed_)
      throw Error(Failure::backend, "hardware pixel format negotiation failed");
  }
  std::vector<Picture> receive() override {
    auto frames = std::move(ready_);
    ready_.clear();
    auto next = drain();
    frames.insert(frames.end(), std::make_move_iterator(next.begin()),
                  std::make_move_iterator(next.end()));
    return frames;
  }

 private:
  void open() {
    int code = avcodec_open2(context_.get(), codec_, nullptr);
    if (code < 0)
      throw Error(Failure::backend,
                  "opening " + std::string(codec_->name) + ": " + errorText(code));
    opened_ = true;
  }
  void probeDimensions(const std::vector<uint8_t>& bytes) {
    std::unique_ptr<AVCodecContext, ContextDeleter> probe(avcodec_alloc_context3(nullptr));
    std::unique_ptr<AVCodecParserContext, decltype(&av_parser_close)> parser(
        av_parser_init(codec_->id), av_parser_close);
    if (!probe || !parser)
      throw Error(Failure::backend, "hardware stream parser unavailable");
    probe->codec_id = codec_->id;
    parser->flags |= PARSER_FLAG_COMPLETE_FRAMES;
    uint8_t* data = nullptr;
    int length = 0;
    // av_parser_parse2 may read AV_INPUT_BUFFER_PADDING_SIZE bytes past input.
    auto padded = bytes;
    padded.resize(bytes.size() + AV_INPUT_BUFFER_PADDING_SIZE, 0);
    av_parser_parse2(parser.get(), probe.get(), &data, &length, padded.data(),
                     static_cast<int>(bytes.size()), AV_NOPTS_VALUE, AV_NOPTS_VALUE, 0);
    int width = parser->width, height = parser->height;
    if (codec_->id == AV_CODEC_ID_MJPEG) {
      // The MJPEG parser splits frames but does not read SOF dimensions.
      for (size_t i = 0; i + 8 < bytes.size(); ++i)
        if (bytes[i] == 0xff &&
            (bytes[i + 1] == 0xc0 || bytes[i + 1] == 0xc1 || bytes[i + 1] == 0xc2)) {
          height = int(bytes[i + 5]) * 256 + bytes[i + 6];
          width = int(bytes[i + 7]) * 256 + bytes[i + 8];
          break;
        }
    }
    if (width <= 0 || height <= 0)
      throw Error(Failure::backend,
                  "cannot determine hardware stream dimensions before initialization");
    context_->width = width;
    context_->height = height;
    context_->coded_width = parser->coded_width > 0 ? parser->coded_width : width;
    context_->coded_height = parser->coded_height > 0 ? parser->coded_height : height;
    if (parser->format >= 0)
      context_->sw_pix_fmt = static_cast<AVPixelFormat>(parser->format);
  }
  static AVPixelFormat chooseFormat(AVCodecContext* ctx, const AVPixelFormat* formats) {
    auto& self = *static_cast<FfmpegBackend*>(ctx->opaque);
    for (auto p = formats; *p != AV_PIX_FMT_NONE; ++p) {
      if (self.pixel_ != AV_PIX_FMT_NONE && *p == self.pixel_)
        return *p;
      if (self.pixel_ == AV_PIX_FMT_NONE) {
        auto* desc = av_pix_fmt_desc_get(*p);
        if (desc && !(desc->flags & AV_PIX_FMT_FLAG_HWACCEL))
          return *p;  // Software codec, or a hardware-only wrapper delivering host frames.
      }
    }
    self.negotiation_failed_ = true;
    return AV_PIX_FMT_NONE;  // A hardware candidate must never silently decode in software.
  }
  [[noreturn]] void fail(const std::string& operation, int code) {
    const auto kind =
        !negotiation_failed_ && code == AVERROR_INVALIDDATA ? Failure::bitstream : Failure::backend;
    throw Error(kind, operation + ": " + errorText(code));
  }
  std::vector<Picture> drain() {
    std::vector<Picture> frames;
    Frame frame(av_frame_alloc()), host(av_frame_alloc());
    if (!frame || !host)
      throw std::bad_alloc();
    for (;;) {
      int code = avcodec_receive_frame(context_.get(), frame.get());
      if (negotiation_failed_)
        throw Error(Failure::backend, "hardware pixel format negotiation failed");
      if (code == AVERROR(EAGAIN) || code == AVERROR_EOF)
        break;
      if (code < 0)
        fail("receiving frame", code);
      if (frame->flags & AV_FRAME_FLAG_CORRUPT)
        throw Error(Failure::bitstream, "corrupt decoded image");
      AVFrame* output = frame.get();
      auto* desc = av_pix_fmt_desc_get(static_cast<AVPixelFormat>(frame->format));
      if (pixel_ != AV_PIX_FMT_NONE && frame->format != pixel_)
        throw Error(Failure::backend, "decoder did not produce the requested hardware surface");
      if (desc && (desc->flags & AV_PIX_FMT_FLAG_HWACCEL)) {
        code = av_hwframe_transfer_data(host.get(), frame.get(), 0);
        if (code < 0)
          throw Error(Failure::backend, "hardware frame download: " + errorText(code));
        // Surface download does not copy range/matrix metadata. Keep it so
        // full-range JPEG and BT.709/2020 streams retain their original colors.
        if (av_frame_copy_props(host.get(), frame.get()) < 0)
          throw std::bad_alloc();
        output = host.get();
      }
      Picture picture;
      picture.token = frame->best_effort_timestamp;
      picture.image = converter_.convert(output);
      frames.push_back(std::move(picture));
      av_frame_unref(frame.get());
      av_frame_unref(host.get());
    }
    return frames;
  }
  std::unique_ptr<AVCodecContext, ContextDeleter> context_;
  bool negotiation_failed_ = false;
  AVPixelFormat pixel_;
  const AVCodec* codec_;
  bool opened_ = false;
  Converter converter_;
  std::vector<Picture> ready_;
};
std::vector<std::string> paths(const char* pattern) {
  glob_t result{};
  std::vector<std::string> out;
  if (glob(pattern, 0, nullptr, &result) == 0)
    for (size_t i = 0; i < result.gl_pathc; ++i)
      out.emplace_back(result.gl_pathv[i]);
  globfree(&result);
  std::sort(out.begin(), out.end(), [](const auto& a, const auto& b) {
    auto number = [](const std::string& s) {
      auto p = s.find_last_not_of("0123456789");
      return p == s.size() - 1 ? 0 : std::stoi(s.substr(p + 1));
    };
    return std::make_pair(number(a), a) < std::make_pair(number(b), b);
  });
  return out;
}
std::vector<std::string> cudaDevices() {
  std::vector<std::string> out;
  void* library = dlopen("libcuda.so.1", RTLD_LAZY | RTLD_LOCAL);
  if (library) {
    auto init = reinterpret_cast<int (*)(unsigned)>(dlsym(library, "cuInit"));
    auto count = reinterpret_cast<int (*)(int*)>(dlsym(library, "cuDeviceGetCount"));
    int n = 0;
    if (init && count && init(0) == 0 && count(&n) == 0)
      for (int i = 0; i < n; ++i)
        out.push_back(std::to_string(i));
    dlclose(library);
  }
  if (out.empty())
    out.push_back("0");  // Probe failures remain visible in diagnostics.
  return out;
}
int priority(AVHWDeviceType type) {
  switch (type) {
    case AV_HWDEVICE_TYPE_CUDA:
      return 1;
    case AV_HWDEVICE_TYPE_VAAPI:
      return 2;
    case AV_HWDEVICE_TYPE_QSV:
      return 3;
    case AV_HWDEVICE_TYPE_VDPAU:
      return 5;
    default:
      return 99;
  }
}
}  // namespace
std::vector<Candidate> discover(sdk::OdinDataFormat format) {
  AVCodecID id = format == sdk::OdinDataFormat::kImageMjpeg  ? AV_CODEC_ID_MJPEG
                 : format == sdk::OdinDataFormat::kImageH264 ? AV_CODEC_ID_H264
                 : format == sdk::OdinDataFormat::kImageH265 ? AV_CODEC_ID_HEVC
                                                             : AV_CODEC_ID_NONE;
  if (id == AV_CODEC_ID_NONE)
    throw std::invalid_argument("unknown compressed camera format");
  const char* software_name = id == AV_CODEC_ID_MJPEG  ? "mjpeg"
                              : id == AV_CODEC_ID_H264 ? "h264"
                                                       : "hevc";
  const AVCodec* software = avcodec_find_decoder_by_name(software_name);
  struct Ranked {
    int rank;
    size_t device;
    Candidate candidate;
  };
  std::vector<Ranked> ranked;
  auto render_nodes = paths("/dev/dri/renderD*");
  if (render_nodes.empty())
    render_nodes.emplace_back("/dev/dri/renderD128");
  const auto cuda = cudaDevices();
  void* cursor = nullptr;
  while (const AVCodec* codec = av_codec_iterate(&cursor)) {
    if (codec->id != id || !av_codec_is_decoder(codec))
      continue;
    bool configured = false;
    for (int i = 0; const auto* config = avcodec_get_hw_config(codec, i); ++i) {
      int rank = priority(config->device_type);
      if (rank == 99 || !(config->methods & AV_CODEC_HW_CONFIG_METHOD_HW_DEVICE_CTX))
        continue;
      configured = true;
      const auto devices = config->device_type == AV_HWDEVICE_TYPE_CUDA ? cuda
                           : config->device_type == AV_HWDEVICE_TYPE_VDPAU
                               ? std::vector<std::string>{"auto"}
                               : render_nodes;
      for (size_t device = 0; device < devices.size(); ++device) {
        const auto path = devices[device];
        auto type = config->device_type;
        auto pixel = config->pix_fmt;
        Candidate candidate;
        candidate.name = std::string(av_hwdevice_get_type_name(type)) + "/" + codec->name;
        candidate.device = path;
        candidate.hardware = true;
        candidate.create = [codec, type, pixel, path] {
          return std::make_unique<FfmpegBackend>(codec, type, pixel, path, true);
        };
        ranked.push_back({rank, device, std::move(candidate)});
      }
    }
    // Older FFmpeg builds do not expose AVHWDeviceContext for these wrappers.
    if (!configured && std::strstr(codec->name, "_v4l2m2m")) {
      Candidate candidate;
      candidate.name = std::string("v4l2m2m/") + codec->name;
      candidate.device = "auto";  // Upstream FFmpeg owns device discovery for this wrapper.
      candidate.hardware = true;
      candidate.create = [codec] {
        return std::make_unique<FfmpegBackend>(codec, AV_HWDEVICE_TYPE_NONE, AV_PIX_FMT_NONE,
                                               "auto", true);
      };
      ranked.push_back({4, 0, std::move(candidate)});
    }
  }
  std::stable_sort(ranked.begin(), ranked.end(), [](const auto& a, const auto& b) {
    return std::tie(a.rank, a.device, a.candidate.name) <
           std::tie(b.rank, b.device, b.candidate.name);
  });
  auto out = jetsonCandidates(format);
  for (auto& entry : ranked)
    out.push_back(std::move(entry.candidate));
  if (software) {
    Candidate cpu;
    cpu.name = std::string("cpu/") + software->name;
    cpu.device = "cpu";
    cpu.create = [software] {
      return std::make_unique<FfmpegBackend>(software, AV_HWDEVICE_TYPE_NONE, AV_PIX_FMT_NONE,
                                             "cpu", false);
    };
    out.push_back(std::move(cpu));
  }
  return out;
}
sensor_msgs::Image convertNv12(const sdk::OdinCameraPacket& p) {
  if (!p.width || !p.height || p.width % 2 || p.height % 2 || p.width > 8192 || p.height > 8192 ||
      p.payload.size() != size_t(p.width) * p.height * 3 / 2)
    throw Error(Failure::bitstream, "NV12 payload size/dimensions mismatch");
  AVFrame frame{};
  frame.width = p.width;
  frame.height = p.height;
  frame.format = AV_PIX_FMT_NV12;
  av_image_fill_arrays(frame.data, frame.linesize, p.payload.data(), AV_PIX_FMT_NV12, p.width,
                       p.height, 1);
  Converter converter;
  return converter.convert(&frame);
}
}  // namespace odin_driver::decoding
