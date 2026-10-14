#include "odin2_ros_driver/decoder.hpp"

#include "odin2_ros_driver/conversion.hpp"
extern "C" {
#include <libavcodec/avcodec.h>
#include <libavutil/imgutils.h>
#include <libswscale/swscale.h>
}
#include <map>
#include <stdexcept>

namespace odin_driver {
struct Decoder::Impl {
  AVCodecContext* ctx = nullptr;
  SwsContext* sws = nullptr;
  sdk::OdinDataFormat format = sdk::OdinDataFormat::kUnknown;
  bool waiting_keyframe = true;
  std::map<unsigned, std::vector<uint8_t>> parameters;
  std::map<int64_t, std_msgs::Header> headers;
  ~Impl() {
    avcodec_free_context(&ctx);
    sws_freeContext(sws);
  }
  sensor_msgs::Image convert(AVFrame* frame, const std_msgs::Header& header) {
    if (frame->width <= 0 || frame->height <= 0 || frame->width > 8192 || frame->height > 8192)
      throw std::runtime_error("invalid decoded dimensions");
    sws = sws_getCachedContext(
        sws, frame->width, frame->height, static_cast<AVPixelFormat>(frame->format), frame->width,
        frame->height, AV_PIX_FMT_BGR24, SWS_BILINEAR, nullptr, nullptr, nullptr);
    if (!sws)
      throw std::runtime_error("sws_getCachedContext failed");
    sensor_msgs::Image out;
    out.header = header;
    out.width = frame->width;
    out.height = frame->height;
    out.encoding = "bgr8";
    out.step = out.width * 3;
    out.data.resize(size_t(out.step) * out.height);
    uint8_t* dst[] = {out.data.data(), nullptr, nullptr, nullptr};
    int lines[] = {static_cast<int>(out.step), 0, 0, 0};
    if (sws_scale(sws, frame->data, frame->linesize, 0, frame->height, dst, lines) != frame->height)
      throw std::runtime_error("incomplete color conversion");
    return out;
  }
};
Decoder::Decoder() : impl_(new Impl) {}
Decoder::~Decoder() = default;
void Decoder::reset(bool preserve_parameters) {
  auto parameters = preserve_parameters ? std::move(impl_->parameters)
                                        : std::map<unsigned, std::vector<uint8_t>>{};
  auto format = impl_->format;
  impl_.reset(new Impl);
  if (preserve_parameters) {
    impl_->parameters = std::move(parameters);
    impl_->format = format;
  }
}
std::vector<sensor_msgs::Image> Decoder::decode(const sdk::OdinCameraPacket& p,
                                                const std_msgs::Header& header) {
  if (p.payload.empty())
    throw std::runtime_error("empty image payload");
  if (p.format != impl_->format) {
    reset();
    impl_->format = p.format;
  }
  std::vector<sensor_msgs::Image> result;
  using FramePtr = std::unique_ptr<AVFrame, void (*)(AVFrame*)>;
  FramePtr frame(av_frame_alloc(), [](AVFrame* f) { av_frame_free(&f); });
  if (!frame)
    throw std::bad_alloc();
  if (p.format == sdk::OdinDataFormat::kImageNv12) {
    if (!p.width || !p.height || p.width % 2 || p.height % 2 || p.width > 8192 || p.height > 8192 ||
        p.payload.size() != size_t(p.width) * p.height * 3 / 2)
      throw std::runtime_error("NV12 payload size/dimensions mismatch");
    frame->width = p.width;
    frame->height = p.height;
    frame->format = AV_PIX_FMT_NV12;
    av_image_fill_arrays(frame->data, frame->linesize, p.payload.data(), AV_PIX_FMT_NV12, p.width,
                         p.height, 1);
    result.push_back(impl_->convert(frame.get(), header));
    return result;
  }
  bool video =
      p.format == sdk::OdinDataFormat::kImageH264 || p.format == sdk::OdinDataFormat::kImageH265;
  bool has_picture = !video;
  if (video) {
    std::vector<std::pair<size_t, size_t>> starts;
    for (size_t i = 0; i + 3 < p.payload.size();) {
      size_t length = 0;
      if (p.payload[i] == 0 && p.payload[i + 1] == 0 && p.payload[i + 2] == 1)
        length = 3;
      else if (i + 4 < p.payload.size() && p.payload[i] == 0 && p.payload[i + 1] == 0 &&
               p.payload[i + 2] == 0 && p.payload[i + 3] == 1)
        length = 4;
      if (length) {
        starts.emplace_back(i, length);
        i += length;
      } else
        ++i;
    }
    for (size_t i = 0; i < starts.size(); ++i) {
      size_t begin = starts[i].first, nal = begin + starts[i].second;
      size_t end = i + 1 < starts.size() ? starts[i + 1].first : p.payload.size();
      if (nal >= end)
        continue;
      unsigned type = p.format == sdk::OdinDataFormat::kImageH264 ? p.payload[nal] & 31
                                                                  : (p.payload[nal] >> 1) & 63;
      bool parameter = p.format == sdk::OdinDataFormat::kImageH264 ? type == 7 || type == 8
                                                                   : type >= 32 && type <= 34;
      has_picture |=
          p.format == sdk::OdinDataFormat::kImageH264 ? type >= 1 && type <= 5 : type <= 31;
      if (parameter && end - begin <= 256 * 1024)
        impl_->parameters[type] = {p.payload.begin() + begin, p.payload.begin() + end};
    }
  }
  if (!has_picture)
    return result;  // Cache separately delivered SPS/PPS/VPS.
  if (video && impl_->waiting_keyframe && !cameraKeyframe(p))
    return result;
  if (!impl_->ctx) {
    AVCodecID id;
    if (p.format == sdk::OdinDataFormat::kImageMjpeg)
      id = AV_CODEC_ID_MJPEG;
    else if (p.format == sdk::OdinDataFormat::kImageH264)
      id = AV_CODEC_ID_H264;
    else if (p.format == sdk::OdinDataFormat::kImageH265)
      id = AV_CODEC_ID_HEVC;
    else
      throw std::runtime_error("unknown camera format");
    const AVCodec* codec = avcodec_find_decoder(id);
    if (!codec)
      throw std::runtime_error("decoder unavailable");
    impl_->ctx = avcodec_alloc_context3(codec);
    if (!impl_->ctx)
      throw std::bad_alloc();
    impl_->ctx->thread_count = 1;
    impl_->ctx->flags |= AV_CODEC_FLAG_LOW_DELAY;
    if (avcodec_open2(impl_->ctx, codec, nullptr) < 0)
      throw std::runtime_error("avcodec_open2 failed");
  }
  using PacketPtr = std::unique_ptr<AVPacket, void (*)(AVPacket*)>;
  PacketPtr pkt(av_packet_alloc(), [](AVPacket* v) { av_packet_free(&v); });
  size_t parameter_bytes = 0;
  if (video && impl_->waiting_keyframe)
    for (const auto& pair : impl_->parameters)
      parameter_bytes += pair.second.size();
  if (!pkt || p.payload.size() + parameter_bytes > 64 * 1024 * 1024 ||
      av_new_packet(pkt.get(), static_cast<int>(p.payload.size() + parameter_bytes)) < 0)
    throw std::runtime_error("encoded frame too large/allocation failed");
  size_t offset = 0;
  if (parameter_bytes)
    for (const auto& pair : impl_->parameters) {
      std::copy(pair.second.begin(), pair.second.end(), pkt->data + offset);
      offset += pair.second.size();
    }
  std::copy(p.payload.begin(), p.payload.end(), pkt->data + offset);
  pkt->pts = pkt->dts = static_cast<int64_t>(p.timestamp);
  impl_->headers[pkt->pts] = header;
  while (impl_->headers.size() > 64)
    impl_->headers.erase(impl_->headers.begin());
  if (avcodec_send_packet(impl_->ctx, pkt.get()) < 0) {
    reset(true);
    throw std::runtime_error("decoder rejected packet");
  }
  impl_->waiting_keyframe = false;
  while (true) {
    int status = avcodec_receive_frame(impl_->ctx, frame.get());
    if (status == AVERROR(EAGAIN) || status == AVERROR_EOF)
      break;
    if (status < 0 || (frame->flags & AV_FRAME_FLAG_CORRUPT)) {
      reset(true);
      throw std::runtime_error("corrupt decoded image");
    }
    auto it = impl_->headers.find(frame->best_effort_timestamp);
    const auto h = it == impl_->headers.end() ? header : it->second;
    result.push_back(impl_->convert(frame.get(), h));
    if (it != impl_->headers.end())
      impl_->headers.erase(it);
    av_frame_unref(frame.get());
  }
  return result;
}
}  // namespace odin_driver
