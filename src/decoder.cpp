#include "odin2_ros_driver/decoder.hpp"

#include <algorithm>
#include <map>
#include <optional>

#include "odin2_ros_driver/conversion.hpp"

namespace odin_driver {
struct Decoder::Impl {
  struct Choice {
    decoding::Candidate candidate;
    bool failed = false;
    std::chrono::steady_clock::time_point retry_at;
  };
  decoding::Discovery discover;
  decoding::Clock clock;
  std::vector<Choice> choices;
  std::unique_ptr<decoding::Backend> backend;
  std::optional<size_t> selected;
  DecoderInfo status;
  sdk::OdinDataFormat format = sdk::OdinDataFormat::kUnknown;
  uint16_t width = 0, height = 0;
  bool parameters_changed = false;
  bool waiting_keyframe = true;
  int64_t next_token = 0;
  std::map<unsigned, std::vector<uint8_t>> parameters;
  std::map<int64_t, DecodeContext> contexts;
  struct Recovery {
    sdk::OdinCameraPacket packet;
    DecodeContext context;
    int64_t token;
  };
  std::optional<Recovery> recovery;
  std::chrono::steady_clock::time_point last_progress;
  Impl(decoding::Discovery discovery, decoding::Clock now)
      : discover(std::move(discovery)), clock(std::move(now)) {}
  void reset(bool preserve) {
    backend.reset();  // Stop/join pipeline before discarding its input contexts.
    selected.reset();
    contexts.clear();
    recovery.reset();
    waiting_keyframe = true;
    status.hardware_active = false;
    if (!preserve) {
      parameters.clear();
      choices.clear();
      format = sdk::OdinDataFormat::kUnknown;
      width = height = 0;
      status = {};
    }
  }
  void fail(size_t index, const std::string& reason) {
    auto& choice = choices.at(index);
    choice.failed = true;
    choice.retry_at = clock() + std::chrono::seconds(30);
    auto message = choice.candidate.name + " [" + choice.candidate.device + "]: " + reason;
    if (message.size() > 1024)
      message.resize(1024);
    if (status.failures.size() == 64)
      status.failures.erase(status.failures.begin());
    status.failures.push_back(message);
    status.fallback_reason = message;
  }
  void choose() {
    if (choices.empty())
      for (auto& candidate : discover(format))
        choices.push_back({std::move(candidate), false, {}});
    for (size_t i = 0; i < choices.size(); ++i) {
      auto& choice = choices[i];
      if (choice.failed && clock() < choice.retry_at)
        continue;
      try {
        auto created = choice.candidate.create();
        if (!created)
          throw std::runtime_error("backend factory returned null");
        backend = std::move(created);
        selected = i;
        choice.failed = false;
        status.backend = choice.candidate.name;
        status.device = choice.candidate.device;
        status.hardware_active = false;  // Confirm only after an actual output frame.
        last_progress = clock();
        if (choice.candidate.hardware)
          status.fallback_reason.clear();
        else if (status.failures.empty())
          status.fallback_reason = "no compiled hardware decoder supports this format";
        return;
      } catch (const std::bad_alloc&) {
        throw;
      } catch (const std::exception& error) {
        fail(i, error.what());
      }
    }
    throw std::runtime_error("all camera decoders unavailable");
  }
  bool retryHardware() const {
    if (!selected)
      return false;
    for (size_t i = 0; i < *selected; ++i)
      if (choices[i].candidate.hardware && choices[i].failed && clock() >= choices[i].retry_at)
        return true;
    return false;
  }
  std::vector<DecodedFrame> receive() {
    auto pictures = backend->receive();
    std::vector<DecodedFrame> out;
    for (auto& picture : pictures) {
      auto found = contexts.find(picture.token);
      if (found == contexts.end())
        throw decoding::Error(decoding::Failure::backend,
                              "decoder output lost its input timestamp/context");
      picture.image.header = found->second.header;
      out.push_back({std::move(picture.image), found->second});
      contexts.erase(found);
      if (recovery && recovery->token == picture.token)
        recovery.reset();
      status.hardware_active = choices.at(*selected).candidate.hardware;
      last_progress = clock();
    }
    // EAGAIN/reorder delay alone is normal; only a sustained backlog is a stall.
    if (backend->asynchronous() && contexts.size() >= 8 &&
        clock() - last_progress > std::chrono::seconds(5))
      throw decoding::Error(decoding::Failure::backend,
                            "hardware decoder made no progress for 5 s");
    return out;
  }
  bool cacheParameters(const sdk::OdinCameraPacket& p) {
    parameters_changed = false;
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
    bool has_picture = false;
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
      if (parameter && end - begin <= 256 * 1024) {
        std::vector<uint8_t> value{p.payload.begin() + begin, p.payload.begin() + end};
        auto old = parameters.find(type);
        parameters_changed |= old != parameters.end() && old->second != value;
        parameters[type] = std::move(value);
      }
    }
    return has_picture;
  }
};
Decoder::Decoder() : Decoder(decoding::discover, [] { return std::chrono::steady_clock::now(); }) {}
Decoder::Decoder(decoding::Discovery discovery, decoding::Clock clock)
    : impl_(new Impl(std::move(discovery), std::move(clock))) {}
Decoder::~Decoder() = default;
void Decoder::reset(bool preserve_parameters) {
  impl_->reset(preserve_parameters);
}
DecoderInfo Decoder::info() const {
  return impl_->status;
}
bool Decoder::active() const {
  return bool(impl_->backend);
}
bool Decoder::pending() const {
  return impl_->backend && impl_->backend->asynchronous() && !impl_->contexts.empty();
}
std::vector<sensor_msgs::Image> Decoder::decode(const sdk::OdinCameraPacket& p,
                                                const std_msgs::Header& header) {
  DecodeContext context;
  context.header = header;
  auto frames = submit(p, context);
  std::vector<sensor_msgs::Image> out;
  for (auto& frame : frames)
    out.push_back(std::move(frame.image));
  return out;
}
std::vector<DecodedFrame> Decoder::submit(const sdk::OdinCameraPacket& p,
                                          const DecodeContext& context) {
  auto& state = *impl_;
  if (p.payload.empty() || p.payload.size() > 64 * 1024 * 1024)
    throw decoding::Error(decoding::Failure::bitstream, "empty/oversized image payload");
  if (p.format != state.format) {
    reset();
    state.format = p.format;
  }
  if (p.format == sdk::OdinDataFormat::kImageNv12) {
    auto image = decoding::convertNv12(p);
    image.header = context.header;
    state.status.backend = "raw/nv12-conversion";
    state.status.device = "cpu";
    return {{std::move(image), context}};
  }
  const bool video =
      p.format == sdk::OdinDataFormat::kImageH264 || p.format == sdk::OdinDataFormat::kImageH265;
  const bool keyframe = !video || cameraKeyframe(p);
  const bool has_picture = !video || state.cacheParameters(p);
  const bool dimensions_changed = p.width && p.height && state.width && state.height &&
                                  (p.width != state.width || p.height != state.height);
  if (dimensions_changed || (video && state.parameters_changed)) {
    // New stream capabilities invalidate failures observed with the old
    // resolution/profile. Keep the newly received parameter sets for recovery.
    state.reset(true);
    state.choices.clear();
    state.status = {};
  }
  if (p.width && p.height) {
    state.width = p.width;
    state.height = p.height;
  }
  if (!has_picture)
    return poll();
  if (video && state.waiting_keyframe && !keyframe)
    return {};
  if (keyframe && state.retryHardware())
    state.reset(true);
  std::vector<uint8_t> bytes;
  if (video && (state.waiting_keyframe || keyframe))
    for (const auto& entry : state.parameters)
      bytes.insert(bytes.end(), entry.second.begin(), entry.second.end());
  if (bytes.size() + p.payload.size() > 64 * 1024 * 1024)
    throw decoding::Error(decoding::Failure::bitstream,
                          "encoded frame with parameters exceeds 64 MiB");
  bytes.insert(bytes.end(), p.payload.begin(), p.payload.end());
  for (;;) {
    if (!state.backend)
      state.choose();
    const auto index = *state.selected;
    try {
      if (state.contexts.size() >= 64)
        throw decoding::Error(decoding::Failure::backend, "decoder exceeded 64 pending frames");
      const auto token = ++state.next_token * 1000000;  // Survives V4L2 microsecond transport.
      state.contexts[token] = context;
      state.backend->send(bytes, token);
      if (state.backend->asynchronous() && keyframe)
        state.recovery = Impl::Recovery{p, context, token};
      auto frames = state.receive();
      state.waiting_keyframe = false;
      return frames;
    } catch (const decoding::Error& error) {
      const bool recoverable =
          error.kind == decoding::Failure::backend && state.choices.at(index).candidate.hardware;
      if (recoverable)
        state.fail(index, error.what());
      state.reset(true);
      if (!recoverable || !keyframe)
        throw;
      // Retry this independent frame on another hardware decoder, then CPU.
    }
  }
}
std::vector<DecodedFrame> Decoder::poll() {
  auto& state = *impl_;
  if (!state.backend)
    return {};
  try {
    return state.receive();
  } catch (const decoding::Error& error) {
    const bool hardware_failure = error.kind == decoding::Failure::backend &&
                                  state.choices.at(*state.selected).candidate.hardware;
    std::optional<Impl::Recovery> retry;
    if (hardware_failure && state.recovery &&
        (state.recovery->packet.format == sdk::OdinDataFormat::kImageMjpeg ||
         state.contexts.size() == 1))
      retry = std::move(state.recovery);
    if (hardware_failure)
      state.fail(*state.selected, error.what());
    state.reset(true);
    if (retry)
      return submit(retry->packet, retry->context);
    throw;
  }
}
}  // namespace odin_driver
