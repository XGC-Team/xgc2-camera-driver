#include "media_pipeline.hpp"

#include <algorithm>
#include <cmath>
#include <random>
#include <stdexcept>

extern "C" {
#include <libavutil/frame.h>
}

namespace xgc2_camera_driver {
namespace {

constexpr std::uint8_t kRtpPayloadType = 96;
constexpr std::int64_t kRtpClockRate = 90000;
constexpr std::int64_t kNanosecondsPerSecond = 1000000000;
constexpr auto kIdleWait = std::chrono::hours(24);

std::int64_t steadyNanoseconds()
{
  return std::chrono::duration_cast<std::chrono::nanoseconds>(
             std::chrono::steady_clock::now().time_since_epoch())
      .count();
}

std::uint32_t randomWord()
{
  static std::random_device device;
  return device();
}

std::uint32_t gopFrames(const double seconds, const double fps)
{
  const double frames = std::round(seconds * fps);
  return static_cast<std::uint32_t>(std::max(1.0, std::min(frames, 1e9)));
}

// The preview keeps the camera's aspect ratio and never upscales; both sides
// stay even as H.264 4:2:0 requires.
void previewGeometry(
    const std::uint32_t width,
    const std::uint32_t height,
    const std::uint32_t max_width,
    const std::uint32_t max_height,
    std::uint32_t* out_width,
    std::uint32_t* out_height)
{
  const double scale = std::min(
      1.0,
      std::min(
          static_cast<double>(max_width) / width, static_cast<double>(max_height) / height));
  *out_width = std::max<std::uint32_t>(
      16U, static_cast<std::uint32_t>(std::lround(width * scale)) & ~1U);
  *out_height = std::max<std::uint32_t>(
      16U, static_cast<std::uint32_t>(std::lround(height * scale)) & ~1U);
}

}  // namespace

MediaPipeline::MediaPipeline(
    MediaPipelineConfig config,
    PreviewSink preview_sink,
    RosClock ros_clock,
    Logger logger)
    : base_config_(config),
      preview_sink_(std::move(preview_sink)),
      ros_clock_(std::move(ros_clock)),
      logger_(std::move(logger)),
      config_(std::move(config))
{
  if (config_.rtp_port == 0U || !(config_.gop_seconds > 0.0) || !(config_.still_fps > 0.0) ||
      !(config_.still_gop_seconds > 0.0) || !(config_.preview_gop_seconds > 0.0) ||
      config_.bitrate < 10000U || config_.preview_bitrate < 10000U ||
      config_.preview_max_width < 16U || config_.preview_max_height < 16U) {
    throw std::invalid_argument("invalid media pipeline configuration");
  }
  if (!preview_sink_ || !ros_clock_) {
    throw std::invalid_argument("the media pipeline needs a preview sink and a ROS clock");
  }
  decode_thread_ = std::thread(&MediaPipeline::decodeLoop, this);
  encode_thread_ = std::thread(&MediaPipeline::encodeLoop, this);
}

MediaPipeline::~MediaPipeline()
{
  {
    std::lock_guard<std::mutex> lock(mutex_);
    stopping_ = true;
  }
  decode_cv_.notify_all();
  encode_cv_.notify_all();
  decode_thread_.join();
  encode_thread_.join();
  // Nobody can be served any more.
  std::vector<Completion> orphaned;
  {
    std::lock_guard<std::mutex> lock(mutex_);
    orphaned.swap(start_waiters_);
    orphaned.insert(orphaned.end(), keyframe_waiters_.begin(), keyframe_waiters_.end());
    keyframe_waiters_.clear();
    orphaned.insert(orphaned.end(), stop_waiters_.begin(), stop_waiters_.end());
    stop_waiters_.clear();
  }
  for (const auto& done : orphaned) {
    done("the camera is shutting down");
  }
}

void MediaPipeline::log(const LogLevel level, const std::string& message) const
{
  if (logger_) {
    logger_(level, message);
  }
}

void MediaPipeline::setSource(const MediaSource& source)
{
  std::lock_guard<std::mutex> lock(mutex_);
  source_ = source;
}

bool MediaPipeline::wantedLocked() const
{
  return state_ == State::Starting || state_ == State::Active ||
         (config_.preview_enabled && preview_wanted_);
}

void MediaPipeline::updateWantedLocked()
{
  wanted_.store(wantedLocked(), std::memory_order_release);
}

void MediaPipeline::submit(LiveFrame frame)
{
  if (!wanted()) {
    return;
  }
  auto shared = std::make_shared<const LiveFrame>(std::move(frame));
  {
    std::lock_guard<std::mutex> lock(mutex_);
    if (pending_) {
      ++stats_.frames_dropped;
    }
    pending_ = std::move(shared);
  }
  decode_cv_.notify_one();
}

void MediaPipeline::setStill(
    std::shared_ptr<const std::vector<std::uint8_t>> jpeg,
    const std::uint32_t width,
    const std::uint32_t height)
{
  JpegDecoder decoder;
  const Picture picture = decoder.decode(jpeg->data(), jpeg->size());
  if (picture.frame()->width != static_cast<int>(width) ||
      picture.frame()->height != static_cast<int>(height)) {
    throw std::invalid_argument("the still JPEG does not have the declared geometry");
  }
  {
    std::lock_guard<std::mutex> lock(mutex_);
    still_picture_ = picture;
    have_still_ = true;
    next_still_ = SteadyClock::now();
  }
  encode_cv_.notify_one();
}

void MediaPipeline::setPreviewWanted(const bool wanted)
{
  {
    std::lock_guard<std::mutex> lock(mutex_);
    if (wanted && !preview_wanted_) {
      preview_idr_pending_ = true;
    }
    preview_wanted_ = wanted;
    updateWantedLocked();
  }
  decode_cv_.notify_all();
  encode_cv_.notify_all();
}

void MediaPipeline::requestPreviewKeyframe()
{
  {
    std::lock_guard<std::mutex> lock(mutex_);
    if (preview_wanted_) {
      preview_idr_pending_ = true;
    }
  }
  encode_cv_.notify_one();
}

void MediaPipeline::start(Completion done)
{
  {
    std::unique_lock<std::mutex> lock(mutex_);
    if (state_ == State::Active) {
      lock.unlock();
      done("");
      return;
    }
    desired_active_ = true;
    state_ = State::Starting;
    last_error_.clear();
    rtp_idr_pending_ = true;
    start_waiters_.push_back(std::move(done));
    updateWantedLocked();
  }
  decode_cv_.notify_all();
  encode_cv_.notify_all();
}

void MediaPipeline::stop(Completion done)
{
  std::vector<Completion> failed;
  {
    std::unique_lock<std::mutex> lock(mutex_);
    desired_active_ = false;
    if (state_ == State::Idle) {
      lock.unlock();
      done("");
      return;
    }
    failed.swap(start_waiters_);
    failed.insert(failed.end(), keyframe_waiters_.begin(), keyframe_waiters_.end());
    keyframe_waiters_.clear();
    rtp_idr_pending_ = false;
    state_ = State::Stopping;
    stop_waiters_.push_back(std::move(done));
    updateWantedLocked();
  }
  for (const auto& waiter : failed) {
    waiter("the stream was stopped before it started");
  }
  encode_cv_.notify_all();
}

void MediaPipeline::requestKeyframe(Completion done)
{
  {
    std::unique_lock<std::mutex> lock(mutex_);
    if (state_ != State::Starting && state_ != State::Active) {
      lock.unlock();
      done("the source is not active");
      return;
    }
    rtp_idr_pending_ = true;
    keyframe_waiters_.push_back(std::move(done));
  }
  encode_cv_.notify_one();
}

bool MediaPipeline::setRtpDestination(const std::string& host, const std::uint16_t port)
{
  std::lock_guard<std::mutex> lock(mutex_);
  if (state_ != State::Idle && state_ != State::Faulted) {
    return false;
  }
  config_.rtp_host = host;
  config_.rtp_port = port;
  return true;
}

bool MediaPipeline::setBitrate(const std::uint32_t bitrate)
{
  std::lock_guard<std::mutex> lock(mutex_);
  if (state_ != State::Idle && state_ != State::Faulted) {
    return false;
  }
  config_.bitrate = bitrate;
  return true;
}

std::string MediaPipeline::rtpHost() const
{
  std::lock_guard<std::mutex> lock(mutex_);
  return config_.rtp_host;
}

std::uint16_t MediaPipeline::rtpPort() const
{
  std::lock_guard<std::mutex> lock(mutex_);
  return config_.rtp_port;
}

std::uint32_t MediaPipeline::bitrate() const
{
  std::lock_guard<std::mutex> lock(mutex_);
  return config_.bitrate;
}

MediaPipeline::Status MediaPipeline::status() const
{
  std::lock_guard<std::mutex> lock(mutex_);
  Status result = stats_;
  result.state = state_;
  result.desired_active = desired_active_;
  result.last_error = last_error_;
  result.encoder = encoder_name_;
  result.encoder_fallback = encoder_fallback_;
  result.preview_active = preview_open_;
  result.encode_milliseconds_average =
      encode_count_ == 0U ? 0.0 : encode_ms_total_ / static_cast<double>(encode_count_);
  return result;
}

void MediaPipeline::decodeLoop()
{
  for (;;) {
    std::shared_ptr<const LiveFrame> frame;
    {
      std::unique_lock<std::mutex> lock(mutex_);
      decode_cv_.wait(lock, [this] { return stopping_ || (pending_ && wantedLocked()); });
      if (stopping_) {
        return;
      }
      frame = std::move(pending_);
    }
    Picture picture;
    try {
      picture = jpeg_decoder_.decode(frame->jpeg->data(), frame->jpeg->size());
    } catch (const std::exception& error) {
      std::uint64_t errors = 0;
      {
        std::lock_guard<std::mutex> lock(mutex_);
        errors = ++stats_.decode_errors;
      }
      if (errors == 1U || errors % 100U == 0U) {
        log(LogLevel::Warn, std::string("JPEG frame could not be decoded: ") + error.what());
      }
      continue;
    }
    {
      std::lock_guard<std::mutex> lock(mutex_);
      if (!wantedLocked()) {
        continue;
      }
      if (have_ready_) {
        ++stats_.frames_dropped;
      }
      ready_picture_ = std::move(picture);
      ready_frame_ = *frame;
      have_ready_ = true;
      ++stats_.pictures_decoded;
    }
    encode_cv_.notify_one();
  }
}

bool MediaPipeline::stillDueLocked(const SteadyClock::time_point now) const
{
  return source_.still && have_still_ && wantedLocked() &&
         (now >= next_still_ || rtp_idr_pending_ || preview_idr_pending_);
}

void MediaPipeline::encodeLoop()
{
  std::unique_lock<std::mutex> lock(mutex_);
  for (;;) {
    const auto deadline =
        source_.still && have_still_ && wantedLocked() ? next_still_
                                                       : SteadyClock::now() + kIdleWait;
    encode_cv_.wait_until(lock, deadline, [this] {
      return stopping_ || state_ == State::Stopping || (have_ready_ && wantedLocked()) ||
             stillDueLocked(SteadyClock::now()) ||
             (!wantedLocked() && (have_ready_ || pending_)) ||
             // a preview encoder whose last subscriber left must be released
             (preview_open_ && !(config_.preview_enabled && preview_wanted_));
    });
    if (stopping_) {
      return;
    }

    bool close_rtp = false;
    std::vector<Completion> stopped;
    if (state_ == State::Stopping) {
      close_rtp = true;
      stopped.swap(stop_waiters_);
    }
    Work work;
    bool have_work = false;
    bool close_preview_encoder = false;
    if (!wantedLocked()) {
      // Nothing is watching: forget frames that were decoded for nobody.
      have_ready_ = false;
      ready_picture_ = Picture();
      pending_.reset();
      close_preview_encoder = true;
    } else {
      close_preview_encoder = !(config_.preview_enabled && preview_wanted_);
      have_work = selectWorkLocked(&work);
    }
    lock.unlock();

    if (close_rtp) {
      closeRtp();
      std::lock_guard<std::mutex> relock(mutex_);
      if (state_ == State::Stopping) {
        state_ = State::Idle;
      }
    }
    for (const auto& done : stopped) {
      done("");
    }
    if (close_preview_encoder) {
      closePreview();
    }
    if (have_work) {
      if (work.want_rtp) {
        rtpStep(&work);
      }
      if (work.want_preview) {
        previewStep(work);
      }
    }
    lock.lock();
  }
}

bool MediaPipeline::selectWorkLocked(Work* work)
{
  const bool rtp = state_ == State::Starting || state_ == State::Active;
  const bool preview = config_.preview_enabled && preview_wanted_;
  if (!rtp && !preview) {
    return false;
  }
  const auto now = SteadyClock::now();
  if (source_.still) {
    if (!stillDueLocked(now)) {
      return false;
    }
    work->still = true;
    work->picture = still_picture_;
    work->capture_ns = steadyNanoseconds();
    work->ros_time_ns = ros_clock_();
    next_still_ = now + std::chrono::nanoseconds(
                            static_cast<std::int64_t>(1e9 / config_.still_fps));
  } else {
    if (!have_ready_) {
      return false;
    }
    work->picture = std::move(ready_picture_);
    work->frame = ready_frame_;
    work->capture_ns = ready_frame_.capture_ns;
    work->ros_time_ns = ready_frame_.ros_time_ns;
    ready_picture_ = Picture();
    have_ready_ = false;
  }
  work->want_rtp = rtp;
  work->want_preview = preview;
  if (rtp) {
    work->rtp_idr = rtp_idr_pending_;
    rtp_idr_pending_ = false;
    work->keyframe_waiters.swap(keyframe_waiters_);
  }
  if (preview) {
    work->preview_idr = preview_idr_pending_;
    preview_idr_pending_ = false;
  }
  return true;
}

std::int64_t MediaPipeline::rtpTicks(const std::int64_t capture_ns)
{
  std::int64_t ticks = 0;
  if (rtp_last_ticks_ >= 0) {
    ticks = (capture_ns - rtp_origin_ns_) * kRtpClockRate / kNanosecondsPerSecond;
  } else {
    rtp_origin_ns_ = capture_ns;
  }
  if (ticks <= rtp_last_ticks_) {
    ticks = rtp_last_ticks_ + 1;
  }
  rtp_last_ticks_ = ticks;
  return ticks;
}

void MediaPipeline::openRtp(const Picture& picture)
{
  MediaPipelineConfig config;
  MediaSource source;
  {
    std::lock_guard<std::mutex> lock(mutex_);
    config = config_;
    source = source_;
  }
  H264EncoderSettings settings;
  settings.width = static_cast<std::uint32_t>(picture.frame()->width);
  settings.height = static_cast<std::uint32_t>(picture.frame()->height);
  if ((settings.width % 2U) != 0U || (settings.height % 2U) != 0U) {
    throw std::runtime_error(
        "H.264 needs an even picture size; the camera delivers " +
        std::to_string(settings.width) + "x" + std::to_string(settings.height));
  }
  settings.fps = source.still ? config.still_fps : source.fps;
  settings.encoder = config.encoder;
  settings.bitrate = config.bitrate;
  settings.constant_quality = config.constant_quality;
  settings.quality = config.quality;
  settings.preset = config.preset;
  settings.gop = gopFrames(source.still ? config.still_gop_seconds : config.gop_seconds, settings.fps);
  auto encoder = H264Encoder::open(settings);
  auto sender = std::unique_ptr<UdpSender>(new UdpSender(config.rtp_host, config.rtp_port));
  packetizer_.reset(new H264RtpPacketizer(
      kRtpPayloadType, randomWord(), static_cast<std::uint16_t>(randomWord())));
  rtp_base_ = randomWord();
  rtp_last_ticks_ = -1;
  rtp_reconstruction_ = Picture();
  sender_ = std::move(sender);
  rtp_encoder_ = std::move(encoder);
  {
    std::lock_guard<std::mutex> lock(mutex_);
    encoder_name_ = rtp_encoder_->name();
    encoder_fallback_ = rtp_encoder_->fallback_reason();
  }
  if (!rtp_encoder_->fallback_reason().empty()) {
    log(LogLevel::Warn, rtp_encoder_->fallback_reason() + "; encoding with libx264");
  }
  log(LogLevel::Info,
      "H.264 stream started: " + rtp_encoder_->name() + " " + std::to_string(settings.width) + "x" +
          std::to_string(settings.height) + " gop=" + std::to_string(settings.gop) + " -> " +
          config.rtp_host + ":" + std::to_string(config.rtp_port));
}

void MediaPipeline::closeRtp()
{
  rtp_encoder_.reset();
  sender_.reset();
  packetizer_.reset();
  rtp_reconstruction_ = Picture();
  std::lock_guard<std::mutex> lock(mutex_);
  encoder_name_.clear();
  encoder_fallback_.clear();
}

void MediaPipeline::closePreview()
{
  if (!preview_encoder_) {
    return;
  }
  preview_encoder_.reset();
  preview_reconstruction_ = Picture();
  preview_last_ticks_ = -1;
  std::lock_guard<std::mutex> lock(mutex_);
  preview_open_ = false;
}

void MediaPipeline::fault(const std::string& error)
{
  std::vector<Completion> failed;
  bool stopped = false;
  {
    std::lock_guard<std::mutex> lock(mutex_);
    stopped = state_ == State::Stopping;
    if (!stopped) {
      state_ = State::Faulted;
      last_error_ = error;
      failed.swap(start_waiters_);
      failed.insert(failed.end(), keyframe_waiters_.begin(), keyframe_waiters_.end());
      keyframe_waiters_.clear();
      updateWantedLocked();
    }
  }
  log(LogLevel::Error, "H.264 stream failed: " + error);
  closeRtp();
  for (const auto& waiter : failed) {
    waiter(error);
  }
}

void MediaPipeline::rtpStep(Work* work)
{
  try {
    if (!rtp_encoder_) {
      openRtp(work->picture);
    }
    Picture input = work->picture;
    bool force = work->rtp_idr;
    if (work->still) {
      // Only the IDR is encoded from the original; its reconstruction feeds
      // every following picture so the scene stays free of refinement bits.
      force = force || !rtp_reconstruction_;
      if (!force) {
        input = rtp_reconstruction_;
      }
    }
    const std::int64_t ticks = rtpTicks(work->capture_ns);
    std::vector<EncodedAccessUnit> units;
    const auto started = std::chrono::steady_clock::now();
    rtp_encoder_->encode(input, ticks, force, &units);
    std::vector<std::vector<std::uint8_t>> packets;
    bool keyframe = false;
    for (const auto& unit : units) {
      packets.clear();
      packetizer_->packetize(
          unit.data.data(),
          unit.data.size(),
          rtp_base_ + static_cast<std::uint32_t>(unit.pts),
          &packets);
      sender_->send(packets);
      if (unit.keyframe) {
        keyframe = true;
        if (work->still) {
          rtp_reconstruction_ = reconstruction_.decode(unit.data);
        }
      }
    }
    const double milliseconds =
        std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - started)
            .count();

    std::vector<Completion> started_waiters;
    std::vector<Completion> keyframe_waiters;
    {
      std::lock_guard<std::mutex> lock(mutex_);
      stats_.access_units += units.size();
      stats_.keyframes += keyframe ? 1U : 0U;
      stats_.rtp_packets = sender_->packets_sent();
      stats_.rtp_bytes = sender_->bytes_sent();
      stats_.rtp_send_errors = sender_->send_errors();
      encode_ms_total_ += milliseconds;
      ++encode_count_;
      if (state_ == State::Starting && !units.empty()) {
        state_ = State::Active;
        started_waiters.swap(start_waiters_);
      }
      if (keyframe) {
        keyframe_waiters.swap(work->keyframe_waiters);
      } else if (!work->keyframe_waiters.empty()) {
        // The requested keyframe has not been produced yet; ask again.
        keyframe_waiters_.insert(
            keyframe_waiters_.end(),
            work->keyframe_waiters.begin(),
            work->keyframe_waiters.end());
        work->keyframe_waiters.clear();
        rtp_idr_pending_ = true;
      }
    }
    for (const auto& waiter : started_waiters) {
      waiter("");
    }
    for (const auto& waiter : keyframe_waiters) {
      waiter("");
    }
  } catch (const std::exception& error) {
    // Waiters of this step get the failure together with everything queued.
    {
      std::lock_guard<std::mutex> lock(mutex_);
      keyframe_waiters_.insert(
          keyframe_waiters_.end(),
          work->keyframe_waiters.begin(),
          work->keyframe_waiters.end());
      work->keyframe_waiters.clear();
    }
    fault(error.what());
  }
}

void MediaPipeline::previewStep(const Work& work)
{
  try {
    MediaPipelineConfig config;
    MediaSource source;
    {
      std::lock_guard<std::mutex> lock(mutex_);
      if (!config_.preview_enabled || !preview_wanted_) {
        return;
      }
      config = config_;
      source = source_;
    }
    if (!preview_encoder_) {
      H264EncoderSettings settings;
      previewGeometry(
          static_cast<std::uint32_t>(work.picture.frame()->width),
          static_cast<std::uint32_t>(work.picture.frame()->height),
          config.preview_max_width,
          config.preview_max_height,
          &settings.width,
          &settings.height);
      settings.fps = source.still ? config.still_fps : source.fps;
      settings.encoder = config.encoder;
      settings.bitrate = config.preview_bitrate;
      settings.constant_quality = source.still || config.constant_quality;
      settings.quality = config.quality;
      settings.preset = config.preset;
      settings.gop = gopFrames(config.preview_gop_seconds, settings.fps);
      preview_encoder_ = H264Encoder::open(settings);
      preview_reconstruction_ = Picture();
      preview_last_ticks_ = -1;
      {
        std::lock_guard<std::mutex> lock(mutex_);
        preview_open_ = true;
      }
      log(LogLevel::Info,
          "H.264 preview started: " + preview_encoder_->name() + " " +
              std::to_string(settings.width) + "x" + std::to_string(settings.height));
    }
    Picture input = work.picture;
    bool force = work.preview_idr;
    if (work.still) {
      force = force || !preview_reconstruction_;
      if (!force) {
        input = preview_reconstruction_;
      }
    }
    std::int64_t ticks = 0;
    if (preview_last_ticks_ >= 0) {
      ticks = (work.capture_ns - preview_origin_ns_) * kRtpClockRate / kNanosecondsPerSecond;
    } else {
      preview_origin_ns_ = work.capture_ns;
    }
    if (ticks <= preview_last_ticks_) {
      ticks = preview_last_ticks_ + 1;
    }
    preview_last_ticks_ = ticks;
    std::vector<EncodedAccessUnit> units;
    preview_encoder_->encode(input, ticks, force, &units);
    std::uint64_t bytes = 0;
    for (auto& unit : units) {
      bytes += unit.data.size();
      if (unit.keyframe && work.still) {
        preview_reconstruction_ = reconstruction_.decode(unit.data);
      }
      PreviewUnit preview;
      preview.ros_time_ns = work.ros_time_ns;
      preview.keyframe = unit.keyframe;
      preview.data = std::move(unit.data);
      preview_sink_(std::move(preview));
    }
    std::lock_guard<std::mutex> lock(mutex_);
    stats_.preview_access_units += units.size();
    stats_.preview_bytes += bytes;
  } catch (const std::exception& error) {
    // A failed preview must not take the media-edge stream down with it.
    preview_encoder_.reset();
    preview_reconstruction_ = Picture();
    preview_last_ticks_ = -1;
    {
      std::lock_guard<std::mutex> lock(mutex_);
      preview_open_ = false;
    }
    if (preview_errors_++ % 100U == 0U) {
      log(LogLevel::Error, std::string("H.264 preview failed: ") + error.what());
    }
  }
}

}  // namespace xgc2_camera_driver
