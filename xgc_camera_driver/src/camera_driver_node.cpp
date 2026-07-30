#include <algorithm>
#include <atomic>
#include <chrono>
#include <condition_variable>
#include <cstdint>
#include <cstring>
#include <deque>
#include <limits>
#include <map>
#include <memory>
#include <mutex>
#include <stdexcept>
#include <string>
#include <thread>
#include <vector>

#include <unistd.h>

extern "C" {
#include <libavcodec/avcodec.h>
#include <libavutil/error.h>
#include <libswscale/swscale.h>
}

#include <camera_info_manager/camera_info_manager.h>
#include <cv_bridge/cv_bridge.h>
#include <diagnostic_msgs/DiagnosticStatus.h>
#include <diagnostic_updater/diagnostic_updater.h>
#include <foxglove_msgs/CompressedVideo.h>
#include <image_transport/image_transport.h>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>
#include <ros/ros.h>
#include <sensor_msgs/CameraInfo.h>
#include <sensor_msgs/CompressedImage.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/image_encodings.h>
#include <std_msgs/Header.h>
#include <xgc_camera_msgs/FrameTiming.h>
#include <xgc_camera_msgs/StreamInfo.h>

#include <xgc2/camera/camera.hpp>

#include "clock_mapper.hpp"
#include "h264_annex_b.hpp"
#include "h264_input_buffer.hpp"

namespace {

using xgc2::camera::Frame;
using xgc2::camera::PixelFormat;
using xgc2::camera::Timestamp;
using xgc2::camera::TimestampReference;
using xgc_camera_driver::timing::Mapping;

constexpr std::uint64_t kNanosecondsPerSecond = 1000000000ULL;

std::uint32_t positiveDimension(const int value, const char* name)
{
  if (value <= 0) {
    throw std::invalid_argument(std::string(name) + " must be positive");
  }
  return static_cast<std::uint32_t>(value);
}

std::uint32_t boundedQueueCapacity(const int value)
{
  if (value < 1 || value > 256) {
    throw std::invalid_argument("encoded_queue_capacity must be in [1, 256]");
  }
  return static_cast<std::uint32_t>(value);
}

ros::Time rosTimeFromNanoseconds(const std::int64_t value)
{
  ros::Time result;
  if (value > 0) {
    result.fromNSec(static_cast<std::uint64_t>(value));
  }
  return result;
}

std::uint64_t nextEpoch(const std::uint64_t value)
{
  return value == std::numeric_limits<std::uint64_t>::max() ? 1U : value + 1U;
}

std::uint64_t initialEpoch()
{
  const auto clocks = xgc_camera_driver::timing::sampleHostClocks();
  std::uint64_t value = static_cast<std::uint64_t>(
      clocks.valid ? clocks.realtime_ns : 1);
  value ^= static_cast<std::uint64_t>(::getpid()) << 32U;
  value ^= static_cast<std::uint64_t>(
      reinterpret_cast<std::uintptr_t>(&value));
  return value == 0U ? 1U : value;
}

std::uint32_t saturatedFrameCount(const std::uint64_t value)
{
  return value > std::numeric_limits<std::uint32_t>::max()
             ? std::numeric_limits<std::uint32_t>::max()
             : static_cast<std::uint32_t>(value);
}

std::uint8_t timingReference(const TimestampReference reference)
{
  switch (reference) {
    case TimestampReference::StartOfExposure:
      return xgc_camera_msgs::FrameTiming::
          TIMESTAMP_REFERENCE_START_OF_EXPOSURE;
    case TimestampReference::EndOfFrame:
      return xgc_camera_msgs::FrameTiming::TIMESTAMP_REFERENCE_END_OF_FRAME;
    case TimestampReference::Unknown:
      return xgc_camera_msgs::FrameTiming::TIMESTAMP_REFERENCE_UNKNOWN;
  }
  return xgc_camera_msgs::FrameTiming::TIMESTAMP_REFERENCE_UNKNOWN;
}

std::uint8_t streamClockDomain(
    const xgc2::camera::TimestampClock clock)
{
  switch (clock) {
    case xgc2::camera::TimestampClock::Realtime:
      return xgc_camera_msgs::StreamInfo::CLOCK_DOMAIN_SYSTEM_REALTIME;
    case xgc2::camera::TimestampClock::Monotonic:
      return xgc_camera_msgs::StreamInfo::CLOCK_DOMAIN_MONOTONIC;
    case xgc2::camera::TimestampClock::Unknown:
      return xgc_camera_msgs::StreamInfo::CLOCK_DOMAIN_UNKNOWN;
  }
  return xgc_camera_msgs::StreamInfo::CLOCK_DOMAIN_UNKNOWN;
}

cv::Mat packedMat(
    const Frame& frame,
    const int type,
    const std::size_t bytes_per_pixel)
{
  const std::size_t minimum_row =
      static_cast<std::size_t>(frame.width()) * bytes_per_pixel;
  const std::size_t stride =
      frame.stride() == 0 ? minimum_row : frame.stride();
  const std::size_t required = stride * frame.height();
  if (!frame.data() || stride < minimum_row || frame.size() < required) {
    throw std::runtime_error(
        "camera frame has an invalid packed buffer layout");
  }
  return cv::Mat(
      static_cast<int>(frame.height()),
      static_cast<int>(frame.width()),
      type,
      const_cast<std::uint8_t*>(frame.data()),
      stride);
}

cv::Mat nv12Mat(const Frame& frame)
{
  const std::size_t width = frame.width();
  const std::size_t height = frame.height();
  if ((width % 2U) != 0U || (height % 2U) != 0U) {
    throw std::runtime_error("NV12 width and height must be even");
  }

  cv::Mat packed(
      static_cast<int>(height + height / 2U),
      static_cast<int>(width),
      CV_8UC1);
  const auto& planes = frame.planes();
  if (planes.size() >= 2U) {
    const auto& y_plane = planes[0];
    const auto& uv_plane = planes[1];
    const std::size_t y_stride =
        y_plane.stride == 0 ? width : y_plane.stride;
    const std::size_t uv_stride =
        uv_plane.stride == 0 ? width : uv_plane.stride;
    if (!y_plane.data || !uv_plane.data || y_stride < width ||
        uv_stride < width ||
        y_plane.bytes_used < y_stride * height ||
        uv_plane.bytes_used < uv_stride * (height / 2U)) {
      throw std::runtime_error(
          "camera frame has an invalid multi-plane NV12 layout");
    }
    for (std::size_t row = 0; row < height; ++row) {
      std::memcpy(
          packed.ptr(static_cast<int>(row)),
          y_plane.data + row * y_stride,
          width);
    }
    for (std::size_t row = 0; row < height / 2U; ++row) {
      std::memcpy(
          packed.ptr(static_cast<int>(height + row)),
          uv_plane.data + row * uv_stride,
          width);
    }
    return packed;
  }

  const std::size_t required = width * height * 3U / 2U;
  if (!frame.data() || frame.size() < required) {
    throw std::runtime_error(
        "camera frame has an invalid contiguous NV12 layout");
  }
  std::memcpy(packed.data, frame.data(), required);
  return packed;
}

cv::Mat decodeToBgr(const Frame& frame)
{
  cv::Mat bgr;
  switch (frame.pixel_format()) {
    case PixelFormat::MJPEG: {
      if (!frame.data() || frame.size() == 0U) {
        throw std::runtime_error("empty MJPEG frame");
      }
      const int encoded_size =
          xgc_camera_driver::detail::checkedCompressedPayloadSize(
              frame.size(), "MJPEG");
      const cv::Mat encoded(
          1,
          encoded_size,
          CV_8UC1,
          const_cast<std::uint8_t*>(frame.data()));
      bgr = cv::imdecode(encoded, cv::IMREAD_COLOR);
      if (bgr.empty()) {
        throw std::runtime_error("OpenCV could not decode MJPEG frame");
      }
      break;
    }
    case PixelFormat::BGR24:
      bgr = packedMat(frame, CV_8UC3, 3U).clone();
      break;
    case PixelFormat::RGB24:
      cv::cvtColor(
          packedMat(frame, CV_8UC3, 3U), bgr, cv::COLOR_RGB2BGR);
      break;
    case PixelFormat::YUYV:
      cv::cvtColor(
          packedMat(frame, CV_8UC2, 2U), bgr, cv::COLOR_YUV2BGR_YUY2);
      break;
    case PixelFormat::UYVY:
      cv::cvtColor(
          packedMat(frame, CV_8UC2, 2U), bgr, cv::COLOR_YUV2BGR_UYVY);
      break;
    case PixelFormat::NV12:
      cv::cvtColor(nv12Mat(frame), bgr, cv::COLOR_YUV2BGR_NV12);
      break;
    case PixelFormat::GREY:
      cv::cvtColor(
          packedMat(frame, CV_8UC1, 1U), bgr, cv::COLOR_GRAY2BGR);
      break;
    case PixelFormat::H264:
      throw std::logic_error(
          "H264 must be decoded by the persistent stream decoder");
    case PixelFormat::Unknown:
      throw std::runtime_error("camera returned an unknown pixel format");
  }
  if (bgr.cols != static_cast<int>(frame.width()) ||
      bgr.rows != static_cast<int>(frame.height())) {
    throw std::runtime_error(
        "decoded image dimensions do not match negotiated dimensions");
  }
  return bgr;
}

std::string ffmpegError(const int code)
{
  char buffer[AV_ERROR_MAX_STRING_SIZE] = {};
  av_strerror(code, buffer, sizeof(buffer));
  return std::string(buffer);
}

struct SourceMetadata {
  Timestamp timestamp;
  TimestampReference timestamp_reference{TimestampReference::Unknown};
  Timestamp dequeue_timestamp;
  Mapping mapping;
  std::uint64_t sequence{0};
};

struct DecodedFrame {
  cv::Mat bgr;
  SourceMetadata source;
};

class H264StreamDecoder {
 public:
  H264StreamDecoder()
  {
    const AVCodec* codec = avcodec_find_decoder(AV_CODEC_ID_H264);
    if (!codec) {
      throw std::runtime_error("FFmpeg H264 decoder is unavailable");
    }
    parser_ = av_parser_init(codec->id);
    context_ = avcodec_alloc_context3(codec);
    frame_ = av_frame_alloc();
    packet_ = av_packet_alloc();
    if (!parser_ || !context_ || !frame_ || !packet_) {
      throw std::runtime_error(
          "could not allocate FFmpeg H264 decoder state");
    }
    context_->thread_count = 0;
    context_->thread_type = FF_THREAD_FRAME | FF_THREAD_SLICE;
    const int result = avcodec_open2(context_, codec, nullptr);
    if (result < 0) {
      throw std::runtime_error(
          "could not open FFmpeg H264 decoder: " + ffmpegError(result));
    }
  }

  ~H264StreamDecoder()
  {
    sws_freeContext(scaler_);
    av_packet_free(&packet_);
    av_frame_free(&frame_);
    avcodec_free_context(&context_);
    if (parser_) {
      av_parser_close(parser_);
    }
  }

  H264StreamDecoder(const H264StreamDecoder&) = delete;
  H264StreamDecoder& operator=(const H264StreamDecoder&) = delete;

  bool decode(
      const Frame& input_frame,
      const Mapping& input_mapping,
      DecodedFrame* output)
  {
    SourceMetadata current{
        input_frame.timestamp(),
        input_frame.timestamp_reference(),
        input_frame.dequeue_timestamp(),
        input_mapping,
        input_frame.sequence()};
    metadata_[static_cast<std::int64_t>(current.sequence)] = current;
    while (metadata_.size() > 512U) {
      metadata_.erase(metadata_.begin());
    }

    const xgc_camera_driver::detail::PaddedH264Input padded_input(
        input_frame.data(), input_frame.size());
    const std::uint8_t* input = padded_input.data();
    int remaining = padded_input.payloadSize();
    while (remaining > 0) {
      std::uint8_t* packet_data = nullptr;
      int packet_size = 0;
      const int consumed = av_parser_parse2(
          parser_,
          context_,
          &packet_data,
          &packet_size,
          input,
          remaining,
          static_cast<std::int64_t>(current.sequence),
          static_cast<std::int64_t>(current.sequence),
          0);
      if (consumed < 0) {
        throw std::runtime_error(
            "FFmpeg could not parse H264 stream: " +
            ffmpegError(consumed));
      }
      input += consumed;
      remaining -= consumed;
      if (packet_size > 0) {
        sendPacket(packet_data, packet_size, current);
      }
      if (consumed == 0 && packet_size == 0) {
        throw std::runtime_error(
            "FFmpeg H264 parser made no progress");
      }
    }

    if (decoded_.empty()) {
      return false;
    }
    *output = std::move(decoded_.front());
    decoded_.pop_front();
    return true;
  }

 private:
  void sendPacket(
      std::uint8_t* data,
      const int size,
      const SourceMetadata& current)
  {
    av_packet_unref(packet_);
    packet_->data = data;
    packet_->size = size;
    packet_->pts =
        parser_->pts == AV_NOPTS_VALUE
            ? static_cast<std::int64_t>(current.sequence)
            : parser_->pts;
    packet_->dts =
        parser_->dts == AV_NOPTS_VALUE ? packet_->pts : parser_->dts;
    int result = avcodec_send_packet(context_, packet_);
    if (result == AVERROR(EAGAIN)) {
      drain(current);
      result = avcodec_send_packet(context_, packet_);
    }
    if (result < 0) {
      throw std::runtime_error(
          "FFmpeg rejected H264 packet: " + ffmpegError(result));
    }
    drain(current);
  }

  void drain(const SourceMetadata& fallback)
  {
    for (;;) {
      const int result = avcodec_receive_frame(context_, frame_);
      if (result == AVERROR(EAGAIN) || result == AVERROR_EOF) {
        return;
      }
      if (result < 0) {
        throw std::runtime_error(
            "FFmpeg failed to decode H264 frame: " +
            ffmpegError(result));
      }
      if (frame_->width <= 0 || frame_->height <= 0) {
        av_frame_unref(frame_);
        throw std::runtime_error(
            "FFmpeg decoded an H264 frame with invalid dimensions");
      }

      scaler_ = sws_getCachedContext(
          scaler_,
          frame_->width,
          frame_->height,
          static_cast<AVPixelFormat>(frame_->format),
          frame_->width,
          frame_->height,
          AV_PIX_FMT_BGR24,
          SWS_BILINEAR,
          nullptr,
          nullptr,
          nullptr);
      if (!scaler_) {
        av_frame_unref(frame_);
        throw std::runtime_error(
            "FFmpeg could not create H264 color conversion context");
      }
      DecodedFrame decoded;
      decoded.bgr.create(frame_->height, frame_->width, CV_8UC3);
      std::uint8_t* output_data[] = {
          decoded.bgr.data, nullptr, nullptr, nullptr};
      int output_stride[] = {
          static_cast<int>(decoded.bgr.step), 0, 0, 0};
      sws_scale(
          scaler_,
          frame_->data,
          frame_->linesize,
          0,
          frame_->height,
          output_data,
          output_stride);

      const std::int64_t key =
          frame_->best_effort_timestamp == AV_NOPTS_VALUE
              ? (frame_->pts == AV_NOPTS_VALUE
                     ? static_cast<std::int64_t>(fallback.sequence)
                     : frame_->pts)
              : frame_->best_effort_timestamp;
      auto found = metadata_.find(key);
      decoded.source =
          found == metadata_.end() ? fallback : found->second;
      if (found != metadata_.end()) {
        metadata_.erase(found);
      }
      decoded_.push_back(std::move(decoded));
      av_frame_unref(frame_);
    }
  }

  AVCodecParserContext* parser_{nullptr};
  AVCodecContext* context_{nullptr};
  AVFrame* frame_{nullptr};
  AVPacket* packet_{nullptr};
  SwsContext* scaler_{nullptr};
  std::map<std::int64_t, SourceMetadata> metadata_;
  std::deque<DecodedFrame> decoded_;
};

class ImageDecoder {
 public:
  bool decode(
      const Frame& frame,
      const Mapping& mapping,
      DecodedFrame* decoded)
  {
    if (frame.pixel_format() == PixelFormat::H264) {
      if (!h264_) {
        h264_.reset(new H264StreamDecoder());
      }
      return h264_->decode(frame, mapping, decoded);
    }
    decoded->bgr = decodeToBgr(frame);
    decoded->source = SourceMetadata{
        frame.timestamp(),
        frame.timestamp_reference(),
        frame.dequeue_timestamp(),
        mapping,
        frame.sequence()};
    return true;
  }

 private:
  std::unique_ptr<H264StreamDecoder> h264_;
};

cv::Mat convertOutput(const cv::Mat& bgr, const std::string& encoding)
{
  if (encoding == sensor_msgs::image_encodings::BGR8) {
    return bgr;
  }
  cv::Mat output;
  if (encoding == sensor_msgs::image_encodings::RGB8) {
    cv::cvtColor(bgr, output, cv::COLOR_BGR2RGB);
  } else if (encoding == sensor_msgs::image_encodings::MONO8) {
    cv::cvtColor(bgr, output, cv::COLOR_BGR2GRAY);
  } else {
    throw std::invalid_argument(
        "output_encoding must be bgr8, rgb8, or mono8");
  }
  return output;
}

enum class RawPublishMode {
  Never,
  OnDemand,
  Always,
};

RawPublishMode rawPublishModeFromString(const std::string& value)
{
  if (value == "never") {
    return RawPublishMode::Never;
  }
  if (value == "on_demand") {
    return RawPublishMode::OnDemand;
  }
  if (value == "always") {
    return RawPublishMode::Always;
  }
  throw std::invalid_argument(
      "raw_publish_mode must be never, on_demand, or always");
}

const char* toString(const RawPublishMode value)
{
  switch (value) {
    case RawPublishMode::Never:
      return "never";
    case RawPublishMode::OnDemand:
      return "on_demand";
    case RawPublishMode::Always:
      return "always";
  }
  return "never";
}

enum class EncodedKind {
  H264,
  MJPEG,
};

struct EncodedPacket {
  EncodedKind kind{EncodedKind::H264};
  std::vector<std::uint8_t> data;
  Mapping mapping;
  TimestampReference timestamp_reference{TimestampReference::Unknown};
  std::int64_t dequeue_monotonic_ns{0};
  std::uint64_t epoch{0};
  std::uint64_t frame_sequence{0};
  std::uint64_t source_sequence{0};
  std::uint32_t dropped_frames_before{0};
  std::uint8_t discontinuity{
      xgc_camera_msgs::FrameTiming::DISCONTINUITY_NONE};
  bool keyframe{false};
};

class CameraDriverNode {
 public:
  CameraDriverNode()
      : nh_(),
        private_nh_("~"),
        image_transport_(nh_),
        start_wall_time_(ros::WallTime::now())
  {
    loadParameters();
    camera_info_manager_.reset(new camera_info_manager::CameraInfoManager(
        nh_, camera_name_, camera_info_url_));

    xgc2::camera::CaptureConfig config;
    config.backend = xgc2::camera::backend_kind_from_string(backend_);
    config.device = video_device_;
    config.width = width_;
    config.height = height_;
    config.frame_rate = framerate_;
    config.pixel_format =
        xgc2::camera::pixel_format_from_string(pixel_format_);
    config.capture_mode =
        xgc2::camera::capture_mode_from_string(capture_mode_);
    config.buffer_count = buffer_count_;
    config.synthetic_seed = synthetic_seed_;
    camera_ = xgc2::camera::make_camera(config);
    width_ = camera_->config().width;
    height_ = camera_->config().height;
    framerate_ = camera_->config().frame_rate;
    native_pixel_format_ = camera_->config().pixel_format;

    raw_image_publisher_ = image_transport_.advertise("image_raw", 1);
    camera_info_publisher_ =
        nh_.advertise<sensor_msgs::CameraInfo>("camera_info", 4);
    compressed_image_publisher_ =
        nh_.advertise<sensor_msgs::CompressedImage>(
            "image_raw/compressed", encoded_queue_capacity_);
    compressed_video_publisher_ =
        nh_.advertise<foxglove_msgs::CompressedVideo>(
            "video", encoded_queue_capacity_);
    frame_timing_publisher_ =
        nh_.advertise<xgc_camera_msgs::FrameTiming>(
            "frame_timing", encoded_queue_capacity_);
    stream_info_publisher_ =
        nh_.advertise<xgc_camera_msgs::StreamInfo>(
            "stream_info", 1, true);

    updater_.setHardwareID(
        backend_ == "v4l2" ? video_device_ : "synthetic");
    updater_.add("camera stream", this, &CameraDriverNode::diagnose);
  }

  ~CameraDriverNode()
  {
    if (camera_) {
      camera_->stop();
    }
    stopPublisher();
  }

  int run()
  {
    camera_->start();
    startPublisher();
    running_.store(true);
    ROS_INFO_STREAM(
        "XGC2 camera started: backend=" << backend_
                                        << " device=" << video_device_
                                        << " " << width_ << "x" << height_
                                        << "@" << framerate_
                                        << " format="
                                        << xgc2::camera::to_string(
                                               native_pixel_format_)
                                        << " raw=" << toString(raw_publish_mode_)
                                        << " encoded_queue="
                                        << encoded_queue_capacity_
                                        << " unknown_clock="
                                        << xgc_camera_driver::timing::toString(
                                               unknown_clock_policy_));

    while (ros::ok()) {
      try {
        const Frame frame = camera_->read(capture_timeout_ms_);
        capture_packet_count_.fetch_add(1U);
        handleFrame(frame);
      } catch (const xgc2::camera::CameraError& error) {
        if (error.code() == xgc2::camera::ErrorCode::Timeout) {
          setError(error.what());
          updater_.force_update();
          continue;
        }
        throw;
      }
      updater_.update();
    }
    running_.store(false);
    camera_->stop();
    stopPublisher();
    return 0;
  }

 private:
  void loadParameters()
  {
    int width = 640;
    int height = 480;
    int buffer_count = 4;
    int synthetic_seed = 1;
    int encoded_queue_capacity = 8;
    std::string raw_publish_mode;
    std::string unknown_timestamp_clock;
    private_nh_.param<std::string>("backend", backend_, "v4l2");
    private_nh_.param<std::string>(
        "video_device", video_device_, "/dev/video0");
    private_nh_.param("width", width, width);
    private_nh_.param("height", height, height);
    private_nh_.param("framerate", framerate_, 30.0);
    private_nh_.param<std::string>(
        "pixel_format", pixel_format_, "mjpeg");
    private_nh_.param<std::string>(
        "capture_mode", capture_mode_, "auto");
    private_nh_.param<std::string>(
        "output_encoding",
        output_encoding_,
        sensor_msgs::image_encodings::BGR8);
    private_nh_.param<std::string>(
        "camera_name", camera_name_, "usb_cam");
    private_nh_.param<std::string>(
        "stream_id", stream_id_, camera_name_);
    private_nh_.param<std::string>(
        "frame_id", frame_id_, "usb_cam_optical_frame");
    private_nh_.param<std::string>(
        "camera_info_url", camera_info_url_, "");
    private_nh_.param<std::string>(
        "raw_publish_mode", raw_publish_mode, "on_demand");
    private_nh_.param<std::string>(
        "unknown_timestamp_clock",
        unknown_timestamp_clock,
        "reject");
    private_nh_.param("publish_encoded", publish_encoded_, true);
    private_nh_.param("buffer_count", buffer_count, buffer_count);
    private_nh_.param(
        "encoded_queue_capacity",
        encoded_queue_capacity,
        encoded_queue_capacity);
    private_nh_.param(
        "synthetic_seed", synthetic_seed, synthetic_seed);
    private_nh_.param(
        "capture_timeout_ms", capture_timeout_ms_, 2000);

    width_ = positiveDimension(width, "width");
    height_ = positiveDimension(height, "height");
    buffer_count_ = positiveDimension(buffer_count, "buffer_count");
    encoded_queue_capacity_ =
        boundedQueueCapacity(encoded_queue_capacity);
    if (buffer_count_ < 2U) {
      throw std::invalid_argument("buffer_count must be at least 2");
    }
    synthetic_seed_ = static_cast<std::uint32_t>(
        synthetic_seed < 0 ? 0 : synthetic_seed);
    if (framerate_ <= 0.0 || capture_timeout_ms_ <= 0 ||
        camera_name_.empty() || stream_id_.empty() || frame_id_.empty()) {
      throw std::invalid_argument(
          "framerate, capture_timeout_ms, camera_name, stream_id, and "
          "frame_id must be valid");
    }
    (void)xgc2::camera::backend_kind_from_string(backend_);
    (void)xgc2::camera::pixel_format_from_string(pixel_format_);
    (void)xgc2::camera::capture_mode_from_string(capture_mode_);
    (void)convertOutput(cv::Mat(1, 1, CV_8UC3), output_encoding_);
    raw_publish_mode_ = rawPublishModeFromString(raw_publish_mode);
    unknown_clock_policy_ =
        xgc_camera_driver::timing::unknownClockPolicyFromString(
            unknown_timestamp_clock);
  }

  bool shouldPublishRaw() const
  {
    return raw_publish_mode_ == RawPublishMode::Always ||
           (raw_publish_mode_ == RawPublishMode::OnDemand &&
            raw_image_publisher_.getNumSubscribers() > 0U);
  }

  Mapping mapTimestamp(const Timestamp& source) const
  {
    return xgc_camera_driver::timing::mapSourceTimestamp(
        source,
        unknown_clock_policy_,
        xgc_camera_driver::timing::sampleHostClocks());
  }

  void handleFrame(const Frame& frame)
  {
    const Mapping mapping = mapTimestamp(frame.timestamp());
    if (!mapping.valid) {
      invalid_timestamp_count_.fetch_add(1U);
      setError(
          std::string("source timestamp has no declared ROS time mapping "
                      "(clock=") +
          xgc2::camera::to_string(frame.timestamp().clock) +
          ", policy=" +
          xgc_camera_driver::timing::toString(unknown_clock_policy_) + ")");
      return;
    }
    clearError();

    const bool native_encoded =
        (frame.pixel_format() == PixelFormat::H264 ||
         frame.pixel_format() == PixelFormat::MJPEG);
    if (publish_encoded_ && native_encoded) {
      handleEncodedFrame(frame, mapping);
    }
    const bool publish_raw = shouldPublishRaw();
    if (!native_encoded && !publish_raw &&
        camera_info_publisher_.getNumSubscribers() > 0U) {
      std_msgs::Header header;
      header.seq = static_cast<std::uint32_t>(frame.sequence());
      header.stamp = rosTimeFromNanoseconds(mapping.ros_time_ns);
      header.frame_id = frame_id_;
      publishCameraInfo(header);
      notePublished(frame.sequence());
    }
    if (publish_raw) {
      try {
        publishRawFrame(frame, mapping);
      } catch (const std::exception& error) {
        raw_decode_error_count_.fetch_add(1U);
        setError(std::string("raw decode failed: ") + error.what());
        ROS_ERROR_STREAM_THROTTLE(
            2.0, "XGC2 camera raw decode failed: " << error.what());
      }
    }
  }

  void beginEncodedEpoch(const std::uint8_t reason)
  {
    if (!epoch_reset_pending_) {
      encoded_epoch_ = nextEpoch(encoded_epoch_);
      encoded_frame_sequence_ = 0U;
    }
    epoch_reset_pending_ = true;
    pending_discontinuity_ = reason;
  }

  bool inspectContinuity(
      const Frame& frame,
      const Mapping& mapping,
      std::uint64_t* dropped)
  {
    bool reset = false;
    if (have_stream_contract_ &&
        (mapping.effective_clock != last_effective_clock_ ||
         frame.timestamp_reference() != last_timestamp_reference_)) {
      beginEncodedEpoch(
          xgc_camera_msgs::FrameTiming::
              DISCONTINUITY_SOURCE_TIME_RESET);
      reset = true;
    }
    if (have_source_sequence_) {
      if (frame.sequence() <= last_source_sequence_) {
        beginEncodedEpoch(
            xgc_camera_msgs::FrameTiming::
                DISCONTINUITY_ENCODER_RESET);
        reset = true;
      } else if (frame.sequence() > last_source_sequence_ + 1U) {
        *dropped +=
            frame.sequence() - last_source_sequence_ - 1U;
        if (frame.pixel_format() == PixelFormat::H264) {
          beginEncodedEpoch(
              xgc_camera_msgs::FrameTiming::
                  DISCONTINUITY_ENCODER_RESET);
          reset = true;
        }
      }
      if (frame.timestamp_ns() <= last_native_source_time_ns_ ||
          mapping.ros_time_ns <= last_mapped_source_time_ns_) {
        beginEncodedEpoch(
            xgc_camera_msgs::FrameTiming::
                DISCONTINUITY_SOURCE_TIME_RESET);
        reset = true;
      }
    }
    have_source_sequence_ = true;
    last_source_sequence_ = frame.sequence();
    last_native_source_time_ns_ = frame.timestamp_ns();
    last_mapped_source_time_ns_ = mapping.ros_time_ns;
    have_stream_contract_ = true;
    last_effective_clock_ = mapping.effective_clock;
    last_timestamp_reference_ = frame.timestamp_reference();
    return reset;
  }

  std::uint64_t clearOverflowedQueue()
  {
    std::lock_guard<std::mutex> lock(encoded_queue_mutex_);
    if (encoded_queue_.size() < encoded_queue_capacity_) {
      return 0U;
    }
    const std::uint64_t dropped = encoded_queue_.size();
    encoded_queue_.clear();
    encoded_queue_drop_count_.fetch_add(dropped);
    return dropped;
  }

  void handleEncodedFrame(const Frame& frame, const Mapping& mapping)
  {
    std::uint64_t dropped = pending_dropped_frames_;
    pending_dropped_frames_ = 0U;
    const bool continuity_reset =
        inspectContinuity(frame, mapping, &dropped);
    const std::uint64_t queue_dropped = clearOverflowedQueue();
    if (queue_dropped > 0U) {
      dropped += queue_dropped;
      if (frame.pixel_format() == PixelFormat::H264) {
        beginEncodedEpoch(
            xgc_camera_msgs::FrameTiming::
                DISCONTINUITY_QUEUE_OVERFLOW);
      }
    }

    EncodedPacket packet;
    packet.mapping = mapping;
    packet.timestamp_reference = frame.timestamp_reference();
    packet.dequeue_monotonic_ns =
        frame.dequeue_timestamp().clock ==
                xgc2::camera::TimestampClock::Monotonic
            ? frame.dequeue_timestamp_ns()
            : 0;
    packet.source_sequence = frame.sequence();
    packet.dropped_frames_before = saturatedFrameCount(dropped);

    if (frame.pixel_format() == PixelFormat::H264) {
      packet.kind = EncodedKind::H264;
      if (continuity_reset || queue_dropped > 0U) {
        h264_gate_.discontinuity();
      }
      xgc_camera_driver::h264::AccessUnitInfo info;
      if (!h264_gate_.prepare(
              frame.data(), frame.size(), &packet.data, &info)) {
        if (!info.annex_b) {
          invalid_encoded_frame_count_.fetch_add(1U);
          setError("native H264 frame is not an Annex-B access unit");
          pending_dropped_frames_ =
              static_cast<std::uint64_t>(
                  packet.dropped_frames_before) +
              1U;
          h264_gate_.discontinuity();
          beginEncodedEpoch(
              xgc_camera_msgs::FrameTiming::
                  DISCONTINUITY_ENCODER_RESET);
        } else if (info.has_vcl) {
          pending_dropped_frames_ =
              static_cast<std::uint64_t>(
                  packet.dropped_frames_before) +
              1U;
          encoded_waiting_for_idr_count_.fetch_add(1U);
        } else {
          pending_dropped_frames_ =
              packet.dropped_frames_before;
        }
        return;
      }
      packet.keyframe = info.keyframe;
    } else {
      packet.kind = EncodedKind::MJPEG;
      packet.keyframe = true;
      if (!frame.data() || frame.size() < 2U ||
          frame.data()[0] != 0xffU || frame.data()[1] != 0xd8U) {
        invalid_encoded_frame_count_.fetch_add(1U);
        setError("native MJPEG frame is not a JPEG bitstream");
        return;
      }
      packet.data.assign(frame.data(), frame.data() + frame.size());
    }

    if (encoded_epoch_ == 0U) {
      encoded_epoch_ = initialEpoch();
      pending_discontinuity_ =
          xgc_camera_msgs::FrameTiming::DISCONTINUITY_STREAM_START;
      epoch_reset_pending_ = true;
    }
    packet.epoch = encoded_epoch_;
    packet.frame_sequence = encoded_frame_sequence_++;
    packet.discontinuity = pending_discontinuity_;
    if (epoch_reset_pending_) {
      epoch_reset_pending_ = false;
      pending_discontinuity_ =
          xgc_camera_msgs::FrameTiming::DISCONTINUITY_NONE;
    }
    enqueueEncoded(std::move(packet));
  }

  void enqueueEncoded(EncodedPacket packet)
  {
    {
      std::lock_guard<std::mutex> lock(encoded_queue_mutex_);
      // Only the capture thread pushes and performs the preflight clear. The
      // defensive branch covers a future second producer without blocking it.
      if (encoded_queue_.size() >= encoded_queue_capacity_) {
        pending_dropped_frames_ += encoded_queue_.size() + 1U;
        encoded_queue_drop_count_.fetch_add(
            encoded_queue_.size() + 1U);
        encoded_queue_.clear();
        if (packet.kind == EncodedKind::H264) {
          h264_gate_.discontinuity();
          beginEncodedEpoch(
              xgc_camera_msgs::FrameTiming::
                  DISCONTINUITY_QUEUE_OVERFLOW);
        }
        return;
      }
      encoded_queue_.push_back(std::move(packet));
    }
    encoded_queue_ready_.notify_one();
  }

  void publishRawFrame(
      const Frame& frame,
      const Mapping& input_mapping)
  {
    DecodedFrame decoded;
    if (!decoder_.decode(frame, input_mapping, &decoded)) {
      return;
    }
    if (decoded.bgr.cols != static_cast<int>(width_) ||
        decoded.bgr.rows != static_cast<int>(height_)) {
      throw std::runtime_error(
          "decoded image dimensions do not match negotiated dimensions");
    }
    const Mapping& mapping = decoded.source.mapping;
    if (!mapping.valid) {
      invalid_timestamp_count_.fetch_add(1U);
      return;
    }
    const cv::Mat image = convertOutput(decoded.bgr, output_encoding_);
    std_msgs::Header header;
    header.seq =
        static_cast<std::uint32_t>(decoded.source.sequence);
    header.stamp = rosTimeFromNanoseconds(mapping.ros_time_ns);
    header.frame_id = frame_id_;
    sensor_msgs::ImagePtr image_message =
        cv_bridge::CvImage(header, output_encoding_, image).toImageMsg();
    raw_image_publisher_.publish(image_message);
    publishCameraInfo(header);
    raw_frame_count_.fetch_add(1U);
    notePublished(decoded.source.sequence);
  }

  void startPublisher()
  {
    {
      std::lock_guard<std::mutex> lock(encoded_queue_mutex_);
      encoded_publisher_running_ = true;
    }
    encoded_publisher_thread_ =
        std::thread(&CameraDriverNode::encodedPublisherLoop, this);
  }

  void stopPublisher()
  {
    {
      std::lock_guard<std::mutex> lock(encoded_queue_mutex_);
      encoded_publisher_running_ = false;
    }
    encoded_queue_ready_.notify_all();
    if (encoded_publisher_thread_.joinable()) {
      encoded_publisher_thread_.join();
    }
  }

  void encodedPublisherLoop()
  {
    for (;;) {
      EncodedPacket packet;
      {
        std::unique_lock<std::mutex> lock(encoded_queue_mutex_);
        encoded_queue_ready_.wait(lock, [this]() {
          return !encoded_publisher_running_ ||
                 !encoded_queue_.empty();
        });
        if (encoded_queue_.empty()) {
          if (!encoded_publisher_running_) {
            return;
          }
          continue;
        }
        packet = std::move(encoded_queue_.front());
        encoded_queue_.pop_front();
      }
      try {
        publishEncodedPacket(packet);
      } catch (const std::exception& error) {
        encoded_publish_error_count_.fetch_add(1U);
        setError(
            std::string("encoded ROS publication failed: ") +
            error.what());
        ROS_ERROR_STREAM_THROTTLE(
            2.0,
            "XGC2 encoded camera publication failed: " << error.what());
      }
    }
  }

  void publishEncodedPacket(const EncodedPacket& packet)
  {
    const ros::Time stamp =
        rosTimeFromNanoseconds(packet.mapping.ros_time_ns);
    std_msgs::Header header;
    header.seq = static_cast<std::uint32_t>(packet.source_sequence);
    header.stamp = stamp;
    header.frame_id = frame_id_;

    if (last_stream_info_epoch_ != packet.epoch) {
      publishStreamInfo(packet);
      last_stream_info_epoch_ = packet.epoch;
    }
    if (packet.kind == EncodedKind::H264) {
      foxglove_msgs::CompressedVideo message;
      message.timestamp = stamp;
      message.frame_id = frame_id_;
      message.format = "h264";
      message.data = packet.data;
      compressed_video_publisher_.publish(message);
    } else {
      sensor_msgs::CompressedImage message;
      message.header = header;
      message.format = "jpeg";
      message.data = packet.data;
      compressed_image_publisher_.publish(message);
    }
    publishCameraInfo(header);
    publishFrameTiming(packet, stamp);
    encoded_frame_count_.fetch_add(1U);
    encoded_bytes_count_.fetch_add(packet.data.size());
    notePublished(packet.source_sequence);
  }

  void publishStreamInfo(const EncodedPacket& packet)
  {
    xgc_camera_msgs::StreamInfo info;
    info.contract_version =
        xgc_camera_msgs::StreamInfo::CONTRACT_VERSION_CURRENT;
    info.stream_id = stream_id_;
    info.frame_id = frame_id_;
    info.epoch = packet.epoch;
    info.clock_domain =
        streamClockDomain(packet.mapping.effective_clock);
    info.timestamp_source =
        xgc_camera_msgs::StreamInfo::TIMESTAMP_SOURCE_DRIVER;
    info.timestamp_reference =
        timingReference(packet.timestamp_reference);
    info.width = width_;
    info.height = height_;
    info.nominal_frame_rate = framerate_;
    info.publisher_queue_capacity = encoded_queue_capacity_;
    if (packet.kind == EncodedKind::H264) {
      info.codec = xgc_camera_msgs::StreamInfo::CODEC_H264;
      info.bitstream_format =
          xgc_camera_msgs::StreamInfo::BITSTREAM_FORMAT_ANNEX_B;
      info.transport_mask =
          xgc_camera_msgs::StreamInfo::
              TRANSPORT_ROS_COMPRESSED_VIDEO;
    } else {
      info.codec = xgc_camera_msgs::StreamInfo::CODEC_MJPEG;
      info.bitstream_format =
          xgc_camera_msgs::StreamInfo::BITSTREAM_FORMAT_JPEG;
      info.transport_mask =
          xgc_camera_msgs::StreamInfo::
              TRANSPORT_ROS_COMPRESSED_IMAGE;
    }
    stream_info_publisher_.publish(info);
  }

  void publishFrameTiming(
      const EncodedPacket& packet,
      const ros::Time& source_time)
  {
    xgc_camera_msgs::FrameTiming timing;
    timing.source_time = source_time;
    timing.source_time_valid = packet.mapping.valid;
    timing.timestamp_reference =
        timingReference(packet.timestamp_reference);
    timing.frame_id = frame_id_;
    timing.stream_id = stream_id_;
    timing.epoch = packet.epoch;
    timing.frame_sequence = packet.frame_sequence;
    timing.source_sequence = packet.source_sequence;
    timing.rtp_timestamp = 0U;
    timing.keyframe = packet.keyframe;
    timing.discontinuity = packet.discontinuity;
    timing.dropped_frames_before =
        packet.dropped_frames_before;
    timing.encoded_size_bytes =
        saturatedFrameCount(packet.data.size());
    timing.native_source_time_ns = packet.mapping.source_ns;
    timing.host_dequeue_monotonic_ns =
        packet.dequeue_monotonic_ns;
    const auto publish_clocks =
        xgc_camera_driver::timing::sampleHostClocks();
    timing.host_publish_realtime_ns =
        publish_clocks.valid ? publish_clocks.realtime_ns : 0;
    timing.source_to_ros_offset_ns =
        packet.mapping.source_to_ros_offset_ns;
    timing.mapping_uncertainty_ns =
        packet.mapping.uncertainty_ns;
    frame_timing_publisher_.publish(timing);
  }

  void publishCameraInfo(const std_msgs::Header& header)
  {
    sensor_msgs::CameraInfo camera_info =
        camera_info_manager_->getCameraInfo();
    camera_info.header = header;
    camera_info.width = width_;
    camera_info.height = height_;
    camera_info_publisher_.publish(camera_info);
  }

  void notePublished(const std::uint64_t sequence)
  {
    last_sequence_.store(sequence);
    last_frame_wall_ns_.store(ros::WallTime::now().toNSec());
  }

  void diagnose(
      diagnostic_updater::DiagnosticStatusWrapper& status)
  {
    const std::uint64_t raw_count = raw_frame_count_.load();
    const std::uint64_t encoded_count = encoded_frame_count_.load();
    const std::uint64_t count = raw_count + encoded_count;
    const double elapsed =
        (ros::WallTime::now() - start_wall_time_).toSec();
    const std::uint64_t last_ns = last_frame_wall_ns_.load();
    const double age =
        last_ns == 0U
            ? elapsed
            : static_cast<double>(
                  ros::WallTime::now().toNSec() - last_ns) *
                  1e-9;
    std::string error;
    {
      std::lock_guard<std::mutex> lock(error_mutex_);
      error = last_error_;
    }

    const bool calibrated =
        camera_info_manager_ && camera_info_manager_->isCalibrated();
    if (!running_.load()) {
      status.summary(
          diagnostic_msgs::DiagnosticStatus::ERROR,
          "camera is not running");
    } else if (!error.empty() ||
               age >
                   static_cast<double>(capture_timeout_ms_) * 2e-3) {
      status.summary(
          diagnostic_msgs::DiagnosticStatus::ERROR,
          error.empty() ? "camera frames are stale" : error);
    } else if (count == 0U) {
      status.summary(
          diagnostic_msgs::DiagnosticStatus::WARN,
          "waiting for first camera publication");
    } else if (encoded_queue_drop_count_.load() > 0U) {
      status.summary(
          diagnostic_msgs::DiagnosticStatus::WARN,
          "camera is publishing after encoded queue drops");
    } else if (!calibrated) {
      status.summary(
          diagnostic_msgs::DiagnosticStatus::WARN,
          "camera is publishing but CameraInfo is not calibrated");
    } else {
      status.summary(
          diagnostic_msgs::DiagnosticStatus::OK,
          "camera is publishing");
    }
    status.add("backend", backend_);
    status.add("device", video_device_);
    status.add(
        "pixel_format",
        xgc2::camera::to_string(native_pixel_format_));
    status.add("output_encoding", output_encoding_);
    status.add("raw_publish_mode", toString(raw_publish_mode_));
    status.add(
        "unknown_timestamp_clock",
        xgc_camera_driver::timing::toString(
            unknown_clock_policy_));
    status.add("frame_id", frame_id_);
    status.add("stream_id", stream_id_);
    status.add("captured_packets", capture_packet_count_.load());
    status.add("published_frames", count);
    status.add("published_raw_frames", raw_count);
    status.add("published_encoded_frames", encoded_count);
    status.add("published_encoded_bytes", encoded_bytes_count_.load());
    status.add("encoded_queue_drops", encoded_queue_drop_count_.load());
    status.add(
        "encoded_frames_waiting_for_idr",
        encoded_waiting_for_idr_count_.load());
    status.add(
        "invalid_source_timestamps",
        invalid_timestamp_count_.load());
    status.add(
        "invalid_encoded_frames",
        invalid_encoded_frame_count_.load());
    status.add("raw_decode_errors", raw_decode_error_count_.load());
    status.add(
        "encoded_publish_errors",
        encoded_publish_error_count_.load());
    status.add("camera_info_calibrated", calibrated);
    status.add("camera_info_url", camera_info_url_);
    status.add("last_sequence", last_sequence_.load());
    status.add(
        "measured_publication_fps",
        elapsed > 0.0 ? static_cast<double>(count) / elapsed : 0.0);
    status.add("last_frame_age_seconds", age);
  }

  void setError(const std::string& message)
  {
    std::lock_guard<std::mutex> lock(error_mutex_);
    last_error_ = message;
  }

  void clearError()
  {
    std::lock_guard<std::mutex> lock(error_mutex_);
    last_error_.clear();
  }

  ros::NodeHandle nh_;
  ros::NodeHandle private_nh_;
  image_transport::ImageTransport image_transport_;
  image_transport::Publisher raw_image_publisher_;
  ros::Publisher camera_info_publisher_;
  ros::Publisher compressed_image_publisher_;
  ros::Publisher compressed_video_publisher_;
  ros::Publisher frame_timing_publisher_;
  ros::Publisher stream_info_publisher_;
  std::unique_ptr<camera_info_manager::CameraInfoManager>
      camera_info_manager_;
  diagnostic_updater::Updater updater_;
  std::unique_ptr<xgc2::camera::Camera> camera_;
  ImageDecoder decoder_;
  xgc_camera_driver::h264::AccessUnitGate h264_gate_;

  std::string backend_;
  std::string video_device_;
  std::string pixel_format_;
  std::string capture_mode_;
  std::string output_encoding_;
  std::string camera_name_;
  std::string stream_id_;
  std::string frame_id_;
  std::string camera_info_url_;
  PixelFormat native_pixel_format_{PixelFormat::Unknown};
  RawPublishMode raw_publish_mode_{RawPublishMode::OnDemand};
  xgc_camera_driver::timing::UnknownClockPolicy
      unknown_clock_policy_{
          xgc_camera_driver::timing::UnknownClockPolicy::Reject};
  std::uint32_t width_{640};
  std::uint32_t height_{480};
  std::uint32_t buffer_count_{4};
  std::uint32_t encoded_queue_capacity_{8};
  std::uint32_t synthetic_seed_{1};
  double framerate_{30.0};
  int capture_timeout_ms_{2000};
  bool publish_encoded_{true};

  std::mutex encoded_queue_mutex_;
  std::condition_variable encoded_queue_ready_;
  std::deque<EncodedPacket> encoded_queue_;
  bool encoded_publisher_running_{false};
  std::thread encoded_publisher_thread_;

  bool have_source_sequence_{false};
  bool have_stream_contract_{false};
  bool epoch_reset_pending_{false};
  std::uint64_t encoded_epoch_{0};
  std::uint64_t encoded_frame_sequence_{0};
  std::uint64_t last_source_sequence_{0};
  std::int64_t last_native_source_time_ns_{0};
  std::int64_t last_mapped_source_time_ns_{0};
  xgc2::camera::TimestampClock last_effective_clock_{
      xgc2::camera::TimestampClock::Unknown};
  TimestampReference last_timestamp_reference_{
      TimestampReference::Unknown};
  std::uint64_t pending_dropped_frames_{0};
  std::uint8_t pending_discontinuity_{
      xgc_camera_msgs::FrameTiming::DISCONTINUITY_NONE};
  std::uint64_t last_stream_info_epoch_{0};

  std::atomic<bool> running_{false};
  std::atomic<std::uint64_t> capture_packet_count_{0};
  std::atomic<std::uint64_t> raw_frame_count_{0};
  std::atomic<std::uint64_t> encoded_frame_count_{0};
  std::atomic<std::uint64_t> encoded_bytes_count_{0};
  std::atomic<std::uint64_t> encoded_queue_drop_count_{0};
  std::atomic<std::uint64_t> encoded_waiting_for_idr_count_{0};
  std::atomic<std::uint64_t> invalid_timestamp_count_{0};
  std::atomic<std::uint64_t> invalid_encoded_frame_count_{0};
  std::atomic<std::uint64_t> raw_decode_error_count_{0};
  std::atomic<std::uint64_t> encoded_publish_error_count_{0};
  std::atomic<std::uint64_t> last_sequence_{0};
  std::atomic<std::uint64_t> last_frame_wall_ns_{0};
  ros::WallTime start_wall_time_;
  std::mutex error_mutex_;
  std::string last_error_;
};

}  // namespace

int main(int argc, char** argv)
{
  ros::init(argc, argv, "xgc_camera_driver");
  ros::AsyncSpinner spinner(1);
  spinner.start();
  try {
    CameraDriverNode node;
    return node.run();
  } catch (const std::exception& error) {
    ROS_FATAL_STREAM(
        "XGC2 camera driver failed: " << error.what());
    return 1;
  }
}
