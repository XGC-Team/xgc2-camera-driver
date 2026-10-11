#include <algorithm>
#include <atomic>
#include <chrono>
#include <condition_variable>
#include <cstdint>
#include <cstring>
#include <climits>
#include <deque>
#include <limits>
#include <memory>
#include <mutex>
#include <stdexcept>
#include <string>
#include <thread>
#include <vector>

#include <unistd.h>

#include <camera_info_manager/camera_info_manager.h>
#include <cv_bridge/cv_bridge.h>
#include <diagnostic_msgs/DiagnosticStatus.h>
#include <diagnostic_updater/diagnostic_updater.h>
#include <geometry_msgs/TransformStamped.h>
#include <image_transport/image_transport.h>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>
#include <ros/ros.h>
#include <sensor_msgs/CameraInfo.h>
#include <sensor_msgs/CompressedImage.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/image_encodings.h>
#include <std_msgs/Header.h>
#include <tf2_ros/static_transform_broadcaster.h>
#include <xgc_camera_msgs/FrameTiming.h>
#include <xgc_camera_msgs/StreamInfo.h>

#include <xgc2/camera/camera.hpp>

#include "camera_info_contract.hpp"
#include "capture_source.hpp"
#include "clock_mapper.hpp"
#include "source_frame.hpp"
#include "still_asset.hpp"

#ifdef XGC2_CAMERA_MEDIA
#include "media_outputs.hpp"
#endif

namespace {

using xgc2::camera::PixelFormat;
using xgc2::camera::Timestamp;
using xgc2::camera::TimestampReference;
using xgc2_camera_driver::SourceFrame;
using xgc2_camera_driver::timing::Mapping;

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
  const auto clocks = xgc2_camera_driver::timing::sampleHostClocks();
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
    const SourceFrame& frame,
    const int type,
    const std::size_t bytes_per_pixel)
{
  const std::size_t minimum_row =
      static_cast<std::size_t>(frame.width) * bytes_per_pixel;
  const std::size_t stride =
      frame.stride == 0 ? minimum_row : frame.stride;
  const std::size_t required = stride * frame.height;
  if (!frame.data || stride < minimum_row || frame.size < required) {
    throw std::runtime_error(
        "camera frame has an invalid packed buffer layout");
  }
  return cv::Mat(
      static_cast<int>(frame.height),
      static_cast<int>(frame.width),
      type,
      const_cast<std::uint8_t*>(frame.data),
      stride);
}

cv::Mat nv12Mat(const SourceFrame& frame)
{
  const std::size_t width = frame.width;
  const std::size_t height = frame.height;
  if ((width % 2U) != 0U || (height % 2U) != 0U) {
    throw std::runtime_error("NV12 width and height must be even");
  }

  cv::Mat packed(
      static_cast<int>(height + height / 2U),
      static_cast<int>(width),
      CV_8UC1);
  const auto& planes = frame.planes;
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
  if (!frame.data || frame.size < required) {
    throw std::runtime_error(
        "camera frame has an invalid contiguous NV12 layout");
  }
  std::memcpy(packed.data, frame.data, required);
  return packed;
}

cv::Mat decodeToBgr(const SourceFrame& frame)
{
  cv::Mat bgr;
  switch (frame.pixel_format) {
    case PixelFormat::MJPEG: {
      if (!frame.data || frame.size == 0U) {
        throw std::runtime_error("empty MJPEG frame");
      }
      if (frame.size > static_cast<std::size_t>(INT_MAX)) {
        throw std::runtime_error("MJPEG frame is too large for the decoder");
      }
      const int encoded_size = static_cast<int>(frame.size);
      const cv::Mat encoded(
          1,
          encoded_size,
          CV_8UC1,
          const_cast<std::uint8_t*>(frame.data));
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
    case PixelFormat::Unknown:
      throw std::runtime_error("camera returned an unsupported pixel format");
  }
  if (bgr.cols != static_cast<int>(frame.width) ||
      bgr.rows != static_cast<int>(frame.height)) {
    throw std::runtime_error(
        "decoded image dimensions do not match negotiated dimensions");
  }
  return bgr;
}

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

struct EncodedPacket {
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
};

enum class SourceKind {
  V4l2,
  Still,
  Synthetic,
};

SourceKind sourceKindFromString(const std::string& value)
{
  if (value == "v4l2") {
    return SourceKind::V4l2;
  }
  if (value == "still") {
    return SourceKind::Still;
  }
  if (value == "synthetic") {
    return SourceKind::Synthetic;
  }
  throw std::invalid_argument("source_kind must be v4l2, still, or synthetic");
}

geometry_msgs::TransformStamped toTransformMessage(
    const xgc2_camera_driver::StaticTransform& transform,
    const ros::Time& stamp)
{
  geometry_msgs::TransformStamped message;
  message.header.stamp = stamp;
  message.header.frame_id = transform.parent_frame;
  message.child_frame_id = transform.child_frame;
  message.transform.translation.x = transform.translation[0];
  message.transform.translation.y = transform.translation[1];
  message.transform.translation.z = transform.translation[2];
  message.transform.rotation.x = transform.quaternion[0];
  message.transform.rotation.y = transform.quaternion[1];
  message.transform.rotation.z = transform.quaternion[2];
  message.transform.rotation.w = transform.quaternion[3];
  return message;
}

#ifdef XGC2_CAMERA_MEDIA
using MediaOutputs = xgc2_camera_driver::MediaOutputs;
#else
class MediaOutputs;  // the H.264/RTP outputs are not part of this build
#endif

class CameraDriverNode {
 public:
  // `media` outlives the node: it keeps the camera-source service and its
  // socket across reopenings of the device.
  explicit CameraDriverNode(MediaOutputs* media)
      : nh_(),
        private_nh_("~"),
        image_transport_(nh_),
        media_(media),
        start_wall_time_(ros::WallTime::now())
  {
    loadParameters();
    const bool still = source_kind_ == SourceKind::Still;
    if (still) {
      openStill();
    } else {
      camera_info_manager_.reset(new camera_info_manager::CameraInfoManager(
          nh_, camera_name_, camera_info_url_));
      openCaptureSource();
      const sensor_msgs::CameraInfo calibration =
          camera_info_manager_->getCameraInfo();
      xgc2_camera_driver::validateCalibrationGeometry(
          camera_info_manager_->isCalibrated(),
          calibration.width,
          calibration.height,
          width_,
          height_);
    }

    // A still image never changes: its JPEG, CameraInfo and stream description
    // are published once and latched, so any late subscriber receives them
    // without the node resending a megabyte-sized message at the camera rate.
    if (!still) {
      raw_image_publisher_ = image_transport_.advertise("image_raw", 1);
      frame_timing_publisher_ =
          nh_.advertise<xgc_camera_msgs::FrameTiming>(
              "frame_timing", encoded_queue_capacity_);
    }
    camera_info_publisher_ =
        nh_.advertise<sensor_msgs::CameraInfo>("camera_info", 4, still);
    compressed_image_publisher_ =
        nh_.advertise<sensor_msgs::CompressedImage>(
            "image_raw/compressed", still ? 1U : encoded_queue_capacity_, still);
    stream_info_publisher_ =
        nh_.advertise<xgc_camera_msgs::StreamInfo>(
            "stream_info", 1, true);

    updater_.setHardwareID(
        source_kind_ == SourceKind::V4l2
            ? video_device_
            : (still ? still_image_ : "synthetic"));
    updater_.add("camera stream", this, &CameraDriverNode::diagnose);
  }

  ~CameraDriverNode()
  {
    if (source_) {
      source_->stop();
    }
    stopPublisher();
    mediaClosed("the camera node stopped");
  }

  int run()
  {
    ROS_INFO_STREAM(
        "XGC2 camera started: source=" << source_kind_name_
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
                                       << xgc2_camera_driver::timing::toString(
                                              unknown_clock_policy_));
    if (source_kind_ == SourceKind::Still) {
      return runStill();
    }

    source_->start();
    startPublisher();
    running_.store(true);
    mediaOpened();
    while (ros::ok()) {
      try {
        const SourceFrame frame = source_->read(capture_timeout_ms_);
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
    source_->stop();
    stopPublisher();
    return 0;
  }

 private:
  void loadParameters()
  {
    int width = 3840;
    int height = 2160;
    int buffer_count = 4;
    int synthetic_seed = 1;
    int synthetic_frames = 30;
    int encoded_queue_capacity = 8;
    std::string raw_publish_mode;
    std::string unknown_timestamp_clock;
    private_nh_.param<std::string>("source_kind", source_kind_name_, "v4l2");
    private_nh_.param<std::string>(
        "video_device",
        video_device_,
        "/dev/v4l/by-id/usb-LRCP_imx415_LRCP_imx415_01.00.00-video-index0");
    private_nh_.param("width", width, width);
    private_nh_.param("height", height, height);
    private_nh_.param("framerate", framerate_, 20.0);
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
        "camera_info_file", camera_info_file_, "");
    private_nh_.param<std::string>("still_image", still_image_, "");
    private_nh_.param<std::string>(
        "still_intrinsic_file", still_intrinsic_file_, "");
    private_nh_.param<std::string>(
        "still_extrinsic_file", still_extrinsic_file_, "");
    private_nh_.param("still_jpeg_quality", still_jpeg_quality_, 95);
    private_nh_.param("still_fps", still_fps_, 1.0);
    private_nh_.param<std::string>(
        "raw_publish_mode", raw_publish_mode, "on_demand");
    private_nh_.param<std::string>(
        "unknown_timestamp_clock",
        unknown_timestamp_clock,
        "assume_monotonic");
    private_nh_.param("publish_encoded", publish_encoded_, true);
    private_nh_.param("buffer_count", buffer_count, buffer_count);
    private_nh_.param(
        "encoded_queue_capacity",
        encoded_queue_capacity,
        encoded_queue_capacity);
    private_nh_.param(
        "synthetic_seed", synthetic_seed, synthetic_seed);
    private_nh_.param(
        "synthetic_jpeg_quality", synthetic_jpeg_quality_, 90);
    private_nh_.param(
        "synthetic_noise_sigma", synthetic_noise_sigma_, 3.0);
    private_nh_.param("synthetic_frames", synthetic_frames, synthetic_frames);
    private_nh_.param(
        "capture_timeout_ms", capture_timeout_ms_, 2000);

    source_kind_ = sourceKindFromString(source_kind_name_);
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
    synthetic_frames_ = positiveDimension(synthetic_frames, "synthetic_frames");
    if (framerate_ <= 0.0 || capture_timeout_ms_ <= 0 ||
        stream_id_.empty() || frame_id_.empty()) {
      throw std::invalid_argument(
          "framerate, capture_timeout_ms, camera_name, stream_id, and "
          "frame_id must be valid");
    }
    camera_name_ = xgc2_camera_driver::validateStableCameraName(camera_name_);
    if (!camera_info_file_.empty()) {
      if (source_kind_ == SourceKind::Still) {
        throw std::invalid_argument(
            "camera_info_file is not used with source_kind=still; the "
            "intrinsics come from still_intrinsic_file");
      }
      camera_info_file_ = xgc2_camera_driver::canonicalPhysicalCameraInfoFile(
          camera_info_file_, camera_name_);
      camera_info_url_ = "file://" + camera_info_file_;
    }
    if (source_kind_ == SourceKind::Still &&
        (still_image_.empty() || still_intrinsic_file_.empty() ||
         still_extrinsic_file_.empty())) {
      throw std::invalid_argument(
          "source_kind=still requires still_image, still_intrinsic_file and "
          "still_extrinsic_file");
    }
    if (xgc2::camera::pixel_format_from_string(pixel_format_) ==
        PixelFormat::H264) {
      throw std::invalid_argument(
          "native H264 capture is not supported; use mjpeg or a raw format");
    }
    if (source_kind_ == SourceKind::Synthetic &&
        xgc2::camera::pixel_format_from_string(pixel_format_) !=
            PixelFormat::MJPEG) {
      throw std::invalid_argument(
          "the synthetic source delivers mjpeg; set pixel_format to mjpeg");
    }
    (void)xgc2::camera::capture_mode_from_string(capture_mode_);
    (void)convertOutput(cv::Mat(1, 1, CV_8UC3), output_encoding_);
    raw_publish_mode_ = rawPublishModeFromString(raw_publish_mode);
    unknown_clock_policy_ =
        xgc2_camera_driver::timing::unknownClockPolicyFromString(
            unknown_timestamp_clock);
  }

  void openCaptureSource()
  {
    if (source_kind_ == SourceKind::V4l2) {
      xgc2::camera::CaptureConfig config;
      config.backend = xgc2::camera::BackendKind::V4L2;
      config.device = video_device_;
      config.width = width_;
      config.height = height_;
      config.frame_rate = framerate_;
      config.pixel_format =
          xgc2::camera::pixel_format_from_string(pixel_format_);
      config.capture_mode =
          xgc2::camera::capture_mode_from_string(capture_mode_);
      config.buffer_count = buffer_count_;
      source_ = xgc2_camera_driver::makeV4l2Source(config);
    } else {
      xgc2_camera_driver::SyntheticConfig config;
      config.width = width_;
      config.height = height_;
      config.frame_rate = framerate_;
      config.seed = synthetic_seed_;
      config.jpeg_quality = synthetic_jpeg_quality_;
      config.noise_sigma = synthetic_noise_sigma_;
      config.ring_frames = synthetic_frames_;
      source_ = xgc2_camera_driver::makeSyntheticSource(config);
    }
    width_ = source_->width();
    height_ = source_->height();
    framerate_ = source_->frame_rate();
    native_pixel_format_ = source_->pixel_format();
  }

  void openStill()
  {
    still_intrinsics_ = xgc2_camera_driver::loadIntrinsics(still_intrinsic_file_);
    const xgc2_camera_driver::Extrinsic extrinsic =
        xgc2_camera_driver::loadExtrinsic(still_extrinsic_file_);
    still_transforms_ = xgc2_camera_driver::composeStaticCameraTransforms(extrinsic);
    still_ = xgc2_camera_driver::loadStillImage(
        still_image_, still_intrinsics_, still_jpeg_quality_);
    // The optical frame of the sealed extrinsic is authoritative; a launch
    // default for frame_id must not relabel the historical pixels.
    frame_id_ = extrinsic.child_frame;
    width_ = still_.width;
    height_ = still_.height;
    framerate_ = 0.0;
    native_pixel_format_ = PixelFormat::MJPEG;
    static_broadcaster_.reset(new tf2_ros::StaticTransformBroadcaster());
  }

  int runStill()
  {
    running_.store(true);
    // The image, CameraInfo and TF carry the current ROS time, which is the
    // simulation clock when /use_sim_time is set. Wait for that clock instead
    // of stamping the only publication with zero.
    while (ros::ok() && ros::Time::now().isZero()) {
      ROS_INFO_STREAM_THROTTLE(5.0, "XGC2 camera still waiting for ROS time");
      ros::WallDuration(0.1).sleep();
    }
    if (ros::ok()) {
      publishStill();
    }
    while (ros::ok()) {
      updater_.update();
      ros::WallDuration(0.2).sleep();
    }
    running_.store(false);
    return 0;
  }

  void publishStill()
  {
    const ros::Time stamp = ros::Time::now();
    std_msgs::Header header;
    header.stamp = stamp;
    header.frame_id = frame_id_;

    xgc_camera_msgs::StreamInfo info;
    info.contract_version = xgc_camera_msgs::StreamInfo::CONTRACT_VERSION_CURRENT;
    info.stream_id = stream_id_;
    info.frame_id = frame_id_;
    info.epoch = initialEpoch();
    info.clock_domain = ros::Time::isSimTime()
                            ? xgc_camera_msgs::StreamInfo::CLOCK_DOMAIN_SIMULATION
                            : xgc_camera_msgs::StreamInfo::CLOCK_DOMAIN_SYSTEM_REALTIME;
    info.timestamp_source = xgc_camera_msgs::StreamInfo::TIMESTAMP_SOURCE_DRIVER;
    info.timestamp_reference =
        xgc_camera_msgs::StreamInfo::TIMESTAMP_REFERENCE_UNKNOWN;
    info.codec = xgc_camera_msgs::StreamInfo::CODEC_MJPEG;
    info.bitstream_format = xgc_camera_msgs::StreamInfo::BITSTREAM_FORMAT_JPEG;
    info.transport_mask =
        xgc_camera_msgs::StreamInfo::TRANSPORT_ROS_COMPRESSED_IMAGE;
    info.width = width_;
    info.height = height_;
    info.nominal_frame_rate = 0.0;
    info.publisher_queue_capacity = 1U;
    stream_info_publisher_.publish(info);

    sensor_msgs::CameraInfo camera_info;
    camera_info.header = header;
    camera_info.width = still_intrinsics_.width;
    camera_info.height = still_intrinsics_.height;
    camera_info.distortion_model = still_intrinsics_.distortion_model;
    camera_info.D = still_intrinsics_.distortion;
    const auto& matrix = still_intrinsics_.camera_matrix;
    std::copy(matrix.begin(), matrix.end(), camera_info.K.begin());
    camera_info.R = {{1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0}};
    camera_info.P = {{matrix[0], 0.0, matrix[2], 0.0, 0.0, matrix[4], matrix[5], 0.0,
                      0.0, 0.0, 1.0, 0.0}};
    camera_info_publisher_.publish(camera_info);

    sensor_msgs::CompressedImage image;
    image.header = header;
    image.format = "jpeg";
    image.data = still_.jpeg;
    compressed_image_publisher_.publish(image);

    static_broadcaster_->sendTransform(std::vector<geometry_msgs::TransformStamped>{
        toTransformMessage(still_transforms_.parent_to_link, stamp),
        toTransformMessage(still_transforms_.link_to_optical, stamp)});

    encoded_frame_count_.fetch_add(1U);
    encoded_bytes_count_.fetch_add(still_.jpeg.size());
    notePublished(0U);
    mediaOpened();
    mediaStill(static_cast<std::int64_t>(stamp.toNSec()));
  }

#ifdef XGC2_CAMERA_MEDIA
  // Description of the picture source for the camera-source service.
  xgc2_camera_driver::SourceInfo sourceInfo() const
  {
    const bool still = source_kind_ == SourceKind::Still;
    xgc2_camera_driver::SourceInfo info;
    info.source_kind = source_kind_name_;
    info.frame_id = frame_id_;
    info.width = width_;
    info.height = height_;
    info.fps = still ? still_fps_ : framerate_;
    info.clock_domain = still && ros::Time::isSimTime() ? "simulation" : "system_realtime";
    if (still) {
      info.calibrated = true;
      info.camera_matrix = still_intrinsics_.camera_matrix;
      info.distortion = still_intrinsics_.distortion;
    } else if (camera_info_manager_ && camera_info_manager_->isCalibrated()) {
      const sensor_msgs::CameraInfo calibration = camera_info_manager_->getCameraInfo();
      std::copy(calibration.K.begin(), calibration.K.end(), info.camera_matrix.begin());
      info.distortion = calibration.D;
      info.calibrated = true;
    }
    // Only a real calibration travels with a capture; a distortion vector
    // outside the 4..16 range the contract allows is no calibration at all.
    if (info.distortion.size() < 4U || info.distortion.size() > 16U) {
      info.calibrated = false;
      info.distortion.clear();
    }
    return info;
  }
#endif

  void mediaOpened()
  {
#ifdef XGC2_CAMERA_MEDIA
    if (media_) {
      media_->sourceOpened(sourceInfo());
    }
#endif
  }

  void mediaFrame(const SourceFrame& frame, const std::int64_t ros_time_ns)
  {
#ifdef XGC2_CAMERA_MEDIA
    if (media_) {
      media_->frameCaptured(frame, ros_time_ns);
    }
#else
    (void)frame;
    (void)ros_time_ns;
#endif
  }

  void mediaStill(const std::int64_t ros_time_ns)
  {
#ifdef XGC2_CAMERA_MEDIA
    if (media_) {
      media_->stillLoaded(still_.jpeg, still_.width, still_.height, ros_time_ns);
    }
#else
    (void)ros_time_ns;
#endif
  }

  void mediaClosed(const std::string& reason)
  {
#ifdef XGC2_CAMERA_MEDIA
    if (media_) {
      media_->sourceClosed(reason);
    }
#else
    (void)reason;
#endif
  }

  bool shouldPublishRaw() const
  {
    return raw_publish_mode_ == RawPublishMode::Always ||
           (raw_publish_mode_ == RawPublishMode::OnDemand &&
            raw_image_publisher_.getNumSubscribers() > 0U);
  }

  Mapping mapTimestamp(const Timestamp& source) const
  {
    return xgc2_camera_driver::timing::mapSourceTimestamp(
        source,
        unknown_clock_policy_,
        xgc2_camera_driver::timing::sampleHostClocks());
  }

  void handleFrame(const SourceFrame& frame)
  {
    const Mapping mapping = mapTimestamp(frame.timestamp);
    if (!mapping.valid) {
      invalid_timestamp_count_.fetch_add(1U);
      setError(
          std::string("source timestamp has no declared ROS time mapping "
                      "(clock=") +
          xgc2::camera::to_string(frame.timestamp.clock) +
          ", policy=" +
          xgc2_camera_driver::timing::toString(unknown_clock_policy_) + ")");
      return;
    }
    clearError();
    mediaFrame(frame, mapping.ros_time_ns);

    const bool native_encoded = frame.pixel_format == PixelFormat::MJPEG;
    if (publish_encoded_ && native_encoded) {
      handleEncodedFrame(frame, mapping);
    }
    const bool publish_raw = shouldPublishRaw();
    if (!native_encoded && !publish_raw &&
        camera_info_publisher_.getNumSubscribers() > 0U) {
      std_msgs::Header header;
      header.seq = static_cast<std::uint32_t>(frame.sequence);
      header.stamp = rosTimeFromNanoseconds(mapping.ros_time_ns);
      header.frame_id = frame_id_;
      publishCameraInfo(header);
      notePublished(frame.sequence);
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

  void inspectContinuity(
      const SourceFrame& frame,
      const Mapping& mapping,
      std::uint64_t* dropped)
  {
    if (have_stream_contract_ &&
        (mapping.effective_clock != last_effective_clock_ ||
         frame.timestamp_reference != last_timestamp_reference_)) {
      beginEncodedEpoch(
          xgc_camera_msgs::FrameTiming::
              DISCONTINUITY_SOURCE_TIME_RESET);
    }
    if (have_source_sequence_) {
      if (frame.sequence <= last_source_sequence_) {
        beginEncodedEpoch(
            xgc_camera_msgs::FrameTiming::
                DISCONTINUITY_ENCODER_RESET);
      } else if (frame.sequence > last_source_sequence_ + 1U) {
        *dropped +=
            frame.sequence - last_source_sequence_ - 1U;
      }
      if (frame.timestamp_ns() <= last_native_source_time_ns_ ||
          mapping.ros_time_ns <= last_mapped_source_time_ns_) {
        beginEncodedEpoch(
            xgc_camera_msgs::FrameTiming::
                DISCONTINUITY_SOURCE_TIME_RESET);
      }
    }
    have_source_sequence_ = true;
    last_source_sequence_ = frame.sequence;
    last_native_source_time_ns_ = frame.timestamp_ns();
    last_mapped_source_time_ns_ = mapping.ros_time_ns;
    have_stream_contract_ = true;
    last_effective_clock_ = mapping.effective_clock;
    last_timestamp_reference_ = frame.timestamp_reference;
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

  void handleEncodedFrame(const SourceFrame& frame, const Mapping& mapping)
  {
    std::uint64_t dropped = pending_dropped_frames_;
    pending_dropped_frames_ = 0U;
    inspectContinuity(frame, mapping, &dropped);
    dropped += clearOverflowedQueue();

    EncodedPacket packet;
    packet.mapping = mapping;
    packet.timestamp_reference = frame.timestamp_reference;
    packet.dequeue_monotonic_ns =
        frame.dequeue_timestamp.clock ==
                xgc2::camera::TimestampClock::Monotonic
            ? frame.dequeue_timestamp_ns()
            : 0;
    packet.source_sequence = frame.sequence;
    packet.dropped_frames_before = saturatedFrameCount(dropped);

    if (!frame.data || frame.size < 2U || frame.data[0] != 0xffU ||
        frame.data[1] != 0xd8U) {
      invalid_encoded_frame_count_.fetch_add(1U);
      setError("native MJPEG frame is not a JPEG bitstream");
      return;
    }
    packet.data.assign(frame.data, frame.data + frame.size);

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
        return;
      }
      encoded_queue_.push_back(std::move(packet));
    }
    encoded_queue_ready_.notify_one();
  }

  void publishRawFrame(const SourceFrame& frame, const Mapping& mapping)
  {
    const cv::Mat image =
        convertOutput(decodeToBgr(frame), output_encoding_);
    std_msgs::Header header;
    header.seq = static_cast<std::uint32_t>(frame.sequence);
    header.stamp = rosTimeFromNanoseconds(mapping.ros_time_ns);
    header.frame_id = frame_id_;
    sensor_msgs::ImagePtr image_message =
        cv_bridge::CvImage(header, output_encoding_, image).toImageMsg();
    raw_image_publisher_.publish(image_message);
    publishCameraInfo(header);
    raw_frame_count_.fetch_add(1U);
    notePublished(frame.sequence);
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
    sensor_msgs::CompressedImage message;
    message.header = header;
    message.format = "jpeg";
    message.data = packet.data;
    compressed_image_publisher_.publish(message);
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
    info.codec = xgc_camera_msgs::StreamInfo::CODEC_MJPEG;
    info.bitstream_format =
        xgc_camera_msgs::StreamInfo::BITSTREAM_FORMAT_JPEG;
    info.transport_mask =
        xgc_camera_msgs::StreamInfo::TRANSPORT_ROS_COMPRESSED_IMAGE;
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
    timing.keyframe = true;  // every JPEG is independently decodable
    timing.discontinuity = packet.discontinuity;
    timing.dropped_frames_before =
        packet.dropped_frames_before;
    timing.encoded_size_bytes =
        saturatedFrameCount(packet.data.size());
    timing.native_source_time_ns = packet.mapping.source_ns;
    timing.host_dequeue_monotonic_ns =
        packet.dequeue_monotonic_ns;
    const auto publish_clocks =
        xgc2_camera_driver::timing::sampleHostClocks();
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
    if (!camera_info_manager_->isCalibrated()) {
      const auto pinhole = xgc2_camera_driver::defaultUncalibratedPinhole(
          width_, height_);
      camera_info.distortion_model = "plumb_bob";
      camera_info.D.assign(5U, 0.0);
      camera_info.K.fill(0.0);
      camera_info.K[0] = pinhole.fx;
      camera_info.K[2] = pinhole.cx;
      camera_info.K[4] = pinhole.fy;
      camera_info.K[5] = pinhole.cy;
      camera_info.K[8] = 1.0;
      camera_info.R.fill(0.0);
      camera_info.R[0] = 1.0;
      camera_info.R[4] = 1.0;
      camera_info.R[8] = 1.0;
      camera_info.P.fill(0.0);
      camera_info.P[0] = pinhole.fx;
      camera_info.P[2] = pinhole.cx;
      camera_info.P[5] = pinhole.fy;
      camera_info.P[6] = pinhole.cy;
      camera_info.P[10] = 1.0;
    }
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

    const bool still = source_kind_ == SourceKind::Still;
    const bool calibrated =
        still || (camera_info_manager_ && camera_info_manager_->isCalibrated());
    if (!running_.load()) {
      status.summary(
          diagnostic_msgs::DiagnosticStatus::ERROR,
          "camera is not running");
    } else if (still) {
      status.summary(
          count == 0U ? diagnostic_msgs::DiagnosticStatus::WARN
                      : diagnostic_msgs::DiagnosticStatus::OK,
          count == 0U ? "waiting to publish the still image"
                      : "still image published once and latched");
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
    status.add("source_kind", source_kind_name_);
    status.add("device", still ? still_image_ : video_device_);
    status.add(
        "pixel_format",
        xgc2::camera::to_string(native_pixel_format_));
    status.add("output_encoding", output_encoding_);
    status.add("raw_publish_mode", toString(raw_publish_mode_));
    status.add(
        "unknown_timestamp_clock",
        xgc2_camera_driver::timing::toString(
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
    status.add("camera_info_file", camera_info_file_);
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
  ros::Publisher frame_timing_publisher_;
  ros::Publisher stream_info_publisher_;
  std::unique_ptr<camera_info_manager::CameraInfoManager>
      camera_info_manager_;
  diagnostic_updater::Updater updater_;
  std::unique_ptr<tf2_ros::StaticTransformBroadcaster> static_broadcaster_;
  std::unique_ptr<xgc2_camera_driver::CaptureSource> source_;

  SourceKind source_kind_{SourceKind::V4l2};
  std::string source_kind_name_;
  std::string video_device_;
  std::string pixel_format_;
  std::string capture_mode_;
  std::string output_encoding_;
  std::string camera_name_;
  std::string stream_id_;
  std::string frame_id_;
  std::string camera_info_file_;
  std::string camera_info_url_;
  std::string still_image_;
  std::string still_intrinsic_file_;
  std::string still_extrinsic_file_;
  int still_jpeg_quality_{95};
  double still_fps_{1.0};
  MediaOutputs* media_{nullptr};
  xgc2_camera_driver::Intrinsics still_intrinsics_;
  xgc2_camera_driver::CameraStaticTransforms still_transforms_;
  xgc2_camera_driver::StillImage still_;
  int synthetic_jpeg_quality_{90};
  double synthetic_noise_sigma_{3.0};
  std::uint32_t synthetic_frames_{30};
  PixelFormat native_pixel_format_{PixelFormat::Unknown};
  RawPublishMode raw_publish_mode_{RawPublishMode::OnDemand};
  xgc2_camera_driver::timing::UnknownClockPolicy
      unknown_clock_policy_{
          xgc2_camera_driver::timing::UnknownClockPolicy::AssumeMonotonic};
  std::uint32_t width_{3840};
  std::uint32_t height_{2160};
  std::uint32_t buffer_count_{4};
  std::uint32_t encoded_queue_capacity_{8};
  std::uint32_t synthetic_seed_{1};
  double framerate_{20.0};
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
  ros::init(argc, argv, "xgc2_camera_driver");
  ros::AsyncSpinner spinner(1);
  spinner.start();

  ros::NodeHandle nh;
  ros::NodeHandle private_nh("~");
  std::string source_kind;
  private_nh.param<std::string>("source_kind", source_kind, "v4l2");

  // The media outputs outlive every opening of the device: the camera-source
  // service keeps its socket and instance identity while a camera is unplugged.
#ifdef XGC2_CAMERA_MEDIA
  std::unique_ptr<MediaOutputs> media_owner;
  try {
    media_owner.reset(new MediaOutputs(nh, xgc2_camera_driver::loadMediaOutputsConfig(private_nh)));
  } catch (const std::exception& error) {
    ROS_FATAL_STREAM("XGC2 camera media outputs failed: " << error.what());
    return 1;
  }
  MediaOutputs* media = media_owner.get();
#else
  MediaOutputs* media = nullptr;
  std::string control_socket;
  private_nh.param<std::string>("control_socket", control_socket, "");
  if (!control_socket.empty()) {
    ROS_FATAL("this build has no camera-source service; control_socket must be empty");
    return 1;
  }
#endif
  while (ros::ok()) {
    try {
      CameraDriverNode node(media);
      return node.run();
    } catch (const xgc2::camera::CameraError& error) {
      if (source_kind != "v4l2") {
        ROS_FATAL_STREAM(
            "XGC2 camera driver failed: " << error.what());
        return 1;
      }
#ifdef XGC2_CAMERA_MEDIA
      media->sourceClosed(std::string("the camera is unavailable: ") + error.what());
#endif
      ROS_WARN_STREAM_THROTTLE(
          10.0,
          "XGC2 camera is unavailable; retaining the configured device and "
          "waiting to reconnect: " << error.what());
      ros::WallDuration(1.0).sleep();
    } catch (const std::exception& error) {
      ROS_FATAL_STREAM(
          "XGC2 camera driver failed: " << error.what());
      return 1;
    }
  }
  return 0;
}
