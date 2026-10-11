#include "media_outputs.hpp"

#include <unistd.h>

#include <sstream>
#include <stdexcept>

namespace xgc2_camera_driver {
namespace {

std::string hostName()
{
  char name[256] = {};
  if (::gethostname(name, sizeof(name) - 1U) != 0 || name[0] == '\0') {
    throw std::runtime_error("cannot determine the host name for the service reference");
  }
  return name;
}

std::int64_t nanosecondsOf(const xgc2::camera::Timestamp& timestamp)
{
  return timestamp.to_nanoseconds();
}

// The process definitions carry private parameters on the command line. Read
// the same configuration before ROS registration so the source socket can
// report the master wait without depending on the parameter server.
struct PrivateParameters {
  const ros::M_string& values;

  template <typename T>
  void param(const std::string& name, T& value, const T& fallback) const
  {
    value = fallback;
    const auto found = values.find(name);
    if (found == values.end()) return;
    std::istringstream input(found->second);
    if (!(input >> value) || !input.eof()) {
      throw std::invalid_argument("invalid private parameter " + name);
    }
  }
};

template <>
void PrivateParameters::param<std::string>(
    const std::string& name, std::string& value, const std::string& fallback) const
{
  const auto found = values.find(name);
  value = found == values.end() ? fallback : found->second;
}

template <>
void PrivateParameters::param<bool>(const std::string& name, bool& value, const bool& fallback) const
{
  const auto found = values.find(name);
  value = fallback;
  if (found == values.end()) return;
  if (found->second == "true" || found->second == "1") value = true;
  else if (found->second == "false" || found->second == "0") value = false;
  else throw std::invalid_argument("invalid private parameter " + name);
}

template <typename Parameters>
MediaOutputsConfig loadConfig(Parameters& nh)
{
  MediaOutputsConfig config;
  MediaPipelineConfig& pipeline = config.pipeline;
  std::string source_kind;
  std::string camera_name;
  std::string rate_control;
  int rtp_port = 5004;
  int bitrate = 25000000;
  int quality = 23;
  int preview_width = 1920;
  int preview_height = 1080;
  int preview_bitrate = 8000000;
  int width = 3840;
  int height = 2160;
  double framerate = 20.0;
  nh.template param<std::string>("source_kind", source_kind, "v4l2");
  nh.template param<std::string>("camera_name", camera_name, "usb_cam");
  config.camera_name = camera_name;
  nh.param("width", width, width);
  nh.param("height", height, height);
  nh.param("framerate", framerate, framerate);
  nh.template param<std::string>("control_socket", config.control_socket, "");
  nh.template param<std::string>("target_id", config.target_id, "");
  nh.template param<std::string>("source_id", config.source_id, camera_name);
  nh.template param<std::string>("rtp_host", pipeline.rtp_host, "127.0.0.1");
  nh.param("rtp_port", rtp_port, rtp_port);
  nh.template param<std::string>("h264_encoder", pipeline.encoder, "h264_nvenc");
  nh.param("h264_bitrate", bitrate, bitrate);
  nh.template param<std::string>("h264_preset", pipeline.preset, "");
  nh.param("h264_gop_seconds", pipeline.gop_seconds, 1.0);
  nh.template param<std::string>("h264_rate_control", rate_control, "auto");
  nh.param("h264_quality", quality, quality);
  nh.param("still_fps", pipeline.still_fps, 1.0);
  nh.param("still_gop_seconds", pipeline.still_gop_seconds, 30.0);
  nh.param("preview_enabled", pipeline.preview_enabled, true);
  nh.template param<std::string>("preview_topic", config.preview_topic, "video_h264");
  nh.param("preview_max_width", preview_width, preview_width);
  nh.param("preview_max_height", preview_height, preview_height);
  nh.param("preview_bitrate", preview_bitrate, preview_bitrate);
  nh.param("preview_gop_seconds", pipeline.preview_gop_seconds, 5.0);

  if (rtp_port < 1 || rtp_port > 65535 || bitrate < 100000 || preview_bitrate < 100000 ||
      quality < 0 || quality > 51 || preview_width < 16 || preview_height < 16 ||
      pipeline.gop_seconds <= 0.0 || pipeline.still_fps <= 0.0 || pipeline.still_fps > 60.0 ||
      pipeline.still_gop_seconds <= 0.0 || pipeline.preview_gop_seconds <= 0.0) {
    throw std::invalid_argument("a media parameter is out of range");
  }
  if (rate_control != "auto" && rate_control != "bitrate" && rate_control != "quality") {
    throw std::invalid_argument("h264_rate_control must be auto, bitrate, or quality");
  }
  if (pipeline.encoder != "h264_nvenc" && pipeline.encoder != "libx264") {
    throw std::invalid_argument("h264_encoder must be h264_nvenc or libx264");
  }
  if (config.preview_topic.empty()) {
    pipeline.preview_enabled = false;
  }
  pipeline.rtp_port = static_cast<std::uint16_t>(rtp_port);
  pipeline.bitrate = static_cast<std::uint32_t>(bitrate);
  pipeline.quality = quality;
  pipeline.preview_bitrate = static_cast<std::uint32_t>(preview_bitrate);
  pipeline.preview_max_width = static_cast<std::uint32_t>(preview_width);
  pipeline.preview_max_height = static_cast<std::uint32_t>(preview_height);
  // A still costs bits only when the picture changes, so it is encoded for
  // constant quality; a live camera has a bandwidth budget instead.
  pipeline.constant_quality =
      rate_control == "quality" || (rate_control == "auto" && source_kind == "still");
  if (config.target_id.empty() && !config.control_socket.empty()) {
    config.target_id = hostName();
  }
  config.nominal.source_kind = source_kind;
  config.nominal.width = static_cast<std::uint32_t>(std::max(width, 0));
  config.nominal.height = static_cast<std::uint32_t>(std::max(height, 0));
  config.nominal.fps = source_kind == "still" ? pipeline.still_fps : framerate;
  nh.template param<std::string>("frame_id", config.nominal.frame_id, "usb_cam_optical_frame");
  config.nominal.ros_master_uri = ros::master::getURI();
  return config;
}

}  // namespace

MediaOutputsConfig loadMediaOutputsConfig(ros::NodeHandle& nh) { return loadConfig(nh); }

MediaOutputsConfig loadMediaOutputsConfig(const ros::M_string& values)
{
  PrivateParameters parameters{values};
  return loadConfig(parameters);
}

MediaOutputs::MediaOutputs(MediaOutputsConfig config)
    : preview_enabled_(config.pipeline.preview_enabled), preview_topic_(config.preview_topic),
      frame_id_(config.nominal.frame_id), state_(config.nominal)
{
  pipeline_.reset(new MediaPipeline(
      config.pipeline,
      [this](PreviewUnit&& unit) {
        if (preview_subscribers_.load() <= 0) {
          return;
        }
        foxglove_msgs::CompressedVideo message;
        message.timestamp.fromNSec(static_cast<std::uint64_t>(std::max<std::int64_t>(unit.ros_time_ns, 0)));
        {
          std::lock_guard<std::mutex> lock(frame_id_mutex_);
          message.frame_id = frame_id_;
        }
        message.format = "h264";
        message.data = std::move(unit.data);
        preview_publisher_.publish(message);
      },
      [] { return static_cast<std::int64_t>(ros::Time::now().toNSec()); },
      [](LogLevel level, const std::string& text) {
        switch (level) {
          case LogLevel::Info:
            ROS_INFO_STREAM("XGC2 camera media: " << text);
            break;
          case LogLevel::Warn:
            ROS_WARN_STREAM("XGC2 camera media: " << text);
            break;
          case LogLevel::Error:
            ROS_ERROR_STREAM("XGC2 camera media: " << text);
            break;
        }
      }));
  if (!config.control_socket.empty()) {
    SourceControlService::Options options;
    options.socket_path = config.control_socket;
    options.target_id = config.target_id;
    options.source_id = config.source_id;
    options.camera_name = config.camera_name;
    service_.reset(new SourceControlService(options, pipeline_.get(), &state_));
    ROS_INFO_STREAM(
        "XGC2 camera-source service on " << config.control_socket
                                         << " instance=" << service_->instanceId());
  }
}

void MediaOutputs::bindROS(ros::NodeHandle& nh, const std::string& run_id, const int master_pid)
{
  state_.bindMaster(ros::master::getURI(), run_id, master_pid);
  if (preview_enabled_) {
    // Encode the preview only while subscribed, with a keyframe on arrival.
    preview_publisher_ = nh.advertise<foxglove_msgs::CompressedVideo>(
        preview_topic_, 2,
        [this](const ros::SingleSubscriberPublisher&) { previewSubscribersChanged(+1); },
        [this](const ros::SingleSubscriberPublisher&) { previewSubscribersChanged(-1); });
  }
}

MediaOutputs::~MediaOutputs()
{
  // The service stops serving before the pipeline it controls goes away; the
  // pipeline stops publishing before the publisher it uses goes away.
  service_.reset();
  pipeline_.reset();
}

void MediaOutputs::previewSubscribersChanged(const int delta)
{
  const int count = preview_subscribers_.fetch_add(delta) + delta;
  ROS_INFO_STREAM("XGC2 camera media: preview subscribers: " << count);
  pipeline_->setPreviewWanted(count > 0);
  if (delta > 0) {
    pipeline_->requestPreviewKeyframe();
  }
}

void MediaOutputs::sourceOpened(const SourceInfo& info)
{
  {
    std::lock_guard<std::mutex> lock(frame_id_mutex_);
    frame_id_ = info.frame_id;
  }
  MediaSource source;
  source.width = info.width;
  source.height = info.height;
  source.fps = info.fps;
  source.still = info.source_kind == "still";
  pipeline_->setSource(source);
  SourceInfo bound = info;
  const SourceInfo previous = state_.info();
  bound.ros_master_uri = previous.ros_master_uri;
  bound.ros_master_run_id = previous.ros_master_run_id;
  bound.ros_master_pid = previous.ros_master_pid;
  state_.sourceOpened(bound);
}

void MediaOutputs::sourceClosed(const std::string& reason)
{
  state_.sourceClosed(reason);
}

void MediaOutputs::frameCaptured(const SourceFrame& frame, const std::int64_t ros_time_ns)
{
  if (frame.pixel_format != xgc2::camera::PixelFormat::MJPEG) {
    return;
  }
  auto jpeg = std::make_shared<const std::vector<std::uint8_t>>(frame.data, frame.data + frame.size);
  const std::uint64_t sequence = sequence_.fetch_add(1U) + 1U;
  auto capture = std::make_shared<CaptureFrame>();
  capture->jpeg = jpeg;
  capture->sequence = sequence;
  capture->timestamp_ns = ros_time_ns;
  capture->width = frame.width;
  capture->height = frame.height;
  state_.framePublished(capture);
  if (pipeline_->wanted()) {
    LiveFrame live;
    live.jpeg = jpeg;
    live.sequence = sequence;
    live.capture_ns = frame.timestamp.clock == xgc2::camera::TimestampClock::Monotonic
                          ? nanosecondsOf(frame.timestamp)
                          : nanosecondsOf(frame.dequeue_timestamp);
    live.ros_time_ns = ros_time_ns;
    pipeline_->submit(std::move(live));
  }
}

void MediaOutputs::stillLoaded(
    const std::vector<std::uint8_t>& jpeg,
    const std::uint32_t width,
    const std::uint32_t height,
    const std::int64_t ros_time_ns)
{
  auto shared = std::make_shared<const std::vector<std::uint8_t>>(jpeg);
  pipeline_->setStill(shared, width, height);
  auto capture = std::make_shared<CaptureFrame>();
  capture->jpeg = shared;
  capture->sequence = sequence_.fetch_add(1U) + 1U;
  capture->timestamp_ns = ros_time_ns;
  capture->width = width;
  capture->height = height;
  state_.framePublished(capture);
}

}  // namespace xgc2_camera_driver
