#pragma once

#include <atomic>
#include <cstdint>
#include <memory>
#include <string>
#include <vector>

#include <foxglove_msgs/CompressedVideo.h>
#include <ros/ros.h>

#include "media_pipeline.hpp"
#include "source_control_service.hpp"
#include "source_frame.hpp"
#include "source_state.hpp"

namespace xgc2_camera_driver {

struct MediaOutputsConfig {
  MediaPipelineConfig pipeline;
  // Empty disables the camera-source service; the preview still works.
  std::string control_socket;
  std::string target_id;
  std::string source_id;
  std::string preview_topic{"video_h264"};
  // Description of the source before it is open, from the parameters.
  SourceInfo nominal;
};

// Reads the media parameters of the private namespace. Throws
// std::invalid_argument for values that cannot work.
MediaOutputsConfig loadMediaOutputsConfig(ros::NodeHandle& private_nh);

// The consumers of a camera that exist only while somebody watches: the
// camera-source service for media-edge and the 1080p H.264 preview for ROS
// subscribers. It outlives individual openings of the device, so the service
// keeps its socket and instance identity while a camera is unplugged and
// reports not ready instead of disappearing.
class MediaOutputs {
 public:
  MediaOutputs(ros::NodeHandle& nh, MediaOutputsConfig config);
  ~MediaOutputs();
  MediaOutputs(const MediaOutputs&) = delete;
  MediaOutputs& operator=(const MediaOutputs&) = delete;

  void sourceOpened(const SourceInfo& info);
  void sourceClosed(const std::string& reason);
  // One captured MJPEG frame of a live source.
  void frameCaptured(const SourceFrame& frame, std::int64_t ros_time_ns);
  // The single picture of a still scene.
  void stillLoaded(
      const std::vector<std::uint8_t>& jpeg,
      std::uint32_t width,
      std::uint32_t height,
      std::int64_t ros_time_ns);

 private:
  void previewSubscribersChanged(int delta);

  ros::Publisher preview_publisher_;
  ros::WallTimer master_monitor_;
  std::atomic<int> preview_subscribers_{0};
  std::mutex frame_id_mutex_;
  std::string frame_id_;
  std::atomic<std::uint64_t> sequence_{0};
  SourceState state_;
  std::unique_ptr<MediaPipeline> pipeline_;
  std::unique_ptr<SourceControlService> service_;
};

}  // namespace xgc2_camera_driver
