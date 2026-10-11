#pragma once

#include <atomic>
#include <chrono>
#include <condition_variable>
#include <cstdint>
#include <functional>
#include <memory>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

#include "h264_decoder.hpp"
#include "h264_encoder.hpp"
#include "jpeg_decoder.hpp"
#include "picture.hpp"
#include "rtp_h264.hpp"

namespace xgc2_camera_driver {

enum class LogLevel { Info, Warn, Error };
using Logger = std::function<void(LogLevel, const std::string&)>;

struct MediaPipelineConfig {
  // H.264/RTP towards media-edge. Loopback only.
  std::string rtp_host{"127.0.0.1"};
  std::uint16_t rtp_port{5004};
  std::string encoder{"h264_nvenc"};
  std::uint32_t bitrate{25000000};
  std::string preset;
  // Live GOP in seconds; an explicit keyframe request is always an IDR.
  double gop_seconds{1.0};
  bool constant_quality{false};
  int quality{23};
  // A still is re-encoded at this rate; its GOP is long because a keyframe is
  // requested for every new viewer anyway.
  double still_fps{1.0};
  double still_gop_seconds{30.0};

  // Lichtblick preview: a downscaled second encode of the same picture,
  // produced only while the ROS topic has subscribers.
  bool preview_enabled{true};
  std::uint32_t preview_max_width{1920};
  std::uint32_t preview_max_height{1080};
  std::uint32_t preview_bitrate{8000000};
  double preview_gop_seconds{5.0};
};

// The geometry and cadence of the camera, known before the first picture.
struct MediaSource {
  std::uint32_t width{0};
  std::uint32_t height{0};
  double fps{30.0};  // capture rate; for a still the re-encode rate
  bool still{false};
};

struct LiveFrame {
  std::shared_ptr<const std::vector<std::uint8_t>> jpeg;
  std::uint64_t sequence{0};
  // Monotonic source time; the RTP time base.
  std::int64_t capture_ns{0};
  // ROS time stamp of the frame, carried to the preview message.
  std::int64_t ros_time_ns{0};
};

struct PreviewUnit {
  std::vector<std::uint8_t> data;
  std::int64_t ros_time_ns{0};
  bool keyframe{false};
};

// Turns captured JPEG frames into H.264 for the two consumers that exist only
// while somebody watches: the RTP stream media-edge leases, and the 1080p
// preview of a ROS subscriber. Every frame is decoded once and the picture is
// shared by both encoders. A decode thread and an encode thread overlap so a
// 4K camera is not limited by their sum.
//
// A still scene is encoded once as an IDR; the pipeline decodes that IDR and
// feeds the reconstruction back, so the following pictures are skip
// macroblocks and a static camera costs a few bytes per second.
class MediaPipeline {
 public:
  // Invoked on the encode thread with an empty string on success.
  using Completion = std::function<void(const std::string& error)>;
  using PreviewSink = std::function<void(PreviewUnit&&)>;
  // ROS time in nanoseconds, used to stamp the preview of a still.
  using RosClock = std::function<std::int64_t()>;

  enum class State { Idle, Starting, Active, Stopping, Faulted };

  struct Status {
    State state{State::Idle};
    bool desired_active{false};
    std::string last_error;
    std::string encoder;          // in use, empty while idle
    std::string encoder_fallback;  // why the hardware encoder was not used
    bool preview_active{false};
    std::uint64_t pictures_decoded{0};
    std::uint64_t decode_errors{0};
    std::uint64_t frames_dropped{0};
    std::uint64_t access_units{0};
    std::uint64_t keyframes{0};
    std::uint64_t rtp_packets{0};
    std::uint64_t rtp_bytes{0};
    std::uint64_t rtp_send_errors{0};
    std::uint64_t preview_access_units{0};
    std::uint64_t preview_bytes{0};
    double encode_milliseconds_average{0.0};
  };

  MediaPipeline(
      MediaPipelineConfig config,
      PreviewSink preview_sink,
      RosClock ros_clock,
      Logger logger);
  ~MediaPipeline();
  MediaPipeline(const MediaPipeline&) = delete;
  MediaPipeline& operator=(const MediaPipeline&) = delete;

  void setSource(const MediaSource& source);

  // Cheap check for the capture thread: is anybody consuming frames?
  bool wanted() const noexcept { return wanted_.load(std::memory_order_acquire); }
  // Newest frame wins; a frame replaced before it was decoded is dropped.
  void submit(LiveFrame frame);
  // Installs the single picture of a still scene (JPEG, decoded once here).
  void setStill(
      std::shared_ptr<const std::vector<std::uint8_t>> jpeg,
      std::uint32_t width,
      std::uint32_t height);

  void setPreviewWanted(bool wanted);
  void requestPreviewKeyframe();

  // Lease operations; each completes after the encode thread applied it.
  void start(Completion done);
  void stop(Completion done);
  // Fails unless the stream is starting or active.
  void requestKeyframe(Completion done);

  // Applied only while idle; false when a stream is running.
  bool setRtpDestination(const std::string& host, std::uint16_t port);
  bool setBitrate(std::uint32_t bitrate);
  std::string rtpHost() const;
  std::uint16_t rtpPort() const;
  std::uint32_t bitrate() const;

  Status status() const;

 private:
  using SteadyClock = std::chrono::steady_clock;

  struct Work {
    Picture picture;      // live: the decoded frame; still: unused
    bool still{false};
    LiveFrame frame;      // live metadata
    std::int64_t capture_ns{0};
    std::int64_t ros_time_ns{0};
    bool want_rtp{false};
    bool rtp_idr{false};
    bool want_preview{false};
    bool preview_idr{false};
    std::vector<Completion> keyframe_waiters;
  };

  void decodeLoop();
  void encodeLoop();
  bool wantedLocked() const;
  bool selectWorkLocked(Work* work);
  bool stillDueLocked(SteadyClock::time_point now) const;
  void rtpStep(Work* work);
  void previewStep(const Work& work);
  void openRtp(const Picture& picture);
  void closeRtp();
  void closePreview();
  void fault(const std::string& error);
  void updateWantedLocked();
  std::int64_t rtpTicks(std::int64_t capture_ns);
  void log(LogLevel level, const std::string& message) const;

  const MediaPipelineConfig base_config_;
  const PreviewSink preview_sink_;
  const RosClock ros_clock_;
  const Logger logger_;

  mutable std::mutex mutex_;
  std::condition_variable decode_cv_;
  std::condition_variable encode_cv_;
  bool stopping_{false};
  MediaPipelineConfig config_;  // the mutable subset changes only while idle
  MediaSource source_;
  State state_{State::Idle};
  bool desired_active_{false};
  std::string last_error_;
  std::vector<Completion> start_waiters_;
  std::vector<Completion> keyframe_waiters_;
  std::vector<Completion> stop_waiters_;
  std::string encoder_name_;
  std::string encoder_fallback_;
  bool preview_open_{false};
  bool rtp_idr_pending_{false};
  bool preview_wanted_{false};
  bool preview_idr_pending_{false};
  std::shared_ptr<const LiveFrame> pending_;
  bool have_ready_{false};
  Picture ready_picture_;
  LiveFrame ready_frame_;
  Picture still_picture_;
  bool have_still_{false};
  SteadyClock::time_point next_still_;
  Status stats_;
  std::atomic<bool> wanted_{false};

  // Owned by the decode thread.
  JpegDecoder jpeg_decoder_;
  // Owned by the encode thread.
  std::unique_ptr<H264Encoder> rtp_encoder_;
  std::unique_ptr<UdpSender> sender_;
  std::unique_ptr<H264RtpPacketizer> packetizer_;
  std::unique_ptr<H264Encoder> preview_encoder_;
  H264Decoder reconstruction_;
  Picture rtp_reconstruction_;
  Picture preview_reconstruction_;
  std::uint32_t rtp_base_{0};
  std::int64_t rtp_origin_ns_{0};
  std::int64_t rtp_last_ticks_{-1};
  bool rtp_fresh_{false};
  std::int64_t preview_origin_ns_{0};
  std::int64_t preview_last_ticks_{-1};
  std::uint64_t preview_errors_{0};
  double encode_ms_total_{0.0};
  std::uint64_t encode_count_{0};

  std::thread decode_thread_;
  std::thread encode_thread_;
};

}  // namespace xgc2_camera_driver
