#pragma once

// Fixtures shared by the media pipeline and XRPC service tests: a loopback RTP
// receiver that reassembles access units, a JPEG frame feeder standing in for
// a camera, and a start/stop waiter for the pipeline's completion callbacks.

#include <arpa/inet.h>
#include <netinet/in.h>
#include <sys/socket.h>
#include <unistd.h>

#include <atomic>
#include <chrono>
#include <condition_variable>
#include <cstdint>
#include <memory>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>

#include "media_pipeline.hpp"
#include "rtp_h264.hpp"

namespace media_test {

using Clock = std::chrono::steady_clock;

struct ReceivedUnit {
  std::vector<std::uint8_t> data;
  std::uint32_t rtp_timestamp{0};
  Clock::time_point arrived;
};

// Receives the RTP of one stream on an ephemeral loopback port.
class RtpCollector {
 public:
  RtpCollector()
  {
    fd_ = ::socket(AF_INET, SOCK_DGRAM, 0);
    sockaddr_in address{};
    address.sin_family = AF_INET;
    address.sin_addr.s_addr = htonl(INADDR_LOOPBACK);
    ::bind(fd_, reinterpret_cast<sockaddr*>(&address), sizeof(address));
    socklen_t length = sizeof(address);
    ::getsockname(fd_, reinterpret_cast<sockaddr*>(&address), &length);
    port_ = ntohs(address.sin_port);
    timeval timeout{0, 50000};
    ::setsockopt(fd_, SOL_SOCKET, SO_RCVTIMEO, &timeout, sizeof(timeout));
    reader_ = std::thread([this] { run(); });
  }

  ~RtpCollector()
  {
    stopping_ = true;
    reader_.join();
    ::close(fd_);
  }

  std::uint16_t port() const { return port_; }

  std::vector<ReceivedUnit> units() const
  {
    std::lock_guard<std::mutex> lock(mutex_);
    return units_;
  }

  std::size_t count() const
  {
    std::lock_guard<std::mutex> lock(mutex_);
    return units_.size();
  }

  std::uint64_t datagrams() const { return datagrams_; }
  std::uint64_t lost() const { return lost_; }

  bool waitForUnits(const std::size_t count, const std::chrono::milliseconds timeout) const
  {
    const auto deadline = Clock::now() + timeout;
    while (Clock::now() < deadline) {
      if (this->count() >= count) {
        return true;
      }
      std::this_thread::sleep_for(std::chrono::milliseconds(5));
    }
    return this->count() >= count;
  }

 private:
  void run()
  {
    xgc2_camera_driver::H264RtpDepacketizer depacketizer;
    while (!stopping_) {
      std::uint8_t packet[2048];
      const ssize_t size = ::recv(fd_, packet, sizeof(packet), 0);
      if (size <= 0) {
        continue;
      }
      ++datagrams_;
      std::vector<std::vector<std::uint8_t>> completed;
      depacketizer.push(packet, static_cast<std::size_t>(size), &completed);
      lost_ = depacketizer.lost_packets();
      if (!completed.empty()) {
        std::lock_guard<std::mutex> lock(mutex_);
        for (auto& unit : completed) {
          units_.push_back(ReceivedUnit{std::move(unit), depacketizer.last_timestamp(), Clock::now()});
        }
      }
    }
  }

  int fd_{-1};
  std::uint16_t port_{0};
  std::atomic<bool> stopping_{false};
  std::atomic<std::uint64_t> datagrams_{0};
  std::atomic<std::uint64_t> lost_{0};
  mutable std::mutex mutex_;
  std::vector<ReceivedUnit> units_;
  std::thread reader_;
};

// A moving textured JPEG, standing in for one camera frame.
inline std::shared_ptr<const std::vector<std::uint8_t>> makeJpeg(
    const int width, const int height, const int shift)
{
  cv::Mat image(height, width, CV_8UC3);
  for (int row = 0; row < height; ++row) {
    for (int column = 0; column < width; ++column) {
      image.at<cv::Vec3b>(row, column) = cv::Vec3b(
          static_cast<std::uint8_t>((column + shift * 4) * 255 / width),
          static_cast<std::uint8_t>(row * 255 / height),
          static_cast<std::uint8_t>(((row / 8 + column / 8 + shift) % 2) * 150 + 50));
    }
  }
  auto bytes = std::make_shared<std::vector<std::uint8_t>>();
  cv::imencode(".jpg", image, *bytes, {cv::IMWRITE_JPEG_QUALITY, 90});
  return bytes;
}

// Submits frames to a pipeline at a fixed rate, like a capture thread.
class Feeder {
 public:
  Feeder(
      xgc2_camera_driver::MediaPipeline* pipeline,
      const int width,
      const int height,
      const double fps)
      : pipeline_(pipeline)
  {
    for (int index = 0; index < 8; ++index) {
      ring_.push_back(makeJpeg(width, height, index));
    }
    thread_ = std::thread([this, fps] {
      const auto period = std::chrono::nanoseconds(static_cast<std::int64_t>(1e9 / fps));
      auto next = Clock::now();
      std::uint64_t sequence = 0;
      while (!stopping_) {
        xgc2_camera_driver::LiveFrame frame;
        frame.jpeg = ring_[sequence % ring_.size()];
        frame.sequence = sequence;
        frame.capture_ns = std::chrono::duration_cast<std::chrono::nanoseconds>(
                               Clock::now().time_since_epoch())
                               .count();
        frame.ros_time_ns = 1000000000LL + static_cast<std::int64_t>(sequence) * 1000000;
        pipeline_->submit(std::move(frame));
        submitted_ = ++sequence;
        next += period;
        std::this_thread::sleep_until(next);
      }
    });
  }

  ~Feeder() { stop(); }

  void stop()
  {
    stopping_ = true;
    if (thread_.joinable()) {
      thread_.join();
    }
  }

  std::uint64_t submitted() const { return submitted_; }

 private:
  xgc2_camera_driver::MediaPipeline* pipeline_;
  std::vector<std::shared_ptr<const std::vector<std::uint8_t>>> ring_;
  std::atomic<bool> stopping_{false};
  std::atomic<std::uint64_t> submitted_{0};
  std::thread thread_;
};

// Blocks for the completion callback of a pipeline operation.
class Waiter {
 public:
  xgc2_camera_driver::MediaPipeline::Completion callback()
  {
    return [this](const std::string& error) {
      std::lock_guard<std::mutex> lock(mutex_);
      error_ = error;
      done_ = true;
      condition_.notify_all();
    };
  }

  // Returns false on timeout.
  bool wait(const std::chrono::milliseconds timeout)
  {
    std::unique_lock<std::mutex> lock(mutex_);
    return condition_.wait_for(lock, timeout, [this] { return done_; });
  }

  std::string error() const
  {
    std::lock_guard<std::mutex> lock(mutex_);
    return error_;
  }

 private:
  mutable std::mutex mutex_;
  std::condition_variable condition_;
  bool done_{false};
  std::string error_;
};

inline xgc2_camera_driver::Logger silentLogger()
{
  return [](xgc2_camera_driver::LogLevel, const std::string&) {};
}

inline xgc2_camera_driver::MediaPipelineConfig testConfig(const std::uint16_t rtp_port)
{
  xgc2_camera_driver::MediaPipelineConfig config;
  config.rtp_port = rtp_port;
  config.encoder = "libx264";
  config.bitrate = 3000000;
  config.preview_enabled = false;
  return config;
}

}  // namespace media_test
