#pragma once

#include <atomic>
#include <chrono>
#include <condition_variable>
#include <cstdint>
#include <deque>
#include <functional>
#include <memory>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

#include <xgc2/xrpc/http.hpp>
#include <xgc2/xrpc/json_http.hpp>

#include "media_pipeline.hpp"
#include "source_state.hpp"

namespace xgc2_camera_driver {

// The media-edge `camera-source` v1 contract (XRPC http.v1 over a Unix socket):
// describe, status, start, stop, request-keyframe, capture and config for the
// one source this camera process owns. The service owns no media itself; it
// maps the contract onto the MediaPipeline lease and the latest captured JPEG.
class SourceControlService {
 public:
  struct Options {
    // Canonical absolute socket path inside an owned 0700 directory.
    std::string socket_path;
    // Execution target identity of the service reference. media-edge compares
    // it with its own host name.
    std::string target_id;
    std::string source_id;
    // Where the stream goes; reported by the descriptor.
    std::uint8_t rtp_payload_type{96};
  };

  // Throws std::runtime_error when the socket cannot be bound.
  SourceControlService(Options options, MediaPipeline* pipeline, SourceState* state);
  ~SourceControlService();
  SourceControlService(const SourceControlService&) = delete;
  SourceControlService& operator=(const SourceControlService&) = delete;

  const std::string& instanceId() const noexcept { return instance_id_; }

 private:
  using Clock = std::chrono::steady_clock;

  struct DescribeWaiter {
    xgc2::xrpc::HttpReply reply;
    Clock::time_point until;
  };
  struct FreshCapture {
    xgc2::xrpc::HttpReply reply;
    xgc2::xrpc::Json request;
    std::uint64_t after_count;
    Clock::time_point until;
  };

  void serve();
  void handle(xgc2::xrpc::HttpRequest request, xgc2::xrpc::HttpReply reply);
  void route(const xgc2::xrpc::HttpRequest& request, const xgc2::xrpc::HttpReply& reply);
  void describeService(
      const std::string& query, const xgc2::xrpc::HttpRequest& request,
      const xgc2::xrpc::HttpReply& reply);
  void describeSource(const xgc2::xrpc::HttpReply& reply) const;
  void sourceStatus(const xgc2::xrpc::HttpReply& reply) const;
  void sourceConfig(const xgc2::xrpc::HttpReply& reply) const;
  void patchConfig(const xgc2::xrpc::Json& body, const xgc2::xrpc::HttpReply& reply);
  void lifecycle(const std::string& operation, const xgc2::xrpc::HttpReply& reply);
  void capture(const xgc2::xrpc::Json& body, const xgc2::xrpc::HttpRequest& request,
               const xgc2::xrpc::HttpReply& reply);
  void deliverCapture(
      const xgc2::xrpc::Json& request,
      const std::shared_ptr<const CaptureFrame>& frame,
      const xgc2::xrpc::HttpReply& reply);
  void processWaiters();
  void waitForDeadlines();
  void post(std::function<void()> work);
  void captureWorker();

  xgc2::xrpc::Json serviceDescription() const;
  xgc2::xrpc::Json serviceRef() const;
  xgc2::xrpc::Json factsJson() const;
  xgc2::xrpc::Json lifecycleReceipt(const char* state_name, bool active) const;
  void respond(const xgc2::xrpc::HttpReply& reply, int status, const xgc2::xrpc::Json& body) const;
  void fail(const xgc2::xrpc::HttpReply& reply, int status, const char* code,
            const std::string& message) const;

  const Options options_;
  const std::string instance_id_;
  MediaPipeline* const pipeline_;
  SourceState* const state_;
  std::atomic<std::uint64_t> revision_{1};
  std::atomic<bool> stopping_{false};
  std::atomic<bool> capture_in_flight_{false};
  // Held calls, answered by the service thread when their condition arrives.
  std::mutex waiters_mutex_;
  std::condition_variable waiters_changed_;
  std::uint64_t waiters_revision_{0};
  std::vector<DescribeWaiter> describe_waiters_;
  std::vector<FreshCapture> fresh_captures_;
  std::atomic<std::size_t> waiting_{0};
  std::unique_ptr<xgc2::xrpc::HttpServer> server_;

  std::mutex work_mutex_;
  std::condition_variable work_ready_;
  std::deque<std::function<void()>> work_;

  std::thread service_thread_;
  std::thread capture_thread_;
  std::thread deadline_thread_;
};

}  // namespace xgc2_camera_driver
