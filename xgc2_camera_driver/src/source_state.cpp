#include "source_state.hpp"

#include <utility>

namespace xgc2_camera_driver {

SourceState::SourceState(SourceInfo nominal) : info_(std::move(nominal)) {}

void SourceState::bindMaster(const std::string& uri, const std::string& run_id, const int pid)
{
  {
    std::lock_guard<std::mutex> lock(mutex_);
    info_.ros_master_uri = uri;
    info_.ros_master_run_id = run_id;
    info_.ros_master_pid = pid;
  }
  notify();
}

void SourceState::sourceOpened(const SourceInfo& info)
{
  {
    std::lock_guard<std::mutex> lock(mutex_);
    info_ = info;
    open_ = true;
    have_frame_ = false;
    reason_ = "waiting for the first picture";
  }
  notify();
}

void SourceState::sourceClosed(const std::string& reason)
{
  {
    std::lock_guard<std::mutex> lock(mutex_);
    open_ = false;
    have_frame_ = false;
    reason_ = reason;
  }
  notify();
}

void SourceState::framePublished(std::shared_ptr<const CaptureFrame> frame)
{
  {
    std::lock_guard<std::mutex> lock(mutex_);
    latest_ = std::move(frame);
    ++frame_count_;
    last_frame_ = std::chrono::steady_clock::now();
    if (open_) {
      have_frame_ = true;
      reason_.clear();
    }
  }
  notify();
}

void SourceState::setChangeHandler(std::function<void()> handler)
{
  std::lock_guard<std::mutex> lock(mutex_);
  on_change_ = std::move(handler);
}

void SourceState::notify()
{
  std::function<void()> handler;
  {
    std::lock_guard<std::mutex> lock(mutex_);
    handler = on_change_;
  }
  if (handler) {
    handler();
  }
}

SourceInfo SourceState::info() const
{
  std::lock_guard<std::mutex> lock(mutex_);
  return info_;
}

bool SourceState::ready() const
{
  std::lock_guard<std::mutex> lock(mutex_);
  return open_ && have_frame_;
}

bool SourceState::open() const
{
  std::lock_guard<std::mutex> lock(mutex_);
  return open_;
}

std::string SourceState::reason() const
{
  std::lock_guard<std::mutex> lock(mutex_);
  return reason_;
}

std::shared_ptr<const CaptureFrame> SourceState::latest() const
{
  std::lock_guard<std::mutex> lock(mutex_);
  return latest_;
}

std::uint64_t SourceState::frameCount() const
{
  std::lock_guard<std::mutex> lock(mutex_);
  return frame_count_;
}

std::chrono::milliseconds SourceState::frameAge() const
{
  std::lock_guard<std::mutex> lock(mutex_);
  if (frame_count_ == 0U) {
    return std::chrono::milliseconds::max();
  }
  return std::chrono::duration_cast<std::chrono::milliseconds>(
      std::chrono::steady_clock::now() - last_frame_);
}

}  // namespace xgc2_camera_driver
