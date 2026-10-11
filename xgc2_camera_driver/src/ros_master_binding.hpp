#pragma once

#include <signal.h>
#include <time.h>

#include <atomic>
#include <chrono>
#include <condition_variable>
#include <cstdio>
#include <functional>
#include <mutex>
#include <random>
#include <string>
#include <thread>

#include <ros/ros.h>
#include <xmlrpcpp/XmlRpcUtil.h>

namespace xgc2_camera_driver {

// Master readiness has no event source. Before registering with ROS, retry
// with full jitter from 250 ms to 30 s; after binding, check identity every
// 10 s and end the process on loss or replacement. Topic discovery stays with
// the normal subscriber's publisher updates.
class RosMasterBinding {
 public:
  struct Identity {
    int pid{0};
    std::string run_id;
  };

  static volatile sig_atomic_t& interrupted()
  {
    static volatile sig_atomic_t flag = 0;
    return flag;
  }

  static void installSignals()
  {
    // XMLRPC's default handler prints each low-level connection failure. The
    // master wait and binding owner below report those failures with context
    // and a bounded log rate instead.
    class TransportErrors : public XmlRpc::XmlRpcErrorHandler {
     public:
      void error(const char*) override {}
    };
    static TransportErrors errors;
    XmlRpc::XmlRpcErrorHandler::setErrorHandler(&errors);
    const auto stop = [](int) {
      interrupted() = 1;
      ros::requestShutdown();
    };
    ::signal(SIGINT, stop);
    ::signal(SIGTERM, stop);
  }

  static bool readIdentity(Identity* identity)
  {
    XmlRpc::XmlRpcValue arguments, response, payload;
    arguments[0] = ros::this_node::getName();
    if (!ros::master::execute("getPid", arguments, response, payload, false) ||
        payload.getType() != XmlRpc::XmlRpcValue::TypeInt) {
      return false;
    }
    identity->pid = static_cast<int>(payload);
    arguments[1] = "/run_id";
    if (ros::master::execute("getParam", arguments, response, payload, false) &&
        payload.getType() == XmlRpc::XmlRpcValue::TypeString) {
      identity->run_id = static_cast<std::string>(payload);
    } else {
      identity->run_id.clear();
    }
    // roscore publishes its run id shortly after opening the master socket.
    // Bind only after that identity exists, rather than treating the master's
    // remaining startup as a replacement on the first monitor tick.
    return identity->pid > 0 && !identity->run_id.empty();
  }

  bool wait()
  {
    std::mt19937 generator(std::random_device{}());
    long backoff_ms = 250;
    auto last_log = std::chrono::steady_clock::time_point::min();
    unsigned suppressed = 0;
    while (!interrupted()) {
      if (readIdentity(&identity_)) {
        return true;
      }
      const auto now = std::chrono::steady_clock::now();
      if (last_log == std::chrono::steady_clock::time_point::min() ||
          now - last_log >= std::chrono::seconds(10)) {
        std::fprintf(stderr, "waiting for ROS master %s (suppressed attempts: %u)\n",
                     ros::master::getURI().c_str(), suppressed);
        last_log = now;
        suppressed = 0;
      } else {
        ++suppressed;
      }
      const long delay = std::uniform_int_distribution<long>(0, backoff_ms)(generator);
      timespec duration{delay / 1000, (delay % 1000) * 1000000};
      ::nanosleep(&duration, nullptr);  // Signals interrupt the startup wait.
      backoff_ms = std::min(30000L, backoff_ms * 2);
    }
    return false;
  }

  void monitor(std::function<void()> on_loss)
  {
    thread_ = std::thread([this, on_loss] {
      std::unique_lock<std::mutex> lock(mutex_);
      while (!changed_.wait_for(lock, std::chrono::seconds(10), [this] { return stopping_; })) {
        lock.unlock();
        Identity current;
        if (!readIdentity(&current) || current.pid != identity_.pid ||
            current.run_id != identity_.run_id) {
          lost_.store(true);
          on_loss();
          ros::shutdown();
          return;
        }
        lock.lock();
      }
    });
  }

  ~RosMasterBinding()
  {
    {
      std::lock_guard<std::mutex> lock(mutex_);
      stopping_ = true;
    }
    changed_.notify_all();
    if (thread_.joinable()) {
      thread_.join();
    }
  }

  const Identity& identity() const { return identity_; }
  bool lost() const { return lost_.load(); }

 private:
  Identity identity_;
  std::atomic<bool> lost_{false};
  std::mutex mutex_;
  std::condition_variable changed_;
  bool stopping_{false};
  std::thread thread_;
};

}  // namespace xgc2_camera_driver
