// xgc2_ros_image_pipe: subscribes a sensor_msgs/Image topic and writes the raw
// frames to standard output, for an encoder pipeline that reads video from a
// pipe (for example `fdsrc ! rawvideoparse`). It exits when the reader goes away
// (EPIPE) or on a fatal error. Standard output carries frames only; every
// message goes to standard error.

#include <errno.h>
#include <signal.h>
#include <unistd.h>

#include <chrono>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <string>
#include <vector>

#include <ros/ros.h>
#include <sensor_msgs/Image.h>

#include "image_pipe.hpp"

namespace {

using xgc2_camera_driver::PipeFormat;

struct Options {
  std::string topic;
  PipeFormat format{PipeFormat::Bgr8};
  std::uint32_t width{0};
  std::uint32_t height{0};
  double max_fps{0.0};
  double depth_max_mm{4000.0};
};

void usage(const char* program)
{
  std::fprintf(
      stderr,
      "usage: %s --topic TOPIC [--format bgr8|rgb8|mono8] [--width N --height N]\n"
      "          [--max-fps F] [--depth-max MM] [ROS remappings]\n"
      "Writes each sensor_msgs/Image of TOPIC to stdout as a raw frame of the\n"
      "chosen format. With --width/--height a frame of another size is fatal.\n",
      program);
}

bool parseOptions(int* argc, char** argv, Options* options)
{
  std::vector<char*> remaining;
  remaining.push_back(argv[0]);
  for (int index = 1; index < *argc; ++index) {
    const std::string argument = argv[index];
    auto value = [&](const char* name) -> const char* {
      if (index + 1 >= *argc) {
        std::fprintf(stderr, "%s needs a value\n", name);
        return nullptr;
      }
      return argv[++index];
    };
    const char* text = nullptr;
    if (argument == "--topic") {
      if (!(text = value("--topic"))) return false;
      options->topic = text;
    } else if (argument == "--format") {
      if (!(text = value("--format"))) return false;
      const std::string format = text;
      if (format == "bgr8") {
        options->format = PipeFormat::Bgr8;
      } else if (format == "rgb8") {
        options->format = PipeFormat::Rgb8;
      } else if (format == "mono8") {
        options->format = PipeFormat::Mono8;
      } else {
        std::fprintf(stderr, "unsupported --format %s\n", text);
        return false;
      }
    } else if (argument == "--width" || argument == "--height") {
      if (!(text = value(argument.c_str()))) return false;
      const long number = std::strtol(text, nullptr, 10);
      if (number < 1 || number > 16384) {
        std::fprintf(stderr, "%s must be in 1..16384\n", argument.c_str());
        return false;
      }
      (argument == "--width" ? options->width : options->height) =
          static_cast<std::uint32_t>(number);
    } else if (argument == "--max-fps") {
      if (!(text = value("--max-fps"))) return false;
      options->max_fps = std::strtod(text, nullptr);
      if (options->max_fps < 0.0) {
        std::fprintf(stderr, "--max-fps must not be negative\n");
        return false;
      }
    } else if (argument == "--depth-max") {
      if (!(text = value("--depth-max"))) return false;
      options->depth_max_mm = std::strtod(text, nullptr);
      if (options->depth_max_mm <= 0.0) {
        std::fprintf(stderr, "--depth-max must be positive\n");
        return false;
      }
    } else if (argument == "--help" || argument == "-h") {
      return false;
    } else {
      remaining.push_back(argv[index]);  // ROS remappings and __name:= and friends
    }
  }
  if (options->topic.empty() || (options->width == 0U) != (options->height == 0U)) {
    return false;
  }
  for (std::size_t index = 0; index < remaining.size(); ++index) {
    argv[index] = remaining[index];
  }
  *argc = static_cast<int>(remaining.size());
  return true;
}

class Pipe {
 public:
  Pipe(const Options& options, const int data_fd) : options_(options), data_fd_(data_fd) {}

  void onImage(const sensor_msgs::Image::ConstPtr& image)
  {
    if (finished_) {
      return;
    }
    const auto now = std::chrono::steady_clock::now();
    if (options_.max_fps > 0.0 && frames_ > 0U &&
        now - last_ < std::chrono::duration<double>(1.0 / options_.max_fps)) {
      return;
    }
    if (frames_ == 0U) {
      std::fprintf(
          stderr, "xgc2_ros_image_pipe: %s %ux%u %s -> %s\n", options_.topic.c_str(), image->width,
          image->height, image->encoding.c_str(), xgc2_camera_driver::pipeFormatName(options_.format));
      width_ = image->width;
      height_ = image->height;
    }
    if (image->width != width_ || image->height != height_ ||
        (options_.width != 0U && (image->width != options_.width || image->height != options_.height))) {
      std::fprintf(
          stderr, "xgc2_ros_image_pipe: image size changed to %ux%u (expected %ux%u)\n",
          image->width, image->height, options_.width != 0U ? options_.width : width_,
          options_.width != 0U ? options_.height : height_);
      finish(1);
      return;
    }
    frame_.clear();
    const std::string error = xgc2_camera_driver::convertImage(
        image->encoding, image->width, image->height, image->step, image->is_bigendian != 0U,
        image->data.data(), image->data.size(), options_.format, options_.depth_max_mm, &frame_);
    if (!error.empty()) {
      std::fprintf(stderr, "xgc2_ros_image_pipe: %s\n", error.c_str());
      finish(1);
      return;
    }
    if (!writeAll()) {
      return;
    }
    last_ = now;
    ++frames_;
  }

  int exitCode() const { return exit_code_; }

 private:
  // Blocks until the reader consumed the frame: a slow encoder throttles this
  // tool, and the subscriber queue of one image drops what it cannot take.
  bool writeAll()
  {
    std::size_t offset = 0;
    while (offset < frame_.size()) {
      const ssize_t written = ::write(data_fd_, frame_.data() + offset, frame_.size() - offset);
      if (written < 0) {
        if (errno == EINTR) {
          continue;
        }
        if (errno == EPIPE) {
          finish(0);  // the reader closed its end: the pipeline is gone
        } else {
          std::fprintf(stderr, "xgc2_ros_image_pipe: write failed: %s\n", std::strerror(errno));
          finish(1);
        }
        return false;
      }
      offset += static_cast<std::size_t>(written);
    }
    return true;
  }

  void finish(const int code)
  {
    finished_ = true;
    exit_code_ = code;
    ros::shutdown();
  }

  const Options options_;
  const int data_fd_;
  std::vector<std::uint8_t> frame_;
  std::uint32_t width_{0};
  std::uint32_t height_{0};
  std::uint64_t frames_{0};
  std::chrono::steady_clock::time_point last_;
  bool finished_{false};
  int exit_code_{0};
};

}  // namespace

int main(int argc, char** argv)
{
  Options options;
  if (!parseOptions(&argc, argv, &options)) {
    usage(argv[0]);
    return 2;
  }
  // Frames own the standard output: keep rosconsole and any other library from
  // writing text into the stream by moving them to standard error.
  const int data_fd = ::dup(STDOUT_FILENO);
  if (data_fd < 0 || ::dup2(STDERR_FILENO, STDOUT_FILENO) < 0) {
    std::fprintf(stderr, "cannot take over standard output: %s\n", std::strerror(errno));
    return 1;
  }
  // A closed reader must surface as EPIPE from write(), not kill the process.
  ::signal(SIGPIPE, SIG_IGN);

  ros::init(argc, argv, "xgc2_ros_image_pipe", ros::init_options::AnonymousName);
  // roscpp handles SIGINT; a SIGTERM from the pipeline owner should also leave
  // the ROS master cleanly.
  ::signal(SIGTERM, [](int) { ros::requestShutdown(); });
  ros::NodeHandle nh;
  Pipe pipe(options, data_fd);
  ros::Subscriber subscriber =
      nh.subscribe(options.topic, 1, &Pipe::onImage, &pipe, ros::TransportHints().tcpNoDelay());
  ros::spin();
  return pipe.exitCode();
}
