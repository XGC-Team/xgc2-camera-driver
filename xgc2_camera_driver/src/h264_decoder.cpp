#include "h264_decoder.hpp"

#include <climits>
#include <memory>
#include <stdexcept>
#include <string>
#include <utility>

extern "C" {
#include <libavcodec/avcodec.h>
#include <libavutil/error.h>
#include <libavutil/frame.h>
}

namespace xgc2_camera_driver {
namespace {

std::string ffmpegMessage(const int code)
{
  char buffer[AV_ERROR_MAX_STRING_SIZE] = {};
  av_strerror(code, buffer, sizeof(buffer));
  return std::string(buffer);
}

}  // namespace

H264Decoder::H264Decoder()
{
  const AVCodec* codec = avcodec_find_decoder(AV_CODEC_ID_H264);
  if (!codec) {
    throw std::runtime_error("FFmpeg H.264 decoder is unavailable");
  }
  context_ = avcodec_alloc_context3(codec);
  packet_ = av_packet_alloc();
  if (!context_ || !packet_) {
    throw std::runtime_error("could not allocate the FFmpeg H.264 decoder");
  }
  // One picture out per access unit, without frame-thread latency.
  context_->flags |= AV_CODEC_FLAG_LOW_DELAY;
  context_->thread_count = 1;
  const int result = avcodec_open2(context_, codec, nullptr);
  if (result < 0) {
    throw std::runtime_error(
        "could not open the FFmpeg H.264 decoder: " + ffmpegMessage(result));
  }
}

H264Decoder::~H264Decoder()
{
  av_packet_free(&packet_);
  avcodec_free_context(&context_);
}

Picture H264Decoder::decode(const std::vector<std::uint8_t>& access_unit)
{
  if (access_unit.empty() || access_unit.size() > static_cast<std::size_t>(INT_MAX)) {
    throw std::runtime_error("H.264 access unit is empty or too large");
  }
  std::vector<std::uint8_t> padded(access_unit);
  padded.resize(access_unit.size() + AV_INPUT_BUFFER_PADDING_SIZE, 0U);
  av_packet_unref(packet_);
  packet_->data = padded.data();
  packet_->size = static_cast<int>(access_unit.size());
  int result = avcodec_send_packet(context_, packet_);
  if (result < 0) {
    throw std::runtime_error("FFmpeg rejected the H.264 access unit: " + ffmpegMessage(result));
  }
  std::shared_ptr<AVFrame> frame(av_frame_alloc(), [](AVFrame* released) {
    av_frame_free(&released);
  });
  result = avcodec_receive_frame(context_, frame.get());
  if (result < 0) {
    throw std::runtime_error("FFmpeg produced no picture: " + ffmpegMessage(result));
  }
  return Picture(std::move(frame));
}

}  // namespace xgc2_camera_driver
