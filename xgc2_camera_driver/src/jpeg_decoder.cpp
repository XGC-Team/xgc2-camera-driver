#include "jpeg_decoder.hpp"

#include <climits>
#include <memory>
#include <cstring>
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

JpegDecoder::JpegDecoder()
{
  const AVCodec* codec = avcodec_find_decoder(AV_CODEC_ID_MJPEG);
  if (!codec) {
    throw std::runtime_error("FFmpeg MJPEG decoder is unavailable");
  }
  context_ = avcodec_alloc_context3(codec);
  packet_ = av_packet_alloc();
  if (!context_ || !packet_) {
    throw std::runtime_error("could not allocate the FFmpeg MJPEG decoder");
  }
  context_->thread_count = 0;
  context_->thread_type = FF_THREAD_SLICE;
  const int result = avcodec_open2(context_, codec, nullptr);
  if (result < 0) {
    throw std::runtime_error(
        "could not open the FFmpeg MJPEG decoder: " + ffmpegMessage(result));
  }
}

JpegDecoder::~JpegDecoder()
{
  av_packet_free(&packet_);
  avcodec_free_context(&context_);
}

Picture JpegDecoder::decode(const std::uint8_t* data, const std::size_t size)
{
  if (!data || size < 4U || size > static_cast<std::size_t>(INT_MAX)) {
    throw std::runtime_error("JPEG payload is empty or too large");
  }
  padded_.assign(size + AV_INPUT_BUFFER_PADDING_SIZE, 0U);
  std::memcpy(padded_.data(), data, size);
  av_packet_unref(packet_);
  packet_->data = padded_.data();
  packet_->size = static_cast<int>(size);
  int result = avcodec_send_packet(context_, packet_);
  if (result < 0) {
    throw std::runtime_error("FFmpeg rejected the JPEG: " + ffmpegMessage(result));
  }
  std::shared_ptr<AVFrame> frame(av_frame_alloc(), [](AVFrame* released) {
    av_frame_free(&released);
  });
  if (!frame) {
    throw std::runtime_error("could not allocate a decoded picture");
  }
  result = avcodec_receive_frame(context_, frame.get());
  if (result < 0) {
    throw std::runtime_error("FFmpeg could not decode the JPEG: " + ffmpegMessage(result));
  }
  if (frame->width <= 0 || frame->height <= 0) {
    throw std::runtime_error("FFmpeg decoded a JPEG with invalid dimensions");
  }
  // The MJPEG decoder emits one picture per packet.
  return Picture(std::move(frame));
}

}  // namespace xgc2_camera_driver
