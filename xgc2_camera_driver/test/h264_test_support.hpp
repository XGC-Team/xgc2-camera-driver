#pragma once

// Test helpers shared by the encoder, packetizer and pipeline tests: a small
// H.264 verifier built on FFmpeg's decoder and NAL unit inspection.

#include <cstdint>
#include <memory>
#include <stdexcept>
#include <vector>

#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp>

extern "C" {
#include <libavcodec/avcodec.h>
#include <libavutil/frame.h>
#include <libswscale/swscale.h>
}

namespace h264_test {

// NAL unit types of an Annex-B access unit, in stream order.
inline std::vector<int> nalTypes(const std::vector<std::uint8_t>& data)
{
  std::vector<int> types;
  for (std::size_t index = 0; index + 3U < data.size(); ++index) {
    if (data[index] == 0U && data[index + 1U] == 0U && data[index + 2U] == 1U) {
      types.push_back(data[index + 3U] & 0x1f);
      index += 2U;
    }
  }
  return types;
}

inline bool contains(const std::vector<int>& types, const int type)
{
  for (const int value : types) {
    if (value == type) {
      return true;
    }
  }
  return false;
}

// True for an access unit that starts a decodable sequence: SPS, PPS and IDR.
inline bool isIdr(const std::vector<std::uint8_t>& data)
{
  const auto types = nalTypes(data);
  return contains(types, 7) && contains(types, 8) && contains(types, 5);
}

class Decoder {
 public:
  Decoder()
  {
    const AVCodec* codec = avcodec_find_decoder(AV_CODEC_ID_H264);
    context_ = avcodec_alloc_context3(codec);
    context_->flags |= AV_CODEC_FLAG_LOW_DELAY;
    if (!codec || !context_ || avcodec_open2(context_, codec, nullptr) < 0) {
      throw std::runtime_error("cannot open the H.264 decoder");
    }
    packet_ = av_packet_alloc();
    frame_ = av_frame_alloc();
  }

  ~Decoder()
  {
    sws_freeContext(scaler_);
    av_frame_free(&frame_);
    av_packet_free(&packet_);
    avcodec_free_context(&context_);
  }

  Decoder(const Decoder&) = delete;
  Decoder& operator=(const Decoder&) = delete;

  // Decodes one access unit and returns the pictures it produced as BGR.
  std::vector<cv::Mat> decode(const std::vector<std::uint8_t>& access_unit)
  {
    std::vector<std::uint8_t> padded(access_unit);
    padded.resize(access_unit.size() + AV_INPUT_BUFFER_PADDING_SIZE, 0U);
    packet_->data = padded.data();
    packet_->size = static_cast<int>(access_unit.size());
    std::vector<cv::Mat> pictures;
    if (avcodec_send_packet(context_, packet_) < 0) {
      return pictures;
    }
    while (avcodec_receive_frame(context_, frame_) == 0) {
      scaler_ = sws_getCachedContext(
          scaler_, frame_->width, frame_->height, static_cast<AVPixelFormat>(frame_->format),
          frame_->width, frame_->height, AV_PIX_FMT_BGR24, SWS_BILINEAR, nullptr, nullptr,
          nullptr);
      cv::Mat bgr(frame_->height, frame_->width, CV_8UC3);
      std::uint8_t* destination[4] = {bgr.data, nullptr, nullptr, nullptr};
      int stride[4] = {static_cast<int>(bgr.step), 0, 0, 0};
      sws_scale(scaler_, frame_->data, frame_->linesize, 0, frame_->height, destination, stride);
      pictures.push_back(bgr);
      av_frame_unref(frame_);
    }
    return pictures;
  }

 private:
  AVCodecContext* context_{nullptr};
  AVPacket* packet_{nullptr};
  AVFrame* frame_{nullptr};
  SwsContext* scaler_{nullptr};
};

// Mean absolute difference, averaged over the three channels.
inline double meanAbsoluteDifference(const cv::Mat& a, const cv::Mat& b)
{
  cv::Mat difference;
  cv::absdiff(a, b, difference);
  const cv::Scalar mean = cv::mean(difference);
  return (mean[0] + mean[1] + mean[2]) / 3.0;
}

}  // namespace h264_test
