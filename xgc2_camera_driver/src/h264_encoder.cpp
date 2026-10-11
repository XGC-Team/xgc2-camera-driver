#include "h264_encoder.hpp"

#include <algorithm>
#include <cmath>
#include <stdexcept>

extern "C" {
#include <libavcodec/avcodec.h>
#include <libavutil/dict.h>
#include <libavutil/error.h>
#include <libavutil/frame.h>
#include <libavutil/mathematics.h>
#include <libavutil/opt.h>
#include <libavutil/pixdesc.h>
#include <libswscale/swscale.h>
}

namespace xgc2_camera_driver {
namespace {

constexpr int kRtpTimeBase = 90000;

std::string ffmpegMessage(const int code)
{
  char buffer[AV_ERROR_MAX_STRING_SIZE] = {};
  av_strerror(code, buffer, sizeof(buffer));
  return std::string(buffer);
}

// The full-range "J" pixel formats are deprecated spellings of the plain
// formats with the JPEG colour range; swscale warns about them on every use.
AVPixelFormat plainFormat(const int format, bool* full_range)
{
  *full_range = true;
  switch (format) {
    case AV_PIX_FMT_YUVJ420P:
      return AV_PIX_FMT_YUV420P;
    case AV_PIX_FMT_YUVJ422P:
      return AV_PIX_FMT_YUV422P;
    case AV_PIX_FMT_YUVJ444P:
      return AV_PIX_FMT_YUV444P;
    case AV_PIX_FMT_YUVJ440P:
      return AV_PIX_FMT_YUV440P;
    default:
      *full_range = false;
      return static_cast<AVPixelFormat>(format);
  }
}

void set(AVDictionary** options, const char* key, const std::string& value)
{
  av_dict_set(options, key, value.c_str(), 0);
}

// Options that depend on the encoder implementation. Everything shared by both
// lives on the codec context.
void setEncoderOptions(
    const std::string& codec_name, const H264EncoderSettings& settings, AVDictionary** options)
{
  if (codec_name == "libx264") {
    set(options, "preset", settings.preset.empty() ? "veryfast" : settings.preset);
    set(options, "tune", "zerolatency");
    set(options, "forced-idr", "1");
    // Keyframes come from the GOP and from explicit requests, not from scene
    // changes that would add unplanned multi-megabyte bursts.
    set(options, "x264-params", "scenecut=0");
    if (settings.constant_quality) {
      set(options, "crf", std::to_string(settings.quality));
    }
    return;
  }
  // h264_nvenc
  set(options, "zerolatency", "1");
  set(options, "forced-idr", "1");
  set(options, "no-scenecut", "1");
  if (settings.constant_quality) {
    set(options, "rc", "vbr");
    set(options, "cq", std::to_string(settings.quality));
  } else {
    set(options, "rc", "cbr");
  }
}

}  // namespace

std::unique_ptr<H264Encoder> H264Encoder::open(const H264EncoderSettings& settings)
{
  if (settings.width < 16U || settings.height < 16U || (settings.width % 2U) != 0U ||
      (settings.height % 2U) != 0U) {
    throw std::invalid_argument("H.264 needs an even width and height of at least 16");
  }
  if (!(settings.fps > 0.0) || !std::isfinite(settings.fps) || settings.bitrate < 10000U ||
      settings.gop < 1U || settings.quality < 0 || settings.quality > 51) {
    throw std::invalid_argument("H.264 fps, bitrate, GOP or quality is out of range");
  }
  if (settings.encoder != "h264_nvenc" && settings.encoder != "libx264") {
    throw std::invalid_argument("h264_encoder must be h264_nvenc or libx264");
  }
  std::unique_ptr<H264Encoder> encoder(new H264Encoder());
  std::string error;
  if (settings.encoder == "h264_nvenc") {
    if (encoder->tryOpen("h264_nvenc", settings, &error)) {
      return encoder;
    }
    encoder->fallback_reason_ = "h264_nvenc is unusable: " + error;
  }
  error.clear();
  if (!encoder->tryOpen("libx264", settings, &error)) {
    throw std::runtime_error("libx264 could not be opened: " + error);
  }
  return encoder;
}

bool H264Encoder::tryOpen(
    const char* codec_name, const H264EncoderSettings& settings, std::string* error)
{
  const AVCodec* codec = avcodec_find_encoder_by_name(codec_name);
  if (!codec) {
    *error = std::string(codec_name) + " is not part of this FFmpeg build";
    return false;
  }
  context_ = avcodec_alloc_context3(codec);
  packet_ = av_packet_alloc();
  converted_ = av_frame_alloc();
  if (!context_ || !packet_ || !converted_) {
    *error = "could not allocate the encoder";
    close();
    return false;
  }
  width_ = settings.width;
  height_ = settings.height;
  context_->width = static_cast<int>(width_);
  context_->height = static_cast<int>(height_);
  context_->time_base = AVRational{1, kRtpTimeBase};
  context_->framerate = av_d2q(settings.fps, 100000);
  context_->pix_fmt = AV_PIX_FMT_YUV420P;
  context_->gop_size = static_cast<int>(std::min<std::uint32_t>(settings.gop, 1U << 30U));
  context_->max_b_frames = 0;
  // The pictures are converted with the BT.601 matrix into limited range, the
  // same convention as the JPEG source. Say so in the stream: a decoder that
  // finds no colour description assumes BT.709 for HD content.
  context_->colorspace = AVCOL_SPC_BT470BG;
  context_->color_range = AVCOL_RANGE_MPEG;
  context_->bit_rate = settings.bitrate;
  context_->rc_max_rate = settings.bitrate;
  context_->rc_buffer_size = static_cast<int>(settings.bitrate);
  context_->thread_count = 0;

  AVDictionary* options = nullptr;
  setEncoderOptions(codec_name, settings, &options);
  const int result = avcodec_open2(context_, codec, &options);
  av_dict_free(&options);
  if (result < 0) {
    *error = ffmpegMessage(result);
    close();
    return false;
  }

  converted_->format = AV_PIX_FMT_YUV420P;
  converted_->width = context_->width;
  converted_->height = context_->height;
  if (av_frame_get_buffer(converted_, 32) < 0) {
    *error = "could not allocate the conversion buffer";
    close();
    return false;
  }
  name_ = codec_name;
  return true;
}

H264Encoder::~H264Encoder()
{
  close();
}

void H264Encoder::close() noexcept
{
  sws_freeContext(scaler_);
  scaler_ = nullptr;
  av_frame_free(&converted_);
  av_packet_free(&packet_);
  avcodec_free_context(&context_);
}

void H264Encoder::encode(
    const Picture& picture,
    const std::int64_t pts,
    const bool force_idr,
    std::vector<EncodedAccessUnit>* out)
{
  const AVFrame* source = picture.frame();
  if (!source) {
    throw std::invalid_argument("cannot encode an empty picture");
  }
  bool full_range = false;
  const AVPixelFormat format = plainFormat(source->format, &full_range);
  full_range = full_range || source->color_range == AVCOL_RANGE_JPEG;
  const bool shrinking = source->width > static_cast<int>(width_) ||
                         source->height > static_cast<int>(height_);
  const bool changed = !scaler_ || source->width != scaler_width_ ||
                       source->height != scaler_height_ || format != scaler_format_ ||
                       full_range != scaler_full_range_;
  if (changed) {
    // Configure the colour ranges before initialisation: with identical source
    // and destination formats swscale picks a plain copy at that point and a
    // range set afterwards would be ignored. Also not sws_getCachedContext,
    // which rebuilds the context on every call when swscale has adjusted the
    // flags during initialisation.
    sws_freeContext(scaler_);
    scaler_ = sws_alloc_context();
    if (!scaler_) {
      throw std::runtime_error("FFmpeg could not allocate the picture conversion");
    }
    av_opt_set_int(scaler_, "srcw", source->width, 0);
    av_opt_set_int(scaler_, "srch", source->height, 0);
    av_opt_set_int(scaler_, "src_format", format, 0);
    av_opt_set_int(scaler_, "src_range", full_range ? 1 : 0, 0);
    av_opt_set_int(scaler_, "dstw", width_, 0);
    av_opt_set_int(scaler_, "dsth", height_, 0);
    av_opt_set_int(scaler_, "dst_format", AV_PIX_FMT_YUV420P, 0);
    av_opt_set_int(scaler_, "dst_range", 0, 0);
    av_opt_set_int(scaler_, "sws_flags", shrinking ? SWS_AREA : SWS_BICUBIC, 0);
    if (sws_init_context(scaler_, nullptr, nullptr) < 0) {
      sws_freeContext(scaler_);
      scaler_ = nullptr;
      throw std::runtime_error("FFmpeg could not create the picture conversion");
    }
    const int* coefficients = sws_getCoefficients(SWS_CS_ITU601);
    sws_setColorspaceDetails(
        scaler_, coefficients, full_range ? 1 : 0, coefficients, 0, 0, 1 << 16, 1 << 16);
    scaler_width_ = source->width;
    scaler_height_ = source->height;
    scaler_format_ = format;
    scaler_full_range_ = full_range;
  }

  if (av_frame_make_writable(converted_) < 0) {
    throw std::runtime_error("the conversion buffer is not writable");
  }
  sws_scale(
      scaler_,
      source->data,
      source->linesize,
      0,
      source->height,
      converted_->data,
      converted_->linesize);
  converted_->pts = pts;
  converted_->pict_type = force_idr ? AV_PICTURE_TYPE_I : AV_PICTURE_TYPE_NONE;

  int result = avcodec_send_frame(context_, converted_);
  if (result < 0) {
    throw std::runtime_error("encoder rejected the picture: " + ffmpegMessage(result));
  }
  for (;;) {
    result = avcodec_receive_packet(context_, packet_);
    if (result == AVERROR(EAGAIN) || result == AVERROR_EOF) {
      return;
    }
    if (result < 0) {
      throw std::runtime_error("encoder failed: " + ffmpegMessage(result));
    }
    EncodedAccessUnit unit;
    unit.data.assign(packet_->data, packet_->data + packet_->size);
    unit.pts = packet_->pts;
    unit.keyframe = (packet_->flags & AV_PKT_FLAG_KEY) != 0;
    out->push_back(std::move(unit));
    av_packet_unref(packet_);
  }
}

}  // namespace xgc2_camera_driver
