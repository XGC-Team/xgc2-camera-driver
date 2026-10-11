#include "media_pipeline.hpp"

#include <gtest/gtest.h>

#include <algorithm>
#include <chrono>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

#include "h264_test_support.hpp"
#include "media_test_support.hpp"

namespace driver = xgc2_camera_driver;
using namespace std::chrono_literals;
using media_test::Feeder;
using media_test::RtpCollector;
using media_test::Waiter;

namespace {

std::int64_t rosNow()
{
  return 1700000000000000000LL;
}

struct Previews {
  std::mutex mutex;
  std::vector<driver::PreviewUnit> units;

  driver::MediaPipeline::PreviewSink sink()
  {
    return [this](driver::PreviewUnit&& unit) {
      std::lock_guard<std::mutex> lock(mutex);
      units.push_back(std::move(unit));
    };
  }

  std::size_t count()
  {
    std::lock_guard<std::mutex> lock(mutex);
    return units.size();
  }
};

driver::MediaSource liveSource(const int width, const int height, const double fps)
{
  driver::MediaSource source;
  source.width = static_cast<std::uint32_t>(width);
  source.height = static_cast<std::uint32_t>(height);
  source.fps = fps;
  return source;
}

}  // namespace

TEST(MediaPipeline, StreamsOnlyWhileALeaseIsHeld)
{
  RtpCollector collector;
  Previews previews;
  driver::MediaPipeline pipeline(
      media_test::testConfig(collector.port()), previews.sink(), rosNow, media_test::silentLogger());
  pipeline.setSource(liveSource(320, 240, 30.0));
  Feeder feeder(&pipeline, 320, 240, 30.0);

  // Nobody holds a lease: frames are not even decoded.
  std::this_thread::sleep_for(300ms);
  EXPECT_EQ(collector.datagrams(), 0U);
  EXPECT_EQ(pipeline.status().pictures_decoded, 0U);
  EXPECT_EQ(pipeline.status().state, driver::MediaPipeline::State::Idle);

  Waiter started;
  pipeline.start(started.callback());
  ASSERT_TRUE(started.wait(5s));
  EXPECT_EQ(started.error(), "");
  EXPECT_EQ(pipeline.status().state, driver::MediaPipeline::State::Active);
  EXPECT_TRUE(pipeline.status().desired_active);
  EXPECT_EQ(pipeline.status().encoder, "libx264");
  ASSERT_TRUE(collector.waitForUnits(20, 5s));

  const auto units = collector.units();
  EXPECT_TRUE(h264_test::isIdr(units[0].data)) << "the stream starts on an IDR";
  h264_test::Decoder decoder;
  std::size_t decoded = 0;
  for (const auto& unit : units) {
    decoded += decoder.decode(unit.data).size();
  }
  EXPECT_EQ(decoded, units.size());
  EXPECT_EQ(collector.lost(), 0U);
  // 90 kHz timestamps follow the capture clock: about 3000 ticks per frame.
  const std::uint32_t span = units[10].rtp_timestamp - units[0].rtp_timestamp;
  EXPECT_GT(span, 10U * 2000U);
  EXPECT_LT(span, 10U * 4500U);

  Waiter stopped;
  pipeline.stop(stopped.callback());
  ASSERT_TRUE(stopped.wait(5s));
  EXPECT_EQ(stopped.error(), "");
  EXPECT_EQ(pipeline.status().state, driver::MediaPipeline::State::Idle);
  EXPECT_FALSE(pipeline.status().desired_active);
  EXPECT_TRUE(pipeline.status().encoder.empty());
  std::this_thread::sleep_for(100ms);
  const std::size_t after_stop = collector.count();
  std::this_thread::sleep_for(300ms);
  EXPECT_EQ(collector.count(), after_stop) << "no packets after the lease ended";
  EXPECT_FALSE(pipeline.wanted());

  // The lease can be taken again; the new stream starts on a fresh IDR.
  Waiter again;
  pipeline.start(again.callback());
  ASSERT_TRUE(again.wait(5s));
  ASSERT_TRUE(collector.waitForUnits(after_stop + 3, 5s));
  EXPECT_TRUE(h264_test::isIdr(collector.units()[after_stop].data));
}

TEST(MediaPipeline, KeyframeRequestProducesARealIdr)
{
  RtpCollector collector;
  Previews previews;
  auto config = media_test::testConfig(collector.port());
  config.gop_seconds = 100.0;  // no periodic IDR inside this test
  driver::MediaPipeline pipeline(config, previews.sink(), rosNow, media_test::silentLogger());
  pipeline.setSource(liveSource(320, 240, 30.0));
  Feeder feeder(&pipeline, 320, 240, 30.0);

  Waiter started;
  pipeline.start(started.callback());
  ASSERT_TRUE(started.wait(5s));
  ASSERT_TRUE(collector.waitForUnits(15, 5s));
  for (const auto& unit : collector.units()) {
    // Only the first unit is an IDR before the request.
    (void)unit;
  }
  std::size_t idr_before = 0;
  for (const auto& unit : collector.units()) {
    idr_before += h264_test::isIdr(unit.data) ? 1U : 0U;
  }
  EXPECT_EQ(idr_before, 1U);

  const std::size_t mark = collector.count();
  Waiter keyframe;
  pipeline.requestKeyframe(keyframe.callback());
  ASSERT_TRUE(keyframe.wait(5s));
  EXPECT_EQ(keyframe.error(), "");
  ASSERT_TRUE(collector.waitForUnits(mark + 3, 5s));
  const auto units = collector.units();
  std::size_t index = mark;
  while (index < units.size() && !h264_test::isIdr(units[index].data)) {
    ++index;
  }
  ASSERT_LT(index, units.size()) << "no IDR after the request";
  EXPECT_LE(index, mark + 1U) << "the IDR comes with the next picture";
  // A decoder that joins at the IDR needs nothing before it.
  h264_test::Decoder late_joiner;
  EXPECT_EQ(late_joiner.decode(units[index].data).size(), 1U);
  EXPECT_EQ(late_joiner.decode(units[index + 1U].data).size(), 1U);

  // A keyframe request without a stream is refused.
  Waiter stopped;
  pipeline.stop(stopped.callback());
  ASSERT_TRUE(stopped.wait(5s));
  Waiter refused;
  pipeline.requestKeyframe(refused.callback());
  ASSERT_TRUE(refused.wait(1s));
  EXPECT_NE(refused.error(), "");
}

TEST(MediaPipeline, ConfigurationChangesApplyOnlyWhileIdle)
{
  RtpCollector collector;
  Previews previews;
  driver::MediaPipeline pipeline(
      media_test::testConfig(collector.port()), previews.sink(), rosNow, media_test::silentLogger());
  pipeline.setSource(liveSource(320, 240, 30.0));
  EXPECT_TRUE(pipeline.setBitrate(1500000));
  EXPECT_EQ(pipeline.bitrate(), 1500000U);
  EXPECT_TRUE(pipeline.setRtpDestination("127.0.0.1", 5999));
  EXPECT_EQ(pipeline.rtpPort(), 5999);
  EXPECT_TRUE(pipeline.setRtpDestination("127.0.0.1", collector.port()));

  Feeder feeder(&pipeline, 320, 240, 30.0);
  Waiter started;
  pipeline.start(started.callback());
  ASSERT_TRUE(started.wait(5s));
  EXPECT_FALSE(pipeline.setBitrate(2000000)) << "restart-required while streaming";
  EXPECT_FALSE(pipeline.setRtpDestination("127.0.0.1", 5998));
  EXPECT_EQ(pipeline.bitrate(), 1500000U);
}

TEST(MediaPipeline, StartFailureIsReportedAndTheNextStartRecovers)
{
  RtpCollector collector;
  Previews previews;
  driver::MediaPipeline pipeline(
      media_test::testConfig(collector.port()), previews.sink(), rosNow, media_test::silentLogger());
  pipeline.setSource(liveSource(321, 241, 30.0));
  {
    Feeder odd(&pipeline, 321, 241, 30.0);
    Waiter failed;
    pipeline.start(failed.callback());
    ASSERT_TRUE(failed.wait(5s));
    EXPECT_NE(failed.error().find("even"), std::string::npos) << failed.error();
    const auto status = pipeline.status();
    EXPECT_EQ(status.state, driver::MediaPipeline::State::Faulted);
    EXPECT_NE(status.last_error.find("even"), std::string::npos);
  }
  pipeline.setSource(liveSource(320, 240, 30.0));
  Feeder valid(&pipeline, 320, 240, 30.0);
  Waiter retried;
  pipeline.start(retried.callback());
  ASSERT_TRUE(retried.wait(5s));
  EXPECT_EQ(retried.error(), "");
  EXPECT_EQ(pipeline.status().state, driver::MediaPipeline::State::Active);
  EXPECT_TRUE(pipeline.status().last_error.empty());
}

TEST(MediaPipeline, HardwarePreferenceFallsBackAndSaysWhy)
{
  RtpCollector collector;
  Previews previews;
  auto config = media_test::testConfig(collector.port());
  config.encoder = "h264_nvenc";
  driver::MediaPipeline pipeline(config, previews.sink(), rosNow, media_test::silentLogger());
  pipeline.setSource(liveSource(320, 240, 30.0));
  Feeder feeder(&pipeline, 320, 240, 30.0);
  Waiter started;
  pipeline.start(started.callback());
  ASSERT_TRUE(started.wait(5s));
  EXPECT_EQ(started.error(), "");
  const auto status = pipeline.status();
  if (status.encoder == "h264_nvenc") {
    EXPECT_TRUE(status.encoder_fallback.empty());
  } else {
    EXPECT_EQ(status.encoder, "libx264");
    EXPECT_NE(status.encoder_fallback.find("h264_nvenc"), std::string::npos);
  }
  ASSERT_TRUE(collector.waitForUnits(5, 5s));
}

TEST(MediaPipeline, PendingStartIsFailedOnStopAndOnShutdown)
{
  RtpCollector collector;
  Previews previews;
  Waiter unanswered;
  {
    driver::MediaPipeline pipeline(
        media_test::testConfig(collector.port()), previews.sink(), rosNow, media_test::silentLogger());
    pipeline.setSource(liveSource(320, 240, 30.0));
    // No frames arrive, so the stream cannot start.
    Waiter pending;
    pipeline.start(pending.callback());
    EXPECT_FALSE(pending.wait(200ms));
    EXPECT_EQ(pipeline.status().state, driver::MediaPipeline::State::Starting);
    Waiter stopped;
    pipeline.stop(stopped.callback());
    ASSERT_TRUE(stopped.wait(5s));
    EXPECT_NE(pending.error(), "") << "the pending start is failed by the stop";

    pipeline.start(unanswered.callback());
    EXPECT_FALSE(unanswered.wait(100ms));
  }
  // Destroying the pipeline answers whoever was still waiting.
  EXPECT_TRUE(unanswered.wait(1s));
  EXPECT_NE(unanswered.error(), "");
}

TEST(MediaPipeline, StillSceneIsEncodedOnceAndRepeatedAsSkipPictures)
{
  RtpCollector collector;
  Previews previews;
  auto config = media_test::testConfig(collector.port());
  config.constant_quality = true;
  config.quality = 23;
  config.still_fps = 4.0;
  config.still_gop_seconds = 600.0;
  driver::MediaPipeline pipeline(config, previews.sink(), rosNow, media_test::silentLogger());
  driver::MediaSource source;
  source.width = 640;
  source.height = 360;
  source.fps = 4.0;
  source.still = true;
  pipeline.setSource(source);
  pipeline.setStill(media_test::makeJpeg(640, 360, 3), 640, 360);

  const auto started_at = media_test::Clock::now();
  Waiter started;
  pipeline.start(started.callback());
  ASSERT_TRUE(started.wait(5s));
  ASSERT_TRUE(collector.waitForUnits(9, 6s));
  const double elapsed =
      std::chrono::duration<double>(media_test::Clock::now() - started_at).count();
  const auto units = collector.units();
  EXPECT_TRUE(h264_test::isIdr(units[0].data));
  std::size_t followers = 0;
  for (std::size_t index = 1; index < 9; ++index) {
    EXPECT_FALSE(h264_test::isIdr(units[index].data)) << "unit " << index;
    EXPECT_LT(units[index].data.size(), units[0].data.size() / 50U) << "unit " << index;
    followers += units[index].data.size();
  }
  // Eight further pictures at 4 Hz take about two seconds, not eight frames' time.
  EXPECT_GT(elapsed, 1.5);
  EXPECT_LT(elapsed, 4.0);
  // RTP time advances one quarter second per picture.
  const std::uint32_t step = units[5].rtp_timestamp - units[4].rtp_timestamp;
  EXPECT_NEAR(static_cast<double>(step), 22500.0, 2500.0);
  std::cout << "still idr=" << units[0].data.size() << " bytes, eight repeats=" << followers
            << " bytes" << std::endl;

  // Every decoded picture is the scene.
  h264_test::Decoder decoder;
  for (std::size_t index = 0; index < 9; ++index) {
    ASSERT_EQ(decoder.decode(units[index].data).size(), 1U) << "unit " << index;
  }

  // A new viewer's keyframe request is answered at once, not at the next tick.
  const std::size_t mark = collector.count();
  const auto asked = media_test::Clock::now();
  Waiter keyframe;
  pipeline.requestKeyframe(keyframe.callback());
  ASSERT_TRUE(keyframe.wait(2s));
  const double answered =
      std::chrono::duration<double>(media_test::Clock::now() - asked).count();
  EXPECT_LT(answered, 0.2);
  ASSERT_TRUE(collector.waitForUnits(mark + 1, 1s));
  const auto after = collector.units();
  std::size_t idr = mark;
  while (idr < after.size() && !h264_test::isIdr(after[idr].data)) {
    ++idr;
  }
  ASSERT_LT(idr, after.size());
  h264_test::Decoder joiner;
  ASSERT_EQ(joiner.decode(after[idr].data).size(), 1U);
  // And the pictures after that second IDR still decode and stay tiny.
  ASSERT_TRUE(collector.waitForUnits(idr + 3, 3s));
  const auto later = collector.units();
  for (std::size_t index = idr + 1; index < idr + 3; ++index) {
    EXPECT_EQ(joiner.decode(later[index].data).size(), 1U);
    EXPECT_LT(later[index].data.size(), later[idr].data.size() / 50U);
  }
}

TEST(MediaPipeline, PreviewSharesTheDecodedPictureAndFollowsSubscribers)
{
  RtpCollector collector;
  Previews previews;
  auto config = media_test::testConfig(collector.port());
  config.preview_enabled = true;
  config.preview_max_width = 320;
  config.preview_max_height = 180;
  config.preview_bitrate = 1000000;
  driver::MediaPipeline pipeline(config, previews.sink(), rosNow, media_test::silentLogger());
  pipeline.setSource(liveSource(640, 360, 30.0));
  Feeder feeder(&pipeline, 640, 360, 30.0);

  // Preview only: no RTP packets, and the first unit is a keyframe.
  std::this_thread::sleep_for(100ms);
  EXPECT_EQ(pipeline.status().pictures_decoded, 0U);
  pipeline.setPreviewWanted(true);
  for (int wait = 0; wait < 100 && previews.count() < 10; ++wait) {
    std::this_thread::sleep_for(50ms);
  }
  ASSERT_GE(previews.count(), 10U);
  EXPECT_EQ(collector.datagrams(), 0U);
  EXPECT_TRUE(pipeline.status().preview_active);
  {
    std::lock_guard<std::mutex> lock(previews.mutex);
    EXPECT_TRUE(previews.units[0].keyframe);
    EXPECT_TRUE(h264_test::isIdr(previews.units[0].data));
    h264_test::Decoder decoder;
    const auto pictures = decoder.decode(previews.units[0].data);
    ASSERT_EQ(pictures.size(), 1U);
    EXPECT_EQ(pictures[0].cols, 320);
    EXPECT_EQ(pictures[0].rows, 180);
    // The stamp is the capture time of the frame that was encoded.
    EXPECT_GE(previews.units[0].ros_time_ns, 1000000000LL);
    EXPECT_EQ((previews.units[0].ros_time_ns - 1000000000LL) % 1000000LL, 0);
  }

  // A new subscriber gets a keyframe soon.
  const std::size_t mark = previews.count();
  pipeline.requestPreviewKeyframe();
  for (int wait = 0; wait < 100 && previews.count() < mark + 2; ++wait) {
    std::this_thread::sleep_for(20ms);
  }
  {
    std::lock_guard<std::mutex> lock(previews.mutex);
    bool saw_keyframe = false;
    for (std::size_t index = mark; index < previews.units.size(); ++index) {
      saw_keyframe = saw_keyframe || previews.units[index].keyframe;
    }
    EXPECT_TRUE(saw_keyframe);
  }

  // RTP and preview together decode each frame once.
  Waiter started;
  pipeline.start(started.callback());
  ASSERT_TRUE(started.wait(5s));
  const auto before = pipeline.status();
  auto after = before;
  for (int wait = 0; wait < 400 && after.access_units < before.access_units + 20; ++wait) {
    std::this_thread::sleep_for(20ms);
    after = pipeline.status();
  }
  const std::uint64_t decoded = after.pictures_decoded - before.pictures_decoded;
  const std::uint64_t rtp_units = after.access_units - before.access_units;
  const std::uint64_t preview_units = after.preview_access_units - before.preview_access_units;
  EXPECT_GE(rtp_units, 20U);
  EXPECT_GE(preview_units, 10U);
  EXPECT_LE(decoded, std::max(rtp_units, preview_units) + 3U)
      << "decoded=" << decoded << " rtp=" << rtp_units << " preview=" << preview_units;

  // Without subscribers the preview stops while RTP continues.
  pipeline.setPreviewWanted(false);
  std::this_thread::sleep_for(150ms);
  const std::size_t frozen = previews.count();
  std::this_thread::sleep_for(300ms);
  EXPECT_EQ(previews.count(), frozen);
  EXPECT_FALSE(pipeline.status().preview_active);
  const std::size_t rtp_mark = collector.count();
  ASSERT_TRUE(collector.waitForUnits(rtp_mark + 3, 3s));
}

TEST(MediaPipeline, PreviewEncoderIsReleasedWhenTheLastSubscriberLeavesAnIdlePipeline)
{
  RtpCollector collector;
  Previews previews;
  auto config = media_test::testConfig(collector.port());
  config.preview_enabled = true;
  config.preview_max_width = 320;
  config.preview_max_height = 180;
  driver::MediaPipeline pipeline(config, previews.sink(), rosNow, media_test::silentLogger());
  pipeline.setSource(liveSource(640, 360, 30.0));
  {
    Feeder feeder(&pipeline, 640, 360, 30.0);
    pipeline.setPreviewWanted(true);
    for (int wait = 0; wait < 100 && previews.count() < 3; ++wait) {
      std::this_thread::sleep_for(50ms);
    }
    ASSERT_GE(previews.count(), 3U);
    EXPECT_TRUE(pipeline.status().preview_active);
    pipeline.setPreviewWanted(false);
    feeder.stop();  // no frame arrives after the subscriber left
  }
  for (int wait = 0; wait < 100 && pipeline.status().preview_active; ++wait) {
    std::this_thread::sleep_for(20ms);
  }
  EXPECT_FALSE(pipeline.status().preview_active);
}

TEST(MediaPipeline, StillPreviewIsPacedLikeTheStream)
{
  RtpCollector collector;
  Previews previews;
  auto config = media_test::testConfig(collector.port());
  config.preview_enabled = true;
  config.preview_max_width = 320;
  config.preview_max_height = 180;
  config.preview_bitrate = 1000000;
  config.constant_quality = true;
  config.still_fps = 5.0;
  driver::MediaPipeline pipeline(config, previews.sink(), rosNow, media_test::silentLogger());
  driver::MediaSource source;
  source.width = 640;
  source.height = 360;
  source.fps = 5.0;
  source.still = true;
  pipeline.setSource(source);
  pipeline.setStill(media_test::makeJpeg(640, 360, 1), 640, 360);
  pipeline.setPreviewWanted(true);
  for (int wait = 0; wait < 100 && previews.count() < 6; ++wait) {
    std::this_thread::sleep_for(50ms);
  }
  ASSERT_GE(previews.count(), 6U);
  EXPECT_EQ(collector.datagrams(), 0U) << "no RTP without a lease";
  std::lock_guard<std::mutex> lock(previews.mutex);
  EXPECT_TRUE(previews.units[0].keyframe);
  for (std::size_t index = 1; index < 6; ++index) {
    EXPECT_FALSE(previews.units[index].keyframe);
    EXPECT_LT(previews.units[index].data.size(), previews.units[0].data.size() / 20U);
    EXPECT_EQ(previews.units[index].ros_time_ns, rosNow());
  }
}

int main(int argc, char** argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
