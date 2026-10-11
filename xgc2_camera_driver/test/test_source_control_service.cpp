#include "source_control_service.hpp"

#include <gtest/gtest.h>

#include <sys/stat.h>
#include <unistd.h>

#include <algorithm>
#include <atomic>
#include <chrono>
#include <cstring>
#include <filesystem>
#include <future>
#include <string>
#include <thread>
#include <vector>

#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>

#include <xgc2/xrpc/json_http.hpp>

#include "h264_test_support.hpp"
#include "media_test_support.hpp"

namespace driver = xgc2_camera_driver;
namespace xrpc = xgc2::xrpc;
using namespace std::chrono_literals;
using Json = xrpc::Json;
using media_test::RtpCollector;

namespace {

std::int64_t rosNow()
{
  return 1700000000000000000LL;
}

struct Reply {
  int status{0};
  std::string body;
  xrpc::Headers headers;

  Json json() const
  {
    Json value;
    EXPECT_TRUE(xrpc::parse_json(body, value, xrpc::JsonLimits{1U << 20U, 32U})) << body;
    return value;
  }

  std::string errorCode() const { return json().at("error").at("code").get<std::string>(); }
  std::string errorMessage() const { return json().at("error").at("message").get<std::string>(); }

  std::string header(const std::string& name) const
  {
    for (const auto& item : headers) {
      if (item.first == name) {
        return item.second;
      }
    }
    return "";
  }
};

class Client {
 public:
  Client(const std::string& socket, const std::string& instance)
      : client_(socket, limits(), instance)
  {
  }

  Reply call(
      const std::string& method,
      const std::string& target,
      const std::string& body = "",
      const std::chrono::milliseconds timeout = 10s)
  {
    xrpc::HttpRequest request;
    request.method = method;
    request.target = target;
    request.body = body;
    if (!body.empty()) {
      request.headers.emplace_back("Content-Type", "application/json");
    }
    const xrpc::HttpResponse response = client_.call(request, xrpc::Clock::now() + timeout);
    return Reply{response.status, response.body, response.headers};
  }

  Reply get(const std::string& target) { return call("GET", target); }
  Reply post(const std::string& target, const std::string& body = "")
  {
    return call("POST", target, body);
  }

 private:
  static xrpc::HttpLimits limits()
  {
    xrpc::HttpLimits value;
    value.response_bytes = 160U << 20U;
    return value;
  }

  xrpc::HttpClient client_;
};

struct Part {
  std::string name;
  std::string type;
  std::string data;
};

// Parses a multipart/mixed body the way media-edge's strict reader expects it.
std::vector<Part> parseMultipart(const Reply& reply)
{
  const std::string type = reply.header("Content-Type");
  const std::string key = "multipart/mixed; boundary=";
  EXPECT_EQ(type.compare(0, key.size(), key), 0) << type;
  const std::string boundary = "--" + type.substr(key.size());
  EXPECT_LE(boundary.size() - 2U, 70U);
  std::vector<Part> parts;
  std::size_t at = reply.body.find(boundary);
  while (at != std::string::npos) {
    at += boundary.size();
    if (reply.body.compare(at, 2, "--") == 0) {
      EXPECT_EQ(reply.body.substr(at), "--\r\n");
      return parts;
    }
    EXPECT_EQ(reply.body.compare(at, 2, "\r\n"), 0);
    at += 2;
    const std::size_t header_end = reply.body.find("\r\n\r\n", at);
    EXPECT_NE(header_end, std::string::npos);
    const std::string headers = reply.body.substr(at, header_end - at);
    Part part;
    const std::string type_key = "Content-Type: ";
    const std::size_t type_at = headers.find(type_key);
    EXPECT_EQ(type_at, 0U);
    part.type = headers.substr(type_key.size(), headers.find("\r\n") - type_key.size());
    const std::string name_key = "Content-Disposition: inline; name=\"";
    const std::size_t name_at = headers.find(name_key);
    EXPECT_NE(name_at, std::string::npos);
    part.name = headers.substr(
        name_at + name_key.size(), headers.find('"', name_at + name_key.size()) - name_at - name_key.size());
    const std::size_t data_begin = header_end + 4U;
    const std::size_t next = reply.body.find("\r\n" + boundary, data_begin);
    EXPECT_NE(next, std::string::npos);
    part.data = reply.body.substr(data_begin, next - data_begin);
    parts.push_back(std::move(part));
    at = next + 2U;
  }
  return parts;
}

driver::SourceInfo sourceInfo(const std::string& kind = "synthetic")
{
  driver::SourceInfo info;
  info.source_kind = kind;
  info.frame_id = "test_optical_frame";
  info.width = 320;
  info.height = 240;
  info.fps = 30.0;
  info.clock_domain = "system_realtime";
  return info;
}

// Everything a camera process wires together, with a test camera feeding it.
class Fixture {
 public:
  explicit Fixture(const std::string& kind = "synthetic", const bool open = true)
      : state_(sourceInfo(kind)),
        pipeline_(
            [&] {
              auto config = media_test::testConfig(collector_.port());
              config.preview_enabled = false;
              return config;
            }(),
            [](driver::PreviewUnit&&) {},
            rosNow,
            media_test::silentLogger())
  {
    char pattern[] = "/tmp/xgc2_camera_service_XXXXXX";
    directory_ = ::mkdtemp(pattern);
    socket_ = directory_ + "/camera.sock";
    driver::MediaSource source;
    source.width = 320;
    source.height = 240;
    source.fps = 30.0;
    source.still = kind == "still";
    pipeline_.setSource(source);
    driver::SourceControlService::Options options;
    options.socket_path = socket_;
    options.target_id = "test-target";
    options.source_id = "front";
    options.camera_name = "usb_front";
    service_.reset(new driver::SourceControlService(options, &pipeline_, &state_));
    if (open) {
      openSource(kind);
    }
  }

  ~Fixture()
  {
    feeder_.reset();
    stop_publishing_ = true;
    if (publisher_.joinable()) {
      publisher_.join();
    }
    service_.reset();
    std::filesystem::remove_all(directory_);
  }

  void openSource(const std::string& kind = "synthetic")
  {
    state_.sourceOpened(sourceInfo(kind));
    if (kind == "still") {
      pipeline_.setStill(media_test::makeJpeg(320, 240, 2), 320, 240);
      publishFrame();
      return;
    }
    publishFrame();
    feeder_.reset(new media_test::Feeder(&pipeline_, 320, 240, 30.0));
    publisher_ = std::thread([this] {
      while (!stop_publishing_) {
        publishFrame();
        std::this_thread::sleep_for(33ms);
      }
    });
  }

  void publishFrame()
  {
    auto frame = std::make_shared<driver::CaptureFrame>();
    const std::uint64_t index = state_.frameCount() + 1U;
    frame->jpeg = media_test::makeJpeg(320, 240, static_cast<int>(index % 8U));
    frame->sequence = index;
    frame->timestamp_ns = 1700000000000000000LL + static_cast<std::int64_t>(index) * 33000000LL;
    frame->width = 320;
    frame->height = 240;
    state_.framePublished(frame);
  }

  Client discover()
  {
    Client plain(socket_, "");
    const Reply description = plain.get("/v1/describe");
    EXPECT_EQ(description.status, 200);
    return Client(socket_, description.json().at("instance_id").get<std::string>());
  }

  std::string socket() const { return socket_; }
  driver::SourceState& state() { return state_; }
  driver::MediaPipeline& pipeline() { return pipeline_; }
  RtpCollector& collector() { return collector_; }

 private:
  RtpCollector collector_;
  driver::SourceState state_;
  driver::MediaPipeline pipeline_;
  std::string directory_;
  std::string socket_;
  std::unique_ptr<driver::SourceControlService> service_;
  std::unique_ptr<media_test::Feeder> feeder_;
  std::atomic<bool> stop_publishing_{false};
  std::thread publisher_;
};

const std::string kSource = "/v1/media/sources/front";

}  // namespace

TEST(SourceControlService, DiscoveryIsTheOnlyUnfencedRoute)
{
  Fixture fixture("synthetic", false);
  Client plain(fixture.socket(), "");
  const Reply description = plain.get("/v1/describe");
  ASSERT_EQ(description.status, 200);
  const Json body = description.json();
  EXPECT_EQ(body.at("service"), "camera-source");
  EXPECT_EQ(body.at("api_version"), "v1");
  EXPECT_EQ(body.at("ready"), false);
  EXPECT_FALSE(body.at("instance_id").get<std::string>().empty());
  EXPECT_EQ(body.size(), 5U);
  const Json ref = body.at("facts").at("service_ref");
  EXPECT_EQ(ref.at("target_id"), "test-target");
  EXPECT_EQ(ref.at("service"), "camera-source");
  EXPECT_EQ(ref.at("api_version"), "v1");
  EXPECT_EQ(ref.at("profile"), "http.v1");
  EXPECT_EQ(ref.at("instance_id"), body.at("instance_id"));
  EXPECT_EQ(ref.at("endpoint").at("kind"), "unix");
  EXPECT_EQ(ref.at("endpoint").at("address"), fixture.socket());
  EXPECT_EQ(body.at("facts").at("sources"), Json::array({"front"}));
  EXPECT_FALSE(body.at("facts").at("source_open").get<bool>());
  EXPECT_FALSE(body.at("facts").at("reason").get<std::string>().empty());

  // Every other route needs the instance fence.
  EXPECT_EQ(plain.get(kSource + "/status").status, 409);
  // A pinned instance that is not the live one: the service answers 409 with its
  // own identity, which the client refuses to accept as the pinned owner.
  Client stale(fixture.socket(), "0123456789abcdef0123456789abcdef");
  EXPECT_THROW(stale.get(kSource + "/status"), xrpc::HttpCallError);
  // Discovery matches the path, including readiness queries without a fence.
  EXPECT_EQ(plain.get("/v1/describe?wait_ready_ms=0").status, 200);
  Client fenced = fixture.discover();
  EXPECT_EQ(fenced.get(kSource + "/status").status, 200);
}

TEST(SourceControlService, ReadinessIsHeldWithoutPollingAndNeverFaked)
{
  Fixture fixture("synthetic", false);
  Client client(fixture.socket(), "");

  // A bounded wait on a source that never opens ends with the current answer.
  const auto before = std::chrono::steady_clock::now();
  Reply unready = client.get("/v1/describe?wait_ready_ms=300");
  ASSERT_EQ(unready.status, 200);
  EXPECT_EQ(unready.json().at("ready"), false);
  EXPECT_GE(std::chrono::steady_clock::now() - before, 280ms);

  // The same call is answered the moment the first picture exists.
  auto held = std::async(std::launch::async, [&] {
    return client.get("/v1/describe?wait_ready_ms=5000");
  });
  std::this_thread::sleep_for(300ms);
  EXPECT_EQ(held.wait_for(50ms), std::future_status::timeout);
  const auto opened = std::chrono::steady_clock::now();
  fixture.openSource("synthetic");
  const Reply ready = held.get();
  ASSERT_EQ(ready.status, 200);
  EXPECT_EQ(ready.json().at("ready"), true);
  EXPECT_LT(std::chrono::steady_clock::now() - opened, 1000ms);
  EXPECT_TRUE(ready.json().at("facts").at("frames_flowing").get<bool>());
  EXPECT_EQ(ready.json().at("facts").at("width"), 320);
  EXPECT_FALSE(ready.json().at("facts").contains("reason"));

  // Losing the source makes the service not ready again, with the reason.
  fixture.state().sourceClosed("camera unplugged");
  const Json lost = client.get("/v1/describe").json();
  EXPECT_EQ(lost.at("ready"), false);
  EXPECT_EQ(lost.at("facts").at("reason"), "camera unplugged");

  for (const char* bad : {"abc", "-1", "05", "31000", "", "1.5", "wait_ready_ms=1&x=2"}) {
    const std::string target = std::string("/v1/describe?wait_ready_ms=") + bad;
    EXPECT_EQ(client.get(target).status, 400) << target;
  }
  EXPECT_EQ(client.get("/v1/describe?other=1").status, 400);
}

TEST(SourceControlService, ReadinessWaitCanOutlastFifteenSeconds)
{
  Fixture fixture("synthetic", false);
  Client client(fixture.socket(), "");
  const auto before = std::chrono::steady_clock::now();
  const Reply reply = client.call("GET", "/v1/describe?wait_ready_ms=15100", "", 17s);
  ASSERT_EQ(reply.status, 200);
  EXPECT_FALSE(reply.json().at("ready").get<bool>());
  EXPECT_GE(std::chrono::steady_clock::now() - before, 15000ms);
}

TEST(SourceControlService, SourceDescriptorMatchesWhatMediaEdgeValidates)
{
  Fixture fixture;
  Client client = fixture.discover();
  const Reply reply = client.get(kSource + "/describe");
  ASSERT_EQ(reply.status, 200);
  const Json body = reply.json();
  EXPECT_EQ(body.at("ok"), true);
  EXPECT_EQ(body.at("protocolVersion"), 1);
  EXPECT_EQ(body.at("sourceId"), "front");
  EXPECT_EQ(body.at("codec"), "H264");
  EXPECT_EQ(body.at("rtpPayloadType"), 96);
  EXPECT_EQ(body.at("rtpClockRate"), 90000);
  EXPECT_EQ(body.at("rtpHost"), "127.0.0.1");
  EXPECT_EQ(body.at("rtpPort"), fixture.collector().port());
  EXPECT_EQ(body.at("width"), 320);
  EXPECT_EQ(body.at("height"), 240);
  EXPECT_EQ(body.at("fps"), 30.0);
  EXPECT_EQ(body.at("frameId"), "test_optical_frame");
  EXPECT_EQ(body.at("keyframeRequestSupported"), true);
  EXPECT_FALSE(body.at("keyframePolicy").get<std::string>().empty());
  const auto capabilities = body.at("capabilities").get<std::vector<std::string>>();
  for (const char* required : {"start", "stop", "capture", "fresh-snapshot", "request-keyframe"}) {
    EXPECT_NE(std::find(capabilities.begin(), capabilities.end(), required), capabilities.end())
        << required;
  }
}

TEST(SourceControlService, LeaseStartsAndStopsTheStreamWithAppliedReceipts)
{
  Fixture closed("synthetic", false);
  Client early = closed.discover();
  const Reply refused = early.post(kSource + "/start", "{}");
  EXPECT_EQ(refused.status, 503) << "a source that is not ready cannot start";
  EXPECT_EQ(refused.errorCode(), "unavailable");

  Fixture fixture;
  Client client = fixture.discover();
  EXPECT_EQ(client.get(kSource + "/status").json().at("state"), "idle");
  EXPECT_EQ(fixture.collector().datagrams(), 0U);

  const Reply started = client.post(kSource + "/start", "{}");
  ASSERT_EQ(started.status, 200) << started.body;
  const Json receipt = started.json();
  EXPECT_EQ(receipt.at("ok"), true);
  EXPECT_EQ(receipt.at("source_id"), "front");
  EXPECT_EQ(receipt.at("state"), "active");
  EXPECT_EQ(receipt.at("active"), true);
  EXPECT_EQ(receipt.at("completion"), "applied");
  EXPECT_GE(receipt.at("configuration_revision").get<std::uint64_t>(), 1U);
  ASSERT_TRUE(fixture.collector().waitForUnits(10, 5s));
  EXPECT_TRUE(h264_test::isIdr(fixture.collector().units()[0].data));

  const Json status = client.get(kSource + "/status").json();
  EXPECT_EQ(status.at("ok"), true);
  EXPECT_EQ(status.at("source_id"), "front");
  EXPECT_EQ(status.at("state"), "active");
  EXPECT_EQ(status.at("desired_active"), true);
  EXPECT_EQ(status.at("applied_active"), true);
  EXPECT_TRUE(status.at("persisted_revision").is_null());
  EXPECT_EQ(status.at("desired_revision"), status.at("applied_revision"));
  EXPECT_EQ(status.at("native").at("encoder"), "libx264");
  EXPECT_GT(status.at("native").at("access_units").get<int>(), 0);

  // Idempotent start, and a keyframe on request.
  EXPECT_EQ(client.post(kSource + "/start", "{}").status, 200);
  const std::size_t mark = fixture.collector().count();
  const Reply keyframe = client.post(kSource + "/request-keyframe", "{}");
  ASSERT_EQ(keyframe.status, 200) << keyframe.body;
  EXPECT_EQ(keyframe.json().at("ok"), true);
  ASSERT_TRUE(fixture.collector().waitForUnits(mark + 3, 5s));
  bool idr = false;
  const auto units = fixture.collector().units();
  for (std::size_t index = mark; index < units.size() && index < mark + 3U; ++index) {
    idr = idr || h264_test::isIdr(units[index].data);
  }
  EXPECT_TRUE(idr);

  const Reply stopped = client.post(kSource + "/stop", "{}");
  ASSERT_EQ(stopped.status, 200) << stopped.body;
  EXPECT_EQ(stopped.json().at("state"), "idle");
  EXPECT_EQ(stopped.json().at("active"), false);
  EXPECT_EQ(stopped.json().at("completion"), "applied");
  const Json after = client.get(kSource + "/status").json();
  EXPECT_EQ(after.at("state"), "idle");
  EXPECT_EQ(after.at("applied_active"), false);
  std::this_thread::sleep_for(150ms);
  const std::size_t quiet = fixture.collector().count();
  std::this_thread::sleep_for(300ms);
  EXPECT_EQ(fixture.collector().count(), quiet);
  EXPECT_EQ(client.post(kSource + "/stop", "{}").status, 200) << "stop is idempotent";

  const Reply idle_keyframe = client.post(kSource + "/request-keyframe", "{}");
  EXPECT_EQ(idle_keyframe.status, 409);
  EXPECT_EQ(idle_keyframe.errorCode(), "conflict");
}

TEST(SourceControlService, CaptureReturnsTheLatestJpegAndExactRgbOfTheSameFrame)
{
  Fixture fixture;
  Client client = fixture.discover();
  const Reply reply =
      client.post(kSource + "/capture", R"({"snapshotId":"snap-1","includeRgb":false})");
  ASSERT_EQ(reply.status, 200) << reply.body.substr(0, 200);
  const std::vector<Part> parts = parseMultipart(reply);
  ASSERT_EQ(parts.size(), 2U);
  EXPECT_EQ(parts[0].name, "metadata");
  EXPECT_EQ(parts[0].type, "application/json");
  EXPECT_EQ(parts[1].name, "jpeg");
  EXPECT_EQ(parts[1].type, "image/jpeg");
  Json metadata;
  ASSERT_TRUE(xrpc::parse_json(parts[0].data, metadata));
  EXPECT_EQ(metadata.at("ok"), true);
  EXPECT_EQ(metadata.at("sourceId"), "front");
  EXPECT_EQ(metadata.at("snapshotId"), "snap-1");
  EXPECT_EQ(metadata.at("frameId"), "test_optical_frame");
  EXPECT_GT(metadata.at("frameSequence").get<std::uint64_t>(), 0U);
  EXPECT_GT(metadata.at("timestampNanoseconds").get<std::int64_t>(), 0);
  EXPECT_EQ(metadata.at("timestampClockDomain"), "system_realtime");
  EXPECT_EQ(metadata.at("width"), 320);
  EXPECT_EQ(metadata.at("height"), 240);
  EXPECT_EQ(metadata.at("pixelFormat"), "rgb8");
  EXPECT_EQ(metadata.at("jpegBytes").get<std::size_t>(), parts[1].data.size());
  EXPECT_EQ(metadata.at("rgbBytes"), 0);
  EXPECT_EQ(metadata.at("calibrationState"), "unavailable");
  EXPECT_FALSE(metadata.contains("cameraMatrix"));
  const cv::Mat decoded = cv::imdecode(
      std::vector<std::uint8_t>(parts[1].data.begin(), parts[1].data.end()), cv::IMREAD_COLOR);
  ASSERT_FALSE(decoded.empty());
  EXPECT_EQ(decoded.cols, 320);

  // With RGB: the third part is the exact decoded pixels of that very JPEG.
  const Reply with_rgb = client.post(kSource + "/capture", R"({"snapshotId":"snap-2"})");
  ASSERT_EQ(with_rgb.status, 200);
  const std::vector<Part> three = parseMultipart(with_rgb);
  ASSERT_EQ(three.size(), 3U);
  EXPECT_EQ(three[2].name, "rgb");
  EXPECT_EQ(three[2].type, "application/octet-stream");
  Json rich;
  ASSERT_TRUE(xrpc::parse_json(three[0].data, rich));
  EXPECT_EQ(rich.at("rgbBytes"), 320 * 240 * 3);
  ASSERT_EQ(three[2].data.size(), static_cast<std::size_t>(320 * 240 * 3));
  cv::Mat bgr = cv::imdecode(
      std::vector<std::uint8_t>(three[1].data.begin(), three[1].data.end()), cv::IMREAD_COLOR);
  cv::Mat rgb;
  cv::cvtColor(bgr, rgb, cv::COLOR_BGR2RGB);
  EXPECT_EQ(std::memcmp(rgb.data, three[2].data.data(), three[2].data.size()), 0);
}

TEST(SourceControlService, CaptureCarriesCalibrationOnlyWhenItIsReal)
{
  Fixture fixture("synthetic", false);
  driver::SourceInfo info = sourceInfo();
  info.calibrated = true;
  info.camera_matrix = {{50.0, 0.0, 159.5, 0.0, 51.0, 119.5, 0.0, 0.0, 1.0}};
  info.distortion = {0.1, -0.05, 0.001, 0.002, 0.0};
  fixture.state().sourceOpened(info);
  fixture.publishFrame();
  Client client = fixture.discover();
  const Reply reply = client.post(kSource + "/capture", R"({"snapshotId":"s","includeRgb":false})");
  ASSERT_EQ(reply.status, 200);
  Json metadata;
  ASSERT_TRUE(xrpc::parse_json(parseMultipart(reply)[0].data, metadata));
  EXPECT_EQ(metadata.at("calibrationState"), "available");
  EXPECT_EQ(metadata.at("cameraMatrix").size(), 9U);
  EXPECT_EQ(metadata.at("cameraMatrix")[2], 159.5);
  EXPECT_EQ(metadata.at("distortion").size(), 5U);
}

TEST(SourceControlService, FreshCaptureWaitsForAPictureAfterAdmission)
{
  Fixture fixture("synthetic", false);
  fixture.state().sourceOpened(sourceInfo());
  fixture.publishFrame();
  Client client = fixture.discover();
  const std::uint64_t latest = fixture.state().frameCount();

  auto fresh = std::async(std::launch::async, [&] {
    return client.post(kSource + "/capture", R"({"snapshotId":"f","includeRgb":false,"requireFresh":true})");
  });
  EXPECT_EQ(fresh.wait_for(200ms), std::future_status::timeout) << "no new picture yet";
  fixture.publishFrame();
  const Reply reply = fresh.get();
  ASSERT_EQ(reply.status, 200) << reply.body.substr(0, 200);
  Json metadata;
  ASSERT_TRUE(xrpc::parse_json(parseMultipart(reply)[0].data, metadata));
  EXPECT_GT(metadata.at("frameSequence").get<std::uint64_t>(), latest);

  // Without any new picture the call ends with a deadline error, not a stale frame.
  const Reply timeout = client.post(
      kSource + "/capture", R"({"snapshotId":"g","includeRgb":false,"requireFresh":true})");
  EXPECT_EQ(timeout.status, 504);
  EXPECT_EQ(timeout.errorCode(), "deadline_exceeded");
}

TEST(SourceControlService, OneCaptureRunsAtATime)
{
  Fixture fixture("synthetic", false);
  fixture.state().sourceOpened(sourceInfo());
  fixture.publishFrame();
  Client first = fixture.discover();
  Client second = fixture.discover();
  auto waiting = std::async(std::launch::async, [&] {
    return first.post(kSource + "/capture", R"({"snapshotId":"a","includeRgb":false,"requireFresh":true})");
  });
  std::this_thread::sleep_for(150ms);
  const Reply rejected = second.post(kSource + "/capture", R"({"snapshotId":"b","includeRgb":false})");
  EXPECT_EQ(rejected.status, 429);
  EXPECT_EQ(rejected.errorCode(), "resource_exhausted");
  fixture.publishFrame();
  EXPECT_EQ(waiting.get().status, 200);
  // The slot is free again.
  EXPECT_EQ(second.post(kSource + "/capture", R"({"snapshotId":"c","includeRgb":false})").status, 200);
}

TEST(SourceControlService, CaptureRequestsAreStrict)
{
  Fixture fixture;
  Client client = fixture.discover();
  for (const char* bad : {
           R"({"unknown":1})",
           R"({"includeRgb":"yes"})",
           R"({"requireFresh":1})",
           R"({"requestKeyframe":null})",
           R"({"snapshotId":5})",
           R"({"snapshotId":"has space"})",
           R"([])",
           "{not json",
       }) {
    EXPECT_EQ(client.post(kSource + "/capture", bad).status, 400) << bad;
  }
  Fixture unopened("synthetic", false);
  Client early = unopened.discover();
  const Reply none = early.post(kSource + "/capture", R"({"snapshotId":"n"})");
  EXPECT_EQ(none.status, 503);
  EXPECT_EQ(none.errorCode(), "unavailable");
}

TEST(SourceControlService, CaptureCanForceAKeyframeOnlyWhileStreaming)
{
  Fixture fixture;
  Client client = fixture.discover();
  const Reply idle = client.post(
      kSource + "/capture", R"({"snapshotId":"k","includeRgb":false,"requestKeyframe":true})");
  EXPECT_EQ(idle.status, 409) << "a forced IDR needs an active stream";

  ASSERT_EQ(client.post(kSource + "/start", "{}").status, 200);
  ASSERT_TRUE(fixture.collector().waitForUnits(10, 5s));
  const std::size_t mark = fixture.collector().count();
  const Reply active = client.post(
      kSource + "/capture", R"({"snapshotId":"k","includeRgb":false,"requestKeyframe":true})");
  ASSERT_EQ(active.status, 200) << active.body.substr(0, 200);
  ASSERT_TRUE(fixture.collector().waitForUnits(mark + 2, 5s));
  bool idr = false;
  const auto units = fixture.collector().units();
  for (std::size_t index = mark; index < units.size() && index < mark + 3U; ++index) {
    idr = idr || h264_test::isIdr(units[index].data);
  }
  EXPECT_TRUE(idr);
}

TEST(SourceControlService, ConfigurationIsRevisionedEphemeralAndAppliedOnlyWhileIdle)
{
  Fixture fixture;
  Client client = fixture.discover();
  const Json initial = client.get(kSource + "/config").json();
  EXPECT_EQ(initial.at("ok"), true);
  EXPECT_EQ(initial.at("persistence"), "ephemeral");
  EXPECT_TRUE(initial.at("persisted_revision").is_null());
  EXPECT_EQ(initial.at("desired"), initial.at("applied"));
  EXPECT_EQ(initial.at("desired_revision"), initial.at("applied_revision"));
  EXPECT_EQ(initial.at("desired").at("rtp_host"), "127.0.0.1");
  EXPECT_EQ(initial.at("mutable_fields"), Json::array({"rtp_host", "rtp_port", "bitrate"}));
  const std::uint64_t revision = initial.at("desired_revision").get<std::uint64_t>();

  auto patch = [&](const std::string& body) {
    return client.call("PATCH", kSource + "/config", body);
  };
  const std::string next = std::to_string(revision);
  EXPECT_EQ(patch(R"({"expected_revision":)" + std::to_string(revision + 5) +
                  R"(,"persist":false,"config":{"bitrate":2000000}})").status, 409);
  EXPECT_EQ(patch(R"({"expected_revision":)" + next + R"(,"persist":true,"config":{"bitrate":2000000}})").status, 400);
  EXPECT_EQ(patch(R"({"expected_revision":)" + next + R"(,"persist":false,"config":{"width":640}})").status, 400);
  EXPECT_EQ(patch(R"({"expected_revision":)" + next + R"(,"persist":false,"config":{"rtp_host":"192.0.2.1"}})").status, 400);
  EXPECT_EQ(patch(R"({"expected_revision":)" + next + R"(,"persist":false,"config":{"rtp_port":70000}})").status, 400);
  EXPECT_EQ(patch(R"({"expected_revision":)" + next + R"(,"persist":false,"config":{"bitrate":"fast"}})").status, 400);
  EXPECT_EQ(patch(R"({"expected_revision":)" + next + R"(,"persist":false,"config":{}})").status, 400);
  EXPECT_EQ(patch(R"({"persist":false,"config":{"bitrate":2000000}})").status, 400);

  const Reply applied = patch(
      R"({"expected_revision":)" + next + R"(,"persist":false,"config":{"bitrate":2000000}})");
  ASSERT_EQ(applied.status, 200) << applied.body;
  EXPECT_EQ(applied.json().at("desired").at("bitrate"), 2000000);
  EXPECT_EQ(applied.json().at("applied_revision").get<std::uint64_t>(), revision + 1U);
  EXPECT_EQ(fixture.pipeline().bitrate(), 2000000U);
  // The old revision is now stale.
  EXPECT_EQ(patch(R"({"expected_revision":)" + next + R"(,"persist":false,"config":{"bitrate":3000000}})").status, 409);

  // A running stream keeps its destination and bitrate: restart required.
  ASSERT_EQ(client.post(kSource + "/start", "{}").status, 200);
  const Reply busy = patch(
      R"({"expected_revision":)" + std::to_string(revision + 1U) +
      R"(,"persist":false,"config":{"bitrate":3000000}})");
  EXPECT_EQ(busy.status, 409);
  EXPECT_NE(busy.errorMessage().find("restart_required"), std::string::npos);
  EXPECT_EQ(fixture.pipeline().bitrate(), 2000000U);
}

TEST(SourceControlService, DescribePublishesDistinctSourceAndCalibrationIdentities)
{
  Fixture fixture;
  Client client = fixture.discover();
  const Json description = client.get("/v1/describe").json();
  const Json& facts = description.at("facts");
  EXPECT_EQ(facts.at("source_id"), "front");
  EXPECT_EQ(facts.at("sourceId"), "front");
  EXPECT_EQ(facts.at("cameraName"), "usb_front");
}

TEST(SourceControlService, UnknownRoutesMethodsAndSourcesAreRefused)
{
  Fixture fixture;
  Client client = fixture.discover();
  EXPECT_EQ(client.get("/v1/other").status, 404);
  EXPECT_EQ(client.get("/v1/media/sources/front").status, 404);
  EXPECT_EQ(client.get("/v1/media/sources/other/status").status, 404);
  EXPECT_EQ(client.get("/v1/media/sources/front/unknown").status, 404);
  EXPECT_EQ(client.post(kSource + "/status").status, 405);
  EXPECT_EQ(client.get(kSource + "/start").status, 405);
  EXPECT_EQ(client.call("DELETE", kSource + "/config").status, 405);
  EXPECT_EQ(client.post(kSource + "/start", R"({"extra":true})").status, 400);
  EXPECT_EQ(client.get(kSource + "/status?x=1").status, 400);
  const Json roster = client.get("/v1/media/sources").json();
  EXPECT_EQ(roster.at("sources").size(), 1U);
  EXPECT_EQ(roster.at("sources")[0].at("source_id"), "front");
  EXPECT_EQ(roster.at("sources")[0].at("ready"), true);
}

TEST(SourceControlService, AStillAnswersFreshCapturesAtOnce)
{
  Fixture fixture("still");
  Client client = fixture.discover();
  const auto started = std::chrono::steady_clock::now();
  const Reply reply = client.post(
      kSource + "/capture", R"({"snapshotId":"s","includeRgb":false,"requireFresh":true})");
  ASSERT_EQ(reply.status, 200);
  EXPECT_LT(std::chrono::steady_clock::now() - started, 400ms);
  EXPECT_EQ(client.get(kSource + "/describe").json().at("sourceKind"), "still");
  ASSERT_EQ(client.post(kSource + "/start", "{}").status, 200);
  ASSERT_TRUE(fixture.collector().waitForUnits(2, 5s));
  EXPECT_TRUE(h264_test::isIdr(fixture.collector().units()[0].data));
}

TEST(SourceControlService, ASecondInstanceCannotTakeTheSocketAndAStaleOneIsReclaimed)
{
  char pattern[] = "/tmp/xgc2_camera_service_XXXXXX";
  const std::string directory = ::mkdtemp(pattern);
  const std::string socket = directory + "/camera.sock";
  RtpCollector collector;
  driver::SourceState state(sourceInfo());
  driver::MediaPipeline pipeline(
      media_test::testConfig(collector.port()), [](driver::PreviewUnit&&) {}, rosNow,
      media_test::silentLogger());
  driver::SourceControlService::Options options;
  options.socket_path = socket;
  options.target_id = "t";
  options.source_id = "front";
  {
    driver::SourceControlService first(options, &pipeline, &state);
    EXPECT_ANY_THROW(driver::SourceControlService(options, &pipeline, &state));
    Client first_client(socket, "");
    EXPECT_EQ(first_client.get("/v1/describe").status, 200);
  }
  // The first instance is gone; its socket path is free for a new incarnation
  // that is distinguishable by its instance id.
  driver::SourceControlService second(options, &pipeline, &state);
  Client client(socket, "");
  EXPECT_EQ(client.get("/v1/describe").status, 200);
  std::filesystem::remove_all(directory);
}

int main(int argc, char** argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
