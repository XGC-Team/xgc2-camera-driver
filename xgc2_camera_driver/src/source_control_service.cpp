#include "source_control_service.hpp"

#include <arpa/inet.h>
#include <unistd.h>

#include <algorithm>
#include <cctype>
#include <cstdio>
#include <random>
#include <sstream>

#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>

namespace xgc2_camera_driver {
namespace {

using xgc2::xrpc::HttpReply;
using xgc2::xrpc::HttpRequest;
using xgc2::xrpc::HttpResponse;
using xgc2::xrpc::Json;

constexpr const char* kService = "camera-source";
constexpr const char* kApiVersion = "v1";
// The contract bounds a capture to a 32 MiB JPEG and 128 MiB of RGB.
constexpr std::size_t kResponseLimit = 160U << 20U;
constexpr std::size_t kJsonLimit = 64U << 10U;
constexpr auto kPollInterval = std::chrono::milliseconds(100);
constexpr int kMaximumWaitMilliseconds = 30000;

std::vector<std::string> split(const std::string& text, const char separator)
{
  std::vector<std::string> parts;
  std::size_t begin = 0;
  while (begin <= text.size()) {
    const std::size_t end = text.find(separator, begin);
    parts.push_back(text.substr(begin, end == std::string::npos ? end : end - begin));
    if (end == std::string::npos) {
      break;
    }
    begin = end + 1U;
  }
  return parts;
}

bool safeIdentifier(const std::string& value)
{
  return !value.empty() && value.size() <= 128U &&
         std::all_of(value.begin(), value.end(), [](const unsigned char c) {
           return std::isalnum(c) != 0 || c == '-' || c == '_' || c == '.' || c == ':';
         });
}

bool loopbackHost(const std::string& host)
{
  if (host == "localhost") {
    return true;
  }
  in_addr address{};
  return inet_pton(AF_INET, host.c_str(), &address) == 1 &&
         (ntohl(address.s_addr) >> 24U) == 127U;
}

// Strict boolean/integer/string field readers; an object with any field outside
// `allowed` is rejected, as the contract requires for every request body.
void requireFields(const Json& body, std::initializer_list<const char*> allowed)
{
  if (!body.is_object()) {
    throw std::invalid_argument("the request body must be a JSON object");
  }
  for (auto item = body.begin(); item != body.end(); ++item) {
    if (std::none_of(allowed.begin(), allowed.end(), [&](const char* name) {
          return item.key() == name;
        })) {
      throw std::invalid_argument("unknown field: " + item.key());
    }
  }
}

bool optionalBool(const Json& body, const char* name, const bool fallback)
{
  const auto found = body.find(name);
  if (found == body.end()) {
    return fallback;
  }
  if (!found->is_boolean()) {
    throw std::invalid_argument(std::string(name) + " must be a boolean");
  }
  return found->get<bool>();
}

std::string boundary()
{
  static std::random_device device;
  char text[40];
  std::snprintf(text, sizeof(text), "xgc2-%08x%08x%08x", device(), device(), device());
  return text;
}

void appendPart(
    std::string* out,
    const std::string& boundary_text,
    const char* content_type,
    const char* name,
    const char* data,
    const std::size_t size)
{
  out->append("--").append(boundary_text).append("\r\n");
  out->append("Content-Type: ").append(content_type).append("\r\n");
  out->append("Content-Disposition: inline; name=\"").append(name).append("\"\r\n\r\n");
  out->append(data, size);
  out->append("\r\n");
}

}  // namespace

SourceControlService::SourceControlService(
    Options options, MediaPipeline* pipeline, SourceState* state)
    : options_(std::move(options)),
      instance_id_(xgc2::xrpc::new_instance_id()),
      pipeline_(pipeline),
      state_(state)
{
  if (!pipeline_ || !state_ || !safeIdentifier(options_.source_id) ||
      !safeIdentifier(options_.target_id) || options_.socket_path.empty() ||
      options_.socket_path.front() != '/' || options_.socket_path.size() >= 108U) {
    throw std::invalid_argument(
        "the camera-source service needs an absolute socket path, a target id and a source id");
  }
  xgc2::xrpc::UnixOptions endpoint;
  endpoint.path = options_.socket_path;
  endpoint.mode = 0600;
  endpoint.existing = xgc2::xrpc::ExistingPath::ReclaimUnreachable;
  xgc2::xrpc::HttpLimits limits;
  limits.connections = 8;
  limits.inflight = 8;
  limits.header_bytes = 8192;
  limits.request_bytes = kJsonLimit;
  limits.response_bytes = kResponseLimit;
  limits.request_timeout = std::chrono::seconds(31);
  limits.shutdown_timeout = std::chrono::seconds(3);
  server_.reset(new xgc2::xrpc::HttpServer(
      endpoint,
      [this](HttpRequest request, HttpReply reply) { handle(std::move(request), std::move(reply)); },
      limits,
      xgc2::xrpc::HttpIdentity{instance_id_, {"/v1/describe"}}));
  server_->set_wakeup_handler([this] { processWaiters(); });
  state_->setChangeHandler([this] {
    if (waiting_.load(std::memory_order_acquire) != 0U) {
      server_->wake();
    }
  });
  capture_thread_ = std::thread(&SourceControlService::captureWorker, this);
  deadline_thread_ = std::thread(&SourceControlService::waitForDeadlines, this);
  service_thread_ = std::thread(&SourceControlService::serve, this);
}

SourceControlService::~SourceControlService()
{
  state_->setChangeHandler(nullptr);
  stopping_.store(true);
  {
    std::lock_guard<std::mutex> lock(work_mutex_);
  }
  work_ready_.notify_all();
  {
    std::lock_guard<std::mutex> lock(waiters_mutex_);
  }
  waiters_changed_.notify_all();
  server_->wake();
  service_thread_.join();
  capture_thread_.join();
  deadline_thread_.join();
  // Whoever still waits is told the camera is going away.
  {
    std::lock_guard<std::mutex> lock(waiters_mutex_);
    for (const auto& waiter : describe_waiters_) {
      fail(waiter.reply, 503, "unavailable", "the camera is shutting down");
    }
    for (const auto& waiter : fresh_captures_) {
      fail(waiter.reply, 503, "unavailable", "the camera is shutting down");
    }
    describe_waiters_.clear();
    fresh_captures_.clear();
  }
  server_->drain_until(Clock::now() + std::chrono::seconds(2));
}

void SourceControlService::serve()
{
  while (!stopping_.load()) {
    server_->poll(kPollInterval);
  }
}

void SourceControlService::waitForDeadlines()
{
  std::unique_lock<std::mutex> lock(waiters_mutex_);
  while (!stopping_.load()) {
    auto until = Clock::time_point::max();
    for (const auto& waiter : describe_waiters_) {
      until = std::min(until, waiter.until);
    }
    for (const auto& waiter : fresh_captures_) {
      until = std::min(until, waiter.until);
    }
    const auto revision = waiters_revision_;
    const auto changed = [this, revision] {
      return stopping_.load() || waiters_revision_ != revision;
    };
    if (until == Clock::time_point::max()) {
      waiters_changed_.wait(lock, changed);
    } else if (!waiters_changed_.wait_until(lock, until, changed)) {
      lock.unlock();
      server_->wake();
      lock.lock();
      // The owner removes expired calls before another deadline is selected.
      waiters_changed_.wait(lock, changed);
    }
  }
}

void SourceControlService::post(std::function<void()> work)
{
  {
    std::lock_guard<std::mutex> lock(work_mutex_);
    work_.push_back(std::move(work));
  }
  work_ready_.notify_one();
}

void SourceControlService::captureWorker()
{
  for (;;) {
    std::function<void()> work;
    {
      std::unique_lock<std::mutex> lock(work_mutex_);
      work_ready_.wait(lock, [this] { return stopping_.load() || !work_.empty(); });
      if (work_.empty()) {
        return;
      }
      work = std::move(work_.front());
      work_.pop_front();
    }
    work();
  }
}

void SourceControlService::respond(const HttpReply& reply, const int status, const Json& body) const
{
  reply.complete(xgc2::xrpc::json_response(body, kResponseLimit, status));
}

void SourceControlService::fail(
    const HttpReply& reply, const int status, const char* code, const std::string& message) const
{
  reply.complete(xgc2::xrpc::http_error(status, code, message));
}

void SourceControlService::handle(HttpRequest request, HttpReply reply)
{
  try {
    route(request, reply);
  } catch (const std::invalid_argument& error) {
    fail(reply, 400, "invalid_argument", error.what());
  } catch (const std::exception& error) {
    fail(reply, 500, "internal", error.what());
  }
  processWaiters();
}

void SourceControlService::route(const HttpRequest& request, const HttpReply& reply)
{
  const std::size_t query_at = request.target.find('?');
  const std::string path = request.target.substr(0, query_at);
  const std::string query =
      query_at == std::string::npos ? std::string() : request.target.substr(query_at + 1U);
  const bool get = request.method == "GET";
  const bool post_method = request.method == "POST";

  if (path == "/v1/describe") {
    if (!get) {
      return fail(reply, 405, "method_not_allowed", "describe is GET");
    }
    return describeService(query, request, reply);
  }
  if (!query.empty()) {
    return fail(reply, 400, "invalid_argument", "this route takes no query");
  }
  if (path == "/v1/media/sources") {
    if (!get) {
      return fail(reply, 405, "method_not_allowed", "the roster is GET");
    }
    Json roster = Json::array();
    roster.push_back({{"source_id", options_.source_id}, {"ready", state_->ready()}});
    return respond(reply, 200, {{"ok", true}, {"sources", roster}});
  }
  static const std::string prefix = "/v1/media/sources/";
  if (path.compare(0, prefix.size(), prefix) != 0) {
    return fail(reply, 404, "not_found", "camera route not found");
  }
  const std::vector<std::string> parts = split(path.substr(prefix.size()), '/');
  if (parts.size() != 2U || parts[1].empty()) {
    return fail(reply, 404, "not_found", "an explicit source route is required");
  }
  if (parts[0] != options_.source_id) {
    return fail(reply, 404, "not_found", "camera source not found");
  }
  const std::string& operation = parts[1];

  Json body = Json::object();
  if (!request.body.empty()) {
    if (!xgc2::xrpc::parse_json(request.body, body)) {
      return fail(reply, 400, "invalid_argument", "the request body is not strict JSON");
    }
  }
  if (operation == "describe" || operation == "status" || (operation == "config" && get)) {
    if (!get) {
      return fail(reply, 405, "method_not_allowed", operation + " is GET");
    }
    if (!request.body.empty()) {
      return fail(reply, 400, "invalid_argument", operation + " takes no body");
    }
    if (operation == "describe") {
      return describeSource(reply);
    }
    if (operation == "status") {
      return sourceStatus(reply);
    }
    return sourceConfig(reply);
  }
  if (operation == "config") {
    if (request.method != "PATCH") {
      return fail(reply, 405, "method_not_allowed", "config is GET or PATCH");
    }
    return patchConfig(body, reply);
  }
  if (operation == "start" || operation == "stop" || operation == "request-keyframe") {
    if (!post_method) {
      return fail(reply, 405, "method_not_allowed", operation + " is POST");
    }
    requireFields(body, {});
    return lifecycle(operation, reply);
  }
  if (operation == "capture") {
    if (!post_method) {
      return fail(reply, 405, "method_not_allowed", "capture is POST");
    }
    return capture(body, request, reply);
  }
  return fail(reply, 404, "not_found", "camera route not found");
}

Json SourceControlService::serviceRef() const
{
  return {
      {"target_id", options_.target_id},
      {"service", kService},
      {"api_version", kApiVersion},
      {"instance_id", instance_id_},
      {"profile", "http.v1"},
      {"endpoint", {{"kind", "unix"}, {"address", options_.socket_path}}},
  };
}

Json SourceControlService::factsJson() const
{
  const SourceInfo info = state_->info();
  const MediaPipeline::Status status = pipeline_->status();
  const auto age = state_->frameAge();
  Json facts = {
      {"source_kind", info.source_kind},
      {"source_id", options_.source_id},
      {"source_open", state_->open()},
      {"ros_master_uri", info.ros_master_uri},
      {"ros_master_run_id", info.ros_master_run_id},
      {"ros_master_pid", info.ros_master_pid},
      {"frames_flowing",
       state_->ready() && (info.source_kind == "still" ||
                          (age != std::chrono::milliseconds::max() &&
                           age < std::chrono::seconds(3)))},
      {"width", info.width},
      {"height", info.height},
      {"fps", info.fps},
      {"frame_id", info.frame_id},
      {"calibrated", info.calibrated},
      {"active", status.state == MediaPipeline::State::Active},
      {"rtp", {{"host", pipeline_->rtpHost()}, {"port", pipeline_->rtpPort()}}},
  };
  if (age != std::chrono::milliseconds::max()) {
    facts["last_frame_age_ms"] = age.count();
  }
  if (!status.encoder.empty()) {
    facts["encoder"] = status.encoder;
  }
  if (!status.encoder_fallback.empty()) {
    facts["encoder_fallback"] = status.encoder_fallback;
  }
  if (!state_->ready()) {
    facts["reason"] = state_->reason();
  }
  return facts;
}

Json SourceControlService::serviceDescription() const
{
  return {
      {"service", kService},
      {"api_version", kApiVersion},
      {"instance_id", instance_id_},
      {"ready", state_->ready()},
      {"facts", factsJson()},
      {"describe_path", "/v1/describe"},
      {"service_ref", serviceRef()},
      {"sources", Json::array({options_.source_id})},
      {"limits", {{"sources", 1}, {"concurrent_captures", 1}}},
  };
}

void SourceControlService::describeService(
    const std::string& query, const HttpRequest& request, const HttpReply& reply)
{
  int wait_ms = 0;
  if (!query.empty()) {
    const std::string key = "wait_ready_ms=";
    const std::string digits = query.compare(0, key.size(), key) == 0 ? query.substr(key.size()) : "";
    if (digits.empty() || digits.size() > 5U || (digits.size() > 1U && digits[0] == '0') ||
        digits.find_first_not_of("0123456789") != std::string::npos) {
      return fail(reply, 400, "invalid_argument", "describe takes only wait_ready_ms=<integer>");
    }
    wait_ms = std::stoi(digits);
    if (wait_ms > kMaximumWaitMilliseconds) {
      return fail(reply, 400, "invalid_argument", "wait_ready_ms exceeds 30000");
    }
  }
  if (wait_ms == 0 || state_->ready()) {
    return respond(reply, 200, serviceDescription());
  }
  // Hold the call without polling; the answer leaves when readiness changes or
  // the wait ends, always before the caller's own deadline.
  const auto now = Clock::now();
  const auto until = std::min(
      now + std::chrono::milliseconds(wait_ms), request.deadline - std::chrono::milliseconds(100));
  std::lock_guard<std::mutex> lock(waiters_mutex_);
  if (describe_waiters_.size() >= 32U) {
    return fail(reply, 429, "resource_exhausted", "too many held describe calls");
  }
  describe_waiters_.push_back(DescribeWaiter{reply, until});
  waiting_.store(describe_waiters_.size() + fresh_captures_.size(), std::memory_order_release);
  ++waiters_revision_;
  waiters_changed_.notify_all();
}

void SourceControlService::describeSource(const HttpReply& reply) const
{
  const SourceInfo info = state_->info();
  Json capabilities = Json::array({"start", "stop", "request-keyframe", "capture", "fresh-snapshot"});
  respond(
      reply,
      200,
      {
          {"ok", true},
          {"protocolVersion", 1},
          {"sourceId", options_.source_id},
          {"sourceKind", info.source_kind},
          {"codec", "H264"},
          {"rtpPayloadType", options_.rtp_payload_type},
          {"rtpClockRate", 90000},
          {"rtpHost", pipeline_->rtpHost()},
          {"rtpPort", pipeline_->rtpPort()},
          {"width", info.width},
          {"height", info.height},
          {"fps", info.fps},
          {"frameId", info.frame_id},
          {"capabilities", capabilities},
          {"keyframeRequestSupported", true},
          {"keyframePolicy", "native-force-idr"},
          {"ready", state_->ready()},
      });
}

void SourceControlService::sourceStatus(const HttpReply& reply) const
{
  const MediaPipeline::Status status = pipeline_->status();
  static const char* const names[] = {"idle", "starting", "active", "stopping", "faulted"};
  const std::uint64_t revision = revision_.load();
  Json native = {
      {"ready", state_->ready()},
      {"encoder", status.encoder},
      {"pictures_decoded", status.pictures_decoded},
      {"decode_errors", status.decode_errors},
      {"frames_dropped", status.frames_dropped},
      {"access_units", status.access_units},
      {"keyframes", status.keyframes},
      {"rtp_packets", status.rtp_packets},
      {"rtp_bytes", status.rtp_bytes},
      {"rtp_send_errors", status.rtp_send_errors},
      {"preview_active", status.preview_active},
      {"preview_access_units", status.preview_access_units},
      {"encode_milliseconds_average", status.encode_milliseconds_average},
  };
  if (!status.encoder_fallback.empty()) {
    native["encoder_fallback"] = status.encoder_fallback;
  }
  Json body = {
      {"ok", true},
      {"source_id", options_.source_id},
      {"state", names[static_cast<int>(status.state)]},
      {"desired_active", status.desired_active},
      {"applied_active", status.state == MediaPipeline::State::Active},
      {"configuration_revision", revision},
      {"desired_revision", revision},
      {"applied_revision", revision},
      {"persisted_revision", nullptr},
      {"native", native},
  };
  if (!status.last_error.empty()) {
    body["last_error"] = status.last_error;
  }
  respond(reply, 200, body);
}

void SourceControlService::sourceConfig(const HttpReply& reply) const
{
  const Json current = {
      {"rtp_host", pipeline_->rtpHost()},
      {"rtp_port", pipeline_->rtpPort()},
      {"bitrate", pipeline_->bitrate()},
  };
  const std::uint64_t revision = revision_.load();
  respond(
      reply,
      200,
      {
          {"ok", true},
          {"source_id", options_.source_id},
          {"desired", current},
          {"applied", current},
          {"desired_revision", revision},
          {"applied_revision", revision},
          {"persisted_revision", nullptr},
          {"persistence", "ephemeral"},
          {"mutable_fields", Json::array({"rtp_host", "rtp_port", "bitrate"})},
      });
}

void SourceControlService::patchConfig(const Json& body, const HttpReply& reply)
{
  requireFields(body, {"expected_revision", "persist", "config"});
  const auto expected = body.find("expected_revision");
  if (expected == body.end() || !expected->is_number_unsigned()) {
    throw std::invalid_argument("expected_revision must be an unsigned integer");
  }
  if (optionalBool(body, "persist", false)) {
    throw std::invalid_argument("persistence is not supported; configuration is ephemeral");
  }
  const auto config = body.find("config");
  if (config == body.end() || !config->is_object() || config->empty()) {
    throw std::invalid_argument("config must be a non-empty object");
  }
  requireFields(*config, {"rtp_host", "rtp_port", "bitrate"});
  std::string host = pipeline_->rtpHost();
  std::uint16_t port = pipeline_->rtpPort();
  std::uint32_t bitrate = pipeline_->bitrate();
  if (config->contains("rtp_host")) {
    const Json& value = (*config)["rtp_host"];
    if (!value.is_string() || !loopbackHost(value.get<std::string>())) {
      throw std::invalid_argument("rtp_host must be a loopback address");
    }
    host = value.get<std::string>();
  }
  if (config->contains("rtp_port")) {
    const Json& value = (*config)["rtp_port"];
    if (!value.is_number_unsigned() || value.get<std::uint64_t>() < 1U ||
        value.get<std::uint64_t>() > 65535U) {
      throw std::invalid_argument("rtp_port must be an integer in 1..65535");
    }
    port = static_cast<std::uint16_t>(value.get<std::uint64_t>());
  }
  if (config->contains("bitrate")) {
    const Json& value = (*config)["bitrate"];
    if (!value.is_number_unsigned() || value.get<std::uint64_t>() < 100000U ||
        value.get<std::uint64_t>() > 1000000000U) {
      throw std::invalid_argument("bitrate must be an integer in 100000..1000000000");
    }
    bitrate = static_cast<std::uint32_t>(value.get<std::uint64_t>());
  }
  if (expected->get<std::uint64_t>() != revision_.load()) {
    return fail(reply, 409, "conflict", "expected_revision is stale");
  }
  // The destination and the bitrate are bound when the stream opens.
  if (!pipeline_->setRtpDestination(host, port) || !pipeline_->setBitrate(bitrate)) {
    return fail(
        reply, 409, "conflict",
        "restart_required: stop the stream before changing rtp_host, rtp_port or bitrate");
  }
  ++revision_;
  sourceConfig(reply);
}

Json SourceControlService::lifecycleReceipt(const char* state_name, const bool active) const
{
  return {
      {"ok", true},
      {"source_id", options_.source_id},
      {"state", state_name},
      {"active", active},
      {"completion", "applied"},
      {"configuration_revision", revision_.load()},
  };
}

void SourceControlService::lifecycle(const std::string& operation, const HttpReply& reply)
{
  if (operation == "start") {
    if (!state_->ready()) {
      return fail(reply, 503, "unavailable", "the source is not ready: " + state_->reason());
    }
    pipeline_->start([this, reply](const std::string& error) {
      if (error.empty()) {
        respond(reply, 200, lifecycleReceipt("active", true));
      } else {
        fail(reply, 503, "unavailable", error);
      }
    });
  } else if (operation == "stop") {
    pipeline_->stop([this, reply](const std::string& error) {
      if (error.empty()) {
        respond(reply, 200, lifecycleReceipt("idle", false));
      } else {
        fail(reply, 500, "internal", error);
      }
    });
  } else {
    pipeline_->requestKeyframe([this, reply](const std::string& error) {
      if (error.empty()) {
        respond(reply, 200, lifecycleReceipt("active", true));
      } else {
        fail(reply, 409, "conflict", error);
      }
    });
  }
}

void SourceControlService::capture(
    const Json& body, const HttpRequest& request, const HttpReply& reply)
{
  requireFields(body, {"snapshotId", "includeRgb", "requireFresh", "requestKeyframe"});
  const auto snapshot = body.find("snapshotId");
  if (snapshot != body.end() &&
      (!snapshot->is_string() || !safeIdentifier(snapshot->get<std::string>()))) {
    throw std::invalid_argument("snapshotId must be a short identifier");
  }
  (void)optionalBool(body, "includeRgb", true);  // reject a bad type before any work starts
  const bool require_fresh = optionalBool(body, "requireFresh", false);
  const bool request_keyframe = optionalBool(body, "requestKeyframe", false);
  if (!state_->ready() || !state_->latest()) {
    return fail(reply, 503, "unavailable", "no picture is available: " + state_->reason());
  }
  bool expected = false;
  if (!capture_in_flight_.compare_exchange_strong(expected, true)) {
    return fail(reply, 429, "resource_exhausted", "a capture is already in progress");
  }
  const SourceInfo info = state_->info();
  const double fps = info.fps > 0.0 ? info.fps : 1.0;
  const auto wait = std::chrono::milliseconds(
      static_cast<std::int64_t>(std::max(0.5, std::min(2.0, 3.0 / fps)) * 1000.0));
  const std::uint64_t after = state_->frameCount();
  const auto until = std::min(Clock::now() + wait, request.deadline - std::chrono::milliseconds(100));
  // A still never produces a newer picture: its single frame is always fresh.
  const bool wait_for_frame = require_fresh && info.source_kind != "still";

  // Runs on the service thread, or on the pipeline thread that completed the
  // keyframe request; both only touch the guarded waiter list or the queue.
  auto proceed = [this, body, reply, wait_for_frame, after, until]() {
    if (wait_for_frame) {
      {
        std::lock_guard<std::mutex> lock(waiters_mutex_);
        fresh_captures_.push_back(FreshCapture{reply, body, after, until});
        waiting_.store(
            describe_waiters_.size() + fresh_captures_.size(), std::memory_order_release);
        ++waiters_revision_;
      }
      waiters_changed_.notify_all();
      server_->wake();
      return;
    }
    const auto frame = state_->latest();
    post([this, body, frame, reply]() { deliverCapture(body, frame, reply); });
  };
  if (!request_keyframe) {
    return proceed();
  }
  pipeline_->requestKeyframe([this, reply, proceed](const std::string& error) {
    if (!error.empty()) {
      capture_in_flight_.store(false);
      return fail(reply, 409, "conflict", "keyframe request failed: " + error);
    }
    proceed();
  });
}

void SourceControlService::processWaiters()
{
  std::vector<std::function<void()>> actions;
  {
    std::lock_guard<std::mutex> lock(waiters_mutex_);
    const auto now = Clock::now();
    const bool ready = state_->ready();
    for (auto waiter = describe_waiters_.begin(); waiter != describe_waiters_.end();) {
      if (waiter->reply.cancelled()) {
        waiter = describe_waiters_.erase(waiter);
      } else if (ready || now >= waiter->until) {
        const HttpReply reply = waiter->reply;
        actions.push_back([this, reply] { respond(reply, 200, serviceDescription()); });
        waiter = describe_waiters_.erase(waiter);
      } else {
        ++waiter;
      }
    }
    const std::uint64_t count = state_->frameCount();
    for (auto waiter = fresh_captures_.begin(); waiter != fresh_captures_.end();) {
      const HttpReply reply = waiter->reply;
      if (reply.cancelled()) {
        capture_in_flight_.store(false);
        waiter = fresh_captures_.erase(waiter);
      } else if (count > waiter->after_count && state_->latest()) {
        const Json request = waiter->request;
        const auto frame = state_->latest();
        actions.push_back([this, request, frame, reply] {
          post([this, request, frame, reply] { deliverCapture(request, frame, reply); });
        });
        waiter = fresh_captures_.erase(waiter);
      } else if (now >= waiter->until) {
        capture_in_flight_.store(false);
        actions.push_back([this, reply] {
          fail(reply, 504, "deadline_exceeded", "no fresh picture arrived in time");
        });
        waiter = fresh_captures_.erase(waiter);
      } else {
        ++waiter;
      }
    }
    waiting_.store(describe_waiters_.size() + fresh_captures_.size(), std::memory_order_release);
    ++waiters_revision_;
  }
  waiters_changed_.notify_all();
  for (const auto& action : actions) {
    action();
  }
}

void SourceControlService::deliverCapture(
    const Json& request, const std::shared_ptr<const CaptureFrame>& frame, const HttpReply& reply)
{
  struct Release {
    std::atomic<bool>* flag;
    ~Release() { flag->store(false); }
  } release{&capture_in_flight_};
  try {
    if (reply.cancelled()) {
      return;
    }
    const SourceInfo info = state_->info();
    const bool include_rgb = optionalBool(request, "includeRgb", true);
    std::string rgb;
    if (include_rgb) {
      cv::Mat bgr = cv::imdecode(*frame->jpeg, cv::IMREAD_COLOR);
      if (bgr.empty() || bgr.cols != static_cast<int>(frame->width) ||
          bgr.rows != static_cast<int>(frame->height)) {
        return fail(reply, 500, "internal", "the captured JPEG cannot be decoded to RGB");
      }
      cv::Mat rgb_image;
      cv::cvtColor(bgr, rgb_image, cv::COLOR_BGR2RGB);
      rgb.assign(reinterpret_cast<const char*>(rgb_image.data), rgb_image.total() * 3U);
    }
    Json metadata = {
        {"ok", true},
        {"sourceId", options_.source_id},
        {"snapshotId", request.value("snapshotId", std::string())},
        {"frameId", info.frame_id},
        {"frameSequence", frame->sequence},
        {"timestampNanoseconds", frame->timestamp_ns},
        {"timestampClockDomain", info.clock_domain},
        {"width", frame->width},
        {"height", frame->height},
        {"pixelFormat", "rgb8"},
        {"jpegBytes", frame->jpeg->size()},
        {"rgbBytes", rgb.size()},
        {"jpegBackend", "camera-jpeg-passthrough"},
        {"jpegReadback", "latest-compressed-frame"},
        {"calibrationState", info.calibrated ? "available" : "unavailable"},
    };
    if (info.calibrated) {
      metadata["cameraMatrix"] = info.camera_matrix;
      metadata["distortion"] = info.distortion;
    }
    const std::string metadata_text = metadata.dump();
    const std::string separator = boundary();
    HttpResponse response;
    response.status = 200;
    response.headers.emplace_back("Content-Type", "multipart/mixed; boundary=" + separator);
    response.headers.emplace_back("Cache-Control", "no-store");
    response.body.reserve(metadata_text.size() + frame->jpeg->size() + rgb.size() + 512U);
    appendPart(&response.body, separator, "application/json", "metadata", metadata_text.data(),
               metadata_text.size());
    appendPart(&response.body, separator, "image/jpeg", "jpeg",
               reinterpret_cast<const char*>(frame->jpeg->data()), frame->jpeg->size());
    if (include_rgb) {
      appendPart(&response.body, separator, "application/octet-stream", "rgb", rgb.data(),
                 rgb.size());
    }
    response.body.append("--").append(separator).append("--\r\n");
    reply.complete(std::move(response));
  } catch (const std::exception& error) {
    fail(reply, 500, "internal", error.what());
  }
}

}  // namespace xgc2_camera_driver
