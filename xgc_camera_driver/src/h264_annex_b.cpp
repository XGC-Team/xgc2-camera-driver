#include "h264_annex_b.hpp"

#include <algorithm>

namespace xgc_camera_driver {
namespace h264 {
namespace {

struct NALSpan {
  std::size_t begin{0};
  std::size_t payload{0};
  std::size_t end{0};
  std::uint8_t type{0};
};

bool startCodeAt(
    const std::uint8_t* data,
    std::size_t size,
    std::size_t offset,
    std::size_t* length) noexcept {
  if (offset + 3U <= size && data[offset] == 0U &&
      data[offset + 1U] == 0U && data[offset + 2U] == 1U) {
    *length = 3U;
    return true;
  }
  if (offset + 4U <= size && data[offset] == 0U &&
      data[offset + 1U] == 0U && data[offset + 2U] == 0U &&
      data[offset + 3U] == 1U) {
    *length = 4U;
    return true;
  }
  return false;
}

std::vector<NALSpan> spans(
    const std::uint8_t* data,
    std::size_t size) {
  std::vector<NALSpan> result;
  if (!data || size < 4U) {
    return result;
  }
  std::size_t offset = 0U;
  while (offset < size) {
    std::size_t prefix = 0U;
    if (!startCodeAt(data, size, offset, &prefix)) {
      ++offset;
      continue;
    }
    const std::size_t payload = offset + prefix;
    if (payload >= size) {
      break;
    }
    std::size_t end = payload + 1U;
    while (end < size) {
      std::size_t next_prefix = 0U;
      if (startCodeAt(data, size, end, &next_prefix)) {
        break;
      }
      ++end;
    }
    result.push_back(
        NALSpan{offset, payload, end,
                static_cast<std::uint8_t>(data[payload] & 0x1fU)});
    offset = end;
  }
  return result;
}

void copySpan(
    const std::uint8_t* data,
    const NALSpan& span,
    std::vector<std::uint8_t>* output) {
  output->assign(data + span.begin, data + span.end);
}

}  // namespace

AccessUnitInfo inspectAccessUnit(
    const std::uint8_t* data,
    std::size_t size) noexcept {
  AccessUnitInfo info;
  try {
    const auto units = spans(data, size);
    info.annex_b = !units.empty() && units.front().begin == 0U;
    for (const auto& unit : units) {
      info.has_vcl = info.has_vcl || (unit.type >= 1U && unit.type <= 5U);
      info.keyframe = info.keyframe || unit.type == 5U;
      info.has_sps = info.has_sps || unit.type == 7U;
      info.has_pps = info.has_pps || unit.type == 8U;
    }
  } catch (...) {
    return AccessUnitInfo{};
  }
  return info;
}

bool AccessUnitGate::prepare(
    const std::uint8_t* data,
    std::size_t size,
    std::vector<std::uint8_t>* output,
    AccessUnitInfo* info) {
  if (!output || !info) {
    return false;
  }
  output->clear();
  *info = inspectAccessUnit(data, size);
  if (!info->annex_b) {
    return false;
  }
  const auto units = spans(data, size);
  for (const auto& unit : units) {
    if (unit.type == 7U) {
      copySpan(data, unit, &sps_);
    } else if (unit.type == 8U) {
      copySpan(data, unit, &pps_);
    }
  }
  if (!info->has_vcl || (waiting_for_idr_ && !info->keyframe)) {
    return false;
  }
  if (info->keyframe) {
    if ((!info->has_sps && sps_.empty()) ||
        (!info->has_pps && pps_.empty())) {
      waiting_for_idr_ = true;
      return false;
    }
    output->reserve(
        size + (info->has_sps ? 0U : sps_.size()) +
        (info->has_pps ? 0U : pps_.size()));
    if (!info->has_sps) {
      output->insert(output->end(), sps_.begin(), sps_.end());
    }
    if (!info->has_pps) {
      output->insert(output->end(), pps_.begin(), pps_.end());
    }
    output->insert(output->end(), data, data + size);
    info->has_sps = true;
    info->has_pps = true;
    waiting_for_idr_ = false;
    return true;
  }
  output->assign(data, data + size);
  return true;
}

}  // namespace h264
}  // namespace xgc_camera_driver
