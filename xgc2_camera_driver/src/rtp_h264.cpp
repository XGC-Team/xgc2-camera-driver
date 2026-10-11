#include "rtp_h264.hpp"

#include <arpa/inet.h>
#include <netinet/in.h>
#include <sys/socket.h>
#include <unistd.h>

#include <algorithm>
#include <cerrno>
#include <chrono>
#include <cstring>
#include <stdexcept>
#include <thread>

namespace xgc2_camera_driver {
namespace {

constexpr std::size_t kRtpHeaderSize = 12;
constexpr std::uint8_t kFuAType = 28;
// Datagrams sent back to back before yielding to the receiver.
constexpr std::size_t kBurstPackets = 32;
constexpr auto kBurstPause = std::chrono::microseconds(100);

struct NalUnit {
  const std::uint8_t* data;
  std::size_t size;
};

// Splits an Annex-B buffer at its 3- or 4-byte start codes.
std::vector<NalUnit> splitAnnexB(const std::uint8_t* data, const std::size_t size)
{
  std::vector<NalUnit> units;
  std::size_t index = 0;
  std::size_t start = size;
  while (index + 3U <= size) {
    if (data[index] == 0U && data[index + 1U] == 0U && data[index + 2U] == 1U) {
      if (start != size) {
        std::size_t end = index;
        while (end > start && data[end - 1U] == 0U) {
          --end;  // the zero before a 3-byte start code belongs to the 4-byte form
        }
        if (end > start) {
          units.push_back(NalUnit{data + start, end - start});
        }
      }
      start = index + 3U;
      index += 3U;
    } else {
      ++index;
    }
  }
  if (start != size) {
    std::size_t end = size;
    while (end > start && data[end - 1U] == 0U) {
      --end;
    }
    if (end > start) {
      units.push_back(NalUnit{data + start, end - start});
    }
  }
  return units;
}

}  // namespace

H264RtpPacketizer::H264RtpPacketizer(
    const std::uint8_t payload_type,
    const std::uint32_t ssrc,
    const std::uint16_t first_sequence,
    const std::size_t max_packet)
    : payload_type_(payload_type),
      ssrc_(ssrc),
      sequence_(first_sequence),
      max_packet_(max_packet)
{
  if (max_packet_ < kRtpHeaderSize + 16U || payload_type_ > 127U) {
    throw std::invalid_argument("invalid RTP packetizer configuration");
  }
}

void H264RtpPacketizer::emit(
    std::vector<std::vector<std::uint8_t>>* packets,
    const std::uint32_t timestamp,
    const bool marker,
    const std::uint8_t* head,
    const std::size_t head_size,
    const std::uint8_t* payload,
    const std::size_t payload_size)
{
  std::vector<std::uint8_t> packet(kRtpHeaderSize + head_size + payload_size);
  packet[0] = 0x80U;  // version 2, no padding, no extension, no CSRC
  packet[1] = static_cast<std::uint8_t>((marker ? 0x80U : 0U) | payload_type_);
  packet[2] = static_cast<std::uint8_t>(sequence_ >> 8U);
  packet[3] = static_cast<std::uint8_t>(sequence_ & 0xffU);
  packet[4] = static_cast<std::uint8_t>(timestamp >> 24U);
  packet[5] = static_cast<std::uint8_t>(timestamp >> 16U);
  packet[6] = static_cast<std::uint8_t>(timestamp >> 8U);
  packet[7] = static_cast<std::uint8_t>(timestamp);
  packet[8] = static_cast<std::uint8_t>(ssrc_ >> 24U);
  packet[9] = static_cast<std::uint8_t>(ssrc_ >> 16U);
  packet[10] = static_cast<std::uint8_t>(ssrc_ >> 8U);
  packet[11] = static_cast<std::uint8_t>(ssrc_);
  std::memcpy(packet.data() + kRtpHeaderSize, head, head_size);
  std::memcpy(packet.data() + kRtpHeaderSize + head_size, payload, payload_size);
  ++sequence_;
  packets->push_back(std::move(packet));
}

void H264RtpPacketizer::packetize(
    const std::uint8_t* annex_b,
    const std::size_t size,
    const std::uint32_t timestamp,
    std::vector<std::vector<std::uint8_t>>* packets)
{
  const std::vector<NalUnit> units = splitAnnexB(annex_b, size);
  const std::size_t room = max_packet_ - kRtpHeaderSize;
  for (std::size_t index = 0; index < units.size(); ++index) {
    const NalUnit& unit = units[index];
    const bool last_unit = index + 1U == units.size();
    if (unit.size <= room) {
      emit(packets, timestamp, last_unit, nullptr, 0, unit.data, unit.size);
      continue;
    }
    // FU-A: the NAL header's F and NRI bits stay in the indicator, its type
    // moves to the FU header together with the start and end flags.
    const std::uint8_t indicator =
        static_cast<std::uint8_t>((unit.data[0] & 0xe0U) | kFuAType);
    const std::uint8_t type = static_cast<std::uint8_t>(unit.data[0] & 0x1fU);
    const std::size_t fragment_room = room - 2U;
    std::size_t offset = 1;  // the NAL header byte is carried by indicator and FU header
    while (offset < unit.size) {
      const std::size_t chunk = std::min(fragment_room, unit.size - offset);
      const bool first = offset == 1U;
      const bool last = offset + chunk == unit.size;
      const std::uint8_t head[2] = {
          indicator,
          static_cast<std::uint8_t>((first ? 0x80U : 0U) | (last ? 0x40U : 0U) | type)};
      emit(packets, timestamp, last && last_unit, head, 2, unit.data + offset, chunk);
      offset += chunk;
    }
  }
}

bool H264RtpDepacketizer::push(
    const std::uint8_t* packet,
    const std::size_t size,
    std::vector<std::vector<std::uint8_t>>* access_units)
{
  if (size < kRtpHeaderSize + 1U || (packet[0] >> 6U) != 2U) {
    return false;
  }
  const bool marker = (packet[1] & 0x80U) != 0U;
  const std::uint16_t sequence =
      static_cast<std::uint16_t>((packet[2] << 8U) | packet[3]);
  if (have_sequence_ && sequence != expected_sequence_) {
    lost_ += static_cast<std::uint16_t>(sequence - expected_sequence_);
    unit_.clear();
    fragmenting_ = false;
  }
  have_sequence_ = true;
  expected_sequence_ = static_cast<std::uint16_t>(sequence + 1U);
  timestamp_ = (static_cast<std::uint32_t>(packet[4]) << 24U) |
               (static_cast<std::uint32_t>(packet[5]) << 16U) |
               (static_cast<std::uint32_t>(packet[6]) << 8U) | packet[7];
  const std::uint8_t* payload = packet + kRtpHeaderSize;
  const std::size_t payload_size = size - kRtpHeaderSize;
  static const std::uint8_t start_code[4] = {0, 0, 0, 1};
  const std::uint8_t type = payload[0] & 0x1fU;
  if (type >= 1U && type <= 23U) {
    unit_.insert(unit_.end(), start_code, start_code + 4);
    unit_.insert(unit_.end(), payload, payload + payload_size);
  } else if (type == kFuAType) {
    if (payload_size < 3U) {
      return false;
    }
    const bool first = (payload[1] & 0x80U) != 0U;
    const bool last = (payload[1] & 0x40U) != 0U;
    if (first) {
      unit_.insert(unit_.end(), start_code, start_code + 4);
      unit_.push_back(static_cast<std::uint8_t>((payload[0] & 0xe0U) | (payload[1] & 0x1fU)));
      fragmenting_ = true;
    } else if (!fragmenting_) {
      return false;
    }
    unit_.insert(unit_.end(), payload + 2, payload + payload_size);
    if (last) {
      fragmenting_ = false;
    }
  } else {
    return false;
  }
  if (marker) {
    access_units->push_back(std::move(unit_));
    unit_.clear();
  }
  return true;
}

UdpSender::UdpSender(const std::string& host, const std::uint16_t port)
{
  in_addr address{};
  if (host == "localhost") {
    address.s_addr = htonl(INADDR_LOOPBACK);
  } else if (inet_pton(AF_INET, host.c_str(), &address) != 1 ||
             (ntohl(address.s_addr) >> 24U) != 127U) {
    throw std::runtime_error("RTP destination must be an IPv4 loopback address: " + host);
  }
  if (port == 0U) {
    throw std::runtime_error("RTP destination port must be non-zero");
  }
  fd_ = ::socket(AF_INET, SOCK_DGRAM | SOCK_CLOEXEC, 0);
  if (fd_ < 0) {
    throw std::runtime_error(std::string("RTP socket: ") + std::strerror(errno));
  }
  // A 4K IDR is hundreds of datagrams. Ask for room so a brief receiver stall
  // does not drop the tail of the burst; the kernel caps this at wmem_max.
  const int send_buffer = 4 << 20;
  (void)::setsockopt(fd_, SOL_SOCKET, SO_SNDBUF, &send_buffer, sizeof(send_buffer));
  sockaddr_in destination{};
  destination.sin_family = AF_INET;
  destination.sin_addr = address;
  destination.sin_port = htons(port);
  if (::connect(fd_, reinterpret_cast<const sockaddr*>(&destination), sizeof(destination)) != 0) {
    const int error = errno;
    ::close(fd_);
    fd_ = -1;
    throw std::runtime_error(std::string("RTP connect: ") + std::strerror(error));
  }
}

UdpSender::~UdpSender()
{
  if (fd_ >= 0) {
    ::close(fd_);
  }
}

void UdpSender::send(const std::vector<std::vector<std::uint8_t>>& packets)
{
  std::size_t in_burst = 0;
  for (const auto& packet : packets) {
    const ssize_t written = ::send(fd_, packet.data(), packet.size(), MSG_NOSIGNAL);
    if (written == static_cast<ssize_t>(packet.size())) {
      ++packets_;
      bytes_ += packet.size();
    } else {
      ++errors_;
    }
    if (++in_burst == kBurstPackets && &packet != &packets.back()) {
      in_burst = 0;
      std::this_thread::sleep_for(kBurstPause);
    }
  }
}

}  // namespace xgc2_camera_driver
