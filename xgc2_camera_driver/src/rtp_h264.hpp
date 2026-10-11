#pragma once

#include <cstddef>
#include <cstdint>
#include <string>
#include <vector>

namespace xgc2_camera_driver {

// Splits Annex-B access units into RTP packets as RFC 6184 packetization-mode 1
// describes: single NAL unit packets, and FU-A fragments for NAL units that do
// not fit. The marker bit is set on the last packet of every access unit.
class H264RtpPacketizer {
 public:
  // max_packet is the largest datagram payload, header included.
  H264RtpPacketizer(
      std::uint8_t payload_type,
      std::uint32_t ssrc,
      std::uint16_t first_sequence,
      std::size_t max_packet = 1200);

  // `timestamp` is the 90 kHz RTP timestamp shared by every packet of the unit.
  void packetize(
      const std::uint8_t* annex_b,
      std::size_t size,
      std::uint32_t timestamp,
      std::vector<std::vector<std::uint8_t>>* packets);

  std::uint16_t next_sequence() const noexcept { return sequence_; }

 private:
  void emit(
      std::vector<std::vector<std::uint8_t>>* packets,
      std::uint32_t timestamp,
      bool marker,
      const std::uint8_t* head,
      std::size_t head_size,
      const std::uint8_t* payload,
      std::size_t payload_size);

  std::uint8_t payload_type_;
  std::uint32_t ssrc_;
  std::uint16_t sequence_;
  std::size_t max_packet_;
};

// Reassembles RTP packets back into NAL units. Used by tests and measurement
// tools to prove that the packetizer output is a decodable stream.
class H264RtpDepacketizer {
 public:
  // Returns false for a malformed packet. A completed access unit (marker bit)
  // is appended to `access_units` as Annex-B.
  bool push(
      const std::uint8_t* packet,
      std::size_t size,
      std::vector<std::vector<std::uint8_t>>* access_units);

  std::uint32_t last_timestamp() const noexcept { return timestamp_; }
  std::uint64_t lost_packets() const noexcept { return lost_; }

 private:
  std::vector<std::uint8_t> unit_;
  std::vector<std::uint8_t> fragment_;
  bool fragmenting_{false};
  bool have_sequence_{false};
  std::uint16_t expected_sequence_{0};
  std::uint32_t timestamp_{0};
  std::uint64_t lost_{0};
};

// A connected loopback UDP socket for RTP. Sending never blocks the caller on
// a missing receiver: ECONNREFUSED from an absent listener is counted, not fatal.
class UdpSender {
 public:
  // Throws std::runtime_error. Only loopback destinations are accepted.
  UdpSender(const std::string& host, std::uint16_t port);
  ~UdpSender();
  UdpSender(const UdpSender&) = delete;
  UdpSender& operator=(const UdpSender&) = delete;

  // Sends the packets of one access unit. A large unit is spread over a few
  // milliseconds so the receiver's socket buffer can drain between bursts.
  void send(const std::vector<std::vector<std::uint8_t>>& packets);

  std::uint64_t packets_sent() const noexcept { return packets_; }
  std::uint64_t bytes_sent() const noexcept { return bytes_; }
  std::uint64_t send_errors() const noexcept { return errors_; }

 private:
  int fd_{-1};
  std::uint64_t packets_{0};
  std::uint64_t bytes_{0};
  std::uint64_t errors_{0};
};

}  // namespace xgc2_camera_driver
