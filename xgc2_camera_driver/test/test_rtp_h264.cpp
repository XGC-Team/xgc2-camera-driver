#include "rtp_h264.hpp"

#include <gtest/gtest.h>

#include <arpa/inet.h>
#include <netinet/in.h>
#include <sys/socket.h>
#include <unistd.h>

#include <cstring>
#include <string>
#include <thread>
#include <vector>

namespace driver = xgc2_camera_driver;

namespace {

std::vector<std::uint8_t> nal(const std::uint8_t header, const std::size_t size, const std::uint8_t fill)
{
  std::vector<std::uint8_t> unit(size, fill);
  unit[0] = header;
  return unit;
}

// Concatenates NAL units behind 4-byte start codes, with the first one using
// the 3-byte form as some encoders do.
std::vector<std::uint8_t> accessUnit(const std::vector<std::vector<std::uint8_t>>& units)
{
  std::vector<std::uint8_t> stream;
  for (std::size_t index = 0; index < units.size(); ++index) {
    if (index != 0U) {
      stream.push_back(0);
    }
    stream.insert(stream.end(), {0, 0, 1});
    stream.insert(stream.end(), units[index].begin(), units[index].end());
  }
  return stream;
}

std::vector<std::uint8_t> normalized(const std::vector<std::vector<std::uint8_t>>& units)
{
  std::vector<std::uint8_t> stream;
  for (const auto& unit : units) {
    stream.insert(stream.end(), {0, 0, 0, 1});
    stream.insert(stream.end(), unit.begin(), unit.end());
  }
  return stream;
}

}  // namespace

TEST(H264RtpPacketizer, SingleAndFragmentedUnitsRoundTrip)
{
  const std::vector<std::vector<std::uint8_t>> units = {
      nal(0x67, 20, 0x11),    // SPS
      nal(0x68, 6, 0x22),     // PPS
      nal(0x65, 4000, 0x33),  // IDR slice, needs FU-A
  };
  driver::H264RtpPacketizer packetizer(96, 0xdeadbeef, 100);
  std::vector<std::vector<std::uint8_t>> packets;
  packetizer.packetize(accessUnit(units).data(), accessUnit(units).size(), 90000, &packets);

  ASSERT_EQ(packets.size(), 2U + 4U);  // 4000 bytes at 1186 per fragment
  for (std::size_t index = 0; index < packets.size(); ++index) {
    const auto& packet = packets[index];
    EXPECT_LE(packet.size(), 1200U);
    EXPECT_EQ(packet[0], 0x80);
    EXPECT_EQ(packet[1] & 0x7f, 96);
    EXPECT_EQ((packet[1] & 0x80) != 0, index + 1U == packets.size()) << "marker " << index;
    const std::uint16_t sequence = static_cast<std::uint16_t>((packet[2] << 8) | packet[3]);
    EXPECT_EQ(sequence, 100 + index);
    EXPECT_EQ(std::memcmp(packet.data() + 4, "\x00\x01\x5f\x90", 4), 0);  // 90000
    EXPECT_EQ(std::memcmp(packet.data() + 8, "\xde\xad\xbe\xef", 4), 0);
  }
  EXPECT_EQ(packets[0][12], 0x67);
  EXPECT_EQ(packets[2][12] & 0x1f, 28);          // FU-A
  EXPECT_EQ(packets[2][13] & 0xc0, 0x80);        // start
  EXPECT_EQ(packets[5][13] & 0xc0, 0x40);        // end
  EXPECT_EQ(packets[2][13] & 0x1f, 5);           // original type
  EXPECT_EQ(packets[2][12] & 0xe0, 0x60);        // NRI preserved
  EXPECT_EQ(packetizer.next_sequence(), 106);

  driver::H264RtpDepacketizer depacketizer;
  std::vector<std::vector<std::uint8_t>> access_units;
  for (const auto& packet : packets) {
    EXPECT_TRUE(depacketizer.push(packet.data(), packet.size(), &access_units));
  }
  ASSERT_EQ(access_units.size(), 1U);
  EXPECT_EQ(access_units[0], normalized(units));
  EXPECT_EQ(depacketizer.last_timestamp(), 90000U);
  EXPECT_EQ(depacketizer.lost_packets(), 0U);
}

TEST(H264RtpPacketizer, SequenceWrapsAndFourByteStartCodesParse)
{
  driver::H264RtpPacketizer packetizer(96, 1, 0xfffe);
  std::vector<std::uint8_t> stream = {0, 0, 0, 1};
  const auto unit = nal(0x41, 30, 0x44);
  stream.insert(stream.end(), unit.begin(), unit.end());
  std::vector<std::vector<std::uint8_t>> packets;
  for (int index = 0; index < 4; ++index) {
    packetizer.packetize(stream.data(), stream.size(), 3000U * index, &packets);
  }
  ASSERT_EQ(packets.size(), 4U);
  EXPECT_EQ(packets[0][3], 0xfe);
  EXPECT_EQ(packets[1][3], 0xff);
  EXPECT_EQ(packets[2][2], 0x00);
  EXPECT_EQ(packets[2][3], 0x00);
  EXPECT_EQ(packets[3][3], 0x01);
  for (const auto& packet : packets) {
    EXPECT_NE(packet[1] & 0x80, 0);  // each single-unit access unit ends with a marker
  }
}

TEST(H264RtpDepacketizer, DetectsLossAndDiscardsTheDamagedUnit)
{
  driver::H264RtpPacketizer packetizer(96, 1, 7);
  const auto unit = accessUnit({nal(0x65, 3000, 0x55)});
  std::vector<std::vector<std::uint8_t>> packets;
  packetizer.packetize(unit.data(), unit.size(), 0, &packets);
  ASSERT_GE(packets.size(), 3U);
  driver::H264RtpDepacketizer depacketizer;
  std::vector<std::vector<std::uint8_t>> access_units;
  EXPECT_TRUE(depacketizer.push(packets[0].data(), packets[0].size(), &access_units));
  // packet 1 is lost
  for (std::size_t index = 2; index < packets.size(); ++index) {
    depacketizer.push(packets[index].data(), packets[index].size(), &access_units);
  }
  EXPECT_EQ(depacketizer.lost_packets(), 1U);
  // The marker packet arrives but its fragment lacks a start: no unit is emitted
  // as a valid one, only an empty or truncated buffer must not decode.
  for (const auto& produced : access_units) {
    EXPECT_NE(produced, normalized({nal(0x65, 3000, 0x55)}));
  }
}

TEST(H264RtpPacketizer, RejectsInvalidConfiguration)
{
  EXPECT_THROW(driver::H264RtpPacketizer(200, 1, 0), std::invalid_argument);
  EXPECT_THROW(driver::H264RtpPacketizer(96, 1, 0, 20), std::invalid_argument);
}

TEST(UdpSender, DeliversEveryDatagramOfALargeUnitToAReadingReceiver)
{
  const int receiver = ::socket(AF_INET, SOCK_DGRAM, 0);
  ASSERT_GE(receiver, 0);
  timeval timeout{1, 0};
  ASSERT_EQ(::setsockopt(receiver, SOL_SOCKET, SO_RCVTIMEO, &timeout, sizeof(timeout)), 0);
  sockaddr_in address{};
  address.sin_family = AF_INET;
  address.sin_addr.s_addr = htonl(INADDR_LOOPBACK);
  ASSERT_EQ(::bind(receiver, reinterpret_cast<sockaddr*>(&address), sizeof(address)), 0);
  socklen_t length = sizeof(address);
  ASSERT_EQ(::getsockname(receiver, reinterpret_cast<sockaddr*>(&address), &length), 0);

  // The receiver keeps its default socket buffer, as an unprivileged consumer
  // would: the sender's pacing must let a concurrent reader keep up with a
  // burst far larger than that buffer.
  std::vector<int> received;
  std::thread reader([&] {
    for (;;) {
      std::uint8_t datagram[1500];
      const ssize_t size = ::recv(receiver, datagram, sizeof(datagram), 0);
      if (size != 1200) {
        return;
      }
      received.push_back(datagram[0]);
    }
  });
  driver::UdpSender sender("127.0.0.1", ntohs(address.sin_port));
  std::vector<std::vector<std::uint8_t>> packets;
  for (int index = 0; index < 600; ++index) {
    packets.emplace_back(1200, static_cast<std::uint8_t>(index));
  }
  sender.send(packets);
  EXPECT_EQ(sender.packets_sent(), 600U);
  EXPECT_EQ(sender.bytes_sent(), 600U * 1200U);
  EXPECT_EQ(sender.send_errors(), 0U);
  reader.join();  // returns after the one-second receive timeout
  ::close(receiver);
  ASSERT_EQ(received.size(), 600U);
  for (std::size_t index = 0; index < received.size(); ++index) {
    EXPECT_EQ(received[index], static_cast<int>(static_cast<std::uint8_t>(index))) << index;
  }
}

TEST(UdpSender, ToleratesAnAbsentReceiverAndRejectsNonLoopback)
{
  EXPECT_THROW(driver::UdpSender("192.0.2.1", 5004), std::runtime_error);
  EXPECT_THROW(driver::UdpSender("not-an-address", 5004), std::runtime_error);
  EXPECT_THROW(driver::UdpSender("127.0.0.1", 0), std::runtime_error);
  // Reserve a port and release it so nothing listens there.
  const int probe = ::socket(AF_INET, SOCK_DGRAM, 0);
  sockaddr_in address{};
  address.sin_family = AF_INET;
  address.sin_addr.s_addr = htonl(INADDR_LOOPBACK);
  ASSERT_EQ(::bind(probe, reinterpret_cast<sockaddr*>(&address), sizeof(address)), 0);
  socklen_t length = sizeof(address);
  ASSERT_EQ(::getsockname(probe, reinterpret_cast<sockaddr*>(&address), &length), 0);
  ::close(probe);
  driver::UdpSender sender("127.0.0.1", ntohs(address.sin_port));
  const std::vector<std::vector<std::uint8_t>> packets(5, std::vector<std::uint8_t>(100, 1));
  EXPECT_NO_THROW(sender.send(packets));
  EXPECT_NO_THROW(sender.send(packets));
  EXPECT_GT(sender.send_errors(), 0U);
}

int main(int argc, char** argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
