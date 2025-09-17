#ifndef RM_SERIAL_DRIVER__PACKET_HPP_
#define RM_SERIAL_DRIVER__PACKET_HPP_

#include <algorithm>
#include <cstdint>
#include <vector>

namespace rm_serial_driver
{

const uint8_t PACKET_VERSION = 0x32;

struct HeaderFrame
{
  uint8_t sof;  // 数据帧版本， START FROM 0x30
  uint8_t len;  // 数据段长度
  uint8_t id;   // 数据段id 0x3 A/B/C/D/E/F
  uint8_t crc;  // 数据帧头的 CRC8 校验
} __attribute__((packed));

struct SendRoboControl  // 0x3A
{
  HeaderFrame frame_header;

  float pitch;
  float yaw;
  bool fire;

  float vx;
  float vy;

  uint16_t crc;

} __attribute__((packed));

struct ReceiveRoboInfo  // 0x3B
{
  HeaderFrame frame_header;

  float roll;
  float pitch;
  float yaw;

  uint16_t encoder_up;
  uint16_t encoder_down;

  float super_cap_voltage;
  bool is_super_cap_work;

  bool is_auto_aim;

  uint16_t crc;

} __attribute__((packed));

template <typename T>
inline T fromVector(const std::vector<uint8_t> & data)
{
  T packet;
  std::copy(data.begin(), data.end(), reinterpret_cast<uint8_t *>(&packet));
  return packet;
}

template <typename T>
inline std::vector<uint8_t> toVector(const T & data)
{
  std::vector<uint8_t> packet(sizeof(T));
  std::copy(
    reinterpret_cast<const uint8_t *>(&data), reinterpret_cast<const uint8_t *>(&data) + sizeof(T),
    packet.begin());
  return packet;
}

}  // namespace rm_serial_driver

#endif  // RM_SERIAL_DRIVER__PACKET_HPP_
