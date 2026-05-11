#include "switch_udp_bridge/crc16.hpp"

namespace switch_udp_bridge::crc16
{

std::uint16_t calculate(const std::uint8_t * data, std::size_t length)
{
  std::uint16_t crc = 0xFFFF;

  for (std::size_t i = 0; i < length; ++i) {
    crc ^= data[i];

    for (std::size_t j = 0; j < 8; ++j) {
      if ((crc & 0x0001) != 0) {
        crc = static_cast<std::uint16_t>((crc >> 1) ^ 0xA001);
      } else {
        crc = static_cast<std::uint16_t>(crc >> 1);
      }
    }
  }

  return crc;
}

}  // namespace switch_udp_bridge::crc16