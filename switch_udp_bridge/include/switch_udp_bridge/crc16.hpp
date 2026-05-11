#pragma once

#include <cstddef>
#include <cstdint>

namespace switch_udp_bridge::crc16
{

std::uint16_t calculate(const std::uint8_t * data, std::size_t length);

}  // namespace switch_udp_bridge::crc16