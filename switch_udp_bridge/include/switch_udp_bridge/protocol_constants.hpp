#pragma once

#include <cstddef>
#include <cstdint>

namespace switch_udp_bridge::protocol
{

constexpr std::size_t ETHERNET_RECEIVE_LEN = 56;
constexpr std::size_t ETHERNET_SEND_LEN = 170;

constexpr std::size_t BOARD_CMD_LEN = 17;
constexpr std::size_t BOARD_STATUS_LEN = 46;

constexpr std::uint8_t FRAME_HEAD = 0xFE;

constexpr std::size_t BOARD_CMD_CRC_OFFSET = 15;
constexpr std::size_t BOARD_CMD_CRC_CALC_LEN = 15;

constexpr std::size_t BOARD_STATUS_CRC_OFFSET = 44;
constexpr std::size_t BOARD_STATUS_CRC_CALC_LEN = 44;

constexpr std::size_t POWER_CMD_OFFSET = 51;
constexpr std::size_t LED_CMD_OFFSET = 55;

constexpr std::size_t POWER_STATUS_OFFSET = 138;
constexpr std::size_t SWITCH_STATUS_OFFSET = 169;

constexpr std::uint8_t CMD_MOTION = 0x01;
constexpr std::uint8_t CMD_READ_STATUS = 0x02;
constexpr std::uint8_t CMD_ZERO = 0x03;

}  // namespace switch_udp_bridge::protocol