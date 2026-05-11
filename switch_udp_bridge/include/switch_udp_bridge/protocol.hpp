#pragma once

#include <array>
#include <cstdint>

#include "builtin_interfaces/msg/time.hpp"
#include "switch_interfaces/msg/board_command.hpp"
#include "switch_interfaces/msg/board_status.hpp"
#include "switch_interfaces/msg/switch_command.hpp"
#include "switch_interfaces/msg/switch_status.hpp"
#include "switch_udp_bridge/protocol_constants.hpp"

namespace switch_udp_bridge::protocol
{

using CommandFrame = std::array<std::uint8_t, ETHERNET_RECEIVE_LEN>;
using StatusFrame = std::array<std::uint8_t, ETHERNET_SEND_LEN>;

void pack_board_command(
  std::uint8_t * dst,
  const switch_interfaces::msg::BoardCommand & cmd);

CommandFrame pack_switch_command(
  const switch_interfaces::msg::SwitchCommand & cmd);

switch_interfaces::msg::BoardStatus unpack_board_status(
  const std::uint8_t * src);

switch_interfaces::msg::SwitchStatus unpack_switch_status(
  const StatusFrame & frame,
  const builtin_interfaces::msg::Time & stamp);

switch_interfaces::msg::SwitchCommand make_idle_command(
  std::uint8_t command_type);

}  // namespace switch_udp_bridge::protocol