#include "switch_udp_bridge/protocol.hpp"

#include <algorithm>

#include "switch_udp_bridge/byte_utils.hpp"
#include "switch_udp_bridge/crc16.hpp"

namespace switch_udp_bridge::protocol
{

void pack_board_command(
  std::uint8_t * dst,
  const switch_interfaces::msg::BoardCommand & cmd)
{
  std::fill(dst, dst + BOARD_CMD_LEN, 0);

  dst[0] = FRAME_HEAD;
  dst[1] = cmd.command_type;
  dst[2] = cmd.id;

  if (cmd.command_type == CMD_MOTION) {
    byte_utils::write_float_le(&dst[3], cmd.steering_angle);
    byte_utils::write_float_le(&dst[7], cmd.wheel_speed);
    byte_utils::write_float_le(&dst[11], cmd.fan_pressure);
  }

  const std::uint16_t crc =
    crc16::calculate(dst, BOARD_CMD_CRC_CALC_LEN);

  byte_utils::write_u16_le(&dst[BOARD_CMD_CRC_OFFSET], crc);
}

CommandFrame pack_switch_command(
  const switch_interfaces::msg::SwitchCommand & cmd)
{
  CommandFrame frame{};
  frame.fill(0);

  for (std::size_t i = 0; i < 3; ++i) {
    const std::size_t offset = i * BOARD_CMD_LEN;
    pack_board_command(&frame[offset], cmd.boards[i]);
  }

  for (std::size_t i = 0; i < 4; ++i) {
    frame[POWER_CMD_OFFSET + i] = cmd.power_board[i];
  }

  frame[LED_CMD_OFFSET] = cmd.led;

  return frame;
}

switch_interfaces::msg::BoardStatus unpack_board_status(
  const std::uint8_t * src)
{
  switch_interfaces::msg::BoardStatus status;

  status.frame_head = src[0];
  status.command_type = src[1];
  status.id = src[2];

  status.steering_position = byte_utils::read_float_le(&src[3]);
  status.steering_velocity = byte_utils::read_float_le(&src[7]);
  status.steering_torque = byte_utils::read_float_le(&src[11]);

  status.wheel_position = byte_utils::read_float_le(&src[15]);
  status.wheel_velocity = byte_utils::read_float_le(&src[19]);
  status.wheel_torque = byte_utils::read_float_le(&src[23]);

  status.fan_speed = byte_utils::read_float_le(&src[27]);
  status.fan_current = byte_utils::read_float_le(&src[31]);

  status.fan_temperature = byte_utils::read_i16_le(&src[35]);
  status.fan_driver_temperature = byte_utils::read_i16_le(&src[37]);

  status.chamber_pressure = byte_utils::read_float_le(&src[39]);

  status.error_code = src[43];

  status.received_crc =
    byte_utils::read_u16_le(&src[BOARD_STATUS_CRC_OFFSET]);

  status.calculated_crc =
    crc16::calculate(src, BOARD_STATUS_CRC_CALC_LEN);

  status.crc_ok =
    status.received_crc == status.calculated_crc;

  return status;
}

switch_interfaces::msg::SwitchStatus unpack_switch_status(
  const StatusFrame & frame,
  const builtin_interfaces::msg::Time & stamp)
{
  switch_interfaces::msg::SwitchStatus status;

  status.stamp = stamp;

  for (std::size_t i = 0; i < 3; ++i) {
    const std::size_t offset = i * BOARD_STATUS_LEN;
    status.boards[i] = unpack_board_status(&frame[offset]);
  }

  for (std::size_t i = 0; i < 31; ++i) {
    status.power_board[i] = frame[POWER_STATUS_OFFSET + i];
  }

  status.switch_state = frame[SWITCH_STATUS_OFFSET];

  status.raw.assign(frame.begin(), frame.end());

  return status;
}

switch_interfaces::msg::SwitchCommand make_idle_command(
  std::uint8_t command_type)
{
  switch_interfaces::msg::SwitchCommand cmd;

  for (std::size_t i = 0; i < 3; ++i) {
    cmd.boards[i].command_type = command_type;
    cmd.boards[i].id = static_cast<std::uint8_t>(i + 1);
    cmd.boards[i].steering_angle = 0.0f;
    cmd.boards[i].wheel_speed = 0.0f;
    cmd.boards[i].fan_pressure = 0.0f;
  }

  for (std::size_t i = 0; i < 4; ++i) {
    cmd.power_board[i] = 0;
  }

  cmd.led = 0;

  return cmd;
}

}  // namespace switch_udp_bridge::protocol