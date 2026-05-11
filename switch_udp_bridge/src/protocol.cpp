// 引入协议处理模块的头文件
#include "switch_udp_bridge/protocol.hpp"

// 引入 algorithm 库，用于 std::fill 等算法
#include <algorithm>

// 引入字节序转换工具函数
#include "switch_udp_bridge/byte_utils.hpp"
// 引入 CRC16 计算函数
#include "switch_udp_bridge/crc16.hpp"

// 通信协议处理模块的命名空间
namespace switch_udp_bridge::protocol
{

// 打包单个通信板命令帧到目标缓冲区
// 参数 dst：目标缓冲区首地址，至少需要 BOARD_CMD_LEN（17）字节空间
// 参数 cmd：要打包的通信板命令消息
void pack_board_command(
  std::uint8_t * dst,
  const switch_interfaces::msg::BoardCommand & cmd)
{
  // 将目标缓冲区清零，确保未使用的字段为 0
  std::fill(dst, dst + BOARD_CMD_LEN, 0);

  // 字节 0：帧头，固定填 0xFE
  dst[0] = FRAME_HEAD;
  // 字节 1：命令类型
  dst[1] = cmd.command_type;
  // 字节 2：通信板 ID
  dst[2] = cmd.id;

  // 仅在运动命令时填充运动参数，其他命令类型保持为零
  if (cmd.command_type == CMD_MOTION) {
    // 字节 3-6：舵轮目标角度，float32 小端格式
    byte_utils::write_float_le(&dst[3], cmd.steering_angle);
    // 字节 7-10：前进轮目标速度，float32 小端格式
    byte_utils::write_float_le(&dst[7], cmd.wheel_speed);
    // 字节 11-14：风箱目标气压，float32 小端格式
    byte_utils::write_float_le(&dst[11], cmd.fan_pressure);
  }

  // 对字节 0-14 计算 CRC16-Modbus 校验值
  const std::uint16_t crc =
    crc16::calculate(dst, BOARD_CMD_CRC_CALC_LEN);

  // 将 CRC 值填入字节 15-16，小端格式
  byte_utils::write_u16_le(&dst[BOARD_CMD_CRC_OFFSET], crc);
}

// 将完整的 SwitchCommand 消息打包为 56 字节命令帧
// 参数 cmd：完整的控制命令消息
// 返回值：56 字节命令帧数组
CommandFrame pack_switch_command(
  const switch_interfaces::msg::SwitchCommand & cmd)
{
  // 创建 56 字节数组并全部清零
  CommandFrame frame{};
  frame.fill(0);

  // 逐板打包 3 个通信板命令，每板占 17 字节
  for (std::size_t i = 0; i < 3; ++i) {
    // 计算当前通信板命令在帧中的起始偏移位置
    const std::size_t offset = i * BOARD_CMD_LEN;
    // 将单个通信板命令打包到对应位置
    pack_board_command(&frame[offset], cmd.boards[i]);
  }

  // 填充电源板命令，位于字节 51-54
  for (std::size_t i = 0; i < 4; ++i) {
    frame[POWER_CMD_OFFSET + i] = cmd.power_board[i];
  }

  // 填充灯带命令，位于字节 55
  frame[LED_CMD_OFFSET] = cmd.led;

  return frame;
}

// 解析单个通信板状态帧
// 参数 src：46 字节状态帧数据首地址
// 返回值：解析后的 BoardStatus 消息
switch_interfaces::msg::BoardStatus unpack_board_status(
  const std::uint8_t * src)
{
  // 创建待返回的状态消息对象
  switch_interfaces::msg::BoardStatus status;

  // 字节 0：帧头
  status.frame_head = src[0];
  // 字节 1：命令类型（回显）
  status.command_type = src[1];
  // 字节 2：通信板 ID
  status.id = src[2];

  // 字节 3-6：舵轮位置，float32 小端格式
  status.steering_position = byte_utils::read_float_le(&src[3]);
  // 字节 7-10：舵轮速度，float32 小端格式
  status.steering_velocity = byte_utils::read_float_le(&src[7]);
  // 字节 11-14：舵轮力矩，float32 小端格式
  status.steering_torque = byte_utils::read_float_le(&src[11]);

  // 字节 15-18：前进轮位置，float32 小端格式
  status.wheel_position = byte_utils::read_float_le(&src[15]);
  // 字节 19-22：前进轮速度，float32 小端格式
  status.wheel_velocity = byte_utils::read_float_le(&src[19]);
  // 字节 23-26：前进轮力矩，float32 小端格式
  status.wheel_torque = byte_utils::read_float_le(&src[23]);

  // 字节 27-30：风机转速，float32 小端格式
  status.fan_speed = byte_utils::read_float_le(&src[27]);
  // 字节 31-34：风机电流，float32 小端格式
  status.fan_current = byte_utils::read_float_le(&src[31]);

  // 字节 35-36：风机温度，int16 小端格式
  status.fan_temperature = byte_utils::read_i16_le(&src[35]);
  // 字节 37-38：风机驱动温度，int16 小端格式
  status.fan_driver_temperature = byte_utils::read_i16_le(&src[37]);

  // 字节 39-42：吸附腔气压，float32 小端格式
  status.chamber_pressure = byte_utils::read_float_le(&src[39]);

  // 字节 43：错误码
  status.error_code = src[43];

  // 字节 44-45：从帧中读取的 CRC16 校验值
  status.received_crc =
    byte_utils::read_u16_le(&src[BOARD_STATUS_CRC_OFFSET]);

  // 对字节 0-43 本地重新计算 CRC16 校验值
  status.calculated_crc =
    crc16::calculate(src, BOARD_STATUS_CRC_CALC_LEN);

  // 比较接收 CRC 与本地计算 CRC，判断数据完整性
  status.crc_ok =
    status.received_crc == status.calculated_crc;

  return status;
}

// 解析完整的 170 字节状态帧
// 参数 frame：170 字节状态帧数组
// 参数 stamp：接收时刻的时间戳
// 返回值：解析后的 SwitchStatus 消息
switch_interfaces::msg::SwitchStatus unpack_switch_status(
  const StatusFrame & frame,
  const builtin_interfaces::msg::Time & stamp)
{
  // 创建待返回的状态消息对象
  switch_interfaces::msg::SwitchStatus status;

  // 记录接收时刻的时间戳
  status.stamp = stamp;

  // 逐板解析 3 个通信板状态，每板占 46 字节
  for (std::size_t i = 0; i < 3; ++i) {
    // 计算当前通信板状态在帧中的起始偏移位置
    const std::size_t offset = i * BOARD_STATUS_LEN;
    // 将 46 字节数据解析为 BoardStatus 消息
    status.boards[i] = unpack_board_status(&frame[offset]);
  }

  // 拷贝电源板状态数据，位于字节 138-168
  for (std::size_t i = 0; i < 31; ++i) {
    status.power_board[i] = frame[POWER_STATUS_OFFSET + i];
  }

  // 拷贝开关状态，位于字节 169
  status.switch_state = frame[SWITCH_STATUS_OFFSET];

  // 保存完整的原始 170 字节数据到 raw 字段
  status.raw.assign(frame.begin(), frame.end());

  return status;
}

// 生成一个安全的空闲命令（所有运动参数置零）
// 参数 command_type：命令类型（通常为超时保护命令类型）
// 返回值：所有运动参数为零的安全命令
switch_interfaces::msg::SwitchCommand make_idle_command(
  std::uint8_t command_type)
{
  // 创建待返回的命令消息对象
  switch_interfaces::msg::SwitchCommand cmd;

  // 配置 3 个通信板为安全状态
  for (std::size_t i = 0; i < 3; ++i) {
    // 设置指定的命令类型
    cmd.boards[i].command_type = command_type;
    // ID 从 1 开始，与通信板编号对应
    cmd.boards[i].id = static_cast<std::uint8_t>(i + 1);
    // 舵轮角度置零
    cmd.boards[i].steering_angle = 0.0f;
    // 前进轮速度置零
    cmd.boards[i].wheel_speed = 0.0f;
    // 风箱气压置零
    cmd.boards[i].fan_pressure = 0.0f;
  }

  // 电源板命令全部置零
  for (std::size_t i = 0; i < 4; ++i) {
    cmd.power_board[i] = 0;
  }

  // 灯带命令置零
  cmd.led = 0;

  return cmd;
}

}  // namespace switch_udp_bridge::protocol