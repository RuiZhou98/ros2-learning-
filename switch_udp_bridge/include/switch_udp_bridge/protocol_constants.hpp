// 头文件保护宏，防止重复包含
#pragma once

// 引入 size_t 类型的定义
#include <cstddef>
// 引入 uint8_t 类型的定义
#include <cstdint>

// 通信协议常量定义的命名空间
namespace switch_udp_bridge::protocol
{

// ROS2 发送给 STM32 的命令帧总长度，单位：字节
constexpr std::size_t ETHERNET_RECEIVE_LEN = 56;
// STM32 返回给 ROS2 的状态帧总长度，单位：字节
constexpr std::size_t ETHERNET_SEND_LEN = 170;

// 单个通信板命令帧的长度，单位：字节
constexpr std::size_t BOARD_CMD_LEN = 17;
// 单个通信板状态帧的长度，单位：字节
constexpr std::size_t BOARD_STATUS_LEN = 46;

// 帧头标识，固定为 0xFE，用于帧同步
constexpr std::uint8_t FRAME_HEAD = 0xFE;

// 单个通信板命令帧中 CRC16 字段的起始字节偏移（从 0 开始计数）
constexpr std::size_t BOARD_CMD_CRC_OFFSET = 15;
// 单个通信板命令帧中参与 CRC 计算的字节数（字节 0 到 14）
constexpr std::size_t BOARD_CMD_CRC_CALC_LEN = 15;

// 单个通信板状态帧中 CRC16 字段的起始字节偏移（从 0 开始计数）
constexpr std::size_t BOARD_STATUS_CRC_OFFSET = 44;
// 单个通信板状态帧中参与 CRC 计算的字节数（字节 0 到 43）
constexpr std::size_t BOARD_STATUS_CRC_CALC_LEN = 44;

// 电源板命令在 56 字节命令帧中的起始偏移位置（从 0 开始计数）
constexpr std::size_t POWER_CMD_OFFSET = 51;
// 灯带命令在 56 字节命令帧中的起始偏移位置（从 0 开始计数）
constexpr std::size_t LED_CMD_OFFSET = 55;

// 电源板状态在 170 字节状态帧中的起始偏移位置（从 0 开始计数）
constexpr std::size_t POWER_STATUS_OFFSET = 138;
// 开关状态在 170 字节状态帧中的起始偏移位置（从 0 开始计数）
constexpr std::size_t SWITCH_STATUS_OFFSET = 169;

// 运动命令，控制运动、风机等
constexpr std::uint8_t CMD_MOTION = 0x01;
// 读取状态命令，只读取状态不控制运动
constexpr std::uint8_t CMD_READ_STATUS = 0x02;
// 归零命令，使舵轮归零
constexpr std::uint8_t CMD_ZERO = 0x03;

}  // namespace switch_udp_bridge::protocol