// 头文件保护宏，防止重复包含
#pragma once

// 引入 size_t 类型的定义
#include <cstddef>
// 引入 uint16_t 和 uint8_t 类型的定义
#include <cstdint>

// CRC16 计算模块的命名空间
namespace switch_udp_bridge::crc16
{

// 计算 CRC16-Modbus 校验值
// 参数 data：待计算数据的首地址
// 参数 length：待计算数据的字节长度
// 返回值：16 位 CRC 校验值
std::uint16_t calculate(const std::uint8_t * data, std::size_t length);

}  // namespace switch_udp_bridge::crc16