// 防止头文件被重复包含
#pragma once

#include <cstdint>
#include <cstring>

// 字节序转换工具函数命名空间，提供小端格式的读写操作
namespace switch_udp_bridge::byte_utils
{

// 将 16 位无符号整数以低位在前、高位在后的顺序写入目标缓冲区两个字节中
inline void write_u16_le(std::uint8_t * dst, std::uint16_t value)
{
  dst[0] = static_cast<std::uint8_t>(value & 0xFF);          // 写入低字节
  dst[1] = static_cast<std::uint8_t>((value >> 8) & 0xFF);   // 写入高字节
}

// 从源缓冲区小端序的两个字节中读取 16 位无符号整数
inline std::uint16_t read_u16_le(const std::uint8_t * src)
{
  return static_cast<std::uint16_t>(
    static_cast<std::uint16_t>(src[0]) |        // 低字节
    static_cast<std::uint16_t>(src[1] << 8));   // 高字节左移 8 位后按位或
}

// 将 32 位无符号整数以低位在前、高位在后的顺序写入目标缓冲区四个字节中
inline void write_u32_le(std::uint8_t * dst, std::uint32_t value)
{
  dst[0] = static_cast<std::uint8_t>(value & 0xFF);          // 字节 0（最低位）
  dst[1] = static_cast<std::uint8_t>((value >> 8) & 0xFF);   // 字节 1
  dst[2] = static_cast<std::uint8_t>((value >> 16) & 0xFF);  // 字节 2
  dst[3] = static_cast<std::uint8_t>((value >> 24) & 0xFF);  // 字节 3（最高位）
}

// 从源缓冲区小端序的四个字节中读取 32 位无符号整数
inline std::uint32_t read_u32_le(const std::uint8_t * src)
{
  return static_cast<std::uint32_t>(
    static_cast<std::uint32_t>(src[0]) |         // 字节 0（最低位）
    static_cast<std::uint32_t>(src[1] << 8) |    // 字节 1 左移 8 位
    static_cast<std::uint32_t>(src[2] << 16) |   // 字节 2 左移 16 位
    static_cast<std::uint32_t>(src[3] << 24));   // 字节 3 左移 24 位（最高位）
}

// 将 IEEE754 单精度浮点数以小端字节序写入目标缓冲区（四个字节）
inline void write_float_le(std::uint8_t * dst, float value)
{
  // 编译期检查，确保 float 类型在当前平台占 4 个字节
  static_assert(sizeof(float) == 4, "float must be 4 bytes");

  std::uint32_t raw = 0;
  // 通过内存拷贝将浮点数的位模式复制到 32 位无符号整数中
  std::memcpy(&raw, &value, sizeof(float));
  // 复用 u32 小端写入函数完成实际字节写入
  write_u32_le(dst, raw);
}

// 从源缓冲区小端序的四个字节中读取 IEEE754 单精度浮点数
inline float read_float_le(const std::uint8_t * src)
{
  // 编译期检查，确保 float 类型在当前平台占 4 个字节
  static_assert(sizeof(float) == 4, "float must be 4 bytes");

  // 先用 u32 小端读取函数获取原始位模式
  const std::uint32_t raw = read_u32_le(src);

  float value = 0.0f;
  // 通过内存拷贝将 32 位位模式还原为浮点数
  std::memcpy(&value, &raw, sizeof(float));
  return value;
}

// 从源缓冲区小端序的两个字节中读取 16 位有符号整数
// 先将其视为无符号 16 位整数读取，再强制转换为有符号类型
inline std::int16_t read_i16_le(const std::uint8_t * src)
{
  return static_cast<std::int16_t>(read_u16_le(src));
}

}  // namespace switch_udp_bridge::byte_utils