#pragma once

#include <cstdint>
#include <cstring>

namespace switch_udp_bridge::byte_utils
{

inline void write_u16_le(std::uint8_t * dst, std::uint16_t value)
{
  dst[0] = static_cast<std::uint8_t>(value & 0xFF);
  dst[1] = static_cast<std::uint8_t>((value >> 8) & 0xFF);
}

inline std::uint16_t read_u16_le(const std::uint8_t * src)
{
  return static_cast<std::uint16_t>(
    static_cast<std::uint16_t>(src[0]) |
    static_cast<std::uint16_t>(src[1] << 8));
}

inline void write_u32_le(std::uint8_t * dst, std::uint32_t value)
{
  dst[0] = static_cast<std::uint8_t>(value & 0xFF);
  dst[1] = static_cast<std::uint8_t>((value >> 8) & 0xFF);
  dst[2] = static_cast<std::uint8_t>((value >> 16) & 0xFF);
  dst[3] = static_cast<std::uint8_t>((value >> 24) & 0xFF);
}

inline std::uint32_t read_u32_le(const std::uint8_t * src)
{
  return static_cast<std::uint32_t>(
    static_cast<std::uint32_t>(src[0]) |
    static_cast<std::uint32_t>(src[1] << 8) |
    static_cast<std::uint32_t>(src[2] << 16) |
    static_cast<std::uint32_t>(src[3] << 24));
}

inline void write_float_le(std::uint8_t * dst, float value)
{
  static_assert(sizeof(float) == 4, "float must be 4 bytes");

  std::uint32_t raw = 0;
  std::memcpy(&raw, &value, sizeof(float));
  write_u32_le(dst, raw);
}

inline float read_float_le(const std::uint8_t * src)
{
  static_assert(sizeof(float) == 4, "float must be 4 bytes");

  const std::uint32_t raw = read_u32_le(src);

  float value = 0.0f;
  std::memcpy(&value, &raw, sizeof(float));
  return value;
}

inline std::int16_t read_i16_le(const std::uint8_t * src)
{
  return static_cast<std::int16_t>(read_u16_le(src));
}

}  // namespace switch_udp_bridge::byte_utils