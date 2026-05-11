// 引入 CRC16 计算函数的声明头文件
#include "switch_udp_bridge/crc16.hpp"

// CRC16 计算模块的命名空间
namespace switch_udp_bridge::crc16
{

// 计算 CRC16-Modbus 校验值
// 参数 data：待计算数据的首地址
// 参数 length：待计算数据的字节长度
// 返回值：16 位 CRC 校验值（小端字节序，低字节在前）
std::uint16_t calculate(const std::uint8_t * data, std::size_t length)
{
  // CRC 初始值，Modbus 协议规定为 0xFFFF
  std::uint16_t crc = 0xFFFF;

  // 逐字节处理输入数据
  for (std::size_t i = 0; i < length; ++i) {
    // 将当前字节与 CRC 低字节异或
    crc ^= data[i];

    // 对每个字节的 8 位逐位处理
    for (std::size_t j = 0; j < 8; ++j) {
      // 检查 CRC 最低位是否为 1
      if ((crc & 0x0001) != 0) {
        // 若最低位为 1，右移一位并与多项式 0xA001 异或
        // 0xA001 是 Modbus 多项式 0x8005 的位反转形式
        crc = static_cast<std::uint16_t>((crc >> 1) ^ 0xA001);
      } else {
        // 若最低位为 0，仅右移一位
        crc = static_cast<std::uint16_t>(crc >> 1);
      }
    }
  }

  // 返回计算得到的 CRC16 校验值
  return crc;
}

}  // namespace switch_udp_bridge::crc16