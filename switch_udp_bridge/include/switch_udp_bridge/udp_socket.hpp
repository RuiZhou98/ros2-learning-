// 头文件保护宏，防止重复包含
#pragma once

// 引入 size_t 类型定义
#include <cstddef>
// 引入 uint8_t 和 uint16_t 类型定义
#include <cstdint>
// 引入互斥锁，用于保护 socket 操作的多线程安全
#include <mutex>
// 引入 optional，用于表示接收操作可能无数据返回
#include <optional>
// 引入 string 类型，用于存储 IP 地址字符串
#include <string>

// 引入 POSIX socket 地址结构体定义
#include <netinet/in.h>

// UDP 桥接功能的命名空间
namespace switch_udp_bridge
{

// UDP Socket 封装类，负责与 STM32 之间的 UDP 通信
class UdpSocket
{
public:
  // 构造函数，创建并绑定 UDP socket
  // 参数 remote_ip：远端（STM32）IP 地址
  // 参数 remote_port：远端（STM32）UDP 端口
  // 参数 local_port：本机 UDP 接收端口
  UdpSocket(
    const std::string & remote_ip,
    std::uint16_t remote_port,
    std::uint16_t local_port);

  // 析构函数，关闭 socket 描述符
  ~UdpSocket();

  // 禁止拷贝构造（socket 资源不可共享）
  UdpSocket(const UdpSocket &) = delete;
  // 禁止拷贝赋值（socket 资源不可共享）
  UdpSocket & operator=(const UdpSocket &) = delete;

  // 发送 UDP 数据到远端（STM32）
  // 参数 data：待发送数据的首地址
  // 参数 length：待发送数据的字节长度
  void send(const std::uint8_t * data, std::size_t length);

  // 从本机端口接收 UDP 数据（非阻塞模式）
  // 参数 buffer：接收缓冲区首地址
  // 参数 buffer_size：接收缓冲区最大容量
  // 返回值：若接收到数据则返回实际接收的字节数，无数据则返回 std::nullopt
  std::optional<std::size_t> receive(
    std::uint8_t * buffer,
    std::size_t buffer_size);

private:
  // socket 文件描述符
  int sock_fd_;
  // 远端（STM32）的 socket 地址结构
  sockaddr_in remote_addr_;
  // 互斥锁，保护 send 和 receive 操作的线程安全
  std::mutex socket_mutex_;
};

}  // namespace switch_udp_bridge