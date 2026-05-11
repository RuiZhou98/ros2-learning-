// 引入 UDP Socket 封装类的头文件
#include "switch_udp_bridge/udp_socket.hpp"

// 引入 errno 错误码定义，用于判断非阻塞模式下的无数据状态
#include <cerrno>
// 引入 cstring，用于 strerror 将错误码转为字符串
#include <cstring>
// 引入标准异常类，用于抛出运行时错误
#include <stdexcept>
// 引入 string 类型
#include <string>

// 引入网络地址转换函数（inet_pton, htons, htonl）
#include <arpa/inet.h>
// 引入文件控制函数（fcntl），用于设置非阻塞模式
#include <fcntl.h>
// 引入 socket API（socket, bind, sendto, recvfrom 等）
#include <sys/socket.h>
// 引入 close 函数，用于关闭 socket 描述符
#include <unistd.h>

// UDP 桥接功能的命名空间
namespace switch_udp_bridge
{

// UdpSocket 构造函数
// 创建 UDP socket，绑定本机端口，配置远端地址，并设置为非阻塞模式
// 参数 remote_ip：远端（STM32）IP 地址
// 参数 remote_port：远端（STM32）UDP 端口
// 参数 local_port：本机 UDP 接收端口
UdpSocket::UdpSocket(
  const std::string & remote_ip,
  std::uint16_t remote_port,
  std::uint16_t local_port)
// 初始化 socket 描述符为 -1，表示尚未创建
: sock_fd_(-1),
  // 将远端地址结构体清零
  remote_addr_{}
{
  // 创建 UDP socket（AF_INET：IPv4，SOCK_DGRAM：UDP 数据报）
  sock_fd_ = ::socket(AF_INET, SOCK_DGRAM, 0);
  if (sock_fd_ < 0) {
    // 创建失败则抛出异常
    throw std::runtime_error("failed to create UDP socket");
  }

  // 设置 SO_REUSEADDR 选项，允许重复绑定端口（避免重启时端口被占用）
  int reuse = 1;
  if (::setsockopt(sock_fd_, SOL_SOCKET, SO_REUSEADDR, &reuse, sizeof(reuse)) < 0) {
    // 设置失败则关闭 socket 并抛出异常
    ::close(sock_fd_);
    throw std::runtime_error("failed to set SO_REUSEADDR");
  }

  // 配置本机监听地址结构
  sockaddr_in local_addr{};
  // 地址族：IPv4
  local_addr.sin_family = AF_INET;
  // 监听所有网卡接口（INADDR_ANY），使用网络字节序
  local_addr.sin_addr.s_addr = htonl(INADDR_ANY);
  // 端口号，转换为网络字节序
  local_addr.sin_port = htons(local_port);

  // 将 socket 绑定到本机地址和端口
  if (::bind(sock_fd_, reinterpret_cast<sockaddr *>(&local_addr), sizeof(local_addr)) < 0) {
    // 绑定失败则关闭 socket 并抛出异常
    ::close(sock_fd_);
    throw std::runtime_error("failed to bind UDP socket");
  }

  // 获取当前 socket 的文件状态标志
  const int flags = ::fcntl(sock_fd_, F_GETFL, 0);
  if (flags < 0) {
    // 获取失败则关闭 socket 并抛出异常
    ::close(sock_fd_);
    throw std::runtime_error("failed to get socket flags");
  }

  // 设置 socket 为非阻塞模式（O_NONBLOCK），recvfrom 无数据时立即返回而不阻塞
  if (::fcntl(sock_fd_, F_SETFL, flags | O_NONBLOCK) < 0) {
    // 设置失败则关闭 socket 并抛出异常
    ::close(sock_fd_);
    throw std::runtime_error("failed to set socket non-blocking");
  }

  // 配置远端地址结构（STM32）
  // 地址族：IPv4
  remote_addr_.sin_family = AF_INET;
  // 远端端口号，转换为网络字节序
  remote_addr_.sin_port = htons(remote_port);

  // 将远端 IP 字符串转换为网络地址结构的二进制格式
  if (::inet_pton(AF_INET, remote_ip.c_str(), &remote_addr_.sin_addr) != 1) {
    // 转换失败（IP 格式非法）则关闭 socket 并抛出异常
    ::close(sock_fd_);
    throw std::runtime_error("invalid remote IP address: " + remote_ip);
  }
}

// UdpSocket 析构函数
// 关闭 socket 描述符，释放系统资源
UdpSocket::~UdpSocket()
{
  // 检查 socket 描述符是否有效
  if (sock_fd_ >= 0) {
    // 关闭 socket
    ::close(sock_fd_);
    // 将描述符置为 -1，防止重复关闭
    sock_fd_ = -1;
  }
}

// 发送 UDP 数据到远端（STM32）
// 参数 data：待发送数据的首地址
// 参数 length：待发送数据的字节长度
void UdpSocket::send(const std::uint8_t * data, std::size_t length)
{
  // 上锁保护，确保多线程环境下发送操作的原子性
  std::lock_guard<std::mutex> lock(socket_mutex_);

  // 调用 sendto 向远端地址发送数据报
  const ssize_t sent = ::sendto(
    sock_fd_,
    data,
    length,
    0,
    reinterpret_cast<sockaddr *>(&remote_addr_),
    sizeof(remote_addr_));

  // 若发送返回值小于 0，说明发送失败
  if (sent < 0) {
    throw std::runtime_error("failed to send UDP packet");
  }

  // 检查实际发送的字节数是否与期望长度一致
  if (static_cast<std::size_t>(sent) != length) {
    throw std::runtime_error("UDP packet length mismatch while sending");
  }
}

// 从本机端口接收 UDP 数据（非阻塞模式）
// 参数 buffer：接收缓冲区首地址
// 参数 buffer_size：接收缓冲区最大容量
// 返回值：若接收到数据则返回实际接收的字节数，无数据则返回 std::nullopt
std::optional<std::size_t> UdpSocket::receive(
  std::uint8_t * buffer,
  std::size_t buffer_size)
{
  // 上锁保护，确保多线程环境下接收操作的原子性
  std::lock_guard<std::mutex> lock(socket_mutex_);

  // 发送方地址结构（本项目中仅用于接收，不检查发送方地址）
  sockaddr_in sender_addr{};
  // 发送方地址结构长度
  socklen_t sender_len = sizeof(sender_addr);

  // 调用 recvfrom 从 socket 接收数据
  const ssize_t received = ::recvfrom(
    sock_fd_,
    buffer,
    buffer_size,
    0,
    reinterpret_cast<sockaddr *>(&sender_addr),
    &sender_len);

  // 若接收返回值小于 0，说明发生错误
  if (received < 0) {
    // 若是 EAGAIN 或 EWOULDBLOCK，表示当前无数据可读（非阻塞模式正常行为）
    if (errno == EAGAIN || errno == EWOULDBLOCK) {
      // 返回空值，表示无数据
      return std::nullopt;
    }

    // 其他错误码，抛出异常
    throw std::runtime_error(
      std::string("failed to receive UDP packet: ") + std::strerror(errno));
  }

  // 成功接收到数据，返回实际接收的字节数
  return static_cast<std::size_t>(received);
}

}  // namespace switch_udp_bridge