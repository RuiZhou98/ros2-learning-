#pragma once

#include <cstddef>
#include <cstdint>
#include <mutex>
#include <optional>
#include <string>

#include <netinet/in.h>

namespace switch_udp_bridge
{

class UdpSocket
{
public:
  UdpSocket(
    const std::string & remote_ip,
    std::uint16_t remote_port,
    std::uint16_t local_port);

  ~UdpSocket();

  UdpSocket(const UdpSocket &) = delete;
  UdpSocket & operator=(const UdpSocket &) = delete;

  void send(const std::uint8_t * data, std::size_t length);

  std::optional<std::size_t> receive(
    std::uint8_t * buffer,
    std::size_t buffer_size);

private:
  int sock_fd_;
  sockaddr_in remote_addr_;
  std::mutex socket_mutex_;
};

}  // namespace switch_udp_bridge