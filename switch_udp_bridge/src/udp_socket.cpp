#include "switch_udp_bridge/udp_socket.hpp"

#include <cerrno>
#include <cstring>
#include <stdexcept>
#include <string>

#include <arpa/inet.h>
#include <fcntl.h>
#include <sys/socket.h>
#include <unistd.h>

namespace switch_udp_bridge
{

UdpSocket::UdpSocket(
  const std::string & remote_ip,
  std::uint16_t remote_port,
  std::uint16_t local_port)
: sock_fd_(-1),
  remote_addr_{}
{
  sock_fd_ = ::socket(AF_INET, SOCK_DGRAM, 0);
  if (sock_fd_ < 0) {
    throw std::runtime_error("failed to create UDP socket");
  }

  int reuse = 1;
  if (::setsockopt(sock_fd_, SOL_SOCKET, SO_REUSEADDR, &reuse, sizeof(reuse)) < 0) {
    ::close(sock_fd_);
    throw std::runtime_error("failed to set SO_REUSEADDR");
  }

  sockaddr_in local_addr{};
  local_addr.sin_family = AF_INET;
  local_addr.sin_addr.s_addr = htonl(INADDR_ANY);
  local_addr.sin_port = htons(local_port);

  if (::bind(sock_fd_, reinterpret_cast<sockaddr *>(&local_addr), sizeof(local_addr)) < 0) {
    ::close(sock_fd_);
    throw std::runtime_error("failed to bind UDP socket");
  }

  const int flags = ::fcntl(sock_fd_, F_GETFL, 0);
  if (flags < 0) {
    ::close(sock_fd_);
    throw std::runtime_error("failed to get socket flags");
  }

  if (::fcntl(sock_fd_, F_SETFL, flags | O_NONBLOCK) < 0) {
    ::close(sock_fd_);
    throw std::runtime_error("failed to set socket non-blocking");
  }

  remote_addr_.sin_family = AF_INET;
  remote_addr_.sin_port = htons(remote_port);

  if (::inet_pton(AF_INET, remote_ip.c_str(), &remote_addr_.sin_addr) != 1) {
    ::close(sock_fd_);
    throw std::runtime_error("invalid remote IP address: " + remote_ip);
  }
}

UdpSocket::~UdpSocket()
{
  if (sock_fd_ >= 0) {
    ::close(sock_fd_);
    sock_fd_ = -1;
  }
}

void UdpSocket::send(const std::uint8_t * data, std::size_t length)
{
  std::lock_guard<std::mutex> lock(socket_mutex_);

  const ssize_t sent = ::sendto(
    sock_fd_,
    data,
    length,
    0,
    reinterpret_cast<sockaddr *>(&remote_addr_),
    sizeof(remote_addr_));

  if (sent < 0) {
    throw std::runtime_error("failed to send UDP packet");
  }

  if (static_cast<std::size_t>(sent) != length) {
    throw std::runtime_error("UDP packet length mismatch while sending");
  }
}

std::optional<std::size_t> UdpSocket::receive(
  std::uint8_t * buffer,
  std::size_t buffer_size)
{
  std::lock_guard<std::mutex> lock(socket_mutex_);

  sockaddr_in sender_addr{};
  socklen_t sender_len = sizeof(sender_addr);

  const ssize_t received = ::recvfrom(
    sock_fd_,
    buffer,
    buffer_size,
    0,
    reinterpret_cast<sockaddr *>(&sender_addr),
    &sender_len);

  if (received < 0) {
    if (errno == EAGAIN || errno == EWOULDBLOCK) {
      return std::nullopt;
    }

    throw std::runtime_error(
      std::string("failed to receive UDP packet: ") + std::strerror(errno));
  }

  return static_cast<std::size_t>(received);
}

}  // namespace switch_udp_bridge