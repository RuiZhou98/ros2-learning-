#include <array>
#include <cerrno>
#include <chrono>
#include <cstdint>
#include <cstring>
#include <iostream>
#include <stdexcept>
#include <string>

#include <arpa/inet.h>
#include <fcntl.h>
#include <netinet/in.h>
#include <sys/socket.h>
#include <unistd.h>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/u_int8_multi_array.hpp"

#include "switch_interfaces/msg/switch_command.hpp"
#include "switch_interfaces/msg/switch_status.hpp"

using namespace std::chrono_literals;

namespace protocol
{

constexpr size_t ETHERNET_RECEIVE_LEN = 56;
constexpr size_t ETHERNET_SEND_LEN = 170;

constexpr size_t BOARD_CMD_LEN = 17;
constexpr size_t BOARD_STATUS_LEN = 46;

constexpr uint8_t FRAME_HEAD = 0xFE;

constexpr size_t BOARD_CMD_CRC_OFFSET = 15;
constexpr size_t BOARD_CMD_CRC_CALC_LEN = 15;

constexpr size_t BOARD_STATUS_CRC_OFFSET = 44;
constexpr size_t BOARD_STATUS_CRC_CALC_LEN = 44;

constexpr size_t POWER_CMD_OFFSET = 51;
constexpr size_t LED_CMD_OFFSET = 55;

constexpr size_t POWER_STATUS_OFFSET = 138;
constexpr size_t SWITCH_STATUS_OFFSET = 169;

// 这里严格按照你给的 STM32 CRC 代码写
uint16_t CalculateCRC(const uint8_t *data, uint16_t length)
{
  uint16_t crc = 0xFFFF;
  uint16_t i;
  uint16_t j;

  for (i = 0; i < length; i++) {
    crc ^= data[i];

    for (j = 0; j < 8; j++) {
      if (crc & 0x0001) {
        crc = static_cast<uint16_t>((crc >> 1) ^ 0xA001);
      } else {
        crc = static_cast<uint16_t>(crc >> 1);
      }
    }
  }

  return crc;
}

// 小端写 uint16
// 如果 STM32 端是直接把 uint16_t crc 拷贝到数组[15],[16]，
// 在常见 STM32 小端系统上就是低字节在前，高字节在后。
void write_u16_le(uint8_t *dst, uint16_t value)
{
  dst[0] = static_cast<uint8_t>(value & 0xFF);
  dst[1] = static_cast<uint8_t>((value >> 8) & 0xFF);
}

uint16_t read_u16_le(const uint8_t *src)
{
  return static_cast<uint16_t>(src[0]) |
         static_cast<uint16_t>(src[1] << 8);
}

void write_u32_le(uint8_t *dst, uint32_t value)
{
  dst[0] = static_cast<uint8_t>(value & 0xFF);
  dst[1] = static_cast<uint8_t>((value >> 8) & 0xFF);
  dst[2] = static_cast<uint8_t>((value >> 16) & 0xFF);
  dst[3] = static_cast<uint8_t>((value >> 24) & 0xFF);
}

uint32_t read_u32_le(const uint8_t *src)
{
  return static_cast<uint32_t>(src[0]) |
         static_cast<uint32_t>(src[1] << 8) |
         static_cast<uint32_t>(src[2] << 16) |
         static_cast<uint32_t>(src[3] << 24);
}

void write_float_le(uint8_t *dst, float value)
{
  uint32_t raw = 0;
  static_assert(sizeof(float) == 4, "float must be 4 bytes");
  std::memcpy(&raw, &value, sizeof(float));
  write_u32_le(dst, raw);
}

float read_float_le(const uint8_t *src)
{
  uint32_t raw = read_u32_le(src);
  float value = 0.0f;
  std::memcpy(&value, &raw, sizeof(float));
  return value;
}

int16_t read_i16_le(const uint8_t *src)
{
  uint16_t raw = read_u16_le(src);
  return static_cast<int16_t>(raw);
}

// 打包单个 17 字节通信板命令帧
void pack_board_command(
  uint8_t *dst,
  const switch_interfaces::msg::BoardCommand &cmd)
{
  for (size_t i = 0; i < BOARD_CMD_LEN; ++i) {
    dst[i] = 0;
  }

  dst[0] = FRAME_HEAD;
  dst[1] = cmd.command_type;
  dst[2] = cmd.id;

  // 0x01 运动命令：写 3 个 float
  // 0x02 读取状态命令：3-14 字节作为占位
  // 0x03 归零命令：3-14 字节作为占位
  if (cmd.command_type == 0x01) {
    write_float_le(&dst[3], cmd.steering_angle);
    write_float_le(&dst[7], cmd.wheel_speed);
    write_float_le(&dst[11], cmd.fan_pressure);
  }

  const uint16_t crc = CalculateCRC(dst, BOARD_CMD_CRC_CALC_LEN);
  write_u16_le(&dst[BOARD_CMD_CRC_OFFSET], crc);
}

// 打包 56 字节上层命令帧
std::array<uint8_t, ETHERNET_RECEIVE_LEN> pack_switch_command(
  const switch_interfaces::msg::SwitchCommand &cmd)
{
  std::array<uint8_t, ETHERNET_RECEIVE_LEN> frame{};
  frame.fill(0);

  pack_board_command(&frame[0], cmd.boards[0]);
  pack_board_command(&frame[17], cmd.boards[1]);
  pack_board_command(&frame[34], cmd.boards[2]);

  for (size_t i = 0; i < 4; ++i) {
    frame[POWER_CMD_OFFSET + i] = cmd.power_board[i];
  }

  frame[LED_CMD_OFFSET] = cmd.led;

  return frame;
}

// 解包单个 46 字节通信板状态帧
switch_interfaces::msg::BoardStatus unpack_board_status(const uint8_t *src)
{
  switch_interfaces::msg::BoardStatus status;

  status.frame_head = src[0];
  status.command_type = src[1];
  status.id = src[2];

  status.steering_position = read_float_le(&src[3]);
  status.steering_velocity = read_float_le(&src[7]);
  status.steering_torque = read_float_le(&src[11]);

  status.wheel_position = read_float_le(&src[15]);
  status.wheel_velocity = read_float_le(&src[19]);
  status.wheel_torque = read_float_le(&src[23]);

  status.fan_speed = read_float_le(&src[27]);
  status.fan_current = read_float_le(&src[31]);

  status.fan_temperature = read_i16_le(&src[35]);
  status.fan_driver_temperature = read_i16_le(&src[37]);

  status.chamber_pressure = read_float_le(&src[39]);

  status.error_code = src[43];

  status.received_crc = read_u16_le(&src[44]);
  status.calculated_crc = CalculateCRC(src, BOARD_STATUS_CRC_CALC_LEN);
  status.crc_ok = status.received_crc == status.calculated_crc;

  return status;
}

// 解包 170 字节上层状态帧
switch_interfaces::msg::SwitchStatus unpack_switch_status(
  const std::array<uint8_t, ETHERNET_SEND_LEN> &frame,
  const rclcpp::Time &stamp)
{
  switch_interfaces::msg::SwitchStatus status;

  status.stamp = stamp;

  status.boards[0] = unpack_board_status(&frame[0]);
  status.boards[1] = unpack_board_status(&frame[46]);
  status.boards[2] = unpack_board_status(&frame[92]);

  for (size_t i = 0; i < 31; ++i) {
    status.power_board[i] = frame[POWER_STATUS_OFFSET + i];
  }

  status.switch_state = frame[SWITCH_STATUS_OFFSET];

  status.raw.assign(frame.begin(), frame.end());

  return status;
}

}  // namespace protocol

class UdpBridgeNode : public rclcpp::Node
{
public:
  UdpBridgeNode()
  : Node("udp_bridge_node")
  {
    stm32_ip_ = this->declare_parameter<std::string>("stm32_ip", "192.168.1.10");
    stm32_port_ = this->declare_parameter<int>("stm32_port", 50000);
    local_port_ = this->declare_parameter<int>("local_port", 50001);

    command_timeout_ms_ = this->declare_parameter<double>("command_timeout_ms", 50.0);
    send_rate_hz_ = this->declare_parameter<double>("send_rate_hz", 200.0);

    setup_socket();

    command_sub_ = this->create_subscription<switch_interfaces::msg::SwitchCommand>(
      "/switch/command",
      rclcpp::QoS(1),
      std::bind(&UdpBridgeNode::command_callback, this, std::placeholders::_1));

    status_pub_ = this->create_publisher<switch_interfaces::msg::SwitchStatus>(
      "/switch/status",
      rclcpp::QoS(10));

    raw_status_pub_ = this->create_publisher<std_msgs::msg::UInt8MultiArray>(
      "/switch/status_raw",
      rclcpp::QoS(10));

    const auto send_period = std::chrono::duration<double>(1.0 / send_rate_hz_);

    tx_timer_ = this->create_wall_timer(
      std::chrono::duration_cast<std::chrono::milliseconds>(send_period),
      std::bind(&UdpBridgeNode::tx_timer_callback, this));

    rx_timer_ = this->create_wall_timer(
      1ms,
      std::bind(&UdpBridgeNode::rx_timer_callback, this));

    RCLCPP_INFO(
      this->get_logger(),
      "UDP bridge started. local_port=%d, stm32=%s:%d, send_rate=%.1f Hz",
      local_port_,
      stm32_ip_.c_str(),
      stm32_port_,
      send_rate_hz_);
  }

  ~UdpBridgeNode() override
  {
    if (sock_fd_ >= 0) {
      close(sock_fd_);
    }
  }

private:
  void setup_socket()
  {
    sock_fd_ = socket(AF_INET, SOCK_DGRAM, 0);

    if (sock_fd_ < 0) {
      throw std::runtime_error("failed to create UDP socket");
    }

    int reuse = 1;
    setsockopt(sock_fd_, SOL_SOCKET, SO_REUSEADDR, &reuse, sizeof(reuse));

    sockaddr_in local_addr{};
    local_addr.sin_family = AF_INET;
    local_addr.sin_addr.s_addr = htonl(INADDR_ANY);
    local_addr.sin_port = htons(static_cast<uint16_t>(local_port_));

    if (bind(sock_fd_, reinterpret_cast<sockaddr *>(&local_addr), sizeof(local_addr)) < 0) {
      throw std::runtime_error("failed to bind UDP socket");
    }

    int flags = fcntl(sock_fd_, F_GETFL, 0);

    if (flags < 0) {
      throw std::runtime_error("failed to get socket flags");
    }

    if (fcntl(sock_fd_, F_SETFL, flags | O_NONBLOCK) < 0) {
      throw std::runtime_error("failed to set non-blocking socket");
    }

    std::memset(&stm32_addr_, 0, sizeof(stm32_addr_));
    stm32_addr_.sin_family = AF_INET;
    stm32_addr_.sin_port = htons(static_cast<uint16_t>(stm32_port_));

    if (inet_pton(AF_INET, stm32_ip_.c_str(), &stm32_addr_.sin_addr) != 1) {
      throw std::runtime_error("invalid stm32_ip");
    }
  }

  void command_callback(const switch_interfaces::msg::SwitchCommand::SharedPtr msg)
  {
    latest_command_ = *msg;
    last_command_time_ = this->now();
    has_command_ = true;
  }

  void tx_timer_callback()
  {
    if (!has_command_) {
      return;
    }

    const double age_ms = (this->now() - last_command_time_).seconds() * 1000.0;

    if (age_ms > command_timeout_ms_) {
      return;
    }

    const auto frame = protocol::pack_switch_command(latest_command_);

    const ssize_t sent = sendto(
      sock_fd_,
      frame.data(),
      frame.size(),
      0,
      reinterpret_cast<sockaddr *>(&stm32_addr_),
      sizeof(stm32_addr_));

    if (sent != static_cast<ssize_t>(frame.size())) {
      RCLCPP_WARN_THROTTLE(
        this->get_logger(),
        *this->get_clock(),
        1000,
        "UDP send length error. sent=%zd, expected=%zu",
        sent,
        frame.size());
    }
  }

  void rx_timer_callback()
  {
    while (true) {
      uint8_t buffer[512]{};
      sockaddr_in from_addr{};
      socklen_t from_len = sizeof(from_addr);

      const ssize_t received = recvfrom(
        sock_fd_,
        buffer,
        sizeof(buffer),
        0,
        reinterpret_cast<sockaddr *>(&from_addr),
        &from_len);

      if (received < 0) {
        if (errno == EAGAIN || errno == EWOULDBLOCK) {
          break;
        }

        RCLCPP_WARN_THROTTLE(
          this->get_logger(),
          *this->get_clock(),
          1000,
          "UDP receive error. errno=%d",
          errno);

        break;
      }

      if (received != static_cast<ssize_t>(protocol::ETHERNET_SEND_LEN)) {
        RCLCPP_WARN(
          this->get_logger(),
          "Ignore UDP packet. len=%zd, expected=%zu",
          received,
          protocol::ETHERNET_SEND_LEN);
        continue;
      }

      std::array<uint8_t, protocol::ETHERNET_SEND_LEN> frame{};
      std::memcpy(frame.data(), buffer, protocol::ETHERNET_SEND_LEN);

      auto status = protocol::unpack_switch_status(frame, this->now());

      for (size_t i = 0; i < 3; ++i) {
        if (status.boards[i].frame_head != protocol::FRAME_HEAD) {
          RCLCPP_WARN(
            this->get_logger(),
            "Board %zu status frame head error: 0x%02X",
            i + 1,
            status.boards[i].frame_head);
        }

        if (!status.boards[i].crc_ok) {
          RCLCPP_WARN(
            this->get_logger(),
            "Board %zu CRC error. received=0x%04X, calculated=0x%04X",
            i + 1,
            status.boards[i].received_crc,
            status.boards[i].calculated_crc);
        }
      }

      status_pub_->publish(status);

      std_msgs::msg::UInt8MultiArray raw_msg;
      raw_msg.data.assign(frame.begin(), frame.end());
      raw_status_pub_->publish(raw_msg);
    }
  }

private:
  std::string stm32_ip_;
  int stm32_port_{50000};
  int local_port_{50001};

  double command_timeout_ms_{50.0};
  double send_rate_hz_{200.0};

  int sock_fd_{-1};
  sockaddr_in stm32_addr_{};

  bool has_command_{false};
  rclcpp::Time last_command_time_;
  switch_interfaces::msg::SwitchCommand latest_command_;

  rclcpp::Subscription<switch_interfaces::msg::SwitchCommand>::SharedPtr command_sub_;
  rclcpp::Publisher<switch_interfaces::msg::SwitchStatus>::SharedPtr status_pub_;
  rclcpp::Publisher<std_msgs::msg::UInt8MultiArray>::SharedPtr raw_status_pub_;

  rclcpp::TimerBase::SharedPtr tx_timer_;
  rclcpp::TimerBase::SharedPtr rx_timer_;
};

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);

  try {
    rclcpp::spin(std::make_shared<UdpBridgeNode>());
  } catch (const std::exception &e) {
    std::cerr << "udp_bridge_node exception: " << e.what() << std::endl;
  }

  rclcpp::shutdown();
  return 0;
}