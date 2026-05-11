#pragma once

#include <array>
#include <atomic>
#include <chrono>
#include <memory>
#include <string>
#include <thread>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/u_int8_multi_array.hpp"
#include "switch_interfaces/msg/switch_command.hpp"
#include "switch_interfaces/msg/switch_status.hpp"

#include "switch_udp_bridge/protocol.hpp"
#include "switch_udp_bridge/tx_scheduler.hpp"
#include "switch_udp_bridge/udp_socket.hpp"

namespace switch_udp_bridge
{

class UdpBridgeNode : public rclcpp::Node
{
public:
  UdpBridgeNode();
  ~UdpBridgeNode() override;

private:
  void command_callback(
    const switch_interfaces::msg::SwitchCommand::SharedPtr msg);

  void rx_timer_callback();

  void tx_loop();

private:
  std::string stm32_ip_;
  std::uint16_t stm32_port_;
  std::uint16_t local_port_;

  double send_rate_hz_;
  double command_timeout_ms_;
  std::uint8_t timeout_command_type_;

  int max_rx_packets_per_spin_;

  std::chrono::nanoseconds tx_period_;

  std::unique_ptr<UdpSocket> udp_socket_;
  std::unique_ptr<TxScheduler> tx_scheduler_;

  rclcpp::Subscription<switch_interfaces::msg::SwitchCommand>::SharedPtr command_sub_;
  rclcpp::Publisher<switch_interfaces::msg::SwitchStatus>::SharedPtr status_pub_;
  rclcpp::Publisher<std_msgs::msg::UInt8MultiArray>::SharedPtr raw_pub_;

  rclcpp::TimerBase::SharedPtr rx_timer_;

  std::thread tx_thread_;
  std::atomic<bool> running_;
};

}  // namespace switch_udp_bridge