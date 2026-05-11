#include "switch_udp_bridge/udp_bridge_node.hpp"

#include <algorithm>
#include <cmath>
#include <exception>

namespace switch_udp_bridge
{

UdpBridgeNode::UdpBridgeNode()
: Node("udp_bridge_node"),
  stm32_ip_("192.168.1.10"),
  stm32_port_(50000),
  local_port_(50001),
  send_rate_hz_(200.0),
  command_timeout_ms_(50.0),
  timeout_command_type_(protocol::CMD_MOTION),
  max_rx_packets_per_spin_(8),
  tx_period_(std::chrono::milliseconds(5)),
  running_(false)
{
  stm32_ip_ = this->declare_parameter<std::string>("stm32_ip", "192.168.1.10");
  stm32_port_ = static_cast<std::uint16_t>(
    this->declare_parameter<int>("stm32_port", 50000));
  local_port_ = static_cast<std::uint16_t>(
    this->declare_parameter<int>("local_port", 50001));

  send_rate_hz_ = this->declare_parameter<double>("send_rate_hz", 200.0);
  command_timeout_ms_ = this->declare_parameter<double>("command_timeout_ms", 50.0);

  timeout_command_type_ = static_cast<std::uint8_t>(
    this->declare_parameter<int>("timeout_command_type", protocol::CMD_MOTION));

  max_rx_packets_per_spin_ =
    this->declare_parameter<int>("max_rx_packets_per_spin", 8);

  if (send_rate_hz_ < 1.0) {
    send_rate_hz_ = 1.0;
  }

  if (command_timeout_ms_ < 1.0) {
    command_timeout_ms_ = 1.0;
  }

  if (max_rx_packets_per_spin_ < 1) {
    max_rx_packets_per_spin_ = 1;
  }

  const auto period_ns =
    static_cast<std::int64_t>(1e9 / send_rate_hz_);

  tx_period_ = std::chrono::nanoseconds(period_ns);

  udp_socket_ = std::make_unique<UdpSocket>(
    stm32_ip_,
    stm32_port_,
    local_port_);

  tx_scheduler_ = std::make_unique<TxScheduler>(
    std::chrono::milliseconds(
      static_cast<int>(std::round(command_timeout_ms_))),
    timeout_command_type_);

  command_sub_ =
    this->create_subscription<switch_interfaces::msg::SwitchCommand>(
      "/switch/command",
      rclcpp::QoS(1).best_effort(),
      std::bind(&UdpBridgeNode::command_callback, this, std::placeholders::_1));

  status_pub_ =
    this->create_publisher<switch_interfaces::msg::SwitchStatus>(
      "/switch/status",
      rclcpp::SensorDataQoS());

  raw_pub_ =
    this->create_publisher<std_msgs::msg::UInt8MultiArray>(
      "/switch/status_raw",
      rclcpp::SensorDataQoS());

  rx_timer_ =
    this->create_wall_timer(
      std::chrono::milliseconds(1),
      std::bind(&UdpBridgeNode::rx_timer_callback, this));

  running_.store(true);
  tx_thread_ = std::thread(&UdpBridgeNode::tx_loop, this);
}

UdpBridgeNode::~UdpBridgeNode()
{
  running_.store(false);

  if (tx_thread_.joinable()) {
    tx_thread_.join();
  }
}

void UdpBridgeNode::command_callback(
  const switch_interfaces::msg::SwitchCommand::SharedPtr msg)
{
  tx_scheduler_->update_command(*msg);
}

void UdpBridgeNode::rx_timer_callback()
{
  for (int i = 0; i < max_rx_packets_per_spin_; ++i) {
    protocol::StatusFrame frame{};

    std::optional<std::size_t> received;

    try {
      received = udp_socket_->receive(frame.data(), frame.size());
    } catch (const std::exception & e) {
      RCLCPP_WARN_THROTTLE(
        this->get_logger(),
        *this->get_clock(),
        1000,
        "UDP receive error: %s",
        e.what());
      return;
    }

    if (!received.has_value()) {
      return;
    }

    if (received.value() != protocol::ETHERNET_SEND_LEN) {
      continue;
    }

    auto status =
      protocol::unpack_switch_status(frame, this->now());

    auto raw_msg = std_msgs::msg::UInt8MultiArray();
    raw_msg.data.assign(frame.begin(), frame.end());

    status_pub_->publish(status);
    raw_pub_->publish(raw_msg);
  }
}

void UdpBridgeNode::tx_loop()
{
  using Clock = std::chrono::steady_clock;

  auto next_time = Clock::now();

  while (rclcpp::ok() && running_.load()) {
    next_time += tx_period_;

    try {
      const auto frame = tx_scheduler_->make_next_frame();
      udp_socket_->send(frame.data(), frame.size());
    } catch (const std::exception & e) {
      RCLCPP_WARN_THROTTLE(
        this->get_logger(),
        *this->get_clock(),
        1000,
        "UDP send error: %s",
        e.what());
    }

    std::this_thread::sleep_until(next_time);

    const auto now = Clock::now();

    if (now > next_time + tx_period_) {
      next_time = now;
    }
  }
}

}  // namespace switch_udp_bridge