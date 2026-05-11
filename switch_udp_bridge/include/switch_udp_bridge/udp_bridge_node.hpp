// 头文件保护宏，防止重复包含
#pragma once

// 引入 array 容器，用于存储固定大小的字节数组
#include <array>
// 引入 atomic 原子类型，用于多线程安全标志位
#include <atomic>
// 引入 chrono 时间库，用于时间周期和超时控制
#include <chrono>
// 引入智能指针，用于自动管理对象生命周期
#include <memory>
// 引入 string 类型
#include <string>
// 引入线程支持，用于创建独立的发送线程
#include <thread>

// 引入 ROS2 核心库
#include "rclcpp/rclcpp.hpp"
// 引入 UInt8MultiArray 消息类型，用于发布原始字节数据
#include "std_msgs/msg/u_int8_multi_array.hpp"
// 引入 SwitchCommand 消息类型，接收控制命令
#include "switch_interfaces/msg/switch_command.hpp"
// 引入 SwitchStatus 消息类型，发布解析后的状态
#include "switch_interfaces/msg/switch_status.hpp"

// 引入协议帧打包与解析功能
#include "switch_udp_bridge/protocol.hpp"
// 引入定频发送调度器
#include "switch_udp_bridge/tx_scheduler.hpp"
// 引入 UDP socket 通信封装
#include "switch_udp_bridge/udp_socket.hpp"

// UDP 桥接功能的命名空间
namespace switch_udp_bridge
{

// UDP 桥接节点类，负责 ROS2 与 STM32 之间的数据转发
class UdpBridgeNode : public rclcpp::Node
{
public:
  // 构造函数，初始化节点、参数、订阅者、发布者、定时器和发送线程
  UdpBridgeNode();
  // 析构函数，停止发送线程并清理资源
  ~UdpBridgeNode() override;

private:
  // 订阅回调函数：接收 /switch/command 话题的控制命令，更新发送调度器缓存
  void command_callback(
    const switch_interfaces::msg::SwitchCommand::SharedPtr msg);

  // 接收定时器回调函数：周期性从 UDP socket 读取 STM32 返回的状态帧
  void rx_timer_callback();

  // 发送线程主循环：按固定频率从调度器获取命令帧并通过 UDP 发送
  void tx_loop();

private:
  // STM32 控制板的 IP 地址
  std::string stm32_ip_;
  // STM32 控制板的 UDP 端口号
  std::uint16_t stm32_port_;
  // ROS2 本机的 UDP 接收端口号
  std::uint16_t local_port_;

  // 定频发送频率，单位：Hz
  double send_rate_hz_;
  // 命令超时时间，单位：毫秒
  double command_timeout_ms_;
  // 超时后发送的命令类型（1=零速运动命令，2=读取状态命令）
  std::uint8_t timeout_command_type_;

  // 单次接收定时器回调中最多处理的 UDP 数据包数量
  int max_rx_packets_per_spin_;

  // 发送周期，由 send_rate_hz_ 换算得到，单位：纳秒
  std::chrono::nanoseconds tx_period_;

  // UDP socket 对象，负责底层网络收发
  std::unique_ptr<UdpSocket> udp_socket_;
  // 发送调度器对象，管理命令缓存和超时保护
  std::unique_ptr<TxScheduler> tx_scheduler_;

  // 控制命令订阅者，监听 /switch/command 话题
  rclcpp::Subscription<switch_interfaces::msg::SwitchCommand>::SharedPtr command_sub_;
  // 解析后状态发布者，发布到 /switch/status 话题
  rclcpp::Publisher<switch_interfaces::msg::SwitchStatus>::SharedPtr status_pub_;
  // 原始数据发布者，发布到 /switch/status_raw 话题
  rclcpp::Publisher<std_msgs::msg::UInt8MultiArray>::SharedPtr raw_pub_;

  // 接收定时器，周期性触发 UDP 数据接收
  rclcpp::TimerBase::SharedPtr rx_timer_;

  // 独立发送线程，确保 UDP 发送不受 ROS2 spin 调度影响
  std::thread tx_thread_;
  // 原子运行标志，控制发送线程的启停
  std::atomic<bool> running_;
};

}  // namespace switch_udp_bridge