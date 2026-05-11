// 头文件保护宏，防止重复包含
#pragma once

// 引入 chrono 时间库，用于时间测量和超时判断
#include <chrono>
// 引入 uint8_t 类型定义
#include <cstdint>
// 引入互斥锁，用于多线程安全保护
#include <mutex>

// 引入 SwitchCommand 消息类型定义
#include "switch_interfaces/msg/switch_command.hpp"
// 引入协议帧打包功能
#include "switch_udp_bridge/protocol.hpp"

// UDP 桥接功能的命名空间
namespace switch_udp_bridge
{

// 定频发送调度器，负责缓存最新命令并按需生成待发送帧
class TxScheduler
{
public:
  // 使用 steady_clock 作为时间源，保证时间单调递增不受系统时间调整影响
  using SteadyClock = std::chrono::steady_clock;

  // 构造函数
  // 参数 command_timeout：命令超时时长，超时后使用安全命令
  // 参数 timeout_command_type：超时后发送的命令类型
  TxScheduler(
    std::chrono::milliseconds command_timeout,
    std::uint8_t timeout_command_type);

  // 更新最新接收到的控制命令（由 ROS2 回调函数调用）
  void update_command(
    const switch_interfaces::msg::SwitchCommand & command);

  // 生成下一帧待发送的命令帧（由定频发送循环调用）
  // 如果命令已超时，则自动生成安全停止帧
  protocol::CommandFrame make_next_frame();

private:
  // 最新缓存的控制命令
  switch_interfaces::msg::SwitchCommand latest_command_;
  // 上一次收到有效命令的时间点
  SteadyClock::time_point last_command_time_;

  // 命令超时阈值
  std::chrono::milliseconds command_timeout_;
  // 超时后发送的命令类型
  std::uint8_t timeout_command_type_;

  // 是否已经收到过至少一次命令（用于判断超时前是否有有效数据）
  bool has_command_;
  // 互斥锁，保护命令和时间的多线程访问
  std::mutex mutex_;
};

}  // namespace switch_udp_bridge