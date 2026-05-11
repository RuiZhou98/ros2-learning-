// 引入定频发送调度器的头文件
#include "switch_udp_bridge/tx_scheduler.hpp"

// UDP 桥接功能的命名空间
namespace switch_udp_bridge
{

// TxScheduler 构造函数
// 参数 command_timeout：命令超时时长，超时后使用安全命令
// 参数 timeout_command_type：超时后发送的命令类型
TxScheduler::TxScheduler(
  std::chrono::milliseconds command_timeout,
  std::uint8_t timeout_command_type)
  // 初始化最新命令为安全的空闲命令，防止启动时发送未定义数据
  : latest_command_(protocol::make_idle_command(timeout_command_type)),
    // 记录当前时间作为初始命令时间
    last_command_time_(SteadyClock::now()),
    // 保存命令超时阈值
    command_timeout_(command_timeout),
    // 保存超时后使用的命令类型
    timeout_command_type_(timeout_command_type),
    // 初始状态为未收到过命令
    has_command_(false)
{
}

// 更新最新接收到的控制命令（由 ROS2 订阅回调函数调用）
// 参数 command：新接收到的 SwitchCommand 消息
void TxScheduler::update_command(
  const switch_interfaces::msg::SwitchCommand & command)
{
  // 上锁保护，确保多线程安全（ROS2 回调与发送循环可能在不同线程）
  std::lock_guard<std::mutex> lock(mutex_);

  // 缓存最新命令
  latest_command_ = command;
  // 更新最后一次收到命令的时间
  last_command_time_ = SteadyClock::now();
  // 标记已收到过至少一次命令
  has_command_ = true;
}

// 生成下一帧待发送的命令帧（由定频发送循环调用）
// 如果从未收到命令或命令已超时，则自动生成安全停止帧
// 返回值：56 字节命令帧数组
protocol::CommandFrame TxScheduler::make_next_frame()
{
  // 上锁保护，确保线程安全
  std::lock_guard<std::mutex> lock(mutex_);

  // 获取当前时间
  const auto now = SteadyClock::now();

  // 如果从未收到过命令，直接返回安全的空闲命令帧
  if (!has_command_) {
    // 生成所有运动参数为零的安全命令
    const auto idle = protocol::make_idle_command(timeout_command_type_);
    // 打包为 56 字节命令帧并返回
    return protocol::pack_switch_command(idle);
  }

  // 计算距离上次收到命令经过的时间
  const auto age =
    std::chrono::duration_cast<std::chrono::milliseconds>(
      now - last_command_time_);

  // 如果经过时间超过超时阈值，触发超时保护
  if (age > command_timeout_) {
    // 生成安全空闲命令帧（运动参数全部置零，防止失控）
    const auto idle = protocol::make_idle_command(timeout_command_type_);
    // 打包为 56 字节命令帧并返回
    return protocol::pack_switch_command(idle);
  }

  // 未超时，打包最新的有效命令并返回
  return protocol::pack_switch_command(latest_command_);
}

}  // namespace switch_udp_bridge