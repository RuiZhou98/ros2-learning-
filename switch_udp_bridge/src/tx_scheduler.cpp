#include "switch_udp_bridge/tx_scheduler.hpp"

namespace switch_udp_bridge
{

TxScheduler::TxScheduler(
  std::chrono::milliseconds command_timeout,
  std::uint8_t timeout_command_type)
: latest_command_(protocol::make_idle_command(timeout_command_type)),
  last_command_time_(SteadyClock::now()),
  command_timeout_(command_timeout),
  timeout_command_type_(timeout_command_type),
  has_command_(false)
{
}

void TxScheduler::update_command(
  const switch_interfaces::msg::SwitchCommand & command)
{
  std::lock_guard<std::mutex> lock(mutex_);

  latest_command_ = command;
  last_command_time_ = SteadyClock::now();
  has_command_ = true;
}

protocol::CommandFrame TxScheduler::make_next_frame()
{
  std::lock_guard<std::mutex> lock(mutex_);

  const auto now = SteadyClock::now();

  if (!has_command_) {
    const auto idle = protocol::make_idle_command(timeout_command_type_);
    return protocol::pack_switch_command(idle);
  }

  const auto age =
    std::chrono::duration_cast<std::chrono::milliseconds>(
      now - last_command_time_);

  if (age > command_timeout_) {
    const auto idle = protocol::make_idle_command(timeout_command_type_);
    return protocol::pack_switch_command(idle);
  }

  return protocol::pack_switch_command(latest_command_);
}

}  // namespace switch_udp_bridge