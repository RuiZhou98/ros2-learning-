#pragma once

#include <chrono>
#include <cstdint>
#include <mutex>

#include "switch_interfaces/msg/switch_command.hpp"
#include "switch_udp_bridge/protocol.hpp"

namespace switch_udp_bridge
{

class TxScheduler
{
public:
  using SteadyClock = std::chrono::steady_clock;

  TxScheduler(
    std::chrono::milliseconds command_timeout,
    std::uint8_t timeout_command_type);

  void update_command(
    const switch_interfaces::msg::SwitchCommand & command);

  protocol::CommandFrame make_next_frame();

private:
  switch_interfaces::msg::SwitchCommand latest_command_;
  SteadyClock::time_point last_command_time_;

  std::chrono::milliseconds command_timeout_;
  std::uint8_t timeout_command_type_;

  bool has_command_;
  std::mutex mutex_;
};

}  // namespace switch_udp_bridge