// 引入 UDP 桥接节点的头文件
#include "switch_udp_bridge/udp_bridge_node.hpp"

// 引入 algorithm 库，用于通用算法操作
#include <algorithm>
// 引入 cmath 库，用于 std::round 取整
#include <cmath>
// 引入 exception 库，用于异常捕获
#include <exception>

// UDP 桥接功能的命名空间
namespace switch_udp_bridge
{

// UdpBridgeNode 构造函数
// 初始化 ROS2 节点、参数声明、UDP 通信组件和收发线程
UdpBridgeNode::UdpBridgeNode()
// 调用基类构造函数，设置节点名称为 "udp_bridge_node"
: Node("udp_bridge_node"),
  // 初始化 STM32 IP 地址（默认值，后续由参数覆盖）
  stm32_ip_("192.168.1.10"),
  // 初始化 STM32 端口号（默认值，后续由参数覆盖）
  stm32_port_(50000),
  // 初始化本机端口号（默认值，后续由参数覆盖）
  local_port_(50001),
  // 初始化发送频率（默认值，后续由参数覆盖）
  send_rate_hz_(200.0),
  // 初始化超时时间（默认值，后续由参数覆盖）
  command_timeout_ms_(50.0),
  // 初始化超时命令类型（默认值为运动命令）
  timeout_command_type_(protocol::CMD_MOTION),
  // 初始化单次 spin 最大接收包数（默认值，后续由参数覆盖）
  max_rx_packets_per_spin_(8),
  // 初始化发送周期为 5ms（200Hz 对应的周期）
  tx_period_(std::chrono::milliseconds(5)),
  // 初始化运行标志为 false，启动后才置为 true
  running_(false)
{
  // 声明并获取 STM32 的 IP 地址参数
  stm32_ip_ = this->declare_parameter<std::string>("stm32_ip", "192.168.1.10");
  // 声明并获取 STM32 端口号参数（声明为 int，使用时转换为 uint16_t）
  stm32_port_ = static_cast<std::uint16_t>(
    this->declare_parameter<int>("stm32_port", 50000));
  // 声明并获取本机端口号参数（声明为 int，使用时转换为 uint16_t）
  local_port_ = static_cast<std::uint16_t>(
    this->declare_parameter<int>("local_port", 50001));

  // 声明并获取定频发送频率参数
  send_rate_hz_ = this->declare_parameter<double>("send_rate_hz", 200.0);
  // 声明并获取命令超时时间参数
  command_timeout_ms_ = this->declare_parameter<double>("command_timeout_ms", 50.0);

  // 声明并获取超时命令类型参数（声明为 int，使用时转换为 uint8_t）
  timeout_command_type_ = static_cast<std::uint8_t>(
    this->declare_parameter<int>("timeout_command_type", protocol::CMD_MOTION));

  // 声明并获取单次 spin 最大接收包数参数
  max_rx_packets_per_spin_ =
    this->declare_parameter<int>("max_rx_packets_per_spin", 8);

  // 参数合法性检查：发送频率最低为 1Hz
  if (send_rate_hz_ < 1.0) {
    send_rate_hz_ = 1.0;
  }

  // 参数合法性检查：超时时间最低为 1ms
  if (command_timeout_ms_ < 1.0) {
    command_timeout_ms_ = 1.0;
  }

  // 参数合法性检查：最大接收包数至少为 1
  if (max_rx_packets_per_spin_ < 1) {
    max_rx_packets_per_spin_ = 1;
  }

  // 根据发送频率计算发送周期，单位为纳秒
  const auto period_ns =
    static_cast<std::int64_t>(1e9 / send_rate_hz_);

  // 保存计算得到的发送周期
  tx_period_ = std::chrono::nanoseconds(period_ns);

  // 创建 UDP socket 对象，连接到 STM32
  udp_socket_ = std::make_unique<UdpSocket>(
    stm32_ip_,
    stm32_port_,
    local_port_);

  // 创建发送调度器对象，配置超时参数
  tx_scheduler_ = std::make_unique<TxScheduler>(
    std::chrono::milliseconds(
      static_cast<int>(std::round(command_timeout_ms_))),
    timeout_command_type_);

  // 创建订阅者，订阅 /switch/command 话题
  // QoS 配置为 best_effort + 队列深度 1，保证低延迟，只保留最新消息
  command_sub_ =
    this->create_subscription<switch_interfaces::msg::SwitchCommand>(
      "/switch/command",
      rclcpp::QoS(1).best_effort(),
      std::bind(&UdpBridgeNode::command_callback, this, std::placeholders::_1));

  // 创建发布者，发布解析后的状态到 /switch/status 话题
  // 使用 SensorDataQoS，适合高频传感器数据发布
  status_pub_ =
    this->create_publisher<switch_interfaces::msg::SwitchStatus>(
      "/switch/status",
      rclcpp::SensorDataQoS());

  // 创建发布者，发布原始 170 字节数据到 /switch/status_raw 话题
  raw_pub_ =
    this->create_publisher<std_msgs::msg::UInt8MultiArray>(
      "/switch/status_raw",
      rclcpp::SensorDataQoS());

  // 创建 1ms 周期的定时器，用于非阻塞接收 UDP 数据
  // 由于 ROS2 定时器最小精度约 1ms，这里用轮询方式替代 select/epoll
  rx_timer_ =
    this->create_wall_timer(
      std::chrono::milliseconds(1),
      std::bind(&UdpBridgeNode::rx_timer_callback, this));

  // 设置运行标志为 true，启动发送线程
  running_.store(true);
  // 创建独立的发送线程，执行定频发送循环
  tx_thread_ = std::thread(&UdpBridgeNode::tx_loop, this);
}

// UdpBridgeNode 析构函数
// 安全停止发送线程并等待线程结束
UdpBridgeNode::~UdpBridgeNode()
{
  // 设置运行标志为 false，通知发送线程退出循环
  running_.store(false);

  // 如果发送线程可 join（尚未 detach），则等待线程结束
  if (tx_thread_.joinable()) {
    tx_thread_.join();
  }
}

// 命令订阅回调函数
// 参数 msg：接收到的 SwitchCommand 消息的共享指针
void UdpBridgeNode::command_callback(
  const switch_interfaces::msg::SwitchCommand::SharedPtr msg)
{
  // 将新命令传递给发送调度器，更新缓存的最新命令
  tx_scheduler_->update_command(*msg);
}

// 接收定时器回调函数（每 1ms 执行一次）
// 在单次回调中最多处理 max_rx_packets_per_spin_ 个 UDP 数据包
void UdpBridgeNode::rx_timer_callback()
{
  // 循环接收，每次最多处理指定数量的 UDP 包
  for (int i = 0; i < max_rx_packets_per_spin_; ++i) {
    // 创建 170 字节的接收缓冲区
    protocol::StatusFrame frame{};

    // 用于存放接收结果的 optional 对象
    std::optional<std::size_t> received;

    // 尝试接收 UDP 数据，捕获可能的异常
    try {
      received = udp_socket_->receive(frame.data(), frame.size());
    } catch (const std::exception & e) {
      // 若接收发生异常，打印告警日志（每秒最多打印一次，防止刷屏）
      RCLCPP_WARN_THROTTLE(
        this->get_logger(),
        *this->get_clock(),
        1000,
        "UDP receive error: %s",
        e.what());
      return;
    }

    // 若无数据（std::nullopt），则提前返回
    if (!received.has_value()) {
      return;
    }

    // 若接收到的数据长度不等于 170 字节，丢弃并继续处理下一包
    if (received.value() != protocol::ETHERNET_SEND_LEN) {
      continue;
    }

    // 解析 170 字节状态帧为 SwitchStatus 消息，附上当前时间戳
    auto status =
      protocol::unpack_switch_status(frame, this->now());

    // 构造原始数据消息用于发布
    auto raw_msg = std_msgs::msg::UInt8MultiArray();
    // 将 170 字节数据拷贝到消息的 data 字段
    raw_msg.data.assign(frame.begin(), frame.end());

    // 发布解析后的状态消息到 /switch/status 话题
    status_pub_->publish(status);
    // 发布原始字节消息到 /switch/status_raw 话题
    raw_pub_->publish(raw_msg);
  }
}

// 发送循环函数（运行在独立的 tx_thread_ 线程中）
// 按固定周期发送命令帧，保证发送间隔的稳定性
void UdpBridgeNode::tx_loop()
{
  // 使用 steady_clock 作为时钟源，不受系统时间调整影响
  using Clock = std::chrono::steady_clock;

  // 记录下一次应该发送的时间点
  auto next_time = Clock::now();

  // 持续运行直到 ROS2 关闭或运行标志被清除
  while (rclcpp::ok() && running_.load()) {
    // 计算下一个发送时间点（按固定周期累加，避免累积误差）
    next_time += tx_period_;

    // 尝试发送一帧命令数据
    try {
      // 从调度器获取下一帧命令（可能为正常命令或超时安全命令）
      const auto frame = tx_scheduler_->make_next_frame();
      // 通过 UDP socket 发送 56 字节命令帧
      udp_socket_->send(frame.data(), frame.size());
    } catch (const std::exception & e) {
      // 若发送发生异常，打印告警日志（每秒最多打印一次）
      RCLCPP_WARN_THROTTLE(
        this->get_logger(),
        *this->get_clock(),
        1000,
        "UDP send error: %s",
        e.what());
    }

    // 休眠直到下一个发送时间点，保证发送周期精确
    std::this_thread::sleep_until(next_time);

    // 检查当前时间是否落后于计划时间超过一个周期
    // （可能因系统负载导致单次循环执行时间过长）
    const auto now = Clock::now();

    if (now > next_time + tx_period_) {
      // 若严重滞后，重置时间基准，避免追赶导致的突发发送
      next_time = now;
    }
  }
}

}  // namespace switch_udp_bridge