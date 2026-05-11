// 引入智能指针相关功能
#include <memory>

// 引入 rclcpp 核心头文件，提供 ROS2 客户端库功能
#include "rclcpp/rclcpp.hpp"
// 引入多线程执行器，用于同时处理节点中的多个回调线程
#include "rclcpp/executors/multi_threaded_executor.hpp"

// 引入 UDP 桥接节点的类定义
#include "switch_udp_bridge/udp_bridge_node.hpp"

// 程序入口函数
// 参数 argc：命令行参数个数
// 参数 argv：命令行参数数组
int main(int argc, char ** argv)
{
  // 初始化 ROS2 客户端库，解析命令行参数
  rclcpp::init(argc, argv);

  // 创建 UDP 桥接节点实例（使用 shared_ptr 管理生命周期）
  // 节点内部会自动创建发送线程和定时器
  auto node = std::make_shared<switch_udp_bridge::UdpBridgeNode>();

  // 创建多线程执行器
  // 使用多线程确保发送线程、接收定时器回调能并行执行
  rclcpp::executors::MultiThreadedExecutor executor;
  // 将节点添加到执行器中
  executor.add_node(node);
  // 进入事件循环，开始处理 ROS2 消息和定时器回调
  // spin 会阻塞当前线程直到节点被关闭
  executor.spin();

  // 关闭 ROS2 客户端库，释放资源
  rclcpp::shutdown();

  // 返回 0 表示程序正常退出
  return 0;
}