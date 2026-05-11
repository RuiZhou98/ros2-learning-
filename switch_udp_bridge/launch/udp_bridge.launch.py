# 导入 LaunchDescription 类，用于描述 launch 文件的结构
from launch import LaunchDescription
# 导入 Node 动作类，用于配置和启动 ROS2 节点
from launch_ros.actions import Node
# 导入包目录查找函数，用于获取 ROS2 包的共享目录路径
from ament_index_python.packages import get_package_share_directory

# 导入 os 模块，用于路径拼接
import os


# 生成 launch 描述的函数，launch 系统启动时会调用此函数
def generate_launch_description():
    # 获取 switch_udp_bridge 包的共享目录路径
    pkg_share = get_package_share_directory("switch_udp_bridge")

    # 拼接配置文件的完整路径
    config_file = os.path.join(
        pkg_share,
        "config",
        "udp_bridge.yaml"
    )

    # 返回 LaunchDescription 对象，包含要启动的节点列表
    return LaunchDescription([
        Node(
            # 节点所属的 ROS2 包名
            package="switch_udp_bridge",
            # 可执行文件名（对应 CMakeLists.txt 中定义的节点可执行文件）
            executable="udp_bridge_node",
            # 节点名称，用于 ROS2 网络中标识此节点
            name="udp_bridge_node",
            # 将节点的标准输出打印到终端屏幕
            output="screen",
            # 加载配置文件作为节点参数
            parameters=[config_file],
        )
    ])