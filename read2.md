
# ROS2 STM32 UDP Bridge

本项目用于实现 ROS2 上位机与 STM32 控制板之间的 Ethernet UDP 通信。

ROS2 侧负责将 `/switch/command` 消息打包为 UDP 命令帧发送给 STM32，并接收 STM32 返回的状态帧，解析后发布为 `/switch/status`。

---

## 1. 通信数据结构

### 1.1 ROS2 → STM32：56 字节命令帧

| 字节范围 | 长度 | 内容 |
|---:|---:|---|
| 0 - 16 | 17 | 1 号通信板命令 |
| 17 - 33 | 17 | 2 号通信板命令 |
| 34 - 50 | 17 | 3 号通信板命令 |
| 51 - 54 | 4 | 电源板命令 |
| 55 | 1 | 灯带命令 |

---

### 1.2 单个通信板命令帧：17 字节

| 字节范围 | 长度 | 内容 |
|---:|---:|---|
| 0 | 1 | 帧头，固定 `0xFE` |
| 1 | 1 | 命令类型 |
| 2 | 1 | 通信板 ID |
| 3 - 6 | 4 | 舵轮目标角度 `float32` |
| 7 - 10 | 4 | 前进轮目标速度 `float32` |
| 11 - 14 | 4 | 风箱目标气压 `float32` |
| 15 - 16 | 2 | CRC16 |

命令类型：

| 命令 | 值 | 说明 |
|---|---:|---|
| 运动命令 | `0x01` | 控制运动、风机等 |
| 读取状态 | `0x02` | 只读取状态 |
| 归零命令 | `0x03` | 舵轮归零 |

CRC 计算范围：

```text
命令帧 CRC = 对字节 0 - 14 计算 CRC16-Modbus
````

---

### 1.3 STM32 → ROS2：170 字节状态帧

|      字节范围 | 长度 | 内容       |
| --------: | -: | -------- |
|    0 - 45 | 46 | 1 号通信板状态 |
|   46 - 91 | 46 | 2 号通信板状态 |
|  92 - 137 | 46 | 3 号通信板状态 |
| 138 - 168 | 31 | 电源板状态    |
|       169 |  1 | 开关状态     |

---

### 1.4 单个通信板状态帧：46 字节

|    字节范围 | 长度 | 内容           |
| ------: | -: | ------------ |
|       0 |  1 | 帧头，固定 `0xFE` |
|       1 |  1 | 命令类型         |
|       2 |  1 | 通信板 ID       |
|   3 - 6 |  4 | 舵轮位置         |
|  7 - 10 |  4 | 舵轮速度         |
| 11 - 14 |  4 | 舵轮力矩         |
| 15 - 18 |  4 | 前进轮位置        |
| 19 - 22 |  4 | 前进轮速度        |
| 23 - 26 |  4 | 前进轮力矩        |
| 27 - 30 |  4 | 风机转速         |
| 31 - 34 |  4 | 风机电流         |
| 35 - 36 |  2 | 风机温度         |
| 37 - 38 |  2 | 风机驱动温度       |
| 39 - 42 |  4 | 吸附腔气压        |
|      43 |  1 | 错误码          |
| 44 - 45 |  2 | CRC16        |

CRC 计算范围：

```text
状态帧 CRC = 对字节 0 - 43 计算 CRC16-Modbus
```

---

### 1.5 字节序

本项目默认使用小端格式：

```text
uint16: 低字节在前，高字节在后
float32: IEEE754 小端格式
CRC16: 低字节在前，高字节在后
```

---

## 2. ROS2 Topic

| Topic                | 消息类型                                  | 说明                   |
| -------------------- | ------------------------------------- | -------------------- |
| `/switch/command`    | `switch_interfaces/msg/SwitchCommand` | 上位机发送给 STM32 的控制命令   |
| `/switch/status`     | `switch_interfaces/msg/SwitchStatus`  | STM32 返回的解析后状态       |
| `/switch/status_raw` | `std_msgs/msg/UInt8MultiArray`        | STM32 返回的原始 170 字节数据 |

---

## 3. 项目文件结构

```text
switch_ros2_ws/
└── src/
    ├── switch_interfaces/
    │   ├── msg/
    │   │   ├── BoardCommand.msg
    │   │   ├── SwitchCommand.msg
    │   │   ├── BoardStatus.msg
    │   │   └── SwitchStatus.msg
    │   ├── CMakeLists.txt
    │   └── package.xml
    │
    └── switch_udp_bridge/
        ├── include/
        │   └── switch_udp_bridge/
        │       ├── protocol_constants.hpp
        │       ├── byte_utils.hpp
        │       ├── crc16.hpp
        │       ├── protocol.hpp
        │       ├── udp_socket.hpp
        │       ├── tx_scheduler.hpp
        │       └── udp_bridge_node.hpp
        │
        ├── src/
        │   ├── crc16.cpp
        │   ├── protocol.cpp
        │   ├── udp_socket.cpp
        │   ├── tx_scheduler.cpp
        │   ├── udp_bridge_node.cpp
        │   └── main.cpp
        │
        ├── config/
        │   └── udp_bridge.yaml
        │
        ├── launch/
        │   └── udp_bridge.launch.py
        │
        ├── CMakeLists.txt
        └── package.xml
```

---

## 4. 模块说明

| 文件                    | 作用                        |
| --------------------- | ------------------------- |
| `crc16.cpp`           | CRC16-Modbus 计算           |
| `protocol.cpp`        | 56 字节命令帧打包，170 字节状态帧解析    |
| `udp_socket.cpp`      | UDP socket 创建、发送、接收       |
| `tx_scheduler.cpp`    | 定频发送控制，保证发送时间间隔稳定         |
| `udp_bridge_node.cpp` | ROS2 节点，连接 Topic 与 UDP 通信 |
| `main.cpp`            | 节点入口                      |

---

## 5. 参数配置

配置文件：

```text
switch_udp_bridge/config/udp_bridge.yaml
```

示例：

```yaml
udp_bridge_node:
  ros__parameters:
    stm32_ip: "192.168.1.10"
    stm32_port: 50000
    local_port: 50001

    send_rate_hz: 200.0
    command_timeout_ms: 50.0
    timeout_command_type: 1

    max_rx_packets_per_spin: 8
```

参数说明：

| 参数                        | 说明                  |
| ------------------------- | ------------------- |
| `stm32_ip`                | STM32 的 IP 地址       |
| `stm32_port`              | STM32 UDP 接收端口      |
| `local_port`              | ROS2 本机 UDP 接收端口    |
| `send_rate_hz`            | ROS2 向 STM32 定频发送频率 |
| `command_timeout_ms`      | 超过该时间没有新命令则进入超时保护   |
| `timeout_command_type`    | 超时后发送的命令类型          |
| `max_rx_packets_per_spin` | 单次接收循环最多处理的 UDP 包数量 |

---

## 6. 使用说明

### 6.1 编译

进入工作空间：

```bash
cd ~/switch_ros2_ws
```

编译：

```bash
colcon build --symlink-install
```

加载环境：

```bash
source install/setup.bash
```

---

### 6.2 启动 UDP 桥接节点

```bash
ros2 launch switch_udp_bridge udp_bridge.launch.py
```

---

### 6.3 发布控制命令

发布读取状态命令：

```bash
ros2 topic pub -r 200 /switch/command switch_interfaces/msg/SwitchCommand "{boards: [
  {command_type: 2, id: 1, steering_angle: 0.0, wheel_speed: 0.0, fan_pressure: 0.0},
  {command_type: 2, id: 2, steering_angle: 0.0, wheel_speed: 0.0, fan_pressure: 0.0},
  {command_type: 2, id: 3, steering_angle: 0.0, wheel_speed: 0.0, fan_pressure: 0.0}
], power_board: [0, 0, 0, 0], led: 0}"
```

发布运动命令：

```bash
ros2 topic pub -r 200 /switch/command switch_interfaces/msg/SwitchCommand "{boards: [
  {command_type: 1, id: 1, steering_angle: 0.0, wheel_speed: 1.0, fan_pressure: 0.0},
  {command_type: 1, id: 2, steering_angle: 0.0, wheel_speed: 1.0, fan_pressure: 0.0},
  {command_type: 1, id: 3, steering_angle: 0.0, wheel_speed: 1.0, fan_pressure: 0.0}
], power_board: [0, 0, 0, 0], led: 1}"
```

---

### 6.4 查看状态

查看解析后的状态：

```bash
ros2 topic echo /switch/status
```

查看原始 170 字节数据：

```bash
ros2 topic echo /switch/status_raw
```

---

## 7. 发送机制说明

本项目中 `/switch/command` 的接收频率与 UDP 实际发送频率相互解耦。

ROS2 回调函数只负责缓存最新命令，真正的 UDP 发送由 `tx_scheduler.cpp` 按 `send_rate_hz` 定频执行。

这样可以避免 ROS2 控制节点发布频率不稳定时，STM32 收到的 UDP 帧间隔发生剧烈抖动。

```text
/switch/command
      |
      v
缓存最新控制命令
      |
      v
TxScheduler 按固定频率取出最新命令
      |
      v
UDP 56 字节命令帧发送给 STM32
```

---

## 8. 简要流程

```text
ROS2 控制节点
    |
    | /switch/command
    v
switch_udp_bridge
    |
    | UDP 56 bytes
    v
STM32 控制板
    |
    | UDP 170 bytes
    v
switch_udp_bridge
    |
    | /switch/status
    v
ROS2 控制节点 / 日志节点
```
