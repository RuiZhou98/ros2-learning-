# ROS2 与 STM32 Ethernet UDP 通信桥接项目

本项目用于实现 **ROS2 上位机** 与 **STM32 交换机控制板** 之间的 UDP 通信。STM32 控制板作为中间交换机，与上位机通过 Ethernet/UDP 通信，同时向下连接 3 路通信板、1 路电源板、灯带和风机开关。

本 README 主要说明 ROS2 侧代码的工程结构、数据结构、通信协议、核心函数、编译运行方法和调试流程，方便后续开发者快速接手项目。

---

## 1. 项目功能概述

系统整体功能如下：

1. 上位机 ROS2 节点通过 UDP 向 STM32 发送 56 字节命令帧。
2. STM32 收到上位机命令后，将命令拆包为：

   * 3 路通信板命令帧，每路 17 字节；
   * 1 路电源板命令帧，4 字节；
   * 1 字节灯带控制命令。
3. STM32 通过 RS485 与 3 路通信板通信，通过 RS485 与电源板通信，通过 SPI 控制灯带，通过 GPIO 读取开关状态。
4. STM32 将下层通信板、电源板和开关状态组合成 170 字节状态帧，并通过 UDP 返回 ROS2 上位机。
5. ROS2 节点接收 170 字节状态帧，解析为 ROS2 消息并发布给其他控制节点使用。

ROS2 侧的核心作用是：

```text
ROS2 topic  <---->  UDP 56/170 字节协议帧
```

也就是说，算法控制节点只需要发布和订阅 ROS2 topic，不需要直接处理 socket、CRC、字节偏移和底层协议。

---

## 2. 系统通信架构

```text
┌──────────────────────────────┐
│        ROS2 上位机            │
│                              │
│  控制算法节点                 │
│      │                       │
│      │ /switch/command       │
│      ▼                       │
│  switch_udp_bridge            │
│      │                       │
│      │ UDP 56 bytes           │
└──────┼───────────────────────┘
       │
       ▼
┌──────────────────────────────┐
│        STM32 交换机控制板      │
│                              │
│  Ethernet UDP                │
│  RS485 x 4                   │
│  SPI                         │
│  GPIO                        │
└──────┼───────────────────────┘
       │
       ├──────── 通信板 1
       ├──────── 通信板 2
       ├──────── 通信板 3
       ├──────── 电源板
       ├──────── 灯带
       └──────── 风机开关

┌──────────────────────────────┐
│        STM32 交换机控制板      │
│      │                       │
│      │ UDP 170 bytes          │
└──────┼───────────────────────┘
       │
       ▼
┌──────────────────────────────┐
│        ROS2 上位机            │
│                              │
│  switch_udp_bridge            │
│      │                       │
│      │ /switch/status         │
│      ▼                       │
│  控制算法节点 / 日志节点       │
└──────────────────────────────┘
```

---

## 3. ROS2 工作空间结构

工作空间结构如下：

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
        ├── src/
        │   └── udp_bridge_node.cpp
        ├── config/
        │   └── udp_bridge.yaml
        ├── launch/
        │   └── udp_bridge.launch.py
        ├── CMakeLists.txt
        └── package.xml
```

---

## 4. ROS2 包说明

### 4.1 `switch_interfaces`

该包用于定义 ROS2 自定义消息接口。

主要消息包括：

| 消息文件                | 作用                   |
| ------------------- | -------------------- |
| `BoardCommand.msg`  | 单个通信板的控制命令           |
| `SwitchCommand.msg` | 上位机发送给 STM32 的整体控制命令 |
| `BoardStatus.msg`   | 单个通信板返回的状态信息         |
| `SwitchStatus.msg`  | STM32 返回给上位机的整体状态信息  |

---

### 4.2 `switch_udp_bridge`

该包用于实现 ROS2 与 STM32 之间的 UDP 通信桥。

核心节点：

```text
udp_bridge_node
```

主要功能：

1. 订阅 `/switch/command`。
2. 将 `SwitchCommand` 消息打包为 56 字节 UDP 命令帧。
3. 对 3 个 17 字节通信板命令帧分别计算 CRC。
4. 通过 UDP 将 56 字节命令发送给 STM32。
5. 接收 STM32 返回的 170 字节状态帧。
6. 解析 3 个 46 字节通信板状态帧。
7. 校验每个状态帧 CRC。
8. 发布 `/switch/status` 和 `/switch/status_raw`。

---

## 5. ROS2 Topic 说明

| Topic                | 消息类型                                  | 方向                | 说明           |
| -------------------- | ------------------------------------- | ----------------- | ------------ |
| `/switch/command`    | `switch_interfaces/msg/SwitchCommand` | ROS2 控制节点 → UDP 桥 | 上位机控制命令      |
| `/switch/status`     | `switch_interfaces/msg/SwitchStatus`  | UDP 桥 → ROS2 控制节点 | 解析后的系统状态     |
| `/switch/status_raw` | `std_msgs/msg/UInt8MultiArray`        | UDP 桥 → 调试工具      | 原始 170 字节状态帧 |

---

## 6. ROS2 消息结构

### 6.1 `BoardCommand.msg`

```text
uint8 command_type
uint8 id
float32 steering_angle
float32 wheel_speed
float32 fan_pressure
```

字段说明：

| 字段               | 类型        | 说明                                       |
| ---------------- | --------- | ---------------------------------------- |
| `command_type`   | `uint8`   | 命令类型，`0x01` 运动命令，`0x02` 读取状态，`0x03` 归零命令 |
| `id`             | `uint8`   | 通信板编号，`0x01` 开始，`0x00` 为广播               |
| `steering_angle` | `float32` | 舵轮目标角度，单位 rad                            |
| `wheel_speed`    | `float32` | 前进轮目标速度，单位 rad/s                         |
| `fan_pressure`   | `float32` | 风箱目标气压，单位 kPa，负值表示负压，0 表示大气压             |

---

### 6.2 `SwitchCommand.msg`

```text
BoardCommand[3] boards
uint8[4] power_board
uint8 led
```

字段说明：

| 字段            | 类型                | 说明       |
| ------------- | ----------------- | -------- |
| `boards`      | `BoardCommand[3]` | 3 个通信板命令 |
| `power_board` | `uint8[4]`        | 电源板控制命令  |
| `led`         | `uint8`           | 灯带命令     |

---

### 6.3 `BoardStatus.msg`

```text
uint8 frame_head
uint8 command_type
uint8 id

float32 steering_position
float32 steering_velocity
float32 steering_torque

float32 wheel_position
float32 wheel_velocity
float32 wheel_torque

float32 fan_speed
float32 fan_current

int16 fan_temperature
int16 fan_driver_temperature

float32 chamber_pressure

uint8 error_code

uint16 received_crc
uint16 calculated_crc
bool crc_ok
```

字段说明：

| 字段                       | 类型        | 说明                |
| ------------------------ | --------- | ----------------- |
| `frame_head`             | `uint8`   | 帧头，正常为 `0xFE`     |
| `command_type`           | `uint8`   | 返回接收到的命令类型        |
| `id`                     | `uint8`   | 返回接收到的编号          |
| `steering_position`      | `float32` | 舵轮位置              |
| `steering_velocity`      | `float32` | 舵轮速度              |
| `steering_torque`        | `float32` | 舵轮力矩              |
| `wheel_position`         | `float32` | 前进轮位置，单位 rad      |
| `wheel_velocity`         | `float32` | 前进轮速度，单位 rad/s    |
| `wheel_torque`           | `float32` | 前进轮力矩，单位 Nm       |
| `fan_speed`              | `float32` | 风机转速，单位 rad/s     |
| `fan_current`            | `float32` | 风机电流，单位 A         |
| `fan_temperature`        | `int16`   | 风机温度              |
| `fan_driver_temperature` | `int16`   | 风机驱动温度            |
| `chamber_pressure`       | `float32` | 吸附腔气压，单位 kPa      |
| `error_code`             | `uint8`   | 错误码               |
| `received_crc`           | `uint16`  | 状态帧中接收到的 CRC      |
| `calculated_crc`         | `uint16`  | ROS2 侧重新计算得到的 CRC |
| `crc_ok`                 | `bool`    | CRC 校验结果          |

---

### 6.4 `SwitchStatus.msg`

```text
builtin_interfaces/Time stamp

BoardStatus[3] boards

uint8[31] power_board
uint8 switch_state

uint8[] raw
```

字段说明：

| 字段             | 类型                        | 说明                |
| -------------- | ------------------------- | ----------------- |
| `stamp`        | `builtin_interfaces/Time` | ROS2 接收到状态帧的时间戳   |
| `boards`       | `BoardStatus[3]`          | 3 个通信板状态          |
| `power_board`  | `uint8[31]`               | 电源板状态帧            |
| `switch_state` | `uint8`                   | 开关状态              |
| `raw`          | `uint8[]`                 | 原始 170 字节 UDP 状态帧 |

---

## 7. UDP 通信协议

## 7.1 上位机 → STM32：56 字节命令帧

对应 STM32 端：

```c
uint8_t ethernetReceive[56];
```

帧结构如下：

|      字节地址 | 长度 | 内容        |
| --------: | -: | --------- |
|  `0 - 16` | 17 | 1 号通信板命令帧 |
| `17 - 33` | 17 | 2 号通信板命令帧 |
| `34 - 50` | 17 | 3 号通信板命令帧 |
| `51 - 54` |  4 | 电源板命令帧    |
|      `55` |  1 | 灯带命令      |

---

## 7.2 单个通信板命令帧：17 字节

|      字节地址 | 长度 | 内容      | 说明                       |
| --------: | -: | ------- | ------------------------ |
|       `0` |  1 | 帧头      | 固定 `0xFE`                |
|       `1` |  1 | 命令类型    | `0x01` / `0x02` / `0x03` |
|       `2` |  1 | 编号      | `0x01` 开始，`0x00` 为广播     |
|   `3 - 6` |  4 | 舵轮目标角度  | `float32`，单位 rad         |
|  `7 - 10` |  4 | 前进轮目标速度 | `float32`，单位 rad/s       |
| `11 - 14` |  4 | 风箱目标气压  | `float32`，单位 kPa         |
| `15 - 16` |  2 | CRC     | 对 `0 - 14` 共 15 字节计算     |

注意：对于读取状态命令 `0x02` 和归零命令 `0x03`，字节 `3 - 14` 作为占位区使用。由于整个命令帧总长为 17 字节，CRC 位于 `15 - 16`，因此占位区实际为 `3 - 14` 共 12 字节。

---

## 7.3 命令类型

| 命令类型   |     编号 | 说明            |
| ------ | -----: | ------------- |
| 运动命令   | `0x01` | 向运动控制板发送运动命令  |
| 读取状态命令 | `0x02` | 只读取，不改变之前运动状态 |
| 归零命令   | `0x03` | 发送后舵轮自动旋转归零   |

---

## 7.4 灯带命令

| 命令类型     |    命令码 |
| -------- | -----: |
| 无命令，保持现状 | `0x00` |
| 通电       | `0x01` |

---

## 7.5 电源命令

| 命令类型         |    命令码 |
| ------------ | -----: |
| 无命令，保持现状     | `0x00` |
| 从电源供电切换为电池供电 | `0x01` |
| 从电池供电切换为电源供电 | `0x02` |

---

## 7.6 STM32 → 上位机：170 字节状态帧

对应 STM32 端：

```c
uint8_t ethernetSend[170];
```

帧结构如下：

|        字节地址 | 长度 | 内容        |
| ----------: | -: | --------- |
|    `0 - 45` | 46 | 1 号通信板状态帧 |
|   `46 - 91` | 46 | 2 号通信板状态帧 |
|  `92 - 137` | 46 | 3 号通信板状态帧 |
| `138 - 168` | 31 | 电源板状态帧    |
|       `169` |  1 | 开关状态      |

---

## 7.7 单个通信板状态帧：46 字节

|      字节地址 | 长度 | 内容     | 说明                   |
| --------: | -: | ------ | -------------------- |
|       `0` |  1 | 帧头     | 固定 `0xFE`            |
|       `1` |  1 | 命令类型   | 返回接收到的命令帧            |
|       `2` |  1 | 编号     | 返回接收到的编号             |
|   `3 - 6` |  4 | 舵轮位置   | `float32`            |
|  `7 - 10` |  4 | 舵轮速度   | `float32`            |
| `11 - 14` |  4 | 舵轮力矩   | `float32`            |
| `15 - 18` |  4 | 前进轮位置  | `float32`，单位 rad     |
| `19 - 22` |  4 | 前进轮速度  | `float32`，单位 rad/s   |
| `23 - 26` |  4 | 前进轮力矩  | `float32`，单位 Nm      |
| `27 - 30` |  4 | 风机转速   | `float32`，单位 rad/s   |
| `31 - 34` |  4 | 风机电流   | `float32`，单位 A       |
| `35 - 36` |  2 | 风机温度   | `int16`              |
| `37 - 38` |  2 | 风机驱动温度 | `int16`              |
| `39 - 42` |  4 | 吸附腔气压  | `float32`，单位 kPa     |
|      `43` |  1 | 错误码    | `uint8`              |
| `44 - 45` |  2 | CRC    | 对 `0 - 43` 共 44 字节计算 |

---

## 7.8 开关状态

| 状态类型     |                 状态码 |
| -------- | ------------------: |
| 主电源开关    | 按下 `0x01`，断开 `0x00` |
| 风机使能开关 1 | 按下 `0x02`，断开 `0x00` |
| 风机使能开关 2 | 按下 `0x03`，断开 `0x00` |

开关状态码由以下方式组合：

```text
switch_state = 主电源开关 | 风机使能开关1 | 风机使能开关2
```

注意：如果开关状态严格按按位或表示，第三个开关通常应使用 `0x04` 才能避免与 `0x01 | 0x02 = 0x03` 冲突。当前 README 按现有协议原文记录，实际工程联调时需要与 STM32 端确认该定义。

---

## 8. CRC 校验说明

本项目采用如下 CRC 算法：

```c
uint16_t CalculateCRC(const uint8_t *data, uint16_t length)
{
    uint16_t crc = 0xFFFF;
    uint16_t i, j;

    for (i = 0; i < length; i++) {
        crc ^= data[i];

        for (j = 0; j < 8; j++) {
            if (crc & 0x0001) {
                crc = (crc >> 1) ^ 0xA001;
            } else {
                crc >>= 1;
            }
        }
    }

    return crc;
}
```

该算法特征：

| 项目     | 值            |
| ------ | ------------ |
| CRC 类型 | CRC16-Modbus |
| 初始值    | `0xFFFF`     |
| 多项式    | `0xA001`     |
| 移位方向   | 右移，低位先处理     |
| 返回值    | `uint16_t`   |

---

## 8.1 CRC 计算范围

| 帧类型    |    总长度 |         CRC 计算范围 |  CRC 存放位置 |
| ------ | -----: | ---------------: | --------: |
| 通信板命令帧 |  17 字节 | `0 - 14`，共 15 字节 | `15 - 16` |
| 通信板状态帧 |  46 字节 | `0 - 43`，共 44 字节 | `44 - 45` |
| 上层命令帧  |  56 字节 |      不额外计算整体 CRC |         无 |
| 上层状态帧  | 170 字节 |      不额外计算整体 CRC |         无 |

---

## 8.2 CRC 字节序

当前 ROS2 侧代码按小端方式写入 CRC：

```cpp
void write_u16_le(uint8_t *dst, uint16_t value)
{
  dst[0] = static_cast<uint8_t>(value & 0xFF);
  dst[1] = static_cast<uint8_t>((value >> 8) & 0xFF);
}
```

也就是说：

```text
低字节在前，高字节在后
```

如果 STM32 侧直接将 `uint16_t crc` 拷贝到数组，并且 STM32 为常见小端系统，那么该顺序通常一致。

例如：

```c
uint16_t crc = CalculateCRC(data, 15);
data[15] = crc & 0xFF;
data[16] = crc >> 8;
```

如果 STM32 端采用高字节在前：

```c
data[15] = crc >> 8;
data[16] = crc & 0xFF;
```

则需要修改 ROS2 侧 CRC 写入和读取函数。

---

## 9. 字节序说明

### 9.1 float 字节序

ROS2 侧代码按小端方式写入 `float32`：

```text
float 1.0f = 00 00 80 3F
```

STM32 通常也是小端架构，因此一般可以直接匹配。

### 9.2 int16 / uint16 字节序

`int16` 和 `uint16` 也按小端解析：

```text
低字节在前，高字节在后
```

联调中若发现温度、CRC、float 数据异常，应优先检查字节序是否一致。

---

## 10. `udp_bridge_node.cpp` 文件说明

文件路径：

```text
switch_udp_bridge/src/udp_bridge_node.cpp
```

该文件包含 UDP 通信、协议打包、协议解析和 ROS2 topic 转换的全部核心逻辑。

---

## 10.1 协议常量

```cpp
constexpr size_t ETHERNET_RECEIVE_LEN = 56;
constexpr size_t ETHERNET_SEND_LEN = 170;

constexpr size_t BOARD_CMD_LEN = 17;
constexpr size_t BOARD_STATUS_LEN = 46;

constexpr uint8_t FRAME_HEAD = 0xFE;

constexpr size_t BOARD_CMD_CRC_OFFSET = 15;
constexpr size_t BOARD_CMD_CRC_CALC_LEN = 15;

constexpr size_t BOARD_STATUS_CRC_OFFSET = 44;
constexpr size_t BOARD_STATUS_CRC_CALC_LEN = 44;

constexpr size_t POWER_CMD_OFFSET = 51;
constexpr size_t LED_CMD_OFFSET = 55;

constexpr size_t POWER_STATUS_OFFSET = 138;
constexpr size_t SWITCH_STATUS_OFFSET = 169;
```

作用：

| 常量                          | 说明                        |
| --------------------------- | ------------------------- |
| `ETHERNET_RECEIVE_LEN`      | 上位机发送给 STM32 的命令帧长度，56 字节 |
| `ETHERNET_SEND_LEN`         | STM32 返回给上位机的状态帧长度，170 字节 |
| `BOARD_CMD_LEN`             | 单个通信板命令帧长度，17 字节          |
| `BOARD_STATUS_LEN`          | 单个通信板状态帧长度，46 字节          |
| `FRAME_HEAD`                | 帧头，固定 `0xFE`              |
| `BOARD_CMD_CRC_OFFSET`      | 命令帧 CRC 起始位置，15           |
| `BOARD_CMD_CRC_CALC_LEN`    | 命令帧 CRC 计算长度，15           |
| `BOARD_STATUS_CRC_OFFSET`   | 状态帧 CRC 起始位置，44           |
| `BOARD_STATUS_CRC_CALC_LEN` | 状态帧 CRC 计算长度，44           |
| `POWER_CMD_OFFSET`          | 电源命令起始位置，51               |
| `LED_CMD_OFFSET`            | 灯带命令位置，55                 |
| `POWER_STATUS_OFFSET`       | 电源状态起始位置，138              |
| `SWITCH_STATUS_OFFSET`      | 开关状态位置，169                |

---

## 10.2 CRC 函数

```cpp
uint16_t CalculateCRC(const uint8_t *data, uint16_t length)
```

作用：

* 使用和 STM32 端一致的 CRC16-Modbus 算法。
* 用于计算通信板命令帧 CRC。
* 用于校验通信板状态帧 CRC。

调用场景：

```cpp
const uint16_t crc = CalculateCRC(dst, 15);
```

```cpp
status.calculated_crc = CalculateCRC(src, 44);
```

---

## 10.3 字节写入和读取函数

### `write_u16_le`

```cpp
void write_u16_le(uint8_t *dst, uint16_t value)
```

作用：

* 将 `uint16_t` 按小端写入字节数组。
* 用于写 CRC。

---

### `read_u16_le`

```cpp
uint16_t read_u16_le(const uint8_t *src)
```

作用：

* 从字节数组中按小端读取 `uint16_t`。
* 用于读取接收到的 CRC。

---

### `write_u32_le`

```cpp
void write_u32_le(uint8_t *dst, uint32_t value)
```

作用：

* 将 `uint32_t` 按小端写入字节数组。
* 是 `write_float_le` 的底层函数。

---

### `read_u32_le`

```cpp
uint32_t read_u32_le(const uint8_t *src)
```

作用：

* 从字节数组中按小端读取 `uint32_t`。
* 是 `read_float_le` 的底层函数。

---

### `write_float_le`

```cpp
void write_float_le(uint8_t *dst, float value)
```

作用：

* 将 `float` 按 IEEE754 小端格式写入字节数组。
* 用于写入舵轮目标角度、前进轮目标速度、风箱目标气压。

---

### `read_float_le`

```cpp
float read_float_le(const uint8_t *src)
```

作用：

* 从字节数组中按小端格式读取 `float`。
* 用于解析通信板状态中的位置、速度、力矩、电流、气压等字段。

---

### `read_i16_le`

```cpp
int16_t read_i16_le(const uint8_t *src)
```

作用：

* 从字节数组中按小端格式读取 `int16_t`。
* 用于解析风机温度和风机驱动温度。

---

## 10.4 协议打包函数

### `pack_board_command`

```cpp
void pack_board_command(
  uint8_t *dst,
  const switch_interfaces::msg::BoardCommand &cmd)
```

作用：

* 将单个 `BoardCommand` 转换为 17 字节通信板命令帧。
* 自动写入帧头 `0xFE`。
* 根据命令类型决定是否写入 3 个 float 数据。
* 自动计算并写入 CRC。

打包结果：

```text
byte[0]      = 0xFE
byte[1]      = command_type
byte[2]      = id
byte[3-6]    = steering_angle
byte[7-10]   = wheel_speed
byte[11-14]  = fan_pressure
byte[15-16]  = CRC
```

对于 `0x02` 和 `0x03` 命令：

```text
byte[3-14] = 0
byte[15-16] = CRC
```

---

### `pack_switch_command`

```cpp
std::array<uint8_t, ETHERNET_RECEIVE_LEN> pack_switch_command(
  const switch_interfaces::msg::SwitchCommand &cmd)
```

作用：

* 将 ROS2 的 `SwitchCommand` 消息打包为完整 56 字节 UDP 命令帧。

打包映射：

```text
frame[0-16]    = boards[0]
frame[17-33]   = boards[1]
frame[34-50]   = boards[2]
frame[51-54]   = power_board
frame[55]      = led
```

---

## 10.5 协议解包函数

### `unpack_board_status`

```cpp
switch_interfaces::msg::BoardStatus unpack_board_status(const uint8_t *src)
```

作用：

* 将单个 46 字节通信板状态帧解析为 ROS2 `BoardStatus` 消息。
* 自动读取所有 float、int16 和 uint8 字段。
* 自动读取接收到的 CRC。
* 自动重新计算 CRC。
* 生成 `crc_ok` 校验结果。

CRC 校验逻辑：

```cpp
status.received_crc = read_u16_le(&src[44]);
status.calculated_crc = CalculateCRC(src, 44);
status.crc_ok = status.received_crc == status.calculated_crc;
```

---

### `unpack_switch_status`

```cpp
switch_interfaces::msg::SwitchStatus unpack_switch_status(
  const std::array<uint8_t, ETHERNET_SEND_LEN> &frame,
  const rclcpp::Time &stamp)
```

作用：

* 将完整 170 字节 UDP 状态帧解析为 ROS2 `SwitchStatus` 消息。

解析映射：

```text
frame[0-45]     = boards[0]
frame[46-91]    = boards[1]
frame[92-137]   = boards[2]
frame[138-168]  = power_board
frame[169]      = switch_state
```

---

## 10.6 `UdpBridgeNode` 类说明

### 构造函数

```cpp
UdpBridgeNode()
```

主要完成：

1. 声明和读取 ROS2 参数。
2. 初始化 UDP socket。
3. 创建 `/switch/command` 订阅者。
4. 创建 `/switch/status` 发布者。
5. 创建 `/switch/status_raw` 发布者。
6. 创建 200Hz 发送定时器。
7. 创建 1ms 接收轮询定时器。

---

### `setup_socket`

```cpp
void setup_socket()
```

作用：

* 创建 UDP socket。
* 绑定本机监听端口。
* 设置 socket 为非阻塞模式。
* 设置 STM32 目标 IP 和端口。

关键逻辑：

```cpp
sock_fd_ = socket(AF_INET, SOCK_DGRAM, 0);
bind(sock_fd_, ...);
fcntl(sock_fd_, F_SETFL, flags | O_NONBLOCK);
inet_pton(AF_INET, stm32_ip_.c_str(), &stm32_addr_.sin_addr);
```

---

### `command_callback`

```cpp
void command_callback(const switch_interfaces::msg::SwitchCommand::SharedPtr msg)
```

作用：

* 接收 `/switch/command` 最新命令。
* 保存为 `latest_command_`。
* 记录最后一次收到命令的时间。
* 标记当前已有有效命令。

---

### `tx_timer_callback`

```cpp
void tx_timer_callback()
```

作用：

* 按 `send_rate_hz` 定时发送 UDP 命令帧。
* 默认频率为 200Hz。
* 如果没有收到过命令，则不发送。
* 如果超过 `command_timeout_ms` 没有收到新命令，则停止发送。
* 将最新 `SwitchCommand` 打包成 56 字节并通过 UDP 发送。

关键逻辑：

```cpp
const auto frame = protocol::pack_switch_command(latest_command_);
sendto(sock_fd_, frame.data(), frame.size(), 0, ...);
```

---

### `rx_timer_callback`

```cpp
void rx_timer_callback()
```

作用：

* 非阻塞接收 UDP 数据。
* 只接受长度为 170 字节的数据包。
* 解析 170 字节状态帧。
* 检查 3 个通信板状态帧的帧头和 CRC。
* 发布 `/switch/status`。
* 发布 `/switch/status_raw`。

关键逻辑：

```cpp
received = recvfrom(sock_fd_, buffer, sizeof(buffer), 0, ...);
```

若长度不是 170 字节：

```cpp
Ignore UDP packet
```

若 CRC 错误，会打印：

```text
Board N CRC error. received=0xXXXX, calculated=0xYYYY
```

---

## 11. 参数文件说明

文件路径：

```text
switch_udp_bridge/config/udp_bridge.yaml
```

内容示例：

```yaml
udp_bridge_node:
  ros__parameters:
    stm32_ip: "192.168.1.10"
    stm32_port: 50000
    local_port: 50001
    send_rate_hz: 200.0
    command_timeout_ms: 50.0
```

参数说明：

| 参数                   |            默认值 | 说明                   |
| -------------------- | -------------: | -------------------- |
| `stm32_ip`           | `192.168.1.10` | STM32 的 IP 地址        |
| `stm32_port`         |        `50000` | STM32 监听 UDP 端口      |
| `local_port`         |        `50001` | ROS2 电脑本机监听 UDP 端口   |
| `send_rate_hz`       |        `200.0` | ROS2 向 STM32 发送命令的频率 |
| `command_timeout_ms` |         `50.0` | 超过该时间没收到新命令，则停止发送    |

---

## 12. Launch 文件说明

文件路径：

```text
switch_udp_bridge/launch/udp_bridge.launch.py
```

作用：

* 启动 `udp_bridge_node`。
* 自动加载 `config/udp_bridge.yaml` 参数文件。

运行命令：

```bash
ros2 launch switch_udp_bridge udp_bridge.launch.py
```

---

## 13. 编译方法

进入工作空间根目录：

```bash
cd ~/switch_ros2_ws
```

安装依赖：

```bash
rosdep install --from-paths src --ignore-src -r -y
```

编译：

```bash
colcon build --symlink-install
```

加载环境：

```bash
source install/setup.bash
```

建议每次新开终端都执行：

```bash
source ~/switch_ros2_ws/install/setup.bash
```

---

## 14. 检查接口是否生成成功

查看接口列表：

```bash
ros2 interface list | grep switch_interfaces
```

应该看到：

```text
switch_interfaces/msg/BoardCommand
switch_interfaces/msg/BoardStatus
switch_interfaces/msg/SwitchCommand
switch_interfaces/msg/SwitchStatus
```

查看 `SwitchCommand`：

```bash
ros2 interface show switch_interfaces/msg/SwitchCommand
```

应该显示类似：

```text
BoardCommand[3] boards
        uint8 command_type
        uint8 id
        float32 steering_angle
        float32 wheel_speed
        float32 fan_pressure
uint8[4] power_board
uint8 led
```

---

## 15. 运行方法

### 15.1 终端 1：启动 UDP 桥接节点

```bash
cd ~/switch_ros2_ws
source install/setup.bash
ros2 launch switch_udp_bridge udp_bridge.launch.py
```

正常启动后应看到：

```text
UDP bridge started. local_port=50001, stm32=192.168.1.10:50000, send_rate=200.0 Hz
```

---

### 15.2 终端 2：发布一次读取状态命令

```bash
cd ~/switch_ros2_ws
source install/setup.bash
ros2 topic pub --once /switch/command switch_interfaces/msg/SwitchCommand "{boards: [
  {command_type: 2, id: 1, steering_angle: 0.0, wheel_speed: 0.0, fan_pressure: 0.0},
  {command_type: 2, id: 2, steering_angle: 0.0, wheel_speed: 0.0, fan_pressure: 0.0},
  {command_type: 2, id: 3, steering_angle: 0.0, wheel_speed: 0.0, fan_pressure: 0.0}
], power_board: [0, 0, 0, 0], led: 0}"
```

---

### 15.3 终端 2：以 200Hz 发布读取状态命令

```bash
ros2 topic pub -r 200 /switch/command switch_interfaces/msg/SwitchCommand "{boards: [
  {command_type: 2, id: 1, steering_angle: 0.0, wheel_speed: 0.0, fan_pressure: 0.0},
  {command_type: 2, id: 2, steering_angle: 0.0, wheel_speed: 0.0, fan_pressure: 0.0},
  {command_type: 2, id: 3, steering_angle: 0.0, wheel_speed: 0.0, fan_pressure: 0.0}
], power_board: [0, 0, 0, 0], led: 0}"
```

---

### 15.4 发布运动命令

```bash
ros2 topic pub -r 200 /switch/command switch_interfaces/msg/SwitchCommand "{boards: [
  {command_type: 1, id: 1, steering_angle: 0.0, wheel_speed: 1.0, fan_pressure: 0.0},
  {command_type: 1, id: 2, steering_angle: 0.0, wheel_speed: 1.0, fan_pressure: 0.0},
  {command_type: 1, id: 3, steering_angle: 0.0, wheel_speed: 1.0, fan_pressure: 0.0}
], power_board: [0, 0, 0, 0], led: 1}"
```

---

### 15.5 查看状态

```bash
ros2 topic echo /switch/status
```

查看原始 170 字节：

```bash
ros2 topic echo /switch/status_raw
```

---

## 16. 常用调试命令

### 16.1 查看 topic 列表

```bash
ros2 topic list
```

应该包含：

```text
/switch/command
/switch/status
/switch/status_raw
```

---

### 16.2 查看 `/switch/command` 是否有订阅者

```bash
ros2 topic info /switch/command
```

正常情况下，在 `udp_bridge_node` 运行后应显示：

```text
Subscription count: 1
```

如果显示：

```text
Subscription count: 0
```

说明 UDP 桥接节点没有运行，或者节点启动失败。

---

### 16.3 查看节点是否存在

```bash
ros2 node list
```

应该看到：

```text
/udp_bridge_node
```

---

### 16.4 直接运行节点

如果 launch 文件有问题，可以直接运行：

```bash
ros2 run switch_udp_bridge udp_bridge_node
```

---

### 16.5 查看包内可执行文件

```bash
ros2 pkg executables switch_udp_bridge
```

应该看到：

```text
switch_udp_bridge udp_bridge_node
```

---

### 16.6 抓包查看 UDP 数据

查看发送给 STM32 的 UDP 数据：

```bash
sudo tcpdump -i any udp port 50000 -X
```

如果只想看本机监听端口：

```bash
sudo tcpdump -i any udp port 50001 -X
```

---

## 17. 推荐调试流程

不要一开始就把所有外设全部接上，建议按以下步骤逐步联调：

```text
第 1 步：电脑 ping 通 STM32

第 2 步：ROS2 启动 udp_bridge_node

第 3 步：ROS2 发布 /switch/command

第 4 步：用 tcpdump 确认 ROS2 发出了 56 字节 UDP 包

第 5 步：STM32 只打印接收到的 UDP 长度和原始字节

第 6 步：STM32 校验 3 个 17 字节命令帧 CRC

第 7 步：STM32 暂时不接下层 RS485，先伪造 170 字节状态帧返回

第 8 步：ROS2 确认能接收到 /switch/status_raw

第 9 步：ROS2 确认 /switch/status 解析正确，crc_ok 为 true

第 10 步：STM32 接入 1 路通信板 RS485

第 11 步：STM32 接入 3 路通信板 RS485

第 12 步：接入电源板 RS485

第 13 步：接入灯带 SPI

第 14 步：接入风机开关 GPIO

第 15 步：做完整 200Hz 联调
```

---

## 18. 常见问题

### 18.1 `The passed message type is invalid`

原因通常是当前终端没有加载工作空间环境。

解决方法：

```bash
cd ~/switch_ros2_ws
source install/setup.bash
```

然后检查：

```bash
ros2 interface show switch_interfaces/msg/SwitchCommand
```

---

### 18.2 `Waiting for at least 1 matching subscription(s)...`

原因：`/switch/command` 没有订阅者。

解决方法：先启动 UDP 桥接节点：

```bash
ros2 launch switch_udp_bridge udp_bridge.launch.py
```

然后再发布：

```bash
ros2 topic pub --once /switch/command switch_interfaces/msg/SwitchCommand "..."
```

---

### 18.3 `Subscription count: 0`

说明 `udp_bridge_node` 没有成功订阅 `/switch/command`。

检查节点是否运行：

```bash
ros2 node list
```

如果没有 `/udp_bridge_node`，重新运行：

```bash
ros2 launch switch_udp_bridge udp_bridge.launch.py
```

---

### 18.4 收不到 `/switch/status`

可能原因：

1. STM32 没有返回 UDP 数据。
2. STM32 返回的数据不是 170 字节。
3. IP 或端口配置不一致。
4. 防火墙阻止 UDP。
5. 网卡不在同一网段。

检查方法：

```bash
sudo tcpdump -i any udp port 50001 -X
```

确认电脑是否收到 STM32 返回的数据。

---

### 18.5 CRC 错误

如果出现：

```text
Board N CRC error. received=0xXXXX, calculated=0xYYYY
```

优先检查以下内容：

1. CRC 计算长度是否正确。

   * 命令帧：对 `0 - 14` 共 15 字节计算。
   * 状态帧：对 `0 - 43` 共 44 字节计算。
2. CRC 字节序是否一致。

   * 当前 ROS2 代码为低字节在前。
3. STM32 是否对同一段数据计算 CRC。
4. 状态帧是否存在错位。
5. UDP 返回长度是否严格为 170 字节。

---

### 18.6 float 数据解析异常

例如速度、角度、气压出现极大值或乱码，优先检查：

1. STM32 和 ROS2 是否都是小端。
2. STM32 是否使用 4 字节 `float`。
3. 字节偏移是否一致。
4. 通信帧是否错位。

常见验证：

```text
float 1.0f 小端字节应为：00 00 80 3F
float 0.0f 小端字节应为：00 00 00 00
```

---

## 19. 安全逻辑说明

风机控制逻辑涉及物理按钮，因此建议最终放在 STM32 端实现。

目标逻辑如下：

```text
1. 没有任何指令时：气压命令为 0，风机不动作。
2. 两个风机按钮都按下，但电脑指令为 0：目标负压设为 -10 kPa。
3. 两个风机按钮没有都按下，但电脑发送目标负压：按电脑目标值向下发送。
4. 两个风机按钮都按下，电脑也发送目标负压：按电脑目标值向下发送。
```

推荐 STM32 端伪代码：

```c
if (no_command_from_pc) {
    target_pressure = 0.0f;
} else if (fan_switch_1_pressed && fan_switch_2_pressed && pc_pressure == 0.0f) {
    target_pressure = -10.0f;
} else if (pc_pressure != 0.0f) {
    target_pressure = pc_pressure;
} else {
    target_pressure = 0.0f;
}
```

ROS2 端当前实现：

```text
如果没有收到 /switch/command，则不发送 UDP。
如果超过 command_timeout_ms 没有收到新的 /switch/command，也停止发送 UDP。
```

这样可以避免 ROS2 端在控制节点停止后继续发送旧命令。

---

## 20. 开发注意事项

1. 每个新终端都要执行：

```bash
source ~/switch_ros2_ws/install/setup.bash
```

2. 修改 `.msg` 文件后必须重新编译：

```bash
cd ~/switch_ros2_ws
colcon build --symlink-install
source install/setup.bash
```

3. 如果接口生成异常，可以清理后重新编译：

```bash
rm -rf build install log
colcon build --symlink-install
source install/setup.bash
```

4. 不建议控制算法节点直接操作 UDP socket。

5. 不建议在多个节点中重复实现协议打包和 CRC，协议处理应集中在 `switch_udp_bridge` 中。

6. 联调时优先使用 `/switch/status_raw` 和 `tcpdump` 定位问题。

7. 状态帧 CRC 错误时，不要先怀疑 ROS2 消息，优先检查字节偏移、长度、CRC 字节序和 STM32 端打包逻辑。

---

## 21. 后续扩展建议

后续可以继续增加以下功能：

1. 新增独立控制节点，例如：

```text
switch_control_node
```

用于根据机器人运动规划生成 `/switch/command`。

2. 新增日志节点，例如：

```text
switch_logger_node
```

用于记录 `/switch/status` 到 CSV 或 rosbag。

3. 新增诊断节点，例如：

```text
switch_diagnostics_node
```

用于统计：

* UDP 发送频率；
* UDP 接收频率；
* 丢包率；
* CRC 错误次数；
* 状态帧超时；
* 各通信板在线状态。

4. 新增 launch 参数，用于快速切换不同 STM32 IP。

5. 新增仿真节点，用软件模拟 STM32 返回 170 字节数据，方便无硬件时调试 ROS2 侧代码。

---

## 22. 当前项目核心检查清单

正式联调前请确认：

```text
[ ] ROS2 能成功编译
[ ] switch_interfaces/msg/SwitchCommand 能正常显示
[ ] udp_bridge_node 能正常启动
[ ] /switch/command 有 1 个订阅者
[ ] ROS2 能以 200Hz 发布 /switch/command
[ ] tcpdump 能看到 56 字节 UDP 包
[ ] STM32 能收到 56 字节 UDP 包
[ ] STM32 能正确校验 3 个 17 字节命令帧 CRC
[ ] STM32 能返回 170 字节 UDP 包
[ ] ROS2 能发布 /switch/status_raw
[ ] ROS2 能发布 /switch/status
[ ] boards[0].crc_ok 为 true
[ ] boards[1].crc_ok 为 true
[ ] boards[2].crc_ok 为 true
[ ] float 数据解析正常
[ ] 开关状态解析正常
[ ] 200Hz 长时间运行稳定
```

---

## 23. 总结

本项目 ROS2 侧的设计目标是将底层通信协议封装为标准 ROS2 topic：

```text
/switch/command  →  UDP 56 字节命令帧
UDP 170 字节状态帧  →  /switch/status
```

协议打包、解包、CRC、字节序和 socket 通信均集中在 `udp_bridge_node` 内部。其他控制节点只需要面对结构化 ROS2 消息，从而降低系统耦合度，方便调试、维护和扩展。
