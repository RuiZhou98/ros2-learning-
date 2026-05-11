# 时间戳，记录状态帧被 ROS2 节点接收到的时间
builtin_interfaces/Time stamp

# 三个通信板的状态数组，索引 0/1/2 分别对应 1/2/3 号通信板
BoardStatus[3] boards

# 电源板状态数据，31 个字节原始数据，具体含义由硬件协议定义
uint8[31] power_board
# 开关状态，1 字节，具体含义由硬件协议定义
uint8 switch_state

# 原始数据缓冲区，通常用于存储 STM32 返回的完整 170 字节状态帧
uint8[] raw