# 三个通信板的命令数组，索引 0/1/2 分别对应 1/2/3 号通信板
BoardCommand[3] boards
# 电源板控制命令，4 个字节，具体含义由硬件协议定义
uint8[4] power_board
# 灯带控制命令，具体含义由硬件协议定义
uint8 led