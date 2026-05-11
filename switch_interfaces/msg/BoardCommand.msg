# 命令类型：1=运动命令，2=读取状态，3=归零命令
uint8 command_type
# 通信板 ID，用于标识目标通信板
uint8 id
# 舵轮目标角度，单位：弧度（float32，小端字节序）
float32 steering_angle
# 前进轮目标速度，单位：米/秒（float32，小端字节序）
float32 wheel_speed
# 风箱目标气压，单位：帕斯卡（float32，小端字节序）
float32 fan_pressure