# 帧头，固定为 0xFE，用于帧同步
uint8 frame_head
# 命令类型，回显接收到的命令类型（1=运动命令，2=读取状态，3=归零命令）
uint8 command_type
# 通信板 ID，标识当前状态帧的来源通信板
uint8 id

# 舵轮当前位置，单位：弧度
float32 steering_position
# 舵轮当前速度，单位：弧度/秒
float32 steering_velocity
# 舵轮当前力矩，单位：牛·米
float32 steering_torque

# 前进轮当前位置，单位：弧度
float32 wheel_position
# 前进轮当前速度，单位：米/秒
float32 wheel_velocity
# 前进轮当前力矩，单位：牛·米
float32 wheel_torque

# 风机当前转速，单位：转/分钟
float32 fan_speed
# 风机当前电流，单位：安培
float32 fan_current

# 风机温度，单位：摄氏度
int16 fan_temperature
# 风机驱动温度，单位：摄氏度
int16 fan_driver_temperature

# 吸附腔当前气压，单位：帕斯卡
float32 chamber_pressure

# 错误码，0 表示正常，非 0 表示存在故障
uint8 error_code

# 接收到的 CRC16 校验值（来自 STM32 状态帧的原始 CRC 字段）
uint16 received_crc
# 本地计算的 CRC16 校验值（用于校验数据完整性）
uint16 calculated_crc
# CRC 校验结果，true 表示校验通过，false 表示校验失败
bool crc_ok