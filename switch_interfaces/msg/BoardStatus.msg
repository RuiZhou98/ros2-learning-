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