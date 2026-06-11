# 机器人状态 (从 Go2W SDK 获取)
std_msgs/Header header
float64 x                  # X位置 (米)
float64 y                  # Y位置 (米)
float64 yaw                # 航向角 (弧度)
float64 vx                 # X方向速度 (m/s)
float64 vy                 # Y方向速度 (m/s)
float64 vyaw               # 角速度 (rad/s)
float64[4] imu_quaternion  # IMU四元数 [w, x, y, z]
uint8 mode                 # 运动模式: 0=行走, 1=轮式
uint8 gait                 # 步态类型
