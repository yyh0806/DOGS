# 目标检测结果
std_msgs/Header header
string class_name          # 目标类别 (person, car, etc.)
float64 confidence         # 置信度 [0, 1]
float64[4] bbox            # 边界框 [x1, y1, x2, y2] 归一化坐标
float64 robot_x            # 发现时机器人 X
float64 robot_y            # 发现时机器人 Y
float64 robot_yaw          # 发现时机器人航向
string image_path          # 保存的检测图片路径
