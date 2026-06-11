# 任务状态
uint8 IDLE=0
uint8 PLANNING=1
uint8 SEARCHING=2
uint8 PAUSED=3
uint8 COMPLETED=4
uint8 FAILED=5

uint8 state
float64 progress           # 完成进度 [0, 1]
uint32 waypoints_total
uint32 waypoints_visited
uint32 targets_found
string message             # 状态描述
