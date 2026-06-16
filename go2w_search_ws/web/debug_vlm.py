#!/usr/bin/env python3
"""调试VLM输出"""
import sys; sys.path.insert(0, '.')
from ai.vlm import VLMEngine

vlm = VLMEngine(); vlm.load()

sys_prompt = """你是一个机器狗助手。将用户指令分解为任务序列。
可用任务类型: move/follow/search_area/stop/return_home
move: 运动, 参数 {"vx":前进速度, "vy":侧移, "vyaw":旋转(正=左转), "duration":秒}
follow: 跟踪, 参数 {"target":"目标描述"}
search_area: 搜索, 参数 {"pattern":"lawnmower/spiral","width":宽米,"height":高米}
stop: 停止, 参数 {}
return_home: 返回, 参数 {}
重要: 转弯用move+vyaw! JSON不能有注释! 纯JSON输出,无markdown。"""

resp = vlm.chat([
    {"role": "system", "content": sys_prompt},
    {"role": "user", "content": "搜索这片区域"}
], max_new_tokens=512)

print("=== VLM 原始输出 ===")
print(repr(resp))
print("=== 可读 ===")
print(resp)
