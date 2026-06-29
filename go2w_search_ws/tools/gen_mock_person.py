#!/usr/bin/env python3
"""一次性生成 web/static/mock_person.png (spec §8.1, H4.3)。

目的: 让 NxAiEngine 的 MockFrameGenerator 把这张人物裁图贴到 mock 帧里,
使 YOLO 在不依赖狗硬件时也能真检出 person (C4.4 mock 视频真检测)。

注: 本仓库开发机无 COCO 数据集, 这里合成一张"类人剪影":
- 站立人物轮廓 (头/躯干/四肢), 配色贴近真实人物照片 (上衣蓝/裤子深灰/肤色头手)
- 高度 720px (贴合 720p 帧), 宽度按人体比例 ~320px
- 背景为浅灰 (与 mock 帧灰底融合), YOLO 在 mock 帧贴图后能检出 person

真狗帧到位后此文件可被任何 COCO person 裁图替换 (路径不变)。
"""
import os
import sys

import cv2
import numpy as np

OUT = os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "web", "static", "mock_person.png")
OUT = os.path.normpath(OUT)


def main():
    H, W = 720, 320
    img = np.full((H, W, 3), 110, dtype=np.uint8)  # 浅灰背景

    SKIN = (190, 160, 130)     # BGR 肤色
    SHIRT = (180, 60, 40)      # BGR 蓝上衣 (注意 BGR: 蓝=B 大)
    PANTS = (60, 50, 40)       # BGR 深灰裤
    SHOES = (20, 20, 20)

    # 头 (圆)
    cv2.circle(img, (W // 2, 110), 55, SKIN, -1)
    # 脖子
    cv2.rectangle(img, (W // 2 - 18, 160), (W // 2 + 18, 195), SKIN, -1)
    # 躯干 (上衣)
    cv2.rectangle(img, (W // 2 - 80, 195), (W // 2 + 80, 420), SHIRT, -1)
    # 手臂 (上衣袖)
    cv2.rectangle(img, (W // 2 - 110, 200), (W // 2 - 80, 380), SHIRT, -1)
    cv2.rectangle(img, (W // 2 + 80, 200), (W // 2 + 110, 380), SHIRT, -1)
    # 手 (肤色)
    cv2.circle(img, (W // 2 - 95, 395), 18, SKIN, -1)
    cv2.circle(img, (W // 2 + 95, 395), 18, SKIN, -1)
    # 裤子 (两腿)
    cv2.rectangle(img, (W // 2 - 75, 420), (W // 2 - 12, 640), PANTS, -1)
    cv2.rectangle(img, (W // 2 + 12, 420), (W // 2 + 75, 640), PANTS, -1)
    # 鞋
    cv2.rectangle(img, (W // 2 - 80, 640), (W // 2 - 8, 690), SHOES, -1)
    cv2.rectangle(img, (W // 2 + 8, 640), (W // 2 + 80, 690), SHOES, -1)

    # 轻微纹理 (避免纯色块 YOLO 不敏感): 加几条阴影线
    for y in range(200, 420, 8):
        cv2.line(img, (W // 2 - 78, y), (W // 2 + 78, y), (140, 50, 30), 1)

    os.makedirs(os.path.dirname(OUT), exist_ok=True)
    cv2.imwrite(OUT, img)
    print(f"wrote {OUT}  ({img.shape[1]}x{img.shape[0]})")


if __name__ == "__main__":
    sys.exit(main())
