#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
two_agents_boustrophedon.py
让两架无人机各自执行半区域牛耕扫描
作者：ChatGPT（2025-05-15）
"""

from typing import List, Tuple
from boustrophedon_path import Waypoint, generate_boustrophedon_path


def generate_two_half_paths(
    width: float = 5.0,
    height: float = 5.0,
    stride: float = 0.5,
    altitude: float = 0.4,
) -> Tuple[List[Waypoint], List[Waypoint]]:
    """
    把矩形区域按 **垂直方向** 一分为二，并生成各自的牛耕路径

    — 左半区域（x < 0）→ path_left
    — 右半区域（x ≥ 0）→ path_right
    两条路径都会从世界中心 (0, 0) 处起步，确保起始点互不冲突。
    """
    half_w = width / 2.0
    half_h = height / 2.0

    # 左半（x ∈ [−half_w, 0]）：从中心往左走
    path_left = generate_boustrophedon_path(
        width=half_w,
        height=height,
        stride=stride,
        origin=(-half_w, -half_h),
        altitude=altitude,
        start_from_left=False,  # 首行从 x = 0 往 −half_w 方向
    )

    # 右半（x ∈ [0, +half_w]）：从中心往右走
    path_right = generate_boustrophedon_path(
        width=half_w,
        height=height,
        stride=stride,
        origin=(0.0, -half_h),
        altitude=altitude,
        start_from_left=True,  # 首行从 x = 0 往 +half_w 方向
    )

    return path_left, path_right


# ------------------ Crazyswarm 调用示例 ------------------
"""
假设 crazyflie1、crazyflie2 的对象是 cf1、cf2，
并且脚本里已经完成 takeoff，且两机都升到 altitude。

用法：
>>> path1, path2 = generate_two_half_paths(width=5, height=5, stride=0.5, altitude=0.5)
>>> for cf, path in zip([cf1, cf2], [path1, path2]):
...     for wp in path:
...         # 根据距离自动计算运动时间也可以
...         cf.goTo([wp.x, wp.y, wp.z], yaw=0.0, duration=2.0)
...         timeHelper.sleep(2.0)   # Crazyswarm 用自己的 TimeHelper
"""

if __name__ == "__main__":
    p1, p2 = generate_two_half_paths()
    print(f"Left half path:  {len(p1)} points")
    print(f"Right half path: {len(p2)} points")
