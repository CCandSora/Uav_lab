#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
two_cf_boustro.py  ·  2025-05-15 修订版
两架 Crazyflie 各自牛耕式扫描半块区域（按 x 轴分割）
"""

import math
import argparse
from typing import List, Tuple
from dataclasses import dataclass
from pycrazyswarm import Crazyswarm   # ← 关键修改

# ====== 配置 ======
WORLD_W, WORLD_H = 5.0, 5.0      # 世界尺寸 (m)
STRIDE           = 0.5           # 行间距 (m)
ALT1, ALT2       = 0.45, 0.55    # 两机高度 (m)
TAKEOFF_T        = 2.0           # 起飞时间 (s)
CRUISE_V         = 0.4           # 巡航速度 (m/s)
LAND_T           = 3.0           # 降落时间 (s)

# ====== 数据结构 & 路径生成 ======
@dataclass
class WP:
    x: float
    y: float
    z: float

def gen_boustro(width: float, height: float, stride: float,
                origin: Tuple[float, float], start_left: bool, z: float) -> List[WP]:
    x0, y0 = origin
    rows = int(height // stride) + 1
    direction = 1 if start_left else -1
    path: List[WP] = []
    for r in range(rows):
        y = y0 + r * stride
        xs = [x0, x0 + width] if direction == 1 else [x0 + width, x0]
        for x in xs:
            path.append(WP(x, y, z))
        direction *= -1
    return path

def gen_two_paths() -> Tuple[List[WP], List[WP]]:
    half_w, half_h = WORLD_W / 2, WORLD_H / 2
    left  = gen_boustro(half_w, WORLD_H, STRIDE,
                        origin=(-half_w, -half_h),
                        start_left=False, z=ALT1)
    right = gen_boustro(half_w, WORLD_H, STRIDE,
                        origin=(0.0, -half_h),
                        start_left=True,  z=ALT2)
    return left, right

def go_segment(cf, wp_prev: WP, wp_next: WP, th):
    dist = math.dist((wp_prev.x, wp_prev.y, wp_prev.z),
                     (wp_next.x, wp_next.y, wp_next.z))
    T = dist / CRUISE_V
    cf.goTo([wp_next.x, wp_next.y, wp_next.z], yaw=0.0, duration=T)
    th.sleep(T)

# ====== 主函数 ======
def main(use_sim_time: bool):
    swarm = Crazyswarm()
    th     = swarm.timeHelper
    th.is_sim = use_sim_time            # 仿真 -> 跳时钟
    allcfs = swarm.allcfs
    if len(allcfs) < 2:
        raise RuntimeError("需要至少两架 Crazyflie")

    cf1, cf2 = allcfs[0], allcfs[1]

    # 起飞
    cf1.takeoff(ALT1, TAKEOFF_T)
    cf2.takeoff(ALT2, TAKEOFF_T)
    th.sleep(TAKEOFF_T)

    # 路径
    path1, path2 = gen_two_paths()

    # 飞到各自首点
    cf1.goTo([path1[0].x, path1[0].y, path1[0].z], 0.0, 2.0)
    cf2.goTo([path2[0].x, path2[0].y, path2[0].z], 0.0, 2.0)
    th.sleep(2.0)

    # 扫描
    for i in range(1, max(len(path1), len(path2))):
        if i < len(path1):
            go_segment(cf1, path1[i-1], path1[i], th)
        if i < len(path2):
            go_segment(cf2, path2[i-1], path2[i], th)

    # 回中心→降落
    cf1.goTo([0, 0, ALT1], 0, 3); cf2.goTo([0, 0, ALT2], 0, 3); th.sleep(3)
    cf1.land(0.04, LAND_T); cf2.land(0.04, LAND_T); th.sleep(LAND_T)

if __name__ == "__main__":
    ap = argparse.ArgumentParser()
    ap.add_argument("--sim", action="store_true", help="使用模拟时间加速")
    args = ap.parse_args()
    main(args.sim)
