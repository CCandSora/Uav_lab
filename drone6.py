#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Drone5 ‒ 任意多边形双机牛耕扫描（加强版）
• 区域按 X 方向二分（面积均分），若失败退化为奇偶分配。
• 子区域基于最长边方向生成牛耕折返路径。
• 飞行Boundary在子区域内侧收缩 BOUNDARY_OFFSET 米，避免贴边碰撞。
"""

import math
import argparse
from dataclasses import dataclass
from typing import List, Tuple

import rospy
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped, Point
from visualization_msgs.msg import Marker

from shapely.geometry import (
    Polygon, LineString, Point as ShpPoint,
    GeometryCollection, MultiLineString
)
from shapely.ops import split
from shapely.affinity import rotate

from pycrazyswarm import Crazyswarm


# ============ 全局参数 ============
LINE_SPACE        = 0.5    # 折返间距 (m)
BOUNDARY_OFFSET   = 0.25   # 子区域边界内侧收缩 (m)
FLY_HEIGHT        = 0.5    # 起飞高度 (m)
ALT_OFFSET        = 0.6    # 第二架相对高度差 (m)
TAKEOFF_TIME      = 2.0    # 起飞时长 (s)
LAND_TIME         = 3.0    # 降落时长 (s)
SAFE_DIST2        = 0.5**2 # 避碰距离阈值平方 (m²)
HOME_XY           = (0.0, 0.0)


@dataclass
class WP:
    x: float
    y: float
    z: float = FLY_HEIGHT


# ---------- 工具函数 ----------
def _clean_intersections(segs):
    """将 shapely.intersection 输出统一为可迭代的线段/点列表"""
    if segs.is_empty:
        return []
    if isinstance(segs, LineString):
        return [segs]
    if isinstance(segs, MultiLineString):
        return list(segs.geoms)
    if isinstance(segs, ShpPoint):
        return [LineString([segs, segs])]
    if isinstance(segs, GeometryCollection):
        return [g for g in segs.geoms
                if isinstance(g, (LineString, ShpPoint))]
    return []


def split_polygon(poly: Polygon, thresh: float = 0.05) -> Tuple[Polygon, Polygon]:
    """
    沿 X 方向二分面积，提取 GeometryCollection 中的两个 Polygon。
    若切割成功且两块面积差 < thresh，返回 (left, right)；
    否则返回 (poly, None)。
    """
    minx, miny, maxx, maxy = poly.bounds
    lo, hi = minx, maxx
    for _ in range(30):
        mid = (lo + hi) / 2.0
        cutter = LineString([(mid, miny - 1.0), (mid, maxy + 1.0)])
        result = split(poly, cutter)
        # 提取多边形部分
        if isinstance(result, GeometryCollection):
            parts = [g for g in result.geoms if isinstance(g, Polygon)]
        else:
            parts = list(result) if isinstance(result, (list, tuple)) else []
        if len(parts) != 2:
            # 切割失败，继续二分
            continue
        a0, a1 = parts[0].area, parts[1].area
        # 面积均衡判断
        if abs(a0 - a1) / poly.area < thresh:
            return parts[0], parts[1]
        # 根据哪个更大来缩小搜索区间
        if a0 > a1:
            hi = mid
        else:
            lo = mid
    # fallback
    return poly, None


def longest_edge_angle(poly: Polygon) -> float:
    """
    返回多边形外轮廓最长边方向（度）。  
    如果前两条最长边：
      • 长度差 <5% 且 方向差 <10°（同向），仍按这方向返回；  
      • 长度差 <5% 且 方向差 ~90°，退化返回 0°；  
    否则直接返回最大边方向。
    """
    coords = list(poly.exterior.coords)
    edges: List[Tuple[float, float]] = []
    for (x1, y1), (x2, y2) in zip(coords, coords[1:]):
        length = math.hypot(x2 - x1, y2 - y1)
        angle  = math.degrees(math.atan2(y2 - y1, x2 - x1))
        edges.append((length, angle))
    # 按长度降序，如果长度相同则角度降序
    edges.sort(key=lambda x: (x[0], x[1]), reverse=True)

    length1, angle1 = edges[0]
    length2, angle2 = edges[1] if len(edges) > 1 else (0.0, 0.0)

    # 规格化到 [0,180) 度
    def norm180(a: float) -> float:
        a = a % 180.0
        return a + 180.0 if a < 0 else a

    a1 = norm180(angle1)
    a2 = norm180(angle2)
    diff = abs(a1 - a2)

    # 前两条长边长度近似相等
    if length1 > 0 and abs(length1 - length2) / length1 < 0.05:
        # 方向近似（共线）
        if diff < 10.0 or abs(diff - 180.0) < 10.0:
            return angle1
        # 方向差 ~90° → 无明显主向
        return 0.0

    # 正常情况：直接返回最长边方向
    return angle1



# ---------- 生成牛耕折返路径 ----------
def gen_boustro_path(poly: Polygon,
                     space: float,
                     start_ltr: bool = True) -> List[WP]:
    """
    1. 按最长边方向旋转 poly → 水平
    2. 按行间距生成多条水平扫描线，取交集
    3. 按折返顺序提取点并反向旋转回原坐标
    """
    θ = longest_edge_angle(poly)
    rot_poly = rotate(poly, -θ, origin='centroid')
    minx, miny, maxx, maxy = rot_poly.bounds
    rows = int((maxy - miny) / space) + 1

    path: List[WP] = []
    ltr = start_ltr
    for i in range(rows):
        y = miny + i * space
        segs = _clean_intersections(
            rot_poly.intersection(LineString([(-1e4, y), (1e4, y)]))
        )
        # 按中心 x 排序
        segs.sort(key=lambda s: s.centroid.x)
        if not ltr:
            segs.reverse()

        for seg in segs:
            xs = [seg.coords[0][0], seg.coords[-1][0]]
            xs.sort() if ltr else xs.reverse()
            for x in xs:
                p_rot = ShpPoint(x, y)
                # 反向旋转回原 poly 坐标
                p_ori = rotate(p_rot, θ, origin=poly.centroid)
                path.append(WP(p_ori.x, p_ori.y))
        ltr = not ltr

    return path


# ---------- 可视化消息构建 ----------
def build_plan_and_marker(path: List[WP],
                          color: Tuple[float, float, float],
                          z_off: float = 0.0):
    now = rospy.Time.now()
    plan = Path()
    mark = Marker()
    plan.header.stamp = mark.header.stamp = now
    plan.header.frame_id = mark.header.frame_id = "world"

    mark.ns = "boustro"
    mark.id = 0
    mark.type = Marker.LINE_STRIP
    mark.action = Marker.ADD
    mark.pose.orientation.w = 1.0
    mark.scale.x = 0.03
    mark.color.r, mark.color.g, mark.color.b, mark.color.a = (*color, 1.0)

    for wp in path:
        ps = PoseStamped()
        ps.header = plan.header
        ps.pose.position.x = wp.x
        ps.pose.position.y = wp.y
        ps.pose.position.z = wp.z + z_off
        plan.poses.append(ps)
        mark.points.append(Point(x=wp.x, y=wp.y, z=wp.z + z_off))

    return plan, mark


# ---------- 主流程 ----------
def main(use_sim: bool, vertices: List[Tuple[float, float]]):
    rospy.init_node("boustro_scan", anonymous=False)
    swarm = Crazyswarm()
    th = swarm.timeHelper
    th.is_sim = use_sim

    cfs = swarm.allcfs.crazyflies
    if len(cfs) < 2:
        raise RuntimeError("需要至少两台 Crazyflie")
    cf1, cf2 = cfs[:2]

    # 构造原始多边形并切分
    original = Polygon(vertices).buffer(0)
    left, right = split_polygon(original)

    # 对每个子区域先收缩 BOUNDARY_OFFSET
    def shrink(poly):
        shr = poly.buffer(-BOUNDARY_OFFSET)
        return shr if shr.is_valid and shr.area > 1e-6 else poly

    if right is not None:
        A = shrink(left)
        B = shrink(right)
        path1 = gen_boustro_path(A, LINE_SPACE, True)
        path2 = gen_boustro_path(B, LINE_SPACE, False)
    else:
        P = shrink(original)
        whole = gen_boustro_path(P, LINE_SPACE, True)
        path1, path2 = whole[::2], whole[1::2]

    # 发布可视化
    pubP1 = rospy.Publisher("/boustro_plan1",   Path,   queue_size=2, latch=True)
    pubM1 = rospy.Publisher("/boustro_marker1", Marker, queue_size=2, latch=True)
    pubP2 = rospy.Publisher("/boustro_plan2",   Path,   queue_size=2, latch=True)
    pubM2 = rospy.Publisher("/boustro_marker2", Marker, queue_size=2, latch=True)
    rospy.sleep(1.0)

    plan1, mark1 = build_plan_and_marker(path1, (0.0, 1.0, 0.0), z_off=0.0)
    plan2, mark2 = build_plan_and_marker(path2, (1.0, 0.0, 0.0), z_off=ALT_OFFSET)
    pubP1.publish(plan1); pubM1.publish(mark1)
    pubP2.publish(plan2); pubM2.publish(mark2)

    # 起飞
    cf1.takeoff(FLY_HEIGHT,   TAKEOFF_TIME)
    cf2.takeoff(FLY_HEIGHT+ALT_OFFSET, TAKEOFF_TIME)
    th.sleep(TAKEOFF_TIME)

    # 并行执行路径
    for i in range(max(len(path1), len(path2))):
        if i < len(path1):
            cf1.goTo([path1[i].x, path1[i].y, FLY_HEIGHT], 0.0, 1.5)
        if i < len(path2):
            cf2.goTo([path2[i].x, path2[i].y, FLY_HEIGHT+ALT_OFFSET], 0.0, 1.5)

        dx = cf1.position()[0] - cf2.position()[0]
        dy = cf1.position()[1] - cf2.position()[1]
        if dx*dx + dy*dy < SAFE_DIST2:
            rospy.loginfo("[避碰] 距离过近，暂停0.5s")
            th.sleep(0.5)
        th.sleep(1.6)

    # 回航 & 降落
    cf1.goTo([*HOME_XY, FLY_HEIGHT],   0.0, 3.0)
    cf2.goTo([*HOME_XY, FLY_HEIGHT+ALT_OFFSET], 0.0, 3.0)
    th.sleep(3.0)
    cf1.land(0.04, LAND_TIME)
    cf2.land(0.04, LAND_TIME)
    th.sleep(LAND_TIME)
    rospy.loginfo("任务完成，脚本退出")


# ---------- 入口 ----------
if __name__ == "__main__":
    parser = argparse.ArgumentParser(
        description="Dual Boustrophedon Coverage for Arbitrary Polygon"
    )
    parser.add_argument("--sim", action="store_true", help="仿真模式")
    parser.add_argument(
        "--poly", type=str, default="",
        help="多边形顶点: x1,y1;x2,y2;... (默认正方形)"
    )
    args = parser.parse_args()

    if args.poly:
        try:
            verts = [tuple(map(float, p.split(",")))
                     for p in args.poly.strip().split(";")]
        except Exception as e:
            raise ValueError("--poly 格式错误，应为 x1,y1;x2,y2;...") from e
    else:
        # 默认 4×4 正方形
        verts = [(-2.0, -2.0), (-2.0, 2.0),
                 (2.0,  2.0), (2.0, -2.0)]

    main(args.sim, verts)

