import math
import argparse
from typing import List, Tuple
from dataclasses import dataclass

import rospy
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped, Point
from visualization_msgs.msg import Marker
from shapely.geometry import Polygon

from pycrazyswarm import Crazyswarm

# ====== 全局参数 ======
LINE_SPACE   = 0.5      # 折返间距 (m)
FLY_HEIGHT   = 0.5      # 主巡航高度 (m)
ALT_OFFSET   = 0.6      # 第二架相对高度差 (m)  ← ★ 修改这里即可
TAKEOFF_TIME = 2.0      # 起飞时长 (s)
LAND_TIME    = 3.0      # 降落时长 (s)
SAFE_DIST2   = 0.5 ** 2 # 平面安全距离平方 (m^2)

HOME_XY = (0.0, 0.0)    # 返航落点

# ====== 数据结构 ======
@dataclass
class WP:
    x: float
    y: float
    z: float = FLY_HEIGHT

# ====== 路径生成 ======
def gen_boustro_path(width: float, length: float, origin_x: float, origin_y: float,
                     start_left_to_right: bool) -> List[WP]:
    """生成等距牛耕式扫描路径。"""
    rows = int(length // LINE_SPACE) + 1
    path: List[WP] = []

    for i in range(rows):
        y = origin_y + i * LINE_SPACE
        if start_left_to_right:
            xs = [origin_x, origin_x + width]
        else:
            xs = [origin_x + width, origin_x]

        for x in xs:
            path.append(WP(x, y))
        start_left_to_right = not start_left_to_right

    return path

# ====== 可视化消息 ======
def build_plan_and_marker(path: List[WP]):
    now = rospy.Time.now()

    plan = Path()
    plan.header.stamp = now
    plan.header.frame_id = "world"

    marker = Marker()
    marker.header.stamp = now
    marker.header.frame_id = "world"
    marker.ns = "boustro"
    marker.id = 0
    marker.type = Marker.LINE_STRIP
    marker.action = Marker.ADD
    marker.pose.orientation.w = 1.0
    marker.scale.x = 0.03
    marker.color.r, marker.color.g, marker.color.b, marker.color.a = 0.0, 1.0, 0.0, 1.0

    for wp in path:
        ps = PoseStamped()
        ps.header.stamp = now
        ps.header.frame_id = "world"
        ps.pose.position.x, ps.pose.position.y, ps.pose.position.z = wp.x, wp.y, wp.z
        plan.poses.append(ps)

        pt = Point(x=wp.x, y=wp.y, z=wp.z)
        marker.points.append(pt)

    return plan, marker

# ====== 主流程 ======
def main(use_sim: bool):
    rospy.init_node("CrazyflieAPI", anonymous=False)

    swarm = Crazyswarm()
    th = swarm.timeHelper
    th.is_sim = use_sim

    cfs = swarm.allcfs.crazyflies
    if len(cfs) < 2:
        raise RuntimeError("需要至少两台 Crazyflie 参与飞行")

    cf1, cf2 = cfs[0], cfs[1]

    # --- 定义 4×4 正方形区域并拆成左右两块 ---
    square_coords = [(-2.0, -2.0), (-2.0, 2.0), (2.0, 2.0), (2.0, -2.0)]
    full_poly = Polygon(square_coords)
    minx, miny, maxx, maxy = full_poly.bounds  # (-2, -2, 2, 2)
    midx = (minx + maxx) / 2.0  # 0

    # 左右子区域 bbox
    width_half  = (maxx - minx) / 2.0  # 2 m
    length_full =  maxy - miny         # 4 m

    # 左侧区域 (cf1)
    path1 = gen_boustro_path(width=width_half,
                             length=length_full,
                             origin_x=minx,
                             origin_y=miny,
                             start_left_to_right=True)

    # 右侧区域 (cf2)
    path2 = gen_boustro_path(width=width_half,
                             length=length_full,
                             origin_x=midx,
                             origin_y=miny,
                             start_left_to_right=False)

    # ---- 发布可视化 ----
    plan_pub   = rospy.Publisher("/boustro_plan",   Path,   queue_size=1, latch=True)
    marker_pub = rospy.Publisher("/boustro_marker", Marker, queue_size=1, latch=True)
    rospy.sleep(1.0)
    for p, m in (build_plan_and_marker(path1), build_plan_and_marker(path2)):
        plan_pub.publish(p)
        marker_pub.publish(m)

    # ==== 同步起飞 ====
    cf1.takeoff(FLY_HEIGHT, TAKEOFF_TIME)
    cf2.takeoff(FLY_HEIGHT + ALT_OFFSET, TAKEOFF_TIME)
    th.sleep(TAKEOFF_TIME)

    # ==== 并行扫描 ====
    max_len = max(len(path1), len(path2))
    for i in range(max_len):
        # --- cf1 指令 ---
        if i < len(path1):
            wp = path1[i]
            cf1.goTo([wp.x, wp.y, FLY_HEIGHT], 0.0, 1.5)

        # --- cf2 指令 ---
        if i < len(path2):
            wp = path2[i]
            cf2.goTo([wp.x, wp.y, FLY_HEIGHT + ALT_OFFSET], 0.0, 1.5)

        # --- 简单平面避碰 ---
        pos1 = cf1.position()
        pos2 = cf2.position()
        dx, dy = pos1[0] - pos2[0], pos1[1] - pos2[1]
        if dx*dx + dy*dy < SAFE_DIST2:
            rospy.loginfo("[避碰] 两机距离过近，增加等待")
            th.sleep(0.5)

        th.sleep(1.6)  # 给足运动时间

    # ==== 返航 ====
    for cf, z_off in ((cf1, 0.0), (cf2, ALT_OFFSET)):
        cf.goTo([HOME_XY[0], HOME_XY[1], FLY_HEIGHT + z_off], 0.0, 3.0)
    th.sleep(3.0)

    # ==== 降落 ====
    for cf, z_off in ((cf1, 0.0), (cf2, ALT_OFFSET)):
        cf.land(0.04, LAND_TIME)
    th.sleep(LAND_TIME)

    rospy.loginfo("任务完成，脚本退出")


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="Dual Crazyflie Boustrophedon Coverage (4×4 square, height offset)")
    parser.add_argument("--sim", action="store_true", help="仿真模式")
    args = parser.parse_args()
    main(args.sim)

