import math
import argparse
from typing import List, Tuple, Any
from dataclasses import dataclass
import rospy
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped
from visualization_msgs.msg import Marker
from pycrazyswarm import Crazyswarm
from shapely.geometry import Polygon, LineString
from shapely.ops import split

# 参数
line_space = 0.5
height = 0.5
takeoff_time = 2
v = 0.4
land_time = 3.0


@dataclass
class WP:
    x: float
    y: float
    z: float


def gen_path(width, length, line_space, origin_x, origin_y, towards, z) -> List[WP]:
    rows = int(length // line_space) + 1
    path: List[WP] = []
    for i in range(rows):
        y = origin_y + i * line_space
        if towards:
            xs = [origin_x, origin_x + width]
        else:
            xs = [origin_x + width, origin_x]
        for x in xs:
            path.append(WP(x, y, z))
        towards = not towards
    return path


def build_plan_msgs(path: List[WP]):
    now = rospy.Time.now()
    plan = Path()
    plan.header.stamp = now
    plan.header.frame_id = "world"

    for wp in path:
        ps = PoseStamped()
        ps.header.stamp = now
        ps.header.frame_id = "world"
        ps.pose.position.x = wp.x
        ps.pose.position.y = wp.y
        ps.pose.position.z = wp.z
        plan.poses.append(ps)

    mk = Marker()
    mk.header.stamp = now
    mk.header.frame_id = "world"
    mk.ns = "boustro"
    mk.id = 0
    mk.type = Marker.LINE_STRIP
    mk.action = Marker.ADD
    mk.pose.orientation.w = 1.0
    mk.scale.x = 0.03
    mk.color.r = 0.0
    mk.color.g = 1.0
    mk.color.b = 0.0
    mk.color.a = 1.0
    for wp in path:
        from geometry_msgs.msg import Point
        pt = Point()
        pt.x, pt.y, pt.z = wp.x, wp.y, wp.z
        mk.points.append(pt)
    return plan, mk


def fly_path(cf, path: List[WP], th, height_offset: float, other_cf=None):
    cf.takeoff(height + height_offset, takeoff_time)
    th.sleep(takeoff_time)

    cf.goTo([path[0].x, path[0].y, height + height_offset], 0.0, 2.0)
    th.sleep(2.0)

    for i in range(1, len(path)):
        wp = path[i]
        target_pos = [wp.x, wp.y, height + height_offset]

        if other_cf:
            other_pos = other_cf.position()
            dx = other_pos[0] - target_pos[0]
            dy = other_pos[1] - target_pos[1]
            dist_sq = dx ** 2 + dy ** 2
            if dist_sq < 0.5 ** 2:
                rospy.loginfo("[避碰] 距离过近，等待...")
                th.sleep(1.0)

        cf.goTo(target_pos, yaw=0.0, duration=1.5)
        th.sleep(1.5)

    cf.goTo([0, 0, height + height_offset], 0, 3)
    th.sleep(3)
    cf.land(0.04, land_time)
    th.sleep(land_time)


def split_polygon(polygon_coords: List[Tuple[float, float]]) -> List[List[Any]]:
    poly = Polygon(polygon_coords)
    minx, miny, maxx, maxy = poly.bounds
    midx = (minx + maxx) / 2
    splitting_line = LineString([(midx, miny - 1), (midx, maxy + 1)])
    result = split(poly, splitting_line)
    if len(result.geoms) != 2:
        raise ValueError("多边形分割失败")
    return [list(p.exterior.coords)[:-1] for p in result.geoms]


def get_bbox_region(poly_coords):
    poly = Polygon(poly_coords)
    minx, miny, maxx, maxy = poly.bounds
    return minx, miny, maxx - minx, maxy - miny


def main(use_sim_time: bool):
    rospy.init_node("dual_cf_search", anonymous=True)
    swarm = Crazyswarm()
    th = swarm.timeHelper
    th.is_sim = use_sim_time

    cf_list = swarm.allcfs.crazyflies
    if len(cf_list) < 2:
        raise RuntimeError("需要至少两台 Crazyflie")

    cf1 = cf_list[0]
    cf2 = cf_list[1]

    polygon_coords = [(-2.0, -2.0), (2.0, -2.0), (2.5, 0.0), (2.0, 2.0), (-2.0, 2.0), (-2.5, 0.0)]
    poly1, poly2 = split_polygon(polygon_coords)

    origin_x1, origin_y1, width1, length1 = get_bbox_region(poly1)
    origin_x2, origin_y2, width2, length2 = get_bbox_region(poly2)

    path1 = gen_path(width1, length1, line_space, origin_x1, origin_y1, True, height)
    path2 = gen_path(width2, length2, line_space, origin_x2, origin_y2, False, height)

    plan_pub = rospy.Publisher("/boustro_plan", Path, queue_size=1, latch=True)
    marker_pub = rospy.Publisher("/boustro_marker", Marker, queue_size=1, latch=True)
    rospy.sleep(1.0)
    p1, m1 = build_plan_msgs(path1)
    p2, m2 = build_plan_msgs(path2)
    plan_pub.publish(p1)
    marker_pub.publish(m1)
    plan_pub.publish(p2)
    marker_pub.publish(m2)

    fly_path(cf1, path1, th, height_offset=0.0, other_cf=cf2)
    fly_path(cf2, path2, th, height_offset=0.2, other_cf=cf1)


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="Dual-CF area coverage with avoidance")
    parser.add_argument("--sim", action="store_true", help="仿真模式")
    args = parser.parse_args()
    main(args.sim)
