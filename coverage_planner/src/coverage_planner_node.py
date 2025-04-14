#!/usr/bin/env python3
# coverage_planner_node.py
import rospy
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Empty

# 示例：定义A*算法函数，用于在grid_map上寻找从start到goal的路径
def a_star_search(grid, start, goal):
    rows, cols = len(grid), len(grid[0])
    # open列表, 存储待探索节点: (f_cost, g_cost, cell, parent)
    open_list = []
    # closed集合, 存储已探索节点
    closed_set = set()
    # 初始节点
    sx, sy = start; gx, gy = goal
    open_list.append((abs(sx-gx)+abs(sy-gy), 0, (sx, sy), None))
    came_from = {}  # 用于重建路径
    
    while open_list:
        # 取出f_cost最小的节点
        open_list.sort(key=lambda x: x[0])
        f, g, current, parent = open_list.pop(0)
        if current in closed_set:
            continue
        closed_set.add(current)
        came_from[current] = parent
        cx, cy = current
        # 是否到达目标
        if current == (gx, gy):
            break
        # 遍历相邻4邻居
        for dx, dy in [(0,1),(0,-1),(1,0),(-1,0)]:
            nx, ny = cx+dx, cy+dy
            if 0 <= nx < rows and 0 <= ny < cols and grid[nx][ny] == 0:
                neighbor = (nx, ny)
                if neighbor in closed_set:
                    continue
                g_new = g + 1  # 假设每步成本为1
                # 计算启发式h_cost
                h_new = abs(nx-gx) + abs(ny-gy)
                f_new = g_new + h_new
                # 将邻居加入open列表
                open_list.append((f_new, g_new, neighbor, current))
    # 重建路径
    path = []
    node = (gx, gy)
    if node not in came_from:
        return []  # 未找到路径
    while node:
        path.append(node)
        node = came_from.get(node)
    path.reverse()
    return path

# 生成覆盖扫描路径（示例实现：之字形扫描）
def generate_coverage_path(area_width, area_height):
    waypoints = []
    y = 0
    direction = 1
    # 根据高度设定固定飞行高度（下视摄像头保持垂直）
    alt = rospy.get_param("~flight_altitude", 10.0)  # 例如10米高度
    # 栅格步长可根据视场覆盖直径设定，比如直径的0.8倍
    step = rospy.get_param("~grid_step", 5.0)
    while y < area_height:
        # x方向往返
        x_range = range(0, area_width, int(step))
        if direction < 0:
            x_range = reversed(x_range)
        for x in x_range:
            waypoints.append((x, y))
        # 转到下一扫掠行
        y += step
        direction *= -1
    # 转换为带高度的三维坐标序列
    waypoints_3d = [(x, y, alt) for (x, y) in waypoints]
    return waypoints_3d

if __name__ == '__main__':
    rospy.init_node('coverage_planner')
    # 发布目标点位姿的topic，例如使用Mavros的setpoint_position
    cmd_pub = rospy.Publisher('move_base_simple/goal', PoseStamped, queue_size=10)
    rospy.sleep(1)  # 等待发布器就绪

    # 获取规划区域大小参数
    area_width = rospy.get_param('~area_width', 50)   # 区域宽度 (米)
    area_height = rospy.get_param('~area_height', 50) # 区域高度 (米)
    # 构建栅格地图 (0表示空地，1表示障碍)，此处简单初始化为空地图
    grid_map = [[0]*area_width for _ in range(area_height)]
    # （可选）如需添加障碍物，可将对应grid_map单元设为1

    # 生成覆盖路径各关键节点（无障碍情况下为连续栅格，若有障碍可划分子区域）
    coverage_points = generate_coverage_path(area_width, area_height)
    rospy.loginfo(f"[Planner] Generated {len(coverage_points)} coverage points.")
    
    # 按顺序依次覆盖访问每个点
    rate = rospy.Rate(0.5)  # 控制发布频率，例如每2秒发布一个新的目标点
    current_pos = coverage_points[0]
    for idx in range(1, len(coverage_points)):
        next_pos = coverage_points[idx]
        # 计算从current_pos到next_pos的路径 (考虑障碍, 无障碍则直接相邻)
        sx, sy, _ = current_pos
        gx, gy, _ = next_pos
        path = a_star_search(grid_map, (int(sy), int(sx)), (int(gy), int(gx)))
        # 沿A*路径发布各步目标 (无障碍时path基本是straight line)
        for (py, px) in path[1:]:  # path包括起点，这里从第二个节点开始
            goal_msg = PoseStamped()
            goal_msg.header.frame_id = "world"
            goal_msg.pose.position.x = px
            goal_msg.pose.position.y = py
            goal_msg.pose.position.z = current_pos[2]  # 保持恒定高度
            goal_msg.pose.orientation.w = 1.0
            cmd_pub.publish(goal_msg)
            rospy.loginfo(f"[Planner] Moving to ({px},{py})")
            rate.sleep()
        current_pos = next_pos

    rospy.loginfo("[Planner] Coverage complete. Landing or loitering...")
    # （实际应用中，路径完成后可触发降落或悬停，此处简略处理）
    rospy.spin()

