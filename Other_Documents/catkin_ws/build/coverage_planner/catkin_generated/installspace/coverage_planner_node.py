#!/usr/bin/env python3
# coverage_planner_node.py
import rospy
from geometry_msgs.msg import PoseStamped
import yaml
import os

def a_star_search(grid, start, goal):
    rows, cols = len(grid), len(grid[0])
    open_list = []
    closed_set = set()
    open_list.append((abs(start[0]-goal[0])+abs(start[1]-goal[1]), 0, start, None))
    came_from = {}

    while open_list:
        open_list.sort(key=lambda x: x[0])
        f, g, current, parent = open_list.pop(0)
        if current in closed_set:
            continue
        closed_set.add(current)
        came_from[current] = parent
        if current == goal:
            break
        for dx, dy in [(0,1),(0,-1),(1,0),(-1,0)]:
            neighbor = (current[0]+dx, current[1]+dy)
            if 0 <= neighbor[0] < rows and 0 <= neighbor[1] < cols and grid[neighbor[0]][neighbor[1]] == 0:
                if neighbor in closed_set:
                    continue
                g_new = g + 1
                h_new = abs(neighbor[0]-goal[0]) + abs(neighbor[1]-goal[1])
                open_list.append((g_new+h_new, g_new, neighbor, current))

    path = []
    node = goal
    while node:
        path.append(node)
        node = came_from.get(node)
    path.reverse()
    return path

def generate_coverage_path(width, height, step, altitude, origin_x, origin_y):
    waypoints = []
    y = origin_y
    direction = 1
    while y <= origin_y + height:
        x_values = list(range(int(origin_x), int(origin_x + width)+1, int(step)))
        if direction == -1:
            x_values.reverse()
        for x in x_values:
            waypoints.append((x, y, altitude))
        y += step
        direction *= -1
    return waypoints

def main():
    rospy.init_node('coverage_planner')
    pub = rospy.Publisher('move_base_simple/goal', PoseStamped, queue_size=10)
    rospy.sleep(1)

    config_path = os.path.join(rospy.get_param('~config_path'), 'area.yaml')
    with open(config_path, 'r') as f:
        params = yaml.safe_load(f)

    waypoints = generate_coverage_path(
        params['area_width'], params['area_height'],
        params['grid_step'], params['flight_altitude'],
        params['origin_x'], params['origin_y']
    )

    rate = rospy.Rate(0.2)  # 0.2 Hz，每5秒发送一次
    for wp in waypoints:
        goal = PoseStamped()
        goal.header.frame_id = "world"
        goal.pose.position.x = wp[0]
        goal.pose.position.y = wp[1]
        goal.pose.position.z = wp[2]
        goal.pose.orientation.w = 1.0
        pub.publish(goal)
        rospy.loginfo(f"[Planner] waypoint: {wp}")
        rate.sleep()

    rospy.loginfo("[Planner] Coverage complete.")
    rospy.spin()

if __name__ == '__main__':
    main()

