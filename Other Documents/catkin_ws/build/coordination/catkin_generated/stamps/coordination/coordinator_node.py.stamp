#!/usr/bin/env python3
# coordinator_node.py
import rospy
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Bool
import math

# 安全距离阈值 (水平安全距离，单位：米)
MIN_SAFE_DISTANCE = 2.0

# 全局存储两架无人机位置信息
uav_positions = {"uav1": None, "uav2": None}

# 位置回调函数
def pose_callback_uav1(msg):
    uav_positions["uav1"] = msg.pose.position

def pose_callback_uav2(msg):
    uav_positions["uav2"] = msg.pose.position

# 计算两架无人机水平距离
def calculate_distance(pos1, pos2):
    dx = pos1.x - pos2.x
    dy = pos1.y - pos2.y
    return math.sqrt(dx*dx + dy*dy)

def coordinator():
    rospy.init_node('uav_coordinator')

    # 订阅无人机的位置
    rospy.Subscriber("/uav1/mavros/local_position/pose", PoseStamped, pose_callback_uav1)
    rospy.Subscriber("/uav2/mavros/local_position/pose", PoseStamped, pose_callback_uav2)

    # 避碰信号发布（如果无人机控制器需要的话）
    avoid_pub_uav1 = rospy.Publisher("/uav1/avoid_collision", Bool, queue_size=1)
    avoid_pub_uav2 = rospy.Publisher("/uav2/avoid_collision", Bool, queue_size=1)

    rate = rospy.Rate(10)  # 10Hz频率检测
    rospy.loginfo("[Coordinator] Started collision avoidance monitoring...")

    while not rospy.is_shutdown():
        pos1 = uav_positions["uav1"]
        pos2 = uav_positions["uav2"]

        if pos1 is not None and pos2 is not None:
            dist = calculate_distance(pos1, pos2)

            if dist < MIN_SAFE_DISTANCE:
                rospy.logwarn(f"[Coordinator] Warning: UAVs too close ({dist:.2f} m)! Initiating collision avoidance.")

                # 简单的避碰策略示例：通知第二架无人机暂停
                avoid_pub_uav1.publish(False)  # 第一架继续飞行
                avoid_pub_uav2.publish(True)   # 第二架暂停或避让（需飞控节点响应）
            else:
                # 若安全距离足够，均可正常飞行
                avoid_pub_uav1.publish(False)
                avoid_pub_uav2.publish(False)

        rate.sleep()

if __name__ == '__main__':
    try:
        coordinator()
    except rospy.ROSInterruptException:
        pass
