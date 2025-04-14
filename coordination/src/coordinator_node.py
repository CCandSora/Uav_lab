#!/usr/bin/env python3
# coordinator_node.py
import rospy
from geometry_msgs.msg import PoseStamped

MIN_SAFE_DIST = 2.0  # 定义水平安全距离阈值(米)
ALT_OFFSET = 2.0     # 高度错层(米)

# 全局变量记录无人机位置
uav1_pos = None
uav2_pos = None

def uav1_pose_callback(msg):
    global uav1_pos
    uav1_pos = msg.pose.position

def uav2_pose_callback(msg):
    global uav2_pos
    uav2_pos = msg.pose.position

if __name__ == '__main__':
    rospy.init_node('uav_coordinator')
    # 订阅两架无人机的位姿/位置话题 (假定各无人机发布自身位置)
    rospy.Subscriber('/uav1/mavros/local_position/pose', PoseStamped, uav1_pose_callback)
    rospy.Subscriber('/uav2/mavros/local_position/pose', PoseStamped, uav2_pose_callback)
    # 发布器，可用于发送调整指令，比如暂停指令或高度调整
    pause_pub_uav1 = rospy.Publisher('/uav1/pause', PoseStamped, queue_size=1)
    pause_pub_uav2 = rospy.Publisher('/uav2/pause', PoseStamped, queue_size=1)
    rate = rospy.Rate(10)
    rospy.loginfo("[Coordinator] Coordination node started, monitoring UAV separation...")
    # 初始高度错层设定（假定规划器已按不同高度规划，此处作为双重保险）
    init_alt_set = False

    while not rospy.is_shutdown():
        if uav1_pos and uav2_pos:
            # 计算水平距离
            dx = uav1_pos.x - uav2_pos.x
            dy = uav1_pos.y - uav2_pos.y
            horiz_dist = (dx**2 + dy**2) ** 0.5
            dz = abs(uav1_pos.z - uav2_pos.z)
            # 初次循环若未设定高度错层，则判断并调整
            if not init_alt_set:
                if dz < ALT_OFFSET*0.9:  # 如果高度差小于预期错层
                    rospy.loginfo("[Coordinator] Setting altitude separation.")
                    # 简单策略：假设uav1按规划飞行高度，调整uav2高度
                    adjust_msg = PoseStamped()
                    adjust_msg.header.frame_id = "world"
                    adjust_msg.pose.position.x = uav2_pos.x
                    adjust_msg.pose.position.y = uav2_pos.y
                    adjust_msg.pose.position.z = uav2_pos.z + ALT_OFFSET
                    adjust_msg.pose.orientation.w = 1.0
                    pause_pub_uav2.publish(adjust_msg)
                init_alt_set = True
            # 检查安全距离
            if horiz_dist < MIN_SAFE_DIST:
                rospy.logwarn(f"[Coordinator] UAVs too close! Distance={horiz_dist:.1f} m")
                # 简单处理：可让一架暂停或避让。这里以暂停uav2为例
                pause_pub_uav2.publish(PoseStamped())  # 发布空消息表示暂停
        rate.sleep()

