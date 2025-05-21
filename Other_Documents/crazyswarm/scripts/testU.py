# goTo_waypoints.py
from pycrazyswarm import Crazyswarm
import numpy as np

WAYPOINTS = [
    [0.0, 0.0, 1.0],   # 起点
    [1.0, 0.0, 1.0],   # 右移
    [1.0, 1.0, 1.0],   # 前进
    [0.0, 1.0, 1.0],   # 左移
]
SEG_DURATION = 2.0

def main():
    swarm = Crazyswarm()
    th    = swarm.timeHelper
    cf1, cf2 = swarm.allcfs.crazyflies      # 假设只有 2 架

    # 同时起飞
    for cf in (cf1, cf2):
        cf.takeoff(1.0, 2.5)
    th.sleep(2.5)

    # 依次走路点
    for p in WAYPOINTS:
        for cf in (cf1, cf2):
            cf.goTo(p, 0, SEG_DURATION)
        th.sleep(SEG_DURATION)

    # 同时降落
    for cf in (cf1, cf2):
        cf.land(0.04, 2.5)
    th.sleep(2.5)

if __name__ == "__main__":
    main()

