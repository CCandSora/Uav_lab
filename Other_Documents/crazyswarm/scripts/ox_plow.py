# ox_plow.py  两机牛耕式搜索示例
from pycrazyswarm import Crazyswarm
import numpy as np

# ----------------- 可调参数 -----------------
WORLD_SIZE = 5.0          # 世界边长 (m)
STRIP      = 0.5          # 相邻扫描线间距 (m)
Z          = 1.0          # 搜索高度  (m)
VEL        = 0.5          # 期望水平速度 (m/s)
CMD_FREQ   = 50           # 指令发送频率 (Hz)
# -------------------------------------------

DT = 1.0 / CMD_FREQ

def lawnmower_path(x_min, x_max, y_min, y_max, strip):
    """生成牛耕式轨迹路点列表（按行进方向给出）。"""
    waypoints = []
    flip = False
    y = y_min
    while y <= y_max:
        if not flip:
            waypoints.append([x_min, y, Z])
            waypoints.append([x_max, y, Z])
        else:
            waypoints.append([x_max, y, Z])
            waypoints.append([x_min, y, Z])
        flip = not flip
        y += strip
    return waypoints

def follow_waypoints(cf, th, wps):
    """按恒速线性插值在路点之间发送 cmdPosition。"""
    for i in range(len(wps) - 1):
        p0 = np.array(wps[i])
        p1 = np.array(wps[i + 1])
        dist = np.linalg.norm(p1 - p0)
        dur  = dist / VEL
        steps = int(dur / DT)
        for k in range(steps):
            p = p0 + (k / steps) * (p1 - p0)
            cf.cmdPosition(p, 0.0)
            th.sleep(DT)
    # 最后一帧锁定终点，避免 marker 消失
    cf.cmdPosition(wps[-1], 0.0)

def main():
    swarm = Crazyswarm()
    th    = swarm.timeHelper
    cfs   = swarm.allcfs.crazyflies
    assert len(cfs) >= 2, "需要至少两架 CF"

    cf1, cf2 = cfs[0], cfs[1]

    # 起飞到中心点上空 Z
    for cf in (cf1, cf2):
        cf.takeoff(Z, 2.5)
    th.sleep(2.5)

    # 为两架飞机生成各自区域的牛耕式路点
    wp1 = lawnmower_path(0.0, WORLD_SIZE/2, 0.0, WORLD_SIZE, STRIP)
    wp2 = lawnmower_path(WORLD_SIZE/2, WORLD_SIZE, 0.0, WORLD_SIZE, STRIP)

    # 并行执行搜索
    t_max = max(len(wp1), len(wp2)) * STRIP / VEL * 2
    t     = 0.0
    idx1 = idx2 = 0
    while t < t_max:
        if idx1 < len(wp1)-1:
            cf1.cmdPosition(wp1[idx1], 0.0)
            idx1 += 1
        if idx2 < len(wp2)-1:
            cf2.cmdPosition(wp2[idx2], 0.0)
            idx2 += 1
        th.sleep(DT)
        t += DT

    # 返回中心并降落
    center = np.array([WORLD_SIZE/2, WORLD_SIZE/2, Z])
    for cf in (cf1, cf2):
        cf.goTo(center, 0.0, 3.0)
    th.sleep(3.0)

    for cf in (cf1, cf2):
        cf.land(0.04, 3.0)
    th.sleep(3.0)

if __name__ == "__main__":
    main()

