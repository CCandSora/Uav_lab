"""
Two Crazyflies, sequential motions, but keep CF1 visible on ground.
"""
from pycrazyswarm import Crazyswarm

TAKEOFF_CF1 = 2.5
HOVER_CF1   = 5.0
TAKEOFF_CF2 = 2.0
HOVER_CF2   = 3.0

def get_cf(allcfs, cid):
    for cf in allcfs.crazyflies:
        if cf.id == cid:
            return cf
    raise RuntimeError(f"CF id={cid} not found")

def main():
    swarm = Crazyswarm()
    th    = swarm.timeHelper
    cf1   = get_cf(swarm.allcfs, 1)
    cf2   = get_cf(swarm.allcfs, 2)

    # ---------- CF1 起飞、悬停、降落 ----------
    cf1.takeoff(1.0, TAKEOFF_CF1)
    th.sleep(TAKEOFF_CF1 + HOVER_CF1)
    cf1.land(0.04, 2.5)
    th.sleep(2.5)

    # ---------- 在地面锁定 CF1 ----------
    ground_pos = [cf1.initialPosition[0],
                  cf1.initialPosition[1], 0.04]

    # ---------- CF2 动作，同时循环保活 CF1 ----------
    cf2.takeoff(0.5, TAKEOFF_CF2)

    total = TAKEOFF_CF2 + HOVER_CF2 + 1.5          # CF2 全流程大约时长
    t     = 0.0
    dt    = 0.1                                    # 每 0.1 s 发一次静止指令
    while t < total:
        cf1.cmdPosition(ground_pos, 0.0)           # 保持 CF1 可见
        th.sleep(dt)
        t += dt

    cf2.land(0.04, 1.5)
    # 睡够 1.5 s 再退出，让 CF2 降完
    th.sleep(1.5)

if __name__ == "__main__":
    main()

