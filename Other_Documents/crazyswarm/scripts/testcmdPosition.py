# circle_stream.py
from pycrazyswarm import Crazyswarm
import numpy as np

R        = 1.0      # 圆半径
ANG_VEL  = 0.5      # rad/s
FLY_TIME = 20.0     # 总时长
DT       = 0.02     # 发送周期 ≈50 Hz

def main():
    swarm = Crazyswarm()
    th    = swarm.timeHelper
    cf1, cf2 = swarm.allcfs.crazyflies

    for cf in (cf1, cf2):
        cf.takeoff(1.0, 2.5)
    th.sleep(2.5)

    t = 0.0
    while t < FLY_TIME:
        angle = ANG_VEL * t
        # 让两机在圆上相差 180°
        p1 = np.array([ R*np.cos(angle),  R*np.sin(angle), 1.0])
        p2 = np.array([-R*np.cos(angle), -R*np.sin(angle), 1.0])

        cf1.cmdPosition(p1, 0)
        cf2.cmdPosition(p2, 0)

        th.sleep(DT)
        t += DT

    for cf in (cf1, cf2):
        cf.land(0.04, 2.5)
    th.sleep(2.5)

if __name__ == "__main__":
    main()

