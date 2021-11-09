
# encoding: utf-8
"""VMC_test_py controller."""

from controller import Robot
import math
import numpy as np
import copy
timestep = 1
# from VMC_seprate import Quadruped_robot_seprate
from VMC_mix import Quadruped_robot_mix
# from VMC_MAT import Quadruped_robot_MAT

if __name__ == '__main__':
    # q = Quadruped_robot_seprate()
    q =Quadruped_robot_mix()
    # q = Quadruped_robot_MAT()  #
    # webot_device_init()
    q.run()