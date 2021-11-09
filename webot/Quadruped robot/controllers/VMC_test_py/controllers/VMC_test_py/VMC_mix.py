
# encoding: utf-8
"""VMC_test_py controller."""

from controller import Robot
import math
import numpy as np
import copy

# y1 = []

    # y1.append(i)  # 每迭代一次，将i放入y1中画出来
    
    # plt.pause(0.1)

# create the Robot instance.
robot = Robot()
timestep = 1

# 机器人所有马达
#   0  1  2  3
#   LF RF RB LB
# 0 跨 ........
# 1 髋 ........
# 2 膝 ........
robot_motor = []  # LF RF RB LB
for i in range(4):
    robot_motor.append([])
# 左前足马达
robot_motor[0].append(robot.createMotor("LFL0_rotational motor"))
robot_motor[0].append(robot.createMotor("LFL1_rotational motor"))
robot_motor[0].append(robot.createMotor("LFL2_rotational motor"))
# 右前足马达
robot_motor[1].append(robot.createMotor("RFL0_rotational motor"))
robot_motor[1].append(robot.createMotor("RFL1_rotational motor"))
robot_motor[1].append(robot.createMotor("RFL2_rotational motor"))
# 右后足马达
robot_motor[2].append(robot.createMotor("RBL0_rotational motor"))
robot_motor[2].append(robot.createMotor("RBL1_rotational motor"))
robot_motor[2].append(robot.createMotor("RBL2_rotational motor"))
# 左后足马达
robot_motor[3].append(robot.createMotor("LBL0_rotational motor"))
robot_motor[3].append(robot.createMotor("LBL1_rotational motor"))
robot_motor[3].append(robot.createMotor("LBL2_rotational motor"))

# 机器人所有位置传感器
#   0  1  2  3
#   LF RF RB LB
# 0 跨 ........
# 1 髋 ........
# 2 膝 ........
robot_pos_sensor = []  # LF RF RB LB
for i in range(4):
    robot_pos_sensor.append([])
# 左前足位置传感器
robot_pos_sensor[0].append(robot.createPositionSensor("LFL0_position sensor"))
robot_pos_sensor[0].append(robot.createPositionSensor("LFL1_position sensor"))
robot_pos_sensor[0].append(robot.createPositionSensor("LFL2_position sensor"))
# 右前足位置传感器
robot_pos_sensor[1].append(robot.createPositionSensor("RFL0_position sensor"))
robot_pos_sensor[1].append(robot.createPositionSensor("RFL1_position sensor"))
robot_pos_sensor[1].append(robot.createPositionSensor("RFL2_position sensor"))
# 右后足位置传感器
robot_pos_sensor[2].append(robot.createPositionSensor("RBL0_position sensor"))
robot_pos_sensor[2].append(robot.createPositionSensor("RBL1_position sensor"))
robot_pos_sensor[2].append(robot.createPositionSensor("RBL2_position sensor"))
# 左后足位置传感器
robot_pos_sensor[3].append(robot.createPositionSensor("LBL0_position sensor"))
robot_pos_sensor[3].append(robot.createPositionSensor("LBL1_position sensor"))
robot_pos_sensor[3].append(robot.createPositionSensor("LBL2_position sensor"))

# 触地传感器
# LF RF RB LB
robot_touch_sensor = []
robot_touch_sensor.append(robot.createTouchSensor("LF_touch sensor"))
robot_touch_sensor.append(robot.createTouchSensor("RF_touch sensor"))
robot_touch_sensor.append(robot.createTouchSensor("RB_touch sensor"))
robot_touch_sensor.append(robot.createTouchSensor("LB_touch sensor"))

# inertial unit
IMU = robot.createInertialUnit("inertial unit")
ACC = robot.createAccelerometer("accelerometer")
KeyBoard = robot.getKeyboard()
# Camera=robot.createCamera("camera")

# ROll Pich Yaw
def get_IMU_Angle():
    data = IMU.getRollPitchYaw()
    # return ROll Pich Yaw
    return [data[1] * 180.0 / math.pi, data[0] * 180.0 / math.pi, data[2] * 180.0 / math.pi]


def webot_device_init():
    # Camera.enable(timestep)
    KeyBoard.enable(timestep)
    ACC.enable(timestep)
    # IMU使能
    IMU.enable(timestep)
    for leg in range(4):
        # 跨关节锁定
        robot_motor[leg][0].setPosition(0)
        # 初始化使能接触传感器
        robot_touch_sensor[leg].enable(timestep)
        # 初始化位置传感器
        for joint in range(3):
            robot_pos_sensor[leg][joint].enable(timestep)


def set_motor_torque(leg, name, torque):
    max = 1800
    if torque > max:
        # print("error torque >max=", torque)
        torque = max
    if torque < -max:
        # print("error torque >max=", torque)
        torque = -max
    robot_motor[leg][name].setTorque(torque)


def get_motor_angle(leg, name):
    angle = 0
    angle = robot_pos_sensor[leg][name].getValue()
    return angle * 180.0 / math.pi


def get_all_motor_angle():
    temp_list = list()
    for leg in range(4):  # 遍历四条腿 每条腿三个关节
        temp_list.append([])
        for joint in range(3):
            temp_list[leg].append(get_motor_angle(leg, joint))
    return temp_list


def is_foot_touching(leg):
    return robot_touch_sensor[leg].getValue()


def all_is_foot_touching():
    temp_list = list()
    for leg in range(4):
        temp_list.append(robot_touch_sensor[leg].getValue())
    return temp_list


# def print_leg_angle():
#     for i in range(4):
#         print("腿", i, end=" ")
#         for j in range(3):
#             temp = get_motor_angle(i, j)
#             print("关节", j, "角度/°", int(temp), end=" ")
#         print("")


class Quadruped_robot_mix():
    def __init__(self):
        self.L = 0
        self.R = 1

        self.zh = -0.4  # 抬腿到高度 0.4
        self.H_des = - 0.55  # 0.5
        self.Tf = 0.25  # 飞行时间
        self.Ts = 0.25  # 支撑时间
        # 机器人机身参数
        self.L = 0.6  # 前后髋关节w
        self.W = 0.48  # 左右髋关节
        self.a0 = 0
        self.a1 = 0.4
        self.a2 = 0.4
        self.m0 = 4
        self.m1 = 4
        self.M = 50 + 8 * 4

        # 四足支撑系数
        self.Kpitch = 1000  # 1000
        self.Dpitch = 300  # 300

        self.Kroll = 1000  # 1000
        self.Droll = 300  # 300

        self.Kyaw = 1000  # 1000
        self.Dyaw = 300  # 300

        self.Kxs = 2000  # 1000
        self.Dxs = 100  # 100

        self.Kys = 2000  # 4000
        self.Dys = 500  # 300

        self.Kzs = 5000  # 4000
        self.Dzs = 200  # 300

        # 行进时系数
        self.Kpitch_r = 2000  # 2000
        self.Dpitch_r = 300  # 300

        self.Kroll_r = 2000  # 1000
        self.Droll_r = 400  # 400

        self.Kyaw_r = 700  # 6000

        # 行进支撑足系数
        self.Kx = 1000  # 1000
        # self.Dx = 100  # 100

        self.Ky = 10  # 4000
        self.Dy = 500  # 300

        self.Kz = 4000  # 3000
        self.Dz = 300  # 300

        # 悬空足系数
        self.Kxk = 1500  # 1000
        self.Dxk = 200  # 200

        self.Kzk = 5000  # 3000
        self.Dzk = 200  # 300

        self.Kyk = 1000  # 1000
        self.Dyk = 200  # 300

        # self.Dyaw = 10
        # 摆动时速度增益
        self.Kvx = 0.2  # 0.1
        self.Kvy = 0.25  # 0.25
        # 期望值
        self.Vx_des = 0
        self.Vy_des = 0
        self.W_des = 0

        self.Vx=0.3
        self.Vy=0.3
        self.W=40

        self.Pitch_des = - 0 / 180.0 * math.pi  # 上负
        self.Roll_des = 0 / 180.0 * math.pi  # 左负
        self.Yaw_des = 0

        # 存储变量
        # 当前欧拉角
        self.Pitch = 0
        self.Roll = 0
        self.Yaw = 0
        # 上一时刻欧拉角
        self.pre_eulerAngle = [0, 0, 0]
        # 欧拉角导数
        self.dot_Pitch = 0
        self.dot_Roll = 0
        self.dot_Yaw = 0
        # 上一时刻欧拉角导数
        self.dot_Pitch_pre = 0
        self.dot_Roll_pre = 0
        self.dot_Yaw_pre = 0
        # 支撑模式 0->L 1->R 3->全着地 4->悬空
        self.standFoot = 4
        self.swallFoot = 4
        # 四脚着地状态 逆时针编号
        self.is_touching = [0, 0, 0, 0]
        # 四足足端当前坐标
        self.pos = [[0, 0, 0], [0, 0, 0], [0, 0, 0], [0, 0, 0]]
        for leg in range(4):
            self.pos[leg] = self.forwardkinematics([0, 0.6, -1.28])
        # 四足关节角
        self.theta = list()
        # 四足足端当前导数
        self.dot_pos = [[0, 0, 0], [0, 0, 0], [0, 0, 0], [0, 0, 0]]
        # 四足足端当前二阶导数
        self.dot_2_pos = [[0, 0, 0], [0, 0, 0], [0, 0, 0], [0, 0, 0]]
        # 四足足端上一时坐标
        self.pre_pos = [[0, 0, 0], [0, 0, 0], [0, 0, 0], [0, 0, 0]]
        # 四足足端上一时刻导数
        self.pre_dot_pos = [[0, 0, 0], [0, 0, 0], [0, 0, 0], [0, 0, 0]]
        # 不态时间
        self.time = 0
        # 是否停止 用于切换行进和四足着地
        self.stop = 1
        # 用于切换悬空 四脚离地一定时间后进入悬空状态
        self.fly_time = 0
        # 键盘控制变量
        self.key_data = [0, 0, 0]
        self.mass_center = 0.5  # 位置系数

    def forwardkinematics(self, theta):
        a0 = self.a0
        a1 = self.a1  # 腿长
        a2 = self.a2  # 腿长
        # 横滚髋关节和俯仰髋关节偏差为0
        # theta= L0 L1 L2
        # 足底位置

        s0 = math.sin(theta[0] / 180.0 * math.pi)
        c0 = math.cos(theta[0] / 180.0 * math.pi)
        s1 = math.sin(theta[1] / 180.0 * math.pi)
        c1 = math.cos(theta[1] / 180.0 * math.pi)

        s12 = math.sin(theta[1] / 180.0 * math.pi + theta[2] / 180.0 * math.pi)
        c12 = math.cos(theta[1] / 180.0 * math.pi + theta[2] / 180.0 * math.pi)

        x = -a1 * s1 - a2 * s12  # + self.L / 2
        y = a0 * s0 + a1 * s0 * c1 + a2 * s0 * c12  # + self.W / 2
        z = - a0 * c0 - a1 * c0 * c1 - a2 * c0 * c12

        return [x, y, z]

    # 腿部坐标转换到机体坐标下
    def convert_pos(self, leg, pos):
        x, y, z = pos
        if leg == 0 or leg == 1:
            x += self.L / 2
        else:
            x -= self.L / 2

        if leg == 0 or leg == 3:
            y += self.W / 2
        else:
            y -= self.W / 2
        return [x, y, z]

    def convert_all_leg(self, pos):
        temp = []
        for i in range(4):
            temp.append(self.convert_pos(i, pos[i]))
        return temp

    def create_transJ(self, theta):
        a0 = self.a0
        a1 = self.a1  # 腿长
        a2 = self.a2  # 腿长
        # 横滚髋关节和俯仰髋关节偏差为0
        # theta= L0 L1 L2
        s0 = math.sin(theta[0] / 180.0 * math.pi)
        c0 = math.cos(theta[0] / 180.0 * math.pi)
        s1 = math.sin(theta[1] / 180.0 * math.pi)
        c1 = math.cos(theta[1] / 180.0 * math.pi)

        s12 = math.sin(theta[1] / 180.0 * math.pi + theta[2] / 180.0 * math.pi)
        c12 = math.cos(theta[1] / 180.0 * math.pi + theta[2] / 180.0 * math.pi)

        transJ = np.zeros((3, 3))
        transJ[0, 0] = 0
        transJ[0, 1] = -a0 * c1 - a2 * c12
        transJ[0, 2] = -a2 * c12

        transJ[1, 0] = c0 * (a0 + a1 * c1 + a2 * c12)
        transJ[1, 1] = -s0 * (a1 * s1 + a2 * s12)
        transJ[1, 2] = - a2 * s0 * s12

        transJ[2, 0] = s0 * (a0 + a1 * c1 + a2 * c12)
        transJ[2, 1] = c0 * (a1 * s1 + a2 * s12)
        transJ[2, 2] = a2 * c0 * s12
        return transJ.T

    def create_Q_inv(self, posf, posb):
        # q_inv = np.zeros((6, 6))

        xf = posf[0]
        yf = posf[1]
        zf = posf[2]

        xh = posb[0]
        yh = posb[1]
        zh = posb[2]

        # q_inv[0, 0] = ((xf - xh) * yh * zf - ((xf + xh) * yf - 2 * xf * yh) * zh) / 2 * (yf - yh) * (xh * zf - xf * zh)
        # q_inv[0, 1] = ((xf + xh) * (xf * yh - xh * yf)) / 2 * (yf - yh) * (xf * zh - xh * zf)
        # q_inv[0, 2] = ((xf - xh) * (xf + xh)) / 2 * (yf - yh) * (xh * zf - xf * zh)
        # q_inv[0, 3] = (xf + xh) / 2 * (xh * zf - xf * zh)
        # q_inv[0, 4] = ((xf - xh) * (zf + zh)) / 2 * (yf - yh) * (xh * zf - xf * zh)
        # q_inv[0, 5] = 0
        #
        # q_inv[1, 0] = (yh * zf - yf * zh) / 2 * (xh * zf - xf * zh)
        # q_inv[1, 1] = (xh * yf - xf * yh) / 2 * (xh * zf - xf * zh)
        # q_inv[1, 2] = (xf - xh) / 2 * (xh * zf - xf * zh)
        # q_inv[1, 3] = (yf - yh) / 2 * (xh * zf - xf * zh)
        # q_inv[1, 4] = (zf - zh) / 2 * (xh * zf - xf * zh)
        # q_inv[1, 5] = 0
        #
        # q_inv[2, 0] = ((zf + zh) * (zf * yh - zh * yf)) / 2 * (yf - yh) * (xh * zf - xf * zh)
        # q_inv[2, 1] = (xf * yh * (zh - zf) + xh * ((zf + zh) * yf - 2 * zf * yh)) / 2 * (yf - yh) * (xh * zf - xf * zh)
        # q_inv[2, 2] = ((xf + xh) * (zf - zh)) / 2 * (yf - yh) * (xh * zf - xf * zh)
        # q_inv[2, 3] = (zf + zh) / 2 * (xh * zf - xf * zh)
        # q_inv[2, 4] = ((zf + zh) * (zf - zh)) / 2 * (yf - yh) * (xh * zf - xf * zh)
        # q_inv[2, 5] = 0
        #
        # q_inv[3, 0] = (yf * zh * (xh - xf) + zf * (2 * yf * xh - (xf + xh) * yh)) / 2 * (yf - yh) * (xh * zf - xf * zh)
        # q_inv[3, 1] = ((xf + xh) * (xf * yh - xh * yf)) / 2 * (yf - yh) * (xh * zf - xf * zh)
        # q_inv[3, 2] = ((xf + xh) * (xf - xh)) / 2 * (yf - yh) * (xh * zf - xf * zh)
        # q_inv[3, 3] = (xf + xh) / 2 * (zh * xf - zf * xh)
        # q_inv[3, 4] = ((zf + zh) * (xh - xf)) / 2 * (yf - yh) * (xh * zf - xf * zh)
        # q_inv[3, 5] = 0
        #
        # q_inv[4, 0] = (yh * zf - yf * zh) / 2 * (xh * zf - xf * zh)
        # q_inv[4, 1] = (xh * yf - xf * yh) / 2 * (xh * zf - xf * zh)
        # q_inv[4, 2] = (xf - xh) / 2 * (xh * zf - xf * zh)
        # q_inv[4, 3] = (yf - yh) / 2 * (xh * zf - xf * zh)
        # q_inv[4, 4] = (zf - zh) / 2 * (xh * zf - xf * zh)
        # q_inv[4, 5] = 0
        #
        # q_inv[5, 0] = ((zf + zh) * (yf * zh - yh * zf)) / 2 * (yf - yh) * (xh * zf - xf * zh)
        # q_inv[5, 1] = (yf * xh * (zf - zh) + xf * ((zf + zh) * yh - 2 * yf * zh)) / 2 * (yf - yh) * (xh * zf - xf * zh)
        # q_inv[5, 2] = ((xf + xh) * (zh - zf)) / 2 * (yf - yh) * (xh * zf - xf * zh)
        # q_inv[5, 3] = (zf + zh) / 2 * (xf * zh - xh * zf)
        # q_inv[5, 4] = ((zf + zh) * (zh - zf)) / 2 * (yf - yh) * (xh * zf - xf * zh)
        # q_inv[5, 5] = 0
        q_inv = np.array([[(xf * yf * zh - xf * yh * zf - 2 * xf * yh * zh + xh * yf * zh + xh * yh * zf) / (
                2 * (xf * yf * zh - xh * yf * zf - xf * yh * zh + xh * yh * zf)),
                           (xf ** 2 * yh - xh ** 2 * yf - xf * xh * yf + xf * xh * yh) / (
                                   2 * (xf * yf * zh - xh * yf * zf - xf * yh * zh + xh * yh * zf)),
                           -((xf + xh) * (xf - xh)) / (2 * (xf * yf * zh - xh * yf * zf - xf * yh * zh + xh * yh * zf)),
                           -(xf + xh) / (2 * (xf * zh - xh * zf)),
                           -((zf + zh) * (xf - xh)) / (2 * (xf * yf * zh - xh * yf * zf - xf * yh * zh + xh * yh * zf)),
                           (xf - xh) / (2 * (yf - yh))],
                          [(yf * zh - yh * zf) / (2 * (xf * zh - xh * zf)),
                           (xf * yh - xh * yf) / (2 * (xf * zh - xh * zf)), -(xf - xh) / (2 * (xf * zh - xh * zf)),
                           -(yf - yh) / (2 * (xf * zh - xh * zf)), -(zf - zh) / (2 * (xf * zh - xh * zf)), 1 / 2],
                          [((yf * zh - yh * zf) * (zf + zh)) / (
                                  2 * (xf * yf * zh - xh * yf * zf - xf * yh * zh + xh * yh * zf)),
                           -(xh * yf * zf - xf * yh * zf + xf * yh * zh + xh * yf * zh - 2 * xh * yh * zf) / (
                                   2 * (xf * yf * zh - xh * yf * zf - xf * yh * zh + xh * yh * zf)),
                           -((xf + xh) * (zf - zh)) / (2 * (xf * yf * zh - xh * yf * zf - xf * yh * zh + xh * yh * zf)),
                           -(zf + zh) / (2 * (xf * zh - xh * zf)),
                           -((zf + zh) * (zf - zh)) / (2 * (xf * yf * zh - xh * yf * zf - xf * yh * zh + xh * yh * zf)),
                           (zf - zh) / (2 * (yf - yh))],
                          [(xf * yf * zh + xf * yh * zf - 2 * xh * yf * zf - xh * yf * zh + xh * yh * zf) / (
                                  2 * (xf * yf * zh - xh * yf * zf - xf * yh * zh + xh * yh * zf)),
                           -(xf ** 2 * yh - xh ** 2 * yf - xf * xh * yf + xf * xh * yh) / (
                                   2 * (xf * yf * zh - xh * yf * zf - xf * yh * zh + xh * yh * zf)),
                           ((xf + xh) * (xf - xh)) / (2 * (xf * yf * zh - xh * yf * zf - xf * yh * zh + xh * yh * zf)),
                           (xf + xh) / (2 * (xf * zh - xh * zf)),
                           ((zf + zh) * (xf - xh)) / (2 * (xf * yf * zh - xh * yf * zf - xf * yh * zh + xh * yh * zf)),
                           -(xf - xh) / (2 * (yf - yh))],
                          [(yf * zh - yh * zf) / (2 * (xf * zh - xh * zf)),
                           (xf * yh - xh * yf) / (2 * (xf * zh - xh * zf)), -(xf - xh) / (2 * (xf * zh - xh * zf)),
                           -(yf - yh) / (2 * (xf * zh - xh * zf)), -(zf - zh) / (2 * (xf * zh - xh * zf)), -1 / 2],
                          [-((yf * zh - yh * zf) * (zf + zh)) / (
                                  2 * (xf * yf * zh - xh * yf * zf - xf * yh * zh + xh * yh * zf)),
                           -(xf * yh * zf - 2 * xf * yf * zh + xh * yf * zf + xf * yh * zh - xh * yf * zh) / (
                                   2 * (xf * yf * zh - xh * yf * zf - xf * yh * zh + xh * yh * zf)),
                           ((xf + xh) * (zf - zh)) / (2 * (xf * yf * zh - xh * yf * zf - xf * yh * zh + xh * yh * zf)),
                           (zf + zh) / (2 * (xf * zh - xh * zf)),
                           ((zf + zh) * (zf - zh)) / (2 * (xf * yf * zh - xh * yf * zf - xf * yh * zh + xh * yh * zf)),
                           -(zf - zh) / (2 * (yf - yh))]
                          ])
        return q_inv

    def update_food_touch_sensor(self):
        self.is_touching = all_is_foot_touching()

    def update_theta(self):
        self.theta = get_all_motor_angle()

    # 相位切换, 通过时间和足底传感器确定支撑对角腿和摆动对角腿
    def phase_swap(self):
        if self.time > 0.75 * self.Tf:
            if self.standFoot == self.L:
                if self.is_touching == [1, 1, 1, 1]:
                    self.standFoot = self.R
                    self.swallFoot = self.L
                    self.time = 0
                    # print("L-->R")

            elif self.standFoot == self.R:
                if self.is_touching == [1, 1, 1, 1]:
                    self.standFoot = self.L
                    self.swallFoot = self.R
                    self.time = 0
                    # print("R-->L")
            elif self.standFoot == 4:
                if self.is_touching == [1, 1, 1, 1]:
                    self.standFoot = 3
                    self.swallFoot = 3
                    self.stop = 1
                    self.time = 0
                    # print("悬空-->四脚着地")
            elif self.standFoot == 3:
                if self.is_touching == [1, 1, 1, 1] and self.stop == 0:
                    self.standFoot = self.R
                    self.swallFoot = self.L
                    self.time = 0
                    # print("四脚着地-->行进")
            if self.is_touching == [1, 1, 1, 1] and self.stop == 1:
                self.standFoot = 3
                self.swallFoot = 3
                self.time = 0
                # print("任意-->四脚着地")
            if self.is_touching == [0, 0, 0, 0]:  # and self.stop == 0
                self.fly_time += 0.001 * timestep
                if self.fly_time > 0.09:
                    self.standFoot = 4
                    self.swallFoot = 4
                    self.stop = 1
                    self.fly_time = 0
                    self.time = 0
                    # print("任意-->悬空")

    # 更新全部状态
    def update_robot_state(self):
        self.key_control()
        # 更新IMU及导数
        self.update_IMU()
        # 足底接触传感器更新
        self.update_food_touch_sensor()
        # 更新关节角
        self.update_theta()
        # 更新四条腿的运动学正解
        for leg in range(4):
            for joint in range(3):
                self.pos[leg] = self.forwardkinematics(self.theta[leg])
        self.update_V_P()
        self.phase_swap()
        self.time += timestep * 0.001

    # IMU获得欧拉角
    def update_IMU(self):
        # print("pre roll pitch yaw", self.pre_eulerAngle)
        eulerAngle = get_IMU_Angle()  # Roll Pitch Yaw 当前欧拉角
        # print("roll pitch yaw",eulerAngle)
        self.Roll = eulerAngle[0]  # 获得当前roll
        self.dot_Roll = (eulerAngle[0] - self.pre_eulerAngle[0]) / (0.001 * timestep)  # 求导数 dot_roll
        self.dot_Roll = self.dot_Roll * 0.3 + self.dot_Roll_pre * 0.7
        self.dot_Roll_pre = self.dot_Roll  # 更新记录上次的变量 dot_Roll_pre

        self.Pitch = eulerAngle[1]  # 获得当前pitch
        self.dot_Pitch = (eulerAngle[1] - self.pre_eulerAngle[1]) / (0.001 * timestep)  # 求导数 dot_pitch
        self.dot_Pitch = self.dot_Pitch * 0.3 + self.dot_Pitch_pre * 0.7  # 低通滤波
        self.dot_Pitch_pre = self.dot_Pitch  # 更新记录上次的变量 dot_Pitch_pre

        self.Yaw = eulerAngle[2]  # 获得当前yaw
        self.dot_Yaw = (eulerAngle[2] - self.pre_eulerAngle[2]) / (0.001 * timestep)  # 求导数 dot_yaw
        self.dot_Yaw = self.dot_Yaw * 0.3 + self.dot_Yaw_pre * 0.7  # 低通滤波
        self.dot_Yaw_pre = self.dot_Yaw  # 更新记录上次的变量 dot_Yaw_pre

        self.pre_eulerAngle = copy.deepcopy(eulerAngle)

    # 速度，加速度更新
    def update_V_P(self):
        dot_pos = (((np.array(self.pos) - np.array(self.pre_pos))
                    / (0.001 * timestep)) * 1.0 + np.array(self.pre_dot_pos) * 0).tolist()
        # 上一步坐标
        self.pre_pos = copy.deepcopy(self.pos)
        # 对速度求导 所有腿和关节 加速度
        dot_2_pos = ((np.array(dot_pos) - np.array(self.pre_dot_pos)) / (0.001 * timestep)).tolist()
        # 上一步导数
        self.pre_dot_pos = copy.deepcopy(dot_pos)
        self.dot_pos = copy.deepcopy(dot_pos)
        self.dot_2_pos = copy.deepcopy(dot_2_pos)
        pass

    # 键盘控制
    def key_control(self):
        key = KeyBoard.getKey()
        if key == KeyBoard.UP:
            self.Pitch_des += 0.05 / 180.0 * math.pi
            print(self.Pitch_des)
        elif key == KeyBoard.DOWN:
            self.Pitch_des -= 0.05 / 180.0 * math.pi
            print(self.Pitch_des)
        elif key == KeyBoard.LEFT:
            self.Roll_des -= 0.05 / 180.0 * math.pi
        elif key == KeyBoard.RIGHT:
            self.Roll_des += 0.05 / 180.0 * math.pi
        elif key == ord('M'):
            self.Yaw_des -= 0.05 / 180.0 * math.pi
        elif key == ord('N'):
            self.Yaw_des += 0.05 / 180.0 * math.pi

        elif key == ord('P'):
            self.stop = 1
            self.Pitch_des = 0
            self.Roll_des = 0
            self.Yaw_des = self.Yaw / 180.0 * math.pi
        # 纯平移
        elif key == ord('W'):
            self.stop = 0
            self.Vx_des = self.acc(0.0004, self.Vx_des, -self.Vx)
            # print(self.Vx_des)
        elif key == ord('S'):
            self.stop = 0
            self.Vx_des = self.acc(0.0004, self.Vx_des, self.Vx)
        elif key == ord('A'):
            self.Vx_des = self.acc(0.0004, self.Vx_des, 0)
            self.Vy_des = self.acc(0.04, self.Vy_des, -self.Vy)
            self.stop = 0
        elif key == ord('D'):
            self.Vx_des = self.acc(0.0004, self.Vx_des, 0)
            self.Vy_des = self.acc(0.04, self.Vy_des, self.Vy)
            self.stop = 0
        elif key == ord('Q'):
            self.stop = 0
            self.Vx_des = self.acc(0.0004, self.Vx_des, -self.Vx)
            self.Vy_des = self.acc(0.04, self.Vy_des, -self.Vy)
        elif key == ord('E'):
            self.stop = 0
            self.Vx_des = self.acc(0.0004, self.Vx_des, -self.Vx)
            self.Vy_des = self.acc(0.04, self.Vy_des, self.Vy)

        elif key == ord('Z'):
            self.stop = 0
            self.Vx_des = self.acc(0.0004, self.Vx_des, self.Vx)
            self.Vy_des = self.acc(0.04, self.Vy_des, -self.Vy)
        elif key == ord('C'):
            self.stop = 0
            self.Vx_des = self.acc(0.0004, self.Vx_des, self.Vx)
            self.Vy_des = self.acc(0.04, self.Vy_des, self.Vy)
            # 选装平移
        elif key == ord('J'):
            self.Vx_des = self.acc(0.0004, self.Vx_des, 0)
            self.W_des = self.acc(0.05 / 180.0 * math.pi, self.W_des, -self.W / 180.0 * math.pi)
            self.stop = 0
        elif key == ord('L'):
            self.Vx_des = self.acc(0.0004, self.Vx_des, 0)
            self.W_des = self.acc(0.05 / 180.0 * math.pi, self.W_des, self.W / 180.0 * math.pi)
            self.stop = 0
        elif key == ord('U'):
            self.Vx_des = self.acc(0.0004, self.Vx_des, -self.Vx)
            self.W_des = self.acc(0.05 / 180.0 * math.pi, self.W_des, -self.W / 180.0 * math.pi)

            self.stop = 0
        elif key == ord('O'):
            self.Vx_des = self.acc(0.0004, self.Vx_des, -self.Vx)
            self.W_des = self.acc(0.05 / 180.0 * math.pi, self.W_des, self.W / 180.0 * math.pi)
            self.stop = 0
            # 高度调整
        elif key == ord('Y'):
            self.H_des -= 0.0001 * timestep
            self.zh = self.H_des + 0.12
            print("目标 ", self.H_des, "当前", self.pos[0][2])
            print("当前方位角 ROLL Pitch Yaw ", self.Roll, self.Pitch, self.Yaw)
        elif key == ord('H'):
            self.H_des += 0.0001 * timestep
            self.zh = self.H_des + 0.12
            print(self.H_des)
            print("目标 ", self.H_des, "当前", self.pos[0][2])
        else:
            # self.stop = 0
            self.Vx_des = self.acc(0.0004, self.Vx_des, -0.07)
            # self.Pitch_des = self.acc(0.00004, self.Pitch_des, 0)
            self.W_des = self.acc(0.05, self.W_des, 0)
            self.Vy_des = self.acc(0.04, self.Vy_des, 0)

    def acc(self, acc_value, dot, vel):
        value = dot
        if (dot < vel):
            value += acc_value
            if (dot > vel):
                value = vel
        else:
            value -= acc_value
            if (dot < vel):
                value = vel
        return value

    # 轨迹
    def curve(self, xt, yt, init_pos, init_dotpos):
        ft = [0, 0, 0]  # x,z
        dft = [0, 0, 0]  # x,z
        xT = xt
        yT = yt
        t = self.time
        x0 = init_pos[0]
        y0 = init_pos[1]
        z0 = init_pos[2]
        dx0 = init_dotpos[0]
        dy0 = init_dotpos[1]
        Tf = self.Tf
        zh = self.zh
        # // x, y
        if t < Tf / 4.0:
            ft[0] = -4 * dx0 * t * t / Tf + dx0 * t + x0
            dft[0] = -8 * dx0 * t / Tf + dx0
        elif (t >= Tf / 4.0) and (t < 3.0 * Tf / 4.0):

            ft[0] = (-4 * Tf * dx0 - 16 * xT + 16 * x0) * t * t * t / (Tf * Tf * Tf) + \
                    (7 * Tf * dx0 + 24 * xT - 24 * x0) * t * t / (Tf * Tf) + \
                    (-15 * Tf * dx0 - 36 * xT + 36 * x0) * t / (4 * Tf) + \
                    (9 * Tf * dx0 + 16 * xT) / 16
            dft[0] = (-4 * Tf * dx0 - 16 * xT + 16 * x0) * 3 * t * t / (Tf * Tf * Tf) + \
                     (7 * Tf * dx0 + 24 * xT - 24 * x0) * 2 * t / (Tf * Tf) + \
                     (-15 * Tf * dx0 - 36 * xT + 36 * x0) / (4 * Tf)
        else:
            ft[0] = xT
            dft[0] = 0
        # y
        if t < Tf / 4.0:
            ft[1] = -4 * dy0 * t * t / Tf + dy0 * t + y0
            dft[1] = -8 * dy0 * t / Tf + dy0
        elif (t >= Tf / 4.0) and (t < 3.0 * Tf / 4.0):

            ft[1] = (-4 * Tf * dy0 - 16 * yT + 16 * y0) * t * t * t / (Tf * Tf * Tf) + \
                    (7 * Tf * dy0 + 24 * yT - 24 * y0) * t * t / (Tf * Tf) + \
                    (-15 * Tf * dy0 - 36 * yT + 36 * y0) * t / (4 * Tf) + \
                    (9 * Tf * dy0 + 16 * yT) / 16
            dft[1] = (-4 * Tf * dy0 - 16 * yT + 16 * y0) * 3 * t * t / (Tf * Tf * Tf) + \
                     (7 * Tf * dy0 + 24 * yT - 24 * y0) * 2 * t / (Tf * Tf) + \
                     (-15 * Tf * dy0 - 36 * yT + 36 * y0) / (4 * Tf)
        else:
            ft[1] = yT
            dft[1] = 0

        # // z
        if t < Tf / 2.0:
            ft[2] = 16 * (z0 - zh) * t * t * t / (Tf * Tf * Tf) + 12 * (zh - z0) * t * t / (Tf * Tf) + z0
            dft[2] = 16 * (z0 - zh) * 3 * t * t / (Tf * Tf * Tf) + 12 * (zh - z0) * 2 * t / (Tf * Tf)
        elif (t >= Tf / 2.0) and (t < 3.0 * Tf / 4.0):
            ft[2] = 4 * (z0 - zh) * t * t / (Tf * Tf) - 4 * (z0 - zh) * t / Tf + z0
            dft[2] = 4 * (z0 - zh) * 2 * t / (Tf * Tf) - 4 * (z0 - zh) / Tf
        else:
            ft[2] = 4 * (z0 - zh) * t * t / (Tf * Tf) - 4 * (z0 - zh) * t / Tf + z0
            dft[2] = 4 * (z0 - zh) * 2 * t / (Tf * Tf) - 4 * (z0 - zh) / Tf

        return [ft, dft]

    def run(self):
        init_pos_sw = [[0, 0, 0], [0, 0, 0], [0, 0, 0], [0, 0, 0]]  # 左右前后
        init_dotpos_sw = [[0, 0, 0], [0, 0, 0], [0, 0, 0], [0, 0, 0]]  # 左右前后

        path_pos = [[0, 0, 0], [0, 0, 0], [0, 0, 0], [0, 0, 0]]  # 左右前后
        dot_path_pos = [[0, 0, 0], [0, 0, 0], [0, 0, 0], [0, 0, 0]]  # 左右前后
        webot_device_init()
        switch = 0
        while robot.step(timestep) != -1:
            self.update_robot_state()
            # 获取支撑腿和摆动腿状态
            sw = [0, 0, 0, 0, 0]  # 摆动足 腿序号 左，右，前，后，特殊位
            st = [0, 0, 0, 0, 0]  # 支撑足 腿序号 左，右，前，后，特殊位
            # 左脚着地
            if self.standFoot == self.L:  # 左对角线

                st[0] = 0  # 左
                st[1] = 2  # 右
                st[2] = 0  # 前
                st[3] = 2  # 后

                sw[0] = 3  # 左
                sw[1] = 1  # 右
                sw[2] = 1  # 前
                sw[3] = 3  # 后
            # 右脚着地
            elif self.standFoot == self.R:  # 右对角线
                st[0] = 3  # 左
                st[1] = 1  # 右
                st[2] = 1  # 前
                st[3] = 3  # 后

                sw[0] = 0  # 左
                sw[1] = 2  # 右
                sw[2] = 0  # 前
                sw[3] = 2  # 后
            # 四脚着地
            elif self.standFoot == 3:
                st[0] = 3  # 左
                st[1] = 1  # 右
                st[2] = 1  # 前
                st[3] = 3  # 后

                sw[0] = 0  # 左
                sw[1] = 2  # 右
                sw[2] = 0  # 前
                sw[3] = 2  # 后
                st[4] = 100
            # 四脚悬空
            elif self.standFoot == 4:
                sw[4] = 200
            # print(self.standFoot, 'sw', sw)
            # 对角步态状态控----------------------------------------------------------------------------------对角步态状态控
            if st[4] == 0 and sw[4] == 0:
                # 支撑足 非stop
                # 计算姿态虚拟力
                pi_angle = math.atan2(self.pos[st[2]][2] - self.pos[st[3]][2],
                                      self.pos[st[2]][0] + 0.3 - self.pos[st[3]][
                                          0] + 0.3) - self.Pitch / 180.0 * math.pi
                # print("pi_angle",pi_angle/math.pi*180,"Pitch",self.Pitch)

                if abs(self.pos[st[2]][2] - self.pos[st[3]][2]) >= 0.16 and switch == 0:
                    switch = 1
                if switch == 1:
                    self.Pitch_des = self.acc(0.0002, self.Pitch_des, -pi_angle)
                    if abs(self.pos[st[2]][2] - self.pos[st[3]][2]) <= 0.0001:
                        switch = 0
                else:
                    # self.Pitch_des = 0
                    switch = 0

                F_pitch = self.Kpitch_r * (self.Pitch_des - self.Pitch / 180.0 * math.pi) + \
                          self.Dpitch_r / 2 * (0 - self.dot_Pitch / 180.0 * math.pi)
                F_roll = self.Kroll_r * (self.Roll_des - self.Roll / 180.0 * math.pi) + \
                         self.Droll_r * (0 - self.dot_Roll / 180.0 * math.pi)
                if not self.W_des == 0:
                    F_yaw = (self.Kyaw_r * (
                            self.W_des - (self.dot_Yaw * 0.5 + self.dot_Yaw_pre * 0.5) / 180.0 * math.pi))
                else:
                    F_yaw = 0

                # 高低叠加控制姿态
                K_pi_z = 0.01
                H_des_front = + K_pi_z * F_pitch
                H_des_back = - K_pi_z * F_pitch

                K_roll_z = 0.01  # 0.01
                H_des_left = - K_roll_z * F_roll
                H_des_right = + K_roll_z * F_roll

                Kyaw_x = 0.0003
                Vx_des_left = self.Vx_des - Kyaw_x * F_yaw
                Vx_des_right = self.Vx_des + Kyaw_x * F_yaw

                Kyaw_y = 0.000005  # 0.000001
                Vy_des_front = self.Vy_des + Kyaw_y * F_yaw
                Vy_des_back = self.Vy_des - Kyaw_y * F_yaw

                Vy_left = 0
                Vy_right = 0
                Hz_left = self.H_des
                Hz_right = self.H_des
                # 左前右后对应
                if st.index(st[2]) == 0:
                    Hz_left = self.H_des + H_des_left + H_des_front
                    Hz_right = self.H_des + H_des_right + H_des_back

                    Vy_left = Vy_des_front
                    Vy_right = Vy_des_back
                # 左后右前对应
                elif st.index(st[2]) == 1:
                    Hz_left = self.H_des + H_des_left + H_des_back
                    Hz_right = self.H_des + H_des_right + H_des_front

                    Vy_left = Vy_des_back
                    Vy_right = Vy_des_front

                    # 计算X轴虚拟力
                Fx_left = self.Kx * (Vx_des_left - self.dot_pos[st[0]][0]) \
                          + self.M * 9.8 * math.sin(self.Pitch / 180.0 * math.pi) * self.mass_center
                Fx_right = self.Kx * (Vx_des_right - self.dot_pos[st[1]][0]) \
                           + self.M * 9.8 * math.sin(self.Pitch / 180.0 * math.pi) * (1 - self.mass_center)

                # 支撑腿Y控制
                # init_pos_st[0][1]
                Fy_left = 0 * self.Ky * (0 - self.pos[st[0]][1]) + \
                          self.Dy * (Vy_left - self.dot_pos[st[0]][1]) \
                          + self.M * 9.81 * math.sin(self.Roll / 180.0 * math.pi) * self.mass_center
                Fy_right = 0 * self.Ky * (0 - self.pos[st[1]][1]) + \
                           self.Dy * (Vy_right - self.dot_pos[st[1]][1]) \
                           + self.M * 9.81 * math.sin(self.Roll / 180.0 * math.pi) * (1 - self.mass_center)

                Fz_left = self.Kz * (Hz_left - self.pos[st[0]][2]) + \
                          self.Dz * (0 - self.dot_pos[st[0]][2]) \
                          - self.M * 9.81 * self.mass_center * math.cos(self.Pitch / 180.0 * math.pi)
                Fz_right = self.Kz * (Hz_right - self.pos[st[1]][2]) + \
                           self.Dz * (0 - self.dot_pos[st[1]][2]) \
                           - self.M * 9.81 * (1 - self.mass_center) * math.cos(self.Pitch / 180.0 * math.pi)

                T_left = np.matmul(self.create_transJ(self.theta[st[0]]), np.array([[Fx_left], [Fy_left], [Fz_left]]))
                T_right = np.matmul(self.create_transJ(self.theta[st[1]]), np.array([[Fx_right], [Fy_right], [Fz_right]]))
                KR = 1
                set_motor_torque(st[0], 0, T_left[0, 0] - KR * F_roll / 2)
                set_motor_torque(st[0], 1, T_left[1, 0])  # -KR*F_roll/2
                set_motor_torque(st[0], 2, T_left[2, 0])

                set_motor_torque(st[1], 0, T_right[0, 0] - KR * F_roll / 2)
                set_motor_torque(st[1], 1, T_right[1, 0])
                set_motor_torque(st[1], 2, T_right[2, 0])

                # 摆动足--------------------------------------------------------------------------------------------------摆动足

                # 获得初始速度和位置

                if self.time == 0.001 * timestep:  # 刚刚切换过对角腿，记录此时的摆动足即可
                    init_pos_sw[0] = self.pos[sw[0]]
                    init_pos_sw[1] = self.pos[sw[1]]
                    init_pos_sw[2] = self.pos[sw[2]]
                    init_pos_sw[3] = self.pos[sw[3]]

                    init_dotpos_sw[0] = self.dot_pos[sw[0]]
                    init_dotpos_sw[1] = self.dot_pos[sw[1]]
                    init_dotpos_sw[2] = self.dot_pos[sw[2]]
                    init_dotpos_sw[3] = self.dot_pos[sw[3]]
                # print("初始点 初始速度", init_pos, init_dotpos)
                est_dot_pos = [[0, 0, 0], [0, 0, 0], [0, 0, 0], [0, 0, 0]]
                if not self.time == 0.001 * timestep:
                    est_dot_pos = (np.array(self.dot_pos) * 0.7 + np.array(self.pre_dot_pos) * 0.3).tolist()
                # print(est_dot_pos[st[2]][1])
                errorL = 0
                error_a = 0
                error_b = 0
                if abs(self.Vx_des - 0) <= 0.02:
                    errorL = 0.1
                elif self.Vx_des > 0.1 :
                    errorL = 0.1+ self.Vx_des * 0.375
                # elif self.Vx_des > 0.4:
                    # errorL =0.05+ self.Vx_des * 0.4
                    # print(errorL)
                else:
                    errorL = self.Vx_des * 0.15
                # 上坡重心修正
                if abs(self.Pitch) > 10 / 180.0 * math.pi:
                    temp = -math.tan(self.Pitch / 180.0 * math.pi) * (self.pos[st[2]][2] + self.pos[st[3]][2]) * 0.5*1.3
                    error_a = self.acc(0.006, error_a, temp)
                if abs(self.Roll) > 10 / 180.0 * math.pi:
                    temp = math.tan(self.Roll / 180.0 * math.pi) * (self.pos[st[2]][2] + self.pos[st[3]][2]) * 0.5
                    error_b = self.acc(0.006, error_b, temp)

                xt = [0.2, 0.2]  # 落足点 前后
                xt[0] = est_dot_pos[st[2]][0] * self.Ts / 2.0 - 1*self.Kvx * (
                        self.dot_pos[st[2]][0] - Vx_des_left) - errorL + error_a
                xt[1] = est_dot_pos[st[3]][0] * self.Ts / 2.0 - 1*self.Kvx * (
                        self.dot_pos[st[3]][0] - Vx_des_right) - errorL + error_a

                yt = [0, 0]  # 落足点 前后
                yt[0] = est_dot_pos[st[2]][1] * self.Ts / 2.0 - self.Kvy * (
                        self.dot_pos[st[2]][1] - Vy_des_front) + error_b
                yt[1] = est_dot_pos[st[3]][1] * self.Ts / 2.0 - self.Kvy * (
                        self.dot_pos[st[3]][1] - Vy_des_back) + error_b

                # 落足点转换
                yT = [0, 0]  # 落足点 左右

                xT = [0, 0]  # 落足点 左右
                if st.index(st[2]) == 0:
                    yT[0] = yt[0]
                    yT[1] = yt[1]
                    xT[0] = xt[0]
                    xT[1] = xt[1]
                # 左后右前对应
                elif st.index(st[2]) == 1:
                    yT[0] = yt[1]
                    yT[1] = yt[0]
                    xT[0] = xt[1]
                    xT[1] = xt[0]

                # 中途摆动触底
                # 左
                if self.is_touching[sw[0]] == 1 and self.time > 0.5 * self.Tf:
                    path_pos[0], dot_path_pos[0] = self.pos[sw[0]], [0, 0, 0]
                    # pass
                else:
                    path_pos[0], dot_path_pos[0] = self.curve(xT[0], yT[0], init_pos_sw[0], init_dotpos_sw[0])
                    # 右
                if self.is_touching[sw[1]] == 1 and self.time > 0.5 * self.Tf:
                    path_pos[1], dot_path_pos[1] = self.pos[sw[1]], [0, 0, 0]
                else:
                    path_pos[1], dot_path_pos[1] = self.curve(xT[1], yT[1], init_pos_sw[1], init_dotpos_sw[1])

                # path_pos = [[0, 0, -0.45], [0, 0, -0.45], [0, 0, -0.45], [0, 0, -0.45]]  # 左右前后
                # dot_path_pos = [[0, 0, 0], [0, 0, 0], [0, 0, 0], [0, 0, 0]]  # 左右前后
                # 左摆动足 索引值sw[0]
                Fx_left_sw = self.Kxk * (path_pos[0][0] - self.pos[sw[0]][0]) + \
                             self.Dxk * (dot_path_pos[0][0] - self.dot_pos[sw[0]][0]) \
                             + 8 * 9.8 * math.sin(self.Pitch / 180.0 * math.pi)
                # 右摆动足 索引值sw[1]
                Fx_right_sw = self.Kxk * (path_pos[1][0] - self.pos[sw[1]][0]) + \
                              self.Dxk * (dot_path_pos[1][0] - self.dot_pos[sw[1]][0]) \
                              + 8 * 9.8 * math.sin(self.Pitch / 180.0 * math.pi)

                Fy_left_sw = self.Kyk * (path_pos[0][1] - self.pos[sw[0]][1]) + \
                             self.Dyk * (dot_path_pos[0][1] - self.dot_pos[sw[0]][1]) \
                             + 8 * 9.8 * math.sin(self.Roll / 180.0 * math.pi)
                # 后摆动足 索引值sw[3]
                Fy_right_sw = self.Kyk * (path_pos[1][1] - self.pos[sw[1]][1]) + \
                              self.Dyk * (dot_path_pos[1][1] - self.dot_pos[sw[1]][1]) \
                              + 8 * 9.8 * math.sin(self.Roll / 180.0 * math.pi)

                if self.time <= 0.55 * self.Tf:
                    KKZ = self.Kzk
                    KKD = self.Dzk
                else:
                    KKZ = 1000
                    KKD = 100

                Fz_left_sw = KKZ * (path_pos[0][2] - self.pos[sw[0]][2]) + \
                             KKD * (dot_path_pos[0][2] - self.dot_pos[sw[0]][2]) - 8 * 9.81 * math.cos(
                    self.Pitch / 180.0 * math.pi)

                Fz_right_sw = KKZ * (path_pos[1][2] - self.pos[sw[1]][2]) + \
                              KKD * (dot_path_pos[1][2] - self.dot_pos[sw[1]][2]) - 8 * 9.81 * math.cos(
                    self.Pitch / 180.0 * math.pi)

                T_left_sw = np.matmul(self.create_transJ(self.theta[sw[0]]), np.array(
                    [[Fx_left_sw], [Fy_left_sw], [Fz_left_sw]]))
                T_right_sw = np.matmul(self.create_transJ(self.theta[sw[1]]), np.array(
                    [[Fx_right_sw], [Fy_right_sw], [Fz_right_sw]]))

                set_motor_torque(sw[0], 0, T_left_sw[0, 0])
                set_motor_torque(sw[0], 1, T_left_sw[1, 0])
                set_motor_torque(sw[0], 2, T_left_sw[2, 0])

                set_motor_torque(sw[1], 0, T_right_sw[0, 0])
                set_motor_torque(sw[1], 1, T_right_sw[1, 0])
                set_motor_torque(sw[1], 2, T_right_sw[2, 0])

            # 四足站立状态控制------------------------------------------------------------------------------------------四足站立状态控制
            if st[4] == 100:
                # print("run")

                T_pitch = self.Kpitch * (self.Pitch_des - self.Pitch / 180.0 * math.pi) + \
                          self.Dpitch / 2 * (0 - self.dot_Pitch / 180.0 * math.pi)
                T_roll = self.Kroll * (self.Roll_des - self.Roll / 180.0 * math.pi) + \
                         self.Droll * (0 - self.dot_Roll / 180.0 * math.pi)
                K_pi_z = 0.01  # 0.02
                H_des_front = + K_pi_z * T_pitch / 2
                H_des_back = - K_pi_z * T_pitch / 2

                K_roll_z = 0.01
                H_des_left = - K_roll_z * T_roll / 2
                H_des_right = + K_roll_z * T_roll / 2

                Fz_left_F = self.Kzs * (self.H_des + H_des_left + H_des_front - self.pos[0][2]) + self.Dzs * (
                        0 - self.dot_pos[0][2])
                Fz_right_F = self.Kzs * (self.H_des + H_des_right + H_des_front - self.pos[1][2]) + self.Dzs * (
                        0 - self.dot_pos[1][2])
                Fz_left_B = self.Kzs * (self.H_des + H_des_left + H_des_back - self.pos[3][2]) + self.Dzs * (
                        0 - self.dot_pos[3][2])
                Fz_right_B = self.Kzs * (self.H_des + H_des_right + H_des_back - self.pos[2][2]) + self.Dzs * (
                        0 - self.dot_pos[2][2])

                Fz_left_F = Fz_left_F - self.M * 10 / 4 * math.cos(self.Pitch / 180.0 * math.pi)
                Fz_right_F = Fz_right_F - self.M * 10 / 4 * math.cos(self.Pitch / 180.0 * math.pi)
                Fz_left_B = Fz_left_B - self.M * 10 / 4 * math.cos(self.Pitch / 180.0 * math.pi)
                Fz_right_B = Fz_right_B - self.M * 10 / 4 * 2 * math.cos(self.Pitch / 180.0 * math.pi)

                T_yaw = self.Kyaw * (self.Yaw_des - self.Yaw / 180.0 * math.pi) + \
                        self.Dyaw * (0 - self.dot_Yaw / 180.0 * math.pi)

                K_y = 0.01
                k_r2y = 0.009
                Y_des_front = - K_y * T_yaw / 2
                Y_des_back = + K_y * T_yaw / 2
                Y_des_left = +k_r2y * T_roll / 2
                Y_des_right = -k_r2y * T_roll / 2

                K_x = 0.002
                k_p2x = 0.001
                X_des_left = + K_x * T_yaw / 2
                X_des_right = - K_x * T_yaw / 2
                X_des_front = 0.07 + k_p2x * T_pitch / 2
                X_des_back = -0.07 - k_p2x * T_pitch / 2

                # print("run3")kpx * (0.0 - x) + kdx * (0 - dx)
                Fx_left_F = self.Kxs * (X_des_left + X_des_front - self.pos[0][0]) \
                            + self.Dxs * (0.0 - self.dot_pos[0][0])
                Fx_right_F = self.Kxs * (X_des_right + X_des_front - self.pos[1][0]) \
                             + self.Dxs * (0.0 - self.dot_pos[1][0])
                Fx_left_B = self.Kxs * (X_des_left + X_des_back - self.pos[3][0]) \
                            + self.Dxs * (0.0 - self.dot_pos[3][0])
                Fx_right_B = self.Kxs * (X_des_right + X_des_back - self.pos[2][0]) \
                             + self.Dxs * (0.0 - self.dot_pos[2][0])

                Fy_left_F = self.Kys * (Y_des_front + Y_des_left - self.pos[0][1]) \
                            + self.Dys * (0.0 - self.dot_pos[0][1])
                Fy_right_F = self.Kys * (Y_des_front + Y_des_right - self.pos[1][1]) \
                             + self.Dys * (0.0 - self.dot_pos[1][1])
                Fy_left_B = self.Kys * (Y_des_back + Y_des_left - self.pos[3][1]) \
                            + self.Dys * (0.0 - self.dot_pos[3][1])
                Fy_right_B = self.Kys * (Y_des_back + Y_des_right - self.pos[2][1]) \
                             + self.Dys * (0.0 - self.dot_pos[2][1])

                T_left_F = np.matmul(self.create_transJ(self.theta[0]), np.array([[Fx_left_F], [Fy_left_F], [Fz_left_F]]))
                T_left_B = np.matmul(self.create_transJ(self.theta[3]), np.array([[Fx_left_B], [Fy_left_B], [Fz_left_B]]))
                T_right_F = np.matmul(self.create_transJ(self.theta[1]), np.array([[Fx_right_F], [Fy_right_F], [Fz_right_F]]))
                T_right_B = np.matmul(self.create_transJ(self.theta[2]), np.array([[Fx_right_B], [Fy_right_B], [Fz_right_B]]))

                set_motor_torque(0, 0, float(T_left_F[0, 0]))
                set_motor_torque(0, 1, float(T_left_F[1, 0]))
                set_motor_torque(0, 2, float(T_left_F[2, 0]))

                set_motor_torque(1, 0, float(T_right_F[0, 0]))
                set_motor_torque(1, 1, float(T_right_F[1, 0]))
                set_motor_torque(1, 2, float(T_right_F[2, 0]))

                set_motor_torque(2, 0, float(T_right_B[0, 0]))
                set_motor_torque(2, 1, float(T_right_B[1, 0]))
                set_motor_torque(2, 2, float(T_right_B[2, 0]))

                set_motor_torque(3, 0, float(T_left_B[0, 0]))
                set_motor_torque(3, 1, float(T_left_B[1, 0]))
                set_motor_torque(3, 2, float(T_left_B[2, 0]))

                pass
            # 四足悬空状态控制------------------------------------------------------------------------------------------四足悬空状态控制
            elif sw[4] == 200:  # 全悬空
                # print("run3")
                Fx_left_F = self.Kxk * (0.0 - self.pos[0][0]) + self.Dxk * (0.0 - self.dot_pos[0][0])
                Fx_right_F = self.Kxk * (0.0 - self.pos[1][0]) + self.Dxk * (0.0 - self.dot_pos[1][0])
                Fx_left_B = self.Kxk * (0.0 - self.pos[3][0]) + self.Dxk * (0.0 - self.dot_pos[3][0])
                Fx_right_B = self.Kxk * (0.0 - self.pos[2][0]) + self.Dxk * (0.0 - self.dot_pos[2][0])

                Fy_left_F = self.Kxk * (0.0 - self.pos[0][1]) + self.Dxk * (0.0 - self.dot_pos[0][1])
                Fy_right_F = self.Kxk * (0.0 - self.pos[1][1]) + self.Dxk * (0.0 - self.dot_pos[1][1])
                Fy_left_B = self.Kxk * (0.0 - self.pos[3][1]) + self.Dxk * (0.0 - self.dot_pos[3][1])
                Fy_right_B = self.Kxk * (0.0 - self.pos[2][1]) + self.Dxk * (0.0 - self.dot_pos[2][1])

                Fz_left_F = self.Kzk * (self.H_des - self.pos[0][2]) + self.Dzk * (0 - self.dot_pos[0][2])
                Fz_right_F = self.Kzk * (self.H_des - self.pos[1][2]) + self.Dzk * (0 - self.dot_pos[1][2])
                Fz_left_B = self.Kzk * (self.H_des - self.pos[3][2]) + self.Dzk * (0 - self.dot_pos[3][2])
                Fz_right_B = self.Kzk * (self.H_des - self.pos[2][2]) + self.Dzk * (0 - self.dot_pos[2][2])

                T_left_F = np.matmul(self.create_transJ(self.theta[0]), np.array([[Fx_left_F], [Fy_left_F], [Fz_left_F]]))
                T_left_B = np.matmul(self.create_transJ(self.theta[3]), np.array([[Fx_left_B], [Fy_left_B], [Fz_left_B]]))
                T_right_F = np.matmul(self.create_transJ(self.theta[1]), np.array([[Fx_right_F], [Fy_right_F], [Fz_right_F]]))
                T_right_B = np.matmul(self.create_transJ(self.theta[2]), np.array([[Fx_right_B], [Fy_right_B], [Fz_right_B]]))

                set_motor_torque(0, 0, float(T_left_F[0, 0]))
                set_motor_torque(0, 1, float(T_left_F[1, 0]))
                set_motor_torque(0, 2, float(T_left_F[2, 0]))

                set_motor_torque(1, 0, float(T_right_F[0, 0]))
                set_motor_torque(1, 1, float(T_right_F[1, 0]))
                set_motor_torque(1, 2, float(T_right_F[2, 0]))

                set_motor_torque(2, 0, float(T_right_B[0, 0]))
                set_motor_torque(2, 1, float(T_right_B[1, 0]))
                set_motor_torque(2, 2, float(T_right_B[2, 0]))

                set_motor_torque(3, 0, float(T_left_B[0, 0]))
                set_motor_torque(3, 1, float(T_left_B[1, 0]))
                set_motor_torque(3, 2, float(T_left_B[2, 0]))


if __name__ == '__main__':
    q = Quadruped_robot_mix()
    # webot_device_init()
    q.run()
