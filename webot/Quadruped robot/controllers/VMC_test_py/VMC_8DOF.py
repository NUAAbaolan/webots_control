"""VMC_test_py controller."""
##8自由度
from controller import Robot
import math
import numpy as np
import copy

# create the Robot instance.
robot = Robot()
timestep = int(1)

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


# ROll Pich Yaw
def get_IMU_Angle():
    data = IMU.getRollPitchYaw()
    # return ROll Pich Yaw
    return [data[1] * 180.0 / math.pi, data[0] * 180.0 / math.pi, data[2] * 180.0 / math.pi]


def webot_device_init():
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
    if torque > 1800:
        torque = 1800
    if torque < -1800:
        torque = -1800
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


class Quadruped_robot_8DOF():
    def __init__(self):
        self.L = 0
        self.R = 1

        self.zh = -0.4  # 抬腿到高度 0.4
        self.H_des = - 0.55  # 0.5
        self.Tf = 0.25  # 飞行时间
        self.Ts = 0.25  # 支撑时间
        # 机器人机身参数
        self.L = 0.6  # 前后髋关节
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

        # 行进时系数
        self.Kpitch_r = 1700  # 1000
        self.Dpitch_r = 300  # 200

        self.Kroll_r = 1000  # 1000
        self.Droll_r = 300  # 200

        self.Kyaw = 170  # 200

        # 悬空足系数
        self.Kxk = 800  # 800
        self.Dxk = 200  # 200

        self.Kzk = 4000  # 1000
        self.Dzk = 200  # 300

        # 支撑足系数
        self.Kx = 900  # 1000
        self.Dx = 100  # 100

        self.Kz = 6000  # 4000
        self.Dz = 500  # 300
        # self.Dyaw = 10
        # 摆动时速度增益
        self.Kvx = 0.3
        # 期望值
        self.Vx_des = -0.01
        self.W_des = 0

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
        self.mass_center = 0.4  # 位置系数

    def forwardkinematics(self, theta):
        a0 = self.a0
        a1 = self.a1  # 腿长
        a2 = self.a2  # 腿长
        # 横滚髋关节和俯仰髋关节偏差为0
        # theta= L0 L1 L2
        # 足底位置
        s1 = math.sin(theta[1] / 180.0 * math.pi)
        c1 = math.cos(theta[1] / 180.0 * math.pi)
        s0 = math.sin(theta[0] / 180.0 * math.pi)
        c0 = math.cos(theta[0] / 180.0 * math.pi)

        s12 = math.sin(theta[1] / 180.0 * math.pi + theta[2] / 180.0 * math.pi)
        c12 = math.cos(theta[1] / 180.0 * math.pi + theta[2] / 180.0 * math.pi)

        x = -a1 * s1 - a2 * s12 #+ self.L / 2
        y = a0 * s0 + a1 * s0 * c1 + a2 * s0 * c12 #+ self.W / 2
        z = - a0 * c0 - a1 * c0 * c1 - a2 * c0 * c12
        return [x, y, z]

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
        transJ[2, 2] =  a2 * c0 * s12
        return transJ.T

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
                if self.fly_time > 0.05:
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
            self.Pitch_des += 0.1 / 180.0 * math.pi
            print(self.Pitch_des)
        elif key == KeyBoard.DOWN:
            self.Pitch_des -= 0.1 / 180.0 * math.pi
            print(self.Pitch_des)
        elif key == KeyBoard.LEFT:
            self.Roll_des -= 0.1 / 180.0 * math.pi
        elif key == KeyBoard.RIGHT:
            self.Roll_des += 0.1 / 180.0 * math.pi
        elif key == ord('P'):
            self.stop = 1
            self.Pitch_des = 0
            self.Roll_des = 0
        elif key == ord('W'):
            self.stop = 0
            self.Vx_des = -0.3
        elif key == ord('S'):
            self.stop = 0
            self.Vx_des = 0.3
        elif key == ord('A'):
            self.Vx_des = 0
            self.W_des = 20 / 180.0 * math.pi
            self.stop = 0
        elif key == ord('D'):
            self.Vx_des = 0
            self.W_des = -20 / 180.0 * math.pi
            self.stop = 0
        elif key == ord('Q'):
            self.Vx_des = -0.2
            self.W_des = 20 / 180.0 * math.pi
            self.stop = 0
        elif key == ord('E'):
            self.Vx_des = -0.2
            self.W_des = -20 / 180.0 * math.pi
            self.stop = 0
        elif key == ord('T'):
            self.stop = 1
            self.Vx_des = -0.01
            self.W_des = 0
        elif key == ord('U'):
            self.H_des -= 0.001 * timestep
            self.zh = self.H_des + 0.12
            print("目标 ", self.H_des, "当前", self.pos[0][2])
        elif key == ord('L'):
            self.H_des += 0.001 * timestep
            self.zh = self.H_des + 0.12
            print(self.H_des)
            print("目标 ", self.H_des, "当前", self.pos[0][2])
        else:
            # self.stop = 0
            self.Vx_des = -0.05
            self.W_des = 0

    # 轨迹
    def curve(self, xt, init_pos, init_dotpos):
        ft = [0, 0, 0]  # x,z
        dft = [0, 0, 0]  # x,z
        xT = xt
        t = self.time
        x0 = init_pos[0]
        z0 = init_pos[2]
        dx0 = init_dotpos[0]
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
        init_pos = [[0, 0, 0], [0, 0, 0]]  # 左右
        init_dotpos = [[0, 0, 0], [0, 0, 0]]  # 左右
        path_pos = [[0, 0, 0], [0, 0, 0]]
        dot_path_pos = [[0, 0, 0], [0, 0, 0]]
        while robot.step(timestep) != -1:
            # print(ACC.getValues())
            for leg in range(4):
                # 跨关节锁定
                robot_motor[leg][0].setPosition(0)
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
                F_pitch = self.Kpitch_r * (self.Pitch_des - self.Pitch / 180.0 * math.pi) + \
                          self.Dpitch_r / 2 * (0 - self.dot_Pitch / 180.0 * math.pi)
                F_roll = self.Kroll_r * (self.Roll_des - self.Roll / 180.0 * math.pi) + \
                         self.Droll_r * (0 - self.dot_Roll / 180.0 * math.pi)

                # 高低叠加控制姿态
                K_pi_z = 0.01
                H_des_front = self.H_des / 2 + K_pi_z * F_pitch
                H_des_back = self.H_des / 2 - K_pi_z * F_pitch

                K_roll_z = 0.01
                H_des_left = self.H_des / 2 - K_roll_z * F_roll
                H_des_right = self.H_des / 2 + K_roll_z * F_roll
                #
                Hz_left = self.H_des
                Hz_right = self.H_des

                # 左前右后对应
                if st.index(st[2]) == 0:
                    Hz_left = H_des_left + H_des_front
                    Hz_right = H_des_right + H_des_back

                    Fz_left = self.Kz * (Hz_left - self.pos[st[0]][2]) + \
                              self.Dz * (0 - self.dot_pos[st[0]][2]) \
                              - self.M * 9.81 * self.mass_center * math.cos(self.Pitch / 180.0 * math.pi)
                    Fz_right = self.Kz * (Hz_right - self.pos[st[1]][2]) + \
                               self.Dz * (0 - self.dot_pos[st[1]][2]) \
                               - self.M * 9.81 * (1 - self.mass_center) * math.cos(-self.Pitch / 180.0 * math.pi)
                # 左后右前对应
                elif st.index(st[2]) == 1:
                    Hz_left = H_des_left + H_des_back
                    Hz_right = H_des_right + H_des_front

                    # 计算Z轴虚拟力
                    Fz_left = self.Kz * (Hz_left - self.pos[st[0]][2]) + \
                              self.Dz * (0 - self.dot_pos[st[0]][2]) \
                              - self.M * 9.81 * (1 - self.mass_center) * math.cos(self.Pitch / 180.0 * math.pi)
                    Fz_right = self.Kz * (Hz_right - self.pos[st[1]][2]) + \
                               self.Dz * (0 - self.dot_pos[st[1]][2]) \
                               - self.M * 9.81 * self.mass_center * math.cos(self.Pitch / 180.0 * math.pi)
                # print(self.pos)
                if not self.W_des == 0:
                    F_yaw = self.Kyaw * (self.W_des - (
                            self.dot_Yaw * 0.1 + self.dot_Yaw_pre * 0.9) / 180.0 * math.pi)
                else:
                    F_yaw = 0
                Kyaw_x = 0.01
                V_des_left = self.Vx_des + Kyaw_x * F_yaw
                V_des_right = self.Vx_des - Kyaw_x * F_yaw

                # est_dot_pos=(np.array(self.dot_pos)*0.1+np.array(self.pre_dot_pos)*0.9).tolist()

                # print(self.dot_pos)
                # 计算X轴虚拟力
                Fx_left = self.Kx * (V_des_left - self.dot_pos[st[0]][0]) \
                          + self.M * 9.8 * math.sin(self.Pitch / 180.0 * math.pi) * self.mass_center
                Fx_right = self.Kx * (V_des_right - self.dot_pos[st[1]][0]) \
                           + self.M * 9.8 * math.sin(self.Pitch / 180.0 * math.pi) * (1 - self.mass_center)

                T_left = self.create_transJ(self.theta[st[0]]) @ np.array([[Fx_left], [Fz_left]])
                T_right = self.create_transJ(self.theta[st[1]]) @ np.array([[Fx_right], [Fz_right]])

                set_motor_torque(st[0], 1, T_left[0, 0])
                set_motor_torque(st[0], 2, T_left[1, 0])

                set_motor_torque(st[1], 1, T_right[0, 0])
                set_motor_torque(st[1], 2, T_right[1, 0])

                # 摆动足--------------------------------------------------------------------------------------------------摆动足

                # 获得初始速度和位置

                if self.time == 0.001 * timestep:  # 刚刚切换过对角腿，记录此时的摆动足即可
                    init_pos[0] = self.pos[sw[0]]
                    init_pos[1] = self.pos[sw[1]]

                    init_dotpos[0] = self.dot_pos[sw[0]]
                    init_dotpos[1] = self.dot_pos[sw[1]]
                # print("初始点 初始速度", init_pos, init_dotpos)
                est_dot_pos = [[0, 0, 0], [0, 0, 0], [0, 0, 0], [0, 0, 0]]
                if not self.time == 0.001 * timestep:
                    est_dot_pos = (np.array(self.dot_pos) * 0.5 + np.array(self.pre_dot_pos) * 0.5).tolist()
                # print("速度估计", est_dot_pos)
                xT = [0.2, 0.2]  # 落足点 左右
                xT[0] = est_dot_pos[st[0]][0] * self.Ts / 2.0 - self.Kvx * (est_dot_pos[st[0]][0] - V_des_left)
                xT[1] = est_dot_pos[st[1]][0] * self.Ts / 2.0 - self.Kvx * (est_dot_pos[st[1]][0] - V_des_right)
                # xT[0] = est_dot_pos[st[0]][0] * self.Ts / 2.0 - self.Kvx * (est_dot_pos[st[0]][0] - self.Vx_des)V_des_left
                # xT[1] = est_dot_pos[st[1]][0] * self.Ts / 2.0 - self.Kvx * (est_dot_pos[st[1]][0] - self.Vx_des)
                # print("落足点", xT)

                if self.is_touching[sw[0]] == 1 and self.time > 0.5 * self.Tf:
                    path_pos[0], dot_path_pos[0] = self.pos[sw[0]], [0, 0, 0]
                    # pass
                else:
                    path_pos[0], dot_path_pos[0] = self.curve(xT[0], init_pos[0], init_dotpos[0])
                if self.is_touching[sw[1]] == 1 and self.time > 0.5 * self.Tf:
                    path_pos[1], dot_path_pos[1] = self.pos[sw[1]], [0, 0, 0]
                else:
                    path_pos[1], dot_path_pos[1] = self.curve(xT[1], init_pos[1], init_dotpos[1])

                # 左摆动足 索引值sw[0]
                Fx_left_sw = self.Kxk * (path_pos[0][0] - self.pos[sw[0]][0]) + \
                             self.Dxk * (dot_path_pos[0][0] - self.dot_pos[sw[0]][0]) \
                    # + self.M * 9.8 * math.sin(self.Pitch / 180.0 * math.pi) * self.mass_center
                # 右摆动足 索引值sw[1]
                Fx_right_sw = self.Kxk * (path_pos[1][0] - self.pos[sw[1]][0]) + \
                              self.Dxk * (dot_path_pos[1][0] - self.dot_pos[sw[1]][0]) \
                    # + self.M * 9.8 * math.sin(self.Pitch / 180.0 * math.pi) * (1 - self.mass_center)

                if self.time <= 0.55 * self.Tf:
                    KKZ = self.Kzk
                    KKD = self.Dzk
                else:
                    KKZ = 600
                    KKD = 100
                # if self.is_touching[sw[0]]==1:
                #     KKZ = self.Kzk * 0.1
                Fz_left_sw = KKZ * (path_pos[0][2] - self.pos[sw[0]][2]) + \
                             KKD * (dot_path_pos[0][2] - self.dot_pos[sw[0]][2])
                Fz_right_sw = KKZ * (path_pos[1][2] - self.pos[sw[1]][2]) + \
                              KKD * (dot_path_pos[1][2] - self.dot_pos[sw[1]][2])

                T_left_sw = self.create_transJ(self.theta[sw[0]]) @ np.array([[Fx_left_sw], [Fz_left_sw]])
                T_right_sw = self.create_transJ(self.theta[sw[1]]) @ np.array([[Fx_right_sw], [Fz_right_sw]])

                set_motor_torque(sw[0], 1, T_left_sw[0, 0])
                set_motor_torque(sw[0], 2, T_left_sw[1, 0])

                set_motor_torque(sw[1], 1, T_right_sw[0, 0])
                set_motor_torque(sw[1], 2, T_right_sw[1, 0])

            # 四足站立状态控制------------------------------------------------------------------------------------------四足站立状态控制
            elif st[4] == 100:
                F_pitch = self.Kpitch * (self.Pitch_des - self.Pitch / 180.0 * math.pi) + \
                          self.Dpitch / 2 * (0 - self.dot_Pitch / 180.0 * math.pi)
                F_roll = self.Kroll * (self.Roll_des - self.Roll / 180.0 * math.pi) + \
                         self.Droll * (0 - self.dot_Roll / 180.0 * math.pi)

                K_pi_z = 0.02
                H_des_front = + K_pi_z * F_pitch / 2
                H_des_back = - K_pi_z * F_pitch / 2

                K_roll_z = 0.01
                H_des_left = - K_roll_z * F_roll / 2
                H_des_right = + K_roll_z * F_roll / 2

                Fz_left_F = self.Kz * (self.H_des + H_des_left + H_des_front - self.pos[0][2]) + self.Dz * (
                        0 - self.dot_pos[0][2])
                Fz_right_F = self.Kz * (self.H_des + H_des_right + H_des_front - self.pos[1][2]) + self.Dz * (
                        0 - self.dot_pos[1][2])
                Fz_left_B = self.Kz * (self.H_des + H_des_left + H_des_back - self.pos[3][2]) + self.Dz * (
                        0 - self.dot_pos[3][2])
                Fz_right_B = self.Kz * (self.H_des + H_des_right + H_des_back - self.pos[2][2]) + self.Dz * (
                        0 - self.dot_pos[2][2])

                Fz_left_F = Fz_left_F - self.M * 10 / 4
                Fz_right_F = Fz_right_F - self.M * 10 / 4
                Fz_left_B = Fz_left_B - self.M * 10 / 4
                Fz_right_B = Fz_right_B - self.M * 10 / 4  # / 2

                # print("run3")kpx * (0.0 - x) + kdx * (0 - dx)
                Fx_left_f = self.Kxk * (0 - self.pos[0][0]) + self.Dxk * (0.0 - self.dot_pos[0][0])
                Fx_right_f = self.Kxk * (0 - self.pos[1][0]) + self.Dxk * (0.0 - self.dot_pos[1][0])
                Fx_left_b = self.Kxk * (0 - self.pos[3][0]) + self.Dxk * (0.0 - self.dot_pos[3][0])
                Fx_right_b = self.Kxk * (0 - self.pos[2][0]) + self.Dxk * (0.0 - self.dot_pos[2][0])

                T_left_F = self.create_transJ(self.theta[0]) @ np.array([[Fx_left_f], [Fz_left_F]])
                T_left_B = self.create_transJ(self.theta[3]) @ np.array([[Fx_left_b], [Fz_left_B]])
                T_right_F = self.create_transJ(self.theta[1]) @ np.array([[Fx_right_f], [Fz_right_F]])
                T_right_B = self.create_transJ(self.theta[2]) @ np.array([[Fx_right_b], [Fz_right_B]])

                set_motor_torque(0, 1, float(T_left_F[0, 0]))
                set_motor_torque(0, 2, float(T_left_F[1, 0]))

                set_motor_torque(1, 1, float(T_right_F[0, 0]))
                set_motor_torque(1, 2, float(T_right_F[1, 0]))

                set_motor_torque(2, 1, float(T_right_B[0, 0]))
                set_motor_torque(2, 2, float(T_right_B[1, 0]))

                set_motor_torque(3, 1, float(T_left_B[0, 0]))
                set_motor_torque(3, 2, float(T_left_B[1, 0]))

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

                T_left_F = self.create_transJ(self.theta[0]) @ np.array([[Fx_left_F],[Fy_left_F], [Fz_left_F]])
                T_left_B = self.create_transJ(self.theta[3]) @ np.array([[Fx_left_B],[Fy_left_B], [Fz_left_B]])
                T_right_F = self.create_transJ(self.theta[1]) @ np.array([[Fx_right_F],[Fy_right_F], [Fz_right_F]])
                T_right_B = self.create_transJ(self.theta[2]) @ np.array([[Fx_right_B], [Fy_right_B],[Fz_right_B]])

                set_motor_torque(0, 0, float(T_left_F[0, 0]))
                set_motor_torque(0, 1, float(T_left_F[1, 0]))
                set_motor_torque(0, 2, float(T_left_F[2, 0]))

                set_motor_torque(0, 0, float(T_right_F[0, 0]))
                set_motor_torque(1, 1, float(T_right_F[1, 0]))
                set_motor_torque(1, 2, float(T_right_F[2, 0]))

                set_motor_torque(0, 0, float(T_right_B[0, 0]))
                set_motor_torque(2, 1, float(T_right_B[1, 0]))
                set_motor_torque(2, 2, float(T_right_B[2, 0]))

                set_motor_torque(0, 0, float(T_left_B[0, 0]))
                set_motor_torque(3, 1, float(T_left_B[1, 0]))
                set_motor_torque(3, 2, float(T_left_B[2, 0]))


if __name__ == '__main__':
    q = Quadruped_robot_8DOF()
    webot_device_init()
    q.run()
