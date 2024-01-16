#!/usr/bin/env python3
#coding=utf-8

import serial
import time
import rospy
import roslib
import actionlib
import numpy as np
from std_msgs.msg import *
from control_msgs.msg import *
from geometry_msgs.msg import *
from sensor_msgs.msg import JointState
from xm_msgs.srv import *


class ARM_SERIAL:

    def __init__(self):
        rospy.init_node('arm_serial', anonymous=True)
        # 端口，GNU / Linux上的/ dev / ttyUSB0 等 或 Windows上的 COM3 等
        portx = "/dev/ttyUSB0"
        # 波特率，标准值之50,75,110,134,150,200,300,600,1200,1800,2400,4800,9600,19200,38400,57600,115200
        bps = 115200
        # 超时设置,None：永远等待操作，0为立即返回请求结果，其他值为等待超时时间(单位为秒）
        timex = 50
        # 打开串口，并得到串口对象
        self.ser = serial.Serial(portx, bps, timeout=timex)
        #检测串口是否打开（这里应该用异常处理！！）
        if (self.ser.isOpen()):
            rospy.loginfo("\033[1;32mSerial is open!\033[0m")
        else:
            rospy.logerr("\033[1;32mSerial is closed!\033[0m")
            rospy.signal_shutdown("check serial!")
#与串口建立通信(shaking_Hands)
        shaking_Hands = [
            '\xff', '\xff', '\x01', '\x11', '\x00', '\x00', '\x00', '\x12'
        ]
        self.ser.write(shaking_Hands)
        rospy.loginfo("\033[1;32mShaking Hands!\033[0m")

        #在这里设置了关节更新的速度
        PI = 3.14
        self.plat_velocity = 0.04  #升降移动的速度             m/s
        self.arm_joint_velocity = [
            PI / 5, PI / 6.43, PI / 3.4, PI / 4.93, PI / 5.57, PI / 3.5
        ]  #不同的关节的速度           radis/s
        self.frequency = float(100.0)  #设置刷新频率               hz

        #初始化机器人位姿
        self.arm_pos = [0, -1.4, 3.14, 0, 0, -1.57]
        self.arm_pos_rightOrder = [0, 0, 0, 0, 0, 0]
        self.gripper_pos = ['\x00']
        self.gripper_pos_1 = [1]
        self.plat_pos = [-0.08]
        self.camera_pos = [0, 0]

        self.joint_states = JointState()
        self.joint_names = [
            'lifting_joint', 'arm_joint_0', 'arm_joint_1', 'arm_joint_2',
            'arm_joint_3', 'arm_joint_4', 'arm_joint_5', 'active_finger_joint',
            'camera_joint_0', 'camera_joint_1'
        ]
        self.joint_states.name = self.joint_names

        #添加接口
        rospy.Subscriber('/set_arm_joints_pos', Float32MultiArray,
                         self.set_arm_pos)
        rospy.Service('/gripper_command', xm_Gripper, self.set_gripper_pos)
        rospy.Service('/plat_command', xm_Plat, self.set_plat_pos)
        rospy.Service('/camera_command', xm_camera, self.set_camera_pos)

        pub = rospy.Publisher('/joint_states', JointState, queue_size=10)

        rospy.loginfo("\033[1;32mSuccessful!\033[0m")

        #定义一个将float转成bytes()的函数
        def floatToBytes(f):
            bs = struct.pack("f", f)
            list1 = list()
            list1 = [bs[0], bs[1], bs[2], bs[3]]
            return list1

#设置发送频率

        rate = rospy.Rate(100)
        while not rospy.is_shutdown():

            #设置时间戳
            header = Header()
            header.stamp = rospy.Time.now()
            self.joint_states.header = header

            #将关节信息整理成串口数据包
            #打包plat
            plat_pos_serial = [
                '\xff', '\xff', '\x01', '\x11', '\x00', '\x05', '\x05'
            ]
            plat_pos_serial += floatToBytes(self.plat_pos[0])
            plat_pos_check = ['\x00']
            for num in plat_pos_serial:
                plat_pos_check[0] = chr(
                    (ord(plat_pos_check[0]) + ord(num)) % 255)
            plat_pos_serial += plat_pos_check
            self.ser.write(plat_pos_serial)
            #打包arm
            arm_pos_serial = [
                '\xff', '\xff', '\x01', '\x11', '\x00', '\x19', '\x03'
            ]
            #arm_pos_rightOrder = [0,0,0,0,0,0]
            self.arm_pos_rightOrder[0] = self.arm_pos[4]
            self.arm_pos_rightOrder[1] = self.arm_pos[3]
            self.arm_pos_rightOrder[2] = self.arm_pos[1]
            self.arm_pos_rightOrder[3] = self.arm_pos[2]
            self.arm_pos_rightOrder[4] = self.arm_pos[0]  #如果有偏差，在这里添加修正值,负值向左
            self.arm_pos_rightOrder[5] = self.arm_pos[5]
            for pos in self.arm_pos_rightOrder:
                arm_pos_serial += floatToBytes(pos)
            arm_pos_check = ['\x00']
            for num in arm_pos_serial:
                arm_pos_check[0] = chr(
                    (ord(arm_pos_check[0]) + ord(num)) % 255)
            arm_pos_serial += arm_pos_check
            self.ser.write(arm_pos_serial)
            #打包gripper
            gripper_pos_serial = [
                '\xff', '\xff', '\x01', '\x11', '\x00', '\x03', '\x01'
            ]
            gripper_pos_serial += self.gripper_pos[0]  #柔性爪
            gripper_pos_serial += ['\x00']  #dora
            gripper_pos_check = ['\x00']
            for num in gripper_pos_serial:
                gripper_pos_check[0] = chr(
                    (ord(gripper_pos_check[0]) + ord(num)) % 255)
            gripper_pos_serial += gripper_pos_check
            #gripper_pos_serial += ['\x17']
            self.ser.write(gripper_pos_serial)
            #打包camera(camera有没有用啊？？)
            #camera_pos_serial = ['\xff','\xff','\x01','\x11','\x00','\x09','\x0f']
            #camera_pos_serial += floatToBytes(self.camera_pos[0])
            #camera_pos_serial += floatToBytes(self.camera_pos[1])
            #camera_pos_check = ['\x00']
            #for num in camera_pos_serial:
            #camera_pos_check[0] = hex(ord(camera_pos_check[0]) + ord(num))
            #camera_pos_serial += camera_pos_check

            #从串口读出实际的关节角度，并发布到/joint_states话题中（这里并没有从串口读）
            self.joint_states.position = list(self.plat_pos) + list(
                self.arm_pos) + list(self.gripper_pos_1) + list(
                    self.camera_pos)
            #rospy.loginfo(self.joint_states.position)
            pub.publish(self.joint_states)

            rate.sleep()

    def set_arm_pos(self, data):
        '''
        更新机械臂关节position
        '''
        arm_direction = [0, 0, 0, 0, 0, 0]
        count = [0, 0, 0, 0, 0, 0]
        tmp_arm_pos = [0, 0, 0, 0, 0, 0]

        #判断各个关节移动的方向并计算其移动步数
        for i in range(len(self.arm_pos)):
            if self.arm_pos[i] < data.data[i]:
                arm_direction[i] = 1
            else:
                arm_direction[i] = -1

            count[i] = int(
                abs(self.arm_pos[i] - data.data[i]) /
                (self.arm_joint_velocity[i] / self.frequency))
            tmp_arm_pos[i] = self.arm_pos[i]

        max_count = max(count)
        for i in range(max_count):

            #对每个未到达位置的关节匀速的增加或减小position
            for j in range(len(self.arm_pos)):

                #arm_joint_[j]关节为到达指定位置，需要继续移动
                if count[j] > 0:
                    tmp_arm_pos[j] = tmp_arm_pos[j] + arm_direction[j] * (
                        self.arm_joint_velocity[j] / self.frequency)
                    count[j] -= 1

            #更新
            self.arm_pos = list(tmp_arm_pos)
            #以一定频率运动
            rospy.sleep(1.0 / self.frequency)

        self.arm_pos = data.data

    def set_gripper_pos(self, req):
        if req.command == 0:
            print("close")
            self.gripper_pos = ['\x00']
            self.gripper_pos_1 = [0]
        elif req.command == 1:
            print("open")
            self.gripper_pos = ['\x01']
            self.gripper_pos_1 = [1]
        else:
            print("release")
            self.gripper_pos = ['\x02']
            self.gripper_pos_1 = [2]
        #TODO 检测爪子是否完成操作

        res = xm_GripperResponse()
        res.result = True
        return res

    def set_plat_pos(self, req):

        #判断增大还是减小
        if self.plat_pos[0] < req.height:
            direction = 1
        else:
            direction = -1

        #进行的步数
        count = int(
            abs(self.plat_pos[0] - req.height) /
            (self.plat_velocity / self.frequency))

        for i in range(count):
            self.plat_pos = [
                self.plat_pos[0] + direction *
                (self.plat_velocity / self.frequency)
            ]
            #rospy.logwarn(self.plat_pos[0])
            rospy.sleep(1.0 / self.frequency)

        self.plat_pos = [req.height]

        #TODO 检测升降台是否到达

        res = xm_PlatResponse()
        res.result = True
        return res

    def set_camera_pos(self, req):
        yaw = req.yaw
        pitch = req.pitch
        self.camera_pos = [yaw, pitch]

        res = xm_cameraResponse()
        res.result = True
        return res

if __name__ == '__main__':

    arm_serial = ARM_SERIAL()
