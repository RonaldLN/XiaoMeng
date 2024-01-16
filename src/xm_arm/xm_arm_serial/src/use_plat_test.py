#!/usr/bin/env python
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

        #初始化机器人位姿
        self.arm_pos = [0, 0, 0, 0, 0, 0]
        self.arm_pos_rightOrder = [0, 0, 0, 0, 0, 0]
        self.gripper_pos = [0]
        self.plat_pos = [-0.1]
        self.camera_pos = [0, 0]

        while not rospy.is_shutdown():

            plat_pos_serial = [
                '\xff', '\xff', '\x01', '\x11', '\x00', '\x05', '\x08'
            ]
            plat_pos_serial += floatToBytes(self.plat_pos[0])
            plat_pos_check = ['\x00']
            for num in plat_pos_serial:
                plat_pos_check[0] = chr(
                    (ord(plat_pos_check[0]) + ord(num)) % 255)
            plat_pos_serial += plat_pos_check
            self.ser.write(plat_pos_serial)

        #定义一个将float转成bytes()的函数
        def floatToBytes(f):
            bs = struct.pack("f", f)
            list1 = list()
            list1 = [bs[0], bs[1], bs[2], bs[3]]
            return list1

        #将关节信息整理成串口数据包


#打包plat

if __name__ == '__main__':

    arm_serial = ARM_SERIAL()
