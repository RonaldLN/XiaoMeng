#!/usr/bin/env python
#coding=utf-8

import serial
import time 
import rospy
import roslib
import actionlib
import struct
import numpy as np
from std_msgs.msg import *
from control_msgs.msg import *
from geometry_msgs.msg import *
from sensor_msgs.msg import JointState
from xm_msgs.srv import *


class ARM_SERIAL:
    def __init__(self):
        rospy.init_node('arm_serial', anonymous = True)
        # 端口，GNU / Linux上的/ dev / ttyUSB0 等 或 Windows上的 COM3 等
        portx = "/dev/ttyUSB0"
        # 波特率，标准值之50,75,110,134,150,200,300,600,1200,1800,2400,4800,9600,19200,38400,57600,115200
        bps = 9600
        # 超时设置,None：永远等待操作，0为立即返回请求结果，其他值为等待超时时间(单位为秒）
        timex = 50
	
	serialpack = ['\xff','\xff','\x01','\x11','\x00','\x19','\x03']
	def floatToBytes(f):
	    bs = struct.pack("f",f)
	    list1 = list()
	    list1 = [bs[0],bs[1],bs[2],bs[3]]
	    return list1
	list2 = floatToBytes(1)
	serialpack += list2
	for i in list2 :
	    print(hex(ord(i)))
	check = ['\x00']

	#x = hex(ord(list2[0]))+ord(list2[1])+ord(list2[2])+ord(list2[3]))
	#print(x)
        # 打开串口，并得到串口对象
        #self.ser = serial.Serial(portx, bps, timeout=timex)
	#检测串口是否打开（这里应该用异常处理！！）
	#if(self.ser.isOpen()):
	#    rospy.loginfo("\033[1;32mSerial is open!\033[0m")
	#else:
	#    rospy.logerr("\033[1;32mSerial is closed!\033[0m")
	#    rospy.signal_shutdown("check serial!")
	 
     

    #def floatToBytesAndPrint(float):
	#bs = struct.pack("f",float)
	#rospy.loginfo(str(bs[0]))
if __name__ == '__main__':
    arm_serial = ARM_SERIAL()
