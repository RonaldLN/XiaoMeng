#!/usr/bin/env python
#coding=utf-8
'''
Author: jacy
Date: 2020-09-03 17:57:38
LastEditTime: 2020-09-04 10:09:15
LastEditors: Please set LastEditors
Description: In User Settings Edit
FilePath: /undefined/home/jacy/gazebo_test_ws/src/xm_arm/xm_arm_controller/xm_arm_controller/src/arm_controller.py
'''


import rospy
import roslib
import time
import actionlib
from std_msgs.msg import *
from control_msgs.msg import *
from geometry_msgs.msg import *
from sensor_msgs.msg import JointState

class Server:
    def __init__(self):

        rospy.init_node('arm_controller') 
        rospy.Subscriber('joint_states', JointState, self.stateCb)
        self.pub = rospy.Publisher('/set_arm_joints_pos', Float32MultiArray, queue_size=10)
        self.server = actionlib.SimpleActionServer('arm_controller/follow_joint_trajectory',FollowJointTrajectoryAction,self.execute,False)
        self.server.start()
        rospy.loginfo("\033[1;32mArm Controller Start!\033[0m")
        rospy.spin()
            
    def execute(self,goal):
        rospy.loginfo("\033[1;33marm controller get the goal\033[0m")
        print(goal)


        #将goal中的数据传输到waypoints中
        points_list = goal.trajectory.points
        print("points_list:",points_list)


        #设置更新发送数据的阈值
        self.tolerance_error = 0.12
        
        #用于记录执行进度
        self.percent = 0
        self.length = len(points_list)
        print('There are '+str(self.length)+' points.')
        
        #路经点
        self.waypoints = list()
        for i in range(self.length):
            self.waypoints.append(points_list.pop().positions)

        #将要发布的数据
        self.pub_joints = self.waypoints.pop()

        #各关节误差列表
        self.point_differ = [.0,.0,.0,.0]

        #要发送的msg
        self.arm_command = Float32MultiArray()
        self.arm_command.data = self.pub_joints


        
        #设置10 Hz 频率，准备发送
        rate = rospy.Rate(10) # 10hz

        #当路径点没有执行完的时候，继续发送
        while True:
            if self.server.is_preempt_requested():
                self.server.set_preempted()
                rospy.loginfo('Arm Controller: Preempted.')
                return

            self.point_differ[0] = abs(self.state[0] - self.pub_joints[0])
            self.point_differ[1] = abs(self.state[1] - self.pub_joints[1])
            self.point_differ[2] = abs(self.state[2] - self.pub_joints[2])
            self.point_differ[3] = abs(self.state[3] - self.pub_joints[3])


            
            #如果上次的路径点已到达阈值允许范围内，路径点还有，更新发送的数据，没有则完成，退出循环
            if max(self.point_differ)<self.tolerance_error:
                self.percent = (1-len(self.waypoints)/float(self.length))*100
                print('percent:'+str(self.percent)+'%')
                if self.waypoints:
                    self.pub_joints = list(self.waypoints.pop())
                    self.arm_command.data = self.pub_joints
                else:
                    break
            else:
                pass
                #max_index = self.point_differ.index(max(self.point_differ))
                #print('arm_joint_'+str(max_index)+'need to move')
                #rospy.logwarn("current_state:")
                #rospy.logwarn(self.state[max_index])
                #rospy.logwarn("need to move:")
                #rospy.logwarn(self.pub_joints[max_index])
            #发布
            self.pub.publish(self.arm_command)
            time.sleep(5)
            rate.sleep()

        
        #当路径点完成时
        if self.percent == 100:
            rospy.loginfo("\033[1;36marm controller complete action successful!\033[0m")
            self.server.set_succeeded()
        else:
            rospy.loginfo("\033[1;31mFailed!\033[0m")

    def stateCb(self, msg):
        self.state = msg.position[1:5]

if __name__ == '__main__':
        
    server = Server()
    
