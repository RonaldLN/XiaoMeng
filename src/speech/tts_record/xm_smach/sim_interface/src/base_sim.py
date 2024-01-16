#!/usr/bin/env python
#coding=utf-8
'''
Author: jacy
Date: 2020-09-04 22:45:39
LastEditTime: 2020-09-07 19:47:15
LastEditors: Please set LastEditors
Description: In User Settings Edit
FilePath: /undefined/home/jacy/gazebo_test_ws/src/xm_smach/sim_interface/src/follow.py
'''

import rospy, sys, select, termios, tty
import roslib
from std_msgs.msg import *
from geometry_msgs.msg import *
from xm_msgs.srv import *
from xm_msgs.msg import *
import math
import tf

if __name__ == '__main__':
    
    
    
    rospy.init_node('follow', anonymous = True)
    
    rospy.Subscriber('/set_arm_joints_pos', Float32MultiArray, self.set_arm_pos)

    pub = rospy.Publisher('/mobile_base/mobile_base_controller/cmd_vel', Twist, queue_size=5)
    tf_listener = tf.TransformListener()
    


    rate = rospy.Rate(200)
    #需要注意的是，tf变换是在循环中进行的，所以频率不能需要大于joint states 发布的100hz的频率，否则会受到错误信息
    while not rospy.is_shutdown():
        #按照键盘命令更新人的位置
        key = getKey()
        if key == 'a':
            #向左转
            direction += PI/90      #一次转2度
            
            q = tf.transformations.quaternion_from_euler(0, 0, direction)
            person_pose.pose.orientation.x = q[0]
            person_pose.pose.orientation.y = q[1]
            person_pose.pose.orientation.z = q[2]
            person_pose.pose.orientation.w = q[3]
            
            pass
        elif key == 'd':
            #向右转
            direction -= PI/90      #一次转2度
            
            q = tf.transformations.quaternion_from_euler(0, 0, direction)
            person_pose.pose.orientation.x = q[0]
            person_pose.pose.orientation.y = q[1]
            person_pose.pose.orientation.z = q[2]
            person_pose.pose.orientation.w = q[3]
            
        elif key == 'w':
            #前进
            person_pose.pose.position.x += math.cos(direction) *0.03        #一次移动3cm
            person_pose.pose.position.y += math.sin(direction) *0.03        #一次移动3cm
            
        elif key =='s':
            person_pose.pose.position.x -= math.cos(direction) *0.03        #一次移动3cm
            person_pose.pose.position.y -= math.sin(direction) *0.03        #一次移动3cm
            #后退
            pass
        else:
            pass
   
        #get the kinect2_rgb_link
        tf_listener.waitForTransform('kinect2_rgb_link','odom',rospy.Time(),rospy.Duration(10.0))
        person_camera_pose = tf_listener.transformPose('kinect2_rgb_link',person_pose)


        pub_person_pos.position.point.x = person_camera_pose.pose.position.x
        pub_person_pos.position.point.y = person_camera_pose.pose.position.y

        camera_pub.publish(pub_person_pos)
        screen_pub.publish(person_pose)
        rate.sleep()
    
