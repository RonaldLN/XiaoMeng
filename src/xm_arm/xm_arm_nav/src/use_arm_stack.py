#!/usr/bin/env python
# encoding:utf8


import rospy
# import sys
import actionlib
from geometry_msgs.msg import PointStamped
from xm_msgs.srv import *
from xm_msgs.msg import *


class USE_ARM_STACK:

    def __init__(self):
        rospy.init_node('use_arm_stack', anonymous=True)

        self.target_camera_point = PointStamped()
        self.target_camera_point.header.frame_id = "camera_link"

        '''
        0.555 -0.35 -0.3 右前方
        0.555 0.35  -0.3 左前方
        y：左正右负
        z：上负下正
        x：前后

        '''
        self.target_camera_point.point.x =-0.5

        self.target_camera_point.point.y = 0.5
        self.target_camera_point.point.z = -0.9

        # # x  0.3 0.8 1.2 1.8 
        # self.target_camera_point.point.x = 0.4
        # # y  0.2 0.6 1.0 1.5
        # self.target_camera_point.point.y = 0.5
        # # z  0.2 0.6 1.0 1.5 2
        # self.target_camera_point.point.z = -0.2
        '''
        [255, 255, 1, 6, 0, 2, 74, 81, 62, 0, 226, 254]
        arm_joint_0_angle: -1.190290,
        arm_joint_1_angle: 0.063602,
        arm_joint_2_angle: 0.196000,
        arm_joint_3_angle: -0.132398
        '''

        self.target_size = [0.1, 0.1, 0.1]
        self.target_size = [0.05, 0.05, 0.05]
        self.table_depth = 0.1
        
    
        self.arm_stack_client = actionlib.SimpleActionClient(
            "/xm_arm/arm_stack", xm_ArmStackAction)
        self.arm_stack_client.wait_for_server()
        rospy.loginfo("arm_stack_client...connected.")

        goal = xm_ArmStackGoal()
        goal.target_camera_point = self.target_camera_point
        goal.target_size_l = self.target_size[0]
        goal.target_size_w = self.target_size[1]
        goal.target_size_h = self.target_size[2]
        goal.table_depth = self.table_depth

        self.arm_stack_client.send_goal(goal)
        self.arm_stack_client.wait_for_result(rospy.Duration(60.0))
        # print('nice')


if __name__ == '__main__':

    use_arm_stack = USE_ARM_STACK()
