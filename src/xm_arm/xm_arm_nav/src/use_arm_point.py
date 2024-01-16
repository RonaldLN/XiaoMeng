#!/usr/bin/env python
# encoding:utf8


import rospy, sys
import actionlib
from geometry_msgs.msg import PointStamped
from xm_msgs.srv import *
from xm_msgs.msg import *

class USE_ARM_POINT:

    def __init__(self):
        rospy.init_node('use_arm_point', anonymous = True)
	
        self.target_camera_point = PointStamped()
	self.target_camera_point.header.frame_id = "kinect2_rgb_link"
	self.target_camera_point.point.x = 1.2
	self.target_camera_point.point.y = -0.054
	self.target_camera_point.point.z = -0.209

	self.target_size = [0.1,0.1,0.1]
	self.table_depth = 0.1

	self.arm_stack_client = actionlib.SimpleActionClient("/xm_arm/arm_stack_point", xm_ArmStackAction)
        self.arm_stack_client.wait_for_server()
        rospy.loginfo("arm_stack_point_client...connected.")

    	goal = xm_ArmStackGoal()
    	goal.target_camera_point  = self.target_camera_point
    	goal.target_size_l = self.target_size[0]
    	goal.target_size_w = self.target_size[1]
    	goal.target_size_h = self.target_size[2]
    	goal.table_depth = self.table_depth
    
    	self.arm_stack_client.send_goal(goal)
    	self.arm_stack_client.wait_for_result(rospy.Duration(60.0))


if __name__ == '__main__':
        
    use_arm_point = USE_ARM_POINT()
