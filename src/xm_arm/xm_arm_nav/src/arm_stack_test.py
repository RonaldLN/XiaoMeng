#!/usr/bin/env python
# encoding:utf8
"""
此脚本用于在仿真中测试机械臂除升降台之外的工作区域，
用以确定进行机械臂抓取时升降台的合适的高度和抓取时候的合适的距离
"""

import rospy, sys
import moveit_commander
from geometry_msgs.msg import PoseStamped, Pose,PointStamped
from moveit_commander import MoveGroupCommander, PlanningSceneInterface
from moveit_msgs.msg import PlanningScene, ObjectColor
from moveit_msgs.msg import Grasp, GripperTranslation, MoveItErrorCodes
from sympy import *
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from tf.transformations import quaternion_from_euler
from copy import deepcopy
import actionlib
from xm_msgs.srv import *
from xm_msgs.msg import *
from tf.transformations import euler_from_quaternion, quaternion_from_euler
import tf
#from xm_tool.color import *
REFERENCE_FRAME = 'base_footprint'

#height指物体中心距离地面的高度 1.2
#distance指物体前表面中距离底盘中心x轴方向的距离
#depth指物体位置的深度（相对于货架）
'''

**********test1**********
height  depth   distance    
1       0.04    0.55~0.83
->the situable distance is set for 0.70

**********test2**********
depth   distance    height
0.04    0.70        0.54~1.26
->the situable distance is set for 0.90(under the unremoving situation)
'''
height = 0.95
distance = 0.70
depth = 0.04
class MoveItDemo:
    def __init__(self):
        # Initialize the move_group API
        moveit_commander.roscpp_initialize(sys.argv)
        
        rospy.init_node('moveit_demo')
        
        tf_listener = tf.TransformListener()

        self.arm_stack_client = actionlib.SimpleActionClient("/xm_arm/arm_stack", xm_ArmStackAction)

        # Use the planning scene object to add or remove objects
        scene = PlanningSceneInterface()
        
        target_size = [0.04, 0.065, 0.105]
        
        target_name = scene.get_known_object_names()
        target_pose_pose = scene.get_object_poses(target_name)['target']

        target_pose = PoseStamped()
        target_pose.header.frame_id = "odom"
        target_pose.pose = target_pose_pose
        
        rospy.logwarn(target_pose)
        
        
        #get the kinect2_rgb_link
        tf_listener.waitForTransform('kinect2_rgb_link','odom',rospy.Time(),rospy.Duration(60.0))
        target_camera_pose = tf_listener.transformPose('kinect2_rgb_link',target_pose)
        
        target_camera_point = PointStamped()
        target_camera_point.point.x = target_camera_pose.pose.position.x
        target_camera_point.point.y = target_camera_pose.pose.position.y
        target_camera_point.point.z = target_camera_pose.pose.position.z
        target_camera_point.header.frame_id = 'kinect2_rgb_link'


        #TODO ask
        goal = xm_ArmStackGoal()
        goal.target_camera_point  = target_camera_point
        goal.target_size.l = target_size[0]
        goal.target_size.w = target_size[1]
        goal.target_size.h = target_size[2]
        goal.table_depth = depth
        self.arm_stack_client.send_goal(goal)
        self.arm_stack_client.wait_for_result(rospy.Duration(60.0))


        # Shut down MoveIt cleanly
        moveit_commander.roscpp_shutdown()
        
        # Exit the script
        moveit_commander.os._exit(0)

if __name__ == "__main__":
    MoveItDemo()

    
