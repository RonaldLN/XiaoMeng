#!/usr/bin/env python
# -*- coding: UTF-8 -*-
import rospy, sys
import moveit_commander
from geometry_msgs.msg import PoseStamped, Pose
from moveit_commander import MoveGroupCommander, PlanningSceneInterface
from moveit_msgs.msg import PlanningScene, ObjectColor
from moveit_msgs.msg import Grasp, GripperTranslation, MoveItErrorCodes
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from tf.transformations import quaternion_from_euler
from math import radians
from copy import deepcopy


class MoveItDemo:

    def __init__(self):
        # 初始化move_group的API
        moveit_commander.roscpp_initialize(sys.argv)

        # 初始化ROS节点
        rospy.init_node('moveit_demo')

        # 初始化场景对象
        #scene = PlanningSceneInterface()

        # Create a scene publisher to push changes to the scene
        #self.scene_pub = rospy.Publisher('planning_scene', PlanningScene)

        # Create a publisher for displaying gripper poses
        #self.gripper_pose_pub = rospy.Publisher('gripper_pose', PoseStamped)

        # 初始化需要使用move group控制的机械臂中的right_arm
        arm = MoveGroupCommander('arm')

        # 初始化move group中的right_gripper
        #gripper = MoveGroupCommander('gripper')

        # 获取终端link的名称
        # end_effector_link = arm.get_end_effector_link()

        #参考坐标系
        arm.set_pose_reference_frame('base_footprint')
        # Create a dictionary to hold object colors
        #self.colors = dict()

        # 设置位置（单位：米）和姿态（单位：弧度）的允许误差
        arm.set_goal_position_tolerance(0.05)
        arm.set_goal_orientation_tolerance(0.01)

        # 运动失败后允许重新规划（允许次数：10）
        # arm.allow_replanning(True)
        # arm.set_planning_time(5)

        # Set a limit on the number of pick attempts before bailing
        #max_pick_attempts = 5

        # Set a limit on the number of place attempts
        #max_place_attempts = 5

        #rospy.sleep(2)

        # 移除场景中之前运行遗留的物体

        #scene.remove_world_object('table')
        #scene.remove_world_object('box1')
        #scene.remove_world_object('box2')
        #scene.remove_world_object('target')

        # 控制机器人回到默认位姿
        # arm.set_named_target('wave')
        # arm.go()
        # rospy.sleep(4)
        arm.set_named_target('rest')
        arm.go()
        rospy.sleep(4)        
        # arm.set_named_target('wave_left')
        # arm.go()
        # rospy.sleep(4)
        arm.set_named_target('rest')
        arm.go()
        rospy.sleep(4)

        # Open the gripper to the neutral position
        #gripper.set_joint_value_target(GRIPPER_NEUTRAL)
        #gripper.go()

        rospy.sleep(1)
        # 设置桌面高度
       

if __name__ == "__main__":
    MoveItDemo()
