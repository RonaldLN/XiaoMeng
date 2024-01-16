#!/usr/bin/env python
# encoding:utf8
import rospy
from smach import *
from tf.transformations import euler_from_quaternion, quaternion_from_euler
import math
import time
import subprocess
from smach_ros import *
import actionlib
from xm_msgs.srv import *
from xm_msgs.msg import *
from geometry_msgs.msg import *
import tf
from control_msgs.msg import *
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from sensor_msgs.msg import JointState
import moveit_commander
from copy import deepcopy
'''
Author: jacy
Date: 2020-09-11 21:17:17
LastEditTime: 2022-11-22 23:43:25
LastEditors: yishui,xpp
Description: In User Settings Edit
FilePath: /undefined/home/jacy/gazebo_test_ws/src/xm_smach/xm_smach/smach_lib/jacy_lib/arm_lib.py
'''

class ArmStackPoint(State):
    '''
    ArmStack
    '''
    def __init__(self):
        State.__init__(self,
                        outcomes=['succeeded','aborted' ,'error'],
                       input_keys=['target_camera_point','target_size','table_depth'])
        # self.arm_stack_client = actionlib.SimpleActionClient("/xm_arm/arm_stack_point", xm_ArmStackAction)  ————两个文件一样？？？
        self.arm_stack_client = actionlib.SimpleActionClient("/xm_arm/arm_stack", xm_ArmStackAction)

    def execute(self,userdata):
        try:
            
            goal = xm_ArmStackGoal()
            goal.target_camera_point  = userdata.target_camera_point
            goal.target_size_l = userdata.target_size[0]
            goal.target_size_w = userdata.target_size[1]
            goal.target_size_h = userdata.target_size[2]
            goal.table_depth = userdata.table_depth
    
            
            self.arm_stack_client.wait_for_server(rospy.Duration(60.0))
            
            self.arm_stack_client.cancel_all_goals()
            rospy.logwarn("send the goal")
            self.arm_stack_client.send_goal(goal)

            # self.arm_stack_client.wait_for_result(rospy.Duration.from_sec(60.0))
            self.arm_stack_client.wait_for_result(rospy.Duration.from_sec(120.0))

            #if self.arm_stack_client.get_state() == True:
            #if self.arm_stack_client.result_bool == True:
            return 'succeeded'
            # else:
            #     rospy.logerr("arm stack ik failed!")
            #     return 'aborted'
        except Exception ,e:
            rospy.logerr('Arm Stack Point have not work!')
            rospy.logerr(e)
            return 'error'
        else:
            return 'succeeded' 

class ArmStack(State):
    '''
    ArmStack
    '''
    def __init__(self):
        State.__init__(self,
                        outcomes=['succeeded','aborted' ,'error'],
                       input_keys=['target_camera_point','target_size','table_depth'])
        self.arm_stack_client = actionlib.SimpleActionClient("/xm_arm/arm_stack", xm_ArmStackAction)
    def execute(self,userdata):
        try:
            
            goal = xm_ArmStackGoal()
            goal.target_camera_point  = userdata.target_camera_point
            goal.target_size_l = userdata.target_size[0]
            goal.target_size_w = userdata.target_size[1]
            goal.target_size_h = userdata.target_size[2]
            goal.table_depth = userdata.table_depth
    
            
            self.arm_stack_client.wait_for_server(rospy.Duration(60.0))
            
            self.arm_stack_client.cancel_all_goals()
            rospy.logwarn("send the goal")
            self.arm_stack_client.send_goal(goal)

            # self.arm_stack_client.wait_for_result(rospy.Duration.from_sec(60.0))
            self.arm_stack_client.wait_for_result(rospy.Duration.from_sec(120.0))

            #if self.arm_stack_client.get_state() == True:
            #if self.arm_stack_client.result_bool == True:
            return 'succeeded'
            # else:
            #     rospy.logerr("arm stack ik failed!")
            #     return 'aborted'
        except Exception ,e:
            rospy.logerr('Arm Stack have not work!')
            rospy.logerr(e)
            return 'error'
        else:
            return 'succeeded' 

class GripperCommond(State):
    def __init__(self):
        State.__init__(self,outcomes=['succeeded', 'error'],input_keys=['commond'])
        self.gripper_client = actionlib.SimpleActionClient("/gripper_controller/follow_joint_trajectory", GripperCommandAction)
        self.gripper_commond = 0
    def execute(self,userdata):
        self.gripper_client.wait_for_server() 
        rospy.sleep(5.0)
        if userdata.commond == 0:
            self.gripper_commond = 0.03
            rospy.loginfo("..open the gripper")
        elif userdata.commond == 1:
            self.gripper_commond == -0.03
            rospy.loginfo("..close the gripper")

        else:
            rospy.logerr('no such commond for the gripper')
            return 'error'

        goal = GripperCommandGoal()
        goal.command.position = self.gripper_commond
        self.gripper_client.send_goal(goal)
        self.gripper_client.wait_for_result(rospy.Duration(60.0))
        time.sleep(0.5)
        #rospy.loginfo("...open the gripper") 

        return 'succeeded'

class PlatCommond(State):
    def __init__(self):
        State.__init__(self,outcomes=['succeeded', 'error'])
        self.plat_client = actionlib.SimpleActionClient("/plat_controller/follow_joint_trajectory", GripperCommandAction)
    
    def execute(self,userdata):
        goal = GripperCommandGoal()
        goal.command.position = -0.2
        self.plat_client.send_goal(goal)
        self.plat_client.wait_for_result(rospy.Duration(60.0))
        rospy.loginfo("...set the plat") 

        return 'succeeded'



# 控制机械臂移动
class ArmTrajectory(State):
    def __init__(self):
        State.__init__(self,outcomes=['succeeded', 'error'],input_keys=['arm_waypoints'])
        self.arm_client = actionlib.SimpleActionClient("/arm_controller/follow_joint_trajectory", FollowJointTrajectoryAction)
    def execute(self,userdata):
        
        trajectory = JointTrajectory()
        joint_names = ["arm_joint_0", "arm_joint_1", "arm_joint_2","arm_joint_3"]	
        trajectory.joint_names = joint_names

        pos_demo = JointTrajectoryPoint()
        pos_demo.positions = [0,-0.65,-1.41,-0.85]######
        pos_demo.velocities = [0.0 for i in joint_names]
        pos_demo.accelerations = [0.0 for i in joint_names]
        pos_demo.time_from_start = rospy.Duration(5.0)

        arm_waypoints = userdata.arm_waypoints
        for i in range( len(arm_waypoints) ):
            pos = deepcopy(pos_demo)
            pos.positions = arm_waypoints[i]
            trajectory.points.append(pos)

        goal = FollowJointTrajectoryGoal()
        goal.trajectory = trajectory
        goal.goal_time_tolerance = rospy.Duration(0.0)

        self.arm_client.send_goal(goal)
        self.arm_client.wait_for_result(rospy.Duration(60.0))

        return 'succeeded'

class GetPickPosBack(State):
    '''
    no use
    根据图像传过来的数据，确定抓取时xm的位置
    up_down变量为1，则使用上面的摄像头；up_down变量为0，则使用下面的摄像头。
    '''
    def __init__(self):
        State.__init__(self,outcomes =['succeeded','error'],
                        input_keys= ['distance','up_down'],
                        output_keys =['pick_pos'],
                        io_keys=['target_camera_point'])
        self.tf_listener = tf.TransformListener()
    def execute(self,userdata):
        try:
            getattr(userdata, 'distance')
            getattr(userdata, 'target_camera_point')
        except:
            rospy.logerr('no param')
            return 'error'
        else :
            if userdata.up_down == 0:
                camera_link = "camera_link_down"
            else :
                camera_link = "camera_link_up"
            target_camera_point = userdata.target_camera_point
            target_camera_point.header.frame_id = camera_link
            print("target_camera_point:",target_camera_point)
            self.tf_listener.waitForTransform('base_link',camera_link,rospy.Time.now(),rospy.Duration(4.0))
            # print(55435)
            target_base_point = self.tf_listener.transformPoint('base_link',target_camera_point)
            #                           y
            #               ————————————————————————————
            #               |                       |
            #               |                   |
            #               |               |
            #          x    |  alpha    |
            #               | ~~~   |
            #               |   |
            #               |
            print("target_base_point:",target_base_point)
            x = target_base_point.point.x
            y = target_base_point.point.y

            distance = math.sqrt(x**2+y**2)
            alpha =  math.atan(y/x)
            #  状态机的userdata.distance 需要确定
            move_in_line = distance - 1.0
            
            print("distance is:"+str(distance))
            print("move_in_line is:"+str(move_in_line))

            pick_base_point = PointStamped()
            pick_base_point.header.frame_id = 'base_link'
            pick_base_point.point.x = move_in_line*math.cos(alpha)
            pick_base_point.point.y = move_in_line*math.sin(alpha) 

            target_camera_point_new = PointStamped()
            target_camera_point_new.header.frame_id = 'base_link'
            target_camera_point_new.point.x = (distance-move_in_line)*math.cos(alpha)
            target_camera_point_new.point.y = (distance-move_in_line)*math.sin(alpha) 
            target_camera_point_new.point.z = target_base_point.point.z


            self.tf_listener.waitForTransform(camera_link,'base_link',rospy.Time(),rospy.Duration(10.0))
            userdata.target_camera_point = self.tf_listener.transformPoint(camera_link,target_camera_point_new)
            print('target_camera_point new',userdata.target_camera_point)
            
            qs = QuaternionStamped()
            qs.header.frame_id = 'base_link'
            qs.quaternion = Quaternion(*quaternion_from_euler(0,0,alpha))
            
            self.tf_listener.waitForTransform('map','base_link',rospy.Time.now(),rospy.Duration(60.0))    
            pick_base_point =self.tf_listener.transformPoint('map',pick_base_point)
            
            qs =self.tf_listener.transformQuaternion('map',qs)
            userdata.pick_pos = Pose(pick_base_point.point,qs.quaternion)

            return 'succeeded'

class GetPickPos(State):
    '''
    根据图像传过来的数据，确定抓取时xm的位置
    '''
    def __init__(self):
        State.__init__(self,outcomes =['succeeded','error'],
                        input_keys= ['distance','target_camera_point','pick_pos'],
                        output_keys =['pick_pos'])
        self.tf_listener = tf.TransformListener()
    def execute(self,userdata):
        try:
            getattr(userdata, 'distance')
            getattr(userdata, 'target_camera_point')
        except:
            rospy.logerr('no param')
            return 'error'
        else :
            target_camera_point = userdata.target_camera_point
            target_camera_point.header.frame_id="camera_link"
            print(target_camera_point)
            self.tf_listener.waitForTransform('base_link','camera_link',rospy.Time(),rospy.Duration(10.0))
            target_base_point = self.tf_listener.transformPoint('base_link',target_camera_point)

            print('target_base_point is ',target_base_point)
            #                           y
            #               ————————————————————————————
            #               |                       |
            #               |                   |
            #               |               |
            #          x    |  alpha    |
            #               | ~~~   |
            #               |   |
            #               |
            x = target_base_point.point.x - 1.0#0.99
            #x = target_base_point.point.x
            y = target_base_point.point.y
            alpha = 0

            pick_base_point = PointStamped()
            pick_base_point.header.frame_id = 'base_link'
            pick_base_point.point.x = x - userdata.distance
            #pick_base_point.point.x = x
            pick_base_point.point.y = y
            
            qs = QuaternionStamped()
            qs.header.frame_id = 'base_link'
            qs.quaternion = Quaternion(*quaternion_from_euler(0,0,-alpha))
            
            self.tf_listener.waitForTransform('map','base_link',rospy.Time(),rospy.Duration(60.0))    
            pick_base_point =self.tf_listener.transformPoint('map',pick_base_point)
            
            qs =self.tf_listener.transformQuaternion('map',qs)
            userdata.pick_pos = Pose(pick_base_point.point,qs.quaternion)
            print("userdata.pick_pos :",userdata.pick_pos)

            return 'succeeded'
