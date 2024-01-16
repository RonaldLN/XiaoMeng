#!/usr/bin/env python2
# encoding:utf8

# use for 通用机器人抓取

import sympy as sp
import rospy
# import sys
# import select
# import termios
import time
# import tty
# # import moveit_commander
import actionlib
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from sensor_msgs.msg import JointState
from control_msgs.msg import FollowJointTrajectoryAction, FollowJointTrajectoryGoal, GripperCommandAction, GripperCommandGoal
from copy import deepcopy
import math
# from tf.transformations import euler_from_quaternion, quaternion_from_euler
import tf
from geometry_msgs.msg import *
from std_msgs.msg import *
from xm_msgs.msg import *
from xm_msgs.srv import *

PI = 3.1415926535

# 准备状态
# pick_pose = [0,0.3200, 0.3000, 0]
pick_pose = [0,0,0,0]
# pick_plat = 0.3
pick_plat = 0.3874


# plat_up = -0.1864
# plat_down = 0

# end_plat_height_to_the_target_height relative altitude
plat_height = -0.7    # 需要测量！！！！！

gripper_open = 1
gripper_close = 0

e0 = 0.300  # dabi length
e1 = 0.355  # xaiobi length
e2 = 0.194  # gripper length
es = 0.050  # gripper jian to catch dian length
offset_length = 0.115 # arm_joint_2 to plat_gan
offset_high = 0.652   # map to plat_origin_height
body_tall = 0      # camera to map\

ARM_LENGTH_MAX = 0.913
ARM_LENGTH_MIN = 0.35
# ARM_LENGTH_MAX = e0 + e1 + e2 + offset_length - es  # 能够伸到的最远距离
# ARM_LENGTH_MIN = e0 * math.sin(PI/4) +offset_length + e2 - es # 蜷缩距离

# plat_limit = [0.3874, -0.5]


def get_singel_pos_trajectory(arm_pose, joint_names):

    trajectory = JointTrajectory()
    trajectory.joint_names = joint_names

    single_pos = JointTrajectoryPoint()
    single_pos.positions = arm_pose
    single_pos.velocities = [0.0 for i in joint_names]
    single_pos.accelerations = [0.0 for i in joint_names]
    single_pos.time_from_start = rospy.Duration(5.0)

    trajectory.points.append(single_pos)

    return trajectory

def get_goal_from_trajectory(trajectory):

    goal = FollowJointTrajectoryGoal()
    goal.trajectory = trajectory
    goal.goal_time_tolerance = rospy.Duration(0.0)

    return goal

def get_goal_from_pos(arm_pose, joint_names):

    goal = FollowJointTrajectoryGoal()
    goal.trajectory = get_singel_pos_trajectory(arm_pose, joint_names)
    goal.goal_time_tolerance = rospy.Duration(0.0)

    return goal

def get_gripper_or_plat_goal_from_position(position):

    goal = GripperCommandGoal()
    goal.command.position = position

    return goal

# 计算arm_joint_3_angle
# 事先测量好的height，即滑台末位置
# 计算滑台要上升的高度：plat_move 
        

# 视觉坐标——>相对于底座的坐标系
# 计算原点到target的水平距离（target_length）
class Target:
    def __init__(self, x, y, z, phi=None, length=None, height=None):
        self.x = x
        self.y = y
        self.z = z

        rospy.loginfo("\033[1;32m%f,%f,%f\033[1;0m",x,y,z)
        rospy.loginfo("\033[1;32m(%f,%f,%f)\033[1;0m",self.x,self.y,self.z)

        if self.y == 0:
            self.phi = 0
        else: 
            self.phi = math.atan(self.x/self.y)
        self.height = self.z - plat_height
        self.length = math.sqrt(self.x**2 + self.y**2)
        rospy.loginfo("\033[1;32m(phi=%f,height=%f,length=%f)\033[1;0m",self.phi,self.height,self.length)


# TODO raise exception to get the err single

class ArmStack:
    '''
    - 输入:
        - target_pose_base
    - 输出
    -0 duo la A meng
    -1  easy arm pick lower triangle
    -2  easy arm pick upper triangle
    -3  easy arm pick cricle
    '''


    def __init__(self):
        rospy.init_node('arm_stack')

        self.joint_names = ["arm_joint_0", "arm_joint_1", "arm_joint_2", "arm_joint_3"]

        rospy.loginfo("Waiting for arm_controller...")
        self.arm_client = actionlib.SimpleActionClient("arm_controller/follow_joint_trajectory", FollowJointTrajectoryAction)
        self.arm_client.wait_for_server()
        rospy.loginfo("arm_controller...connected.")

        rospy.loginfo("Waiting for plat_controller...")
        self.plat_client = actionlib.SimpleActionClient("plat_controller/follow_joint_trajectory", GripperCommandAction)
        self.plat_client.wait_for_server()
        rospy.loginfo("plat_controller...connected.")

        rospy.loginfo("Waiting for gripper_controller...")
        self.gripper_client = actionlib.SimpleActionClient("gripper_controller/follow_joint_trajectory", GripperCommandAction)
        self.gripper_client.wait_for_server()
        rospy.loginfo("gripper_controller...connected.")

        self.tf_listener = tf.TransformListener()       

        # Create a publisher for displaying target poses
        self.target_pose_pub = rospy.Publisher(
            'target_pose', PoseStamped, queue_size=10)

        rospy.loginfo("start the arm_stack...")
        self.arm_stack_server = actionlib.SimpleActionServer(
            '/xm_arm/arm_stack', xm_ArmStackAction, self.action_cb, False)
        self.arm_stack_server.start()
        rospy.loginfo("\033[1;32mArm Stack  Start!\033[0m")

        rospy.spin()

    def get_compute_result(self, target):

        beta = 0
        alpha = PI/2
        plat_move = pick_plat
        arm_joint_3_angle = pick_pose[3]
        arm_joint_2_angle = pick_pose[2]
        arm_joint_1_angle = pick_pose[1]
        arm_joint_0_angle = pick_pose[0]

        if target.length >= ARM_LENGTH_MAX:
            rospy.loginfo("\033[1;33mlose the grip potision\033[0m")

        elif target.length <= ARM_LENGTH_MIN:
            rospy.loginfo("\033[1;33m the  target position is too close\033[0m")

        else:
            a, b = sp.symbols('alpha beta')
            eq1 = e1 * sp.cos(b) - e0 * sp.cos(a) - target.height
            eq2 = e0 * sp.sin(a) + e1 * sp.sin(b) + e2 + es - target.length

            solution = sp.solve((eq1, eq2), (a, b))
            print("\033[1;33m")
            print(solution)
            print("\033[0m")
            # rospy.loginfo("\033[1;37m [alpha: %f,beta: %f]\033[1;0m",solution[0][0],solution[0][1])
            for a_val, b_val in solution:
                alpha = round(sp.re(a_val.evalf()), 3)
                beta = round(sp.re(b_val.evalf()), 3)
                rospy.loginfo("\033[1;37m solution [alpha: %f,beta: %f]\033[1;0m",alpha,beta)

                if solution == [] :
                    rospy.loginfo("\033[1;31m Don't have a solution !")
                    continue

                if alpha > PI or beta > PI or alpha<0 or beta<0:
                    rospy.loginfo("\033[1;31m wrong_solution [alpha: %f,beta: %f]\033[1;0m",alpha,beta)
                    continue
                else:
                    rospy.loginfo("\033[1;32m real_solution [alpha: %f,beta: %f]\033[1;0m",alpha,beta)
                    arm_joint_3_angle = alpha-beta-PI/4  # gripper 向下为正
                    arm_joint_2_angle = beta             # xiaobi，向下为正
                    arm_joint_1_angle = alpha - PI/4     # dabi，向上为正
                    arm_joint_0_angle = target.phi       # 平动的轴，逆时针为正
                    break
            
        arm_solution = list()
        arm_solution.append(plat_move)
        arm_solution.append(arm_joint_0_angle)
        arm_solution.append(arm_joint_1_angle)
        arm_solution.append(arm_joint_2_angle)
        arm_solution.append(arm_joint_3_angle)


        rospy.loginfo("""
        \033[1;32m arm_solution [
                plat_move: %f,
                arm_joint_0_angle: %f,
                arm_joint_1_angle: %f,
                arm_joint_2_angle: %f,
                arm_joint_3_angle: %f
                ]\033[1;0m""",
        plat_move,arm_joint_0_angle,arm_joint_1_angle,arm_joint_2_angle,arm_joint_3_angle)

        # print("arm_joint_0_angle",arm_solution)

        get_solution = True
        return arm_solution, get_solution
        
    def action_cb(self, goal):
        rospy.loginfo("\033[1;33mxm arm stack  get the goal\033[0m")
        # rospy.logwarn(goal)
        target_camera_point = goal.target_camera_point
        # 保存camera link id
        camera_link_id = target_camera_point.header.frame_id

        # easy transfer the target_camera_point to the target_camera_pose
        target_camera_pose = PoseStamped()
        target_camera_pose.header.frame_id = target_camera_point.header.frame_id
        target_camera_pose.pose.position.x = target_camera_point.point.x
        target_camera_pose.pose.position.y = target_camera_point.point.y
        target_camera_pose.pose.position.z = target_camera_point.point.z
        target = Target(target_camera_pose.pose.position.x,target_camera_pose.pose.position.y,target_camera_pose.pose.position.z)
        
        # get the target_base_pose
        self.tf_listener.waitForTransform(
            'base_footprint', camera_link_id, rospy.Time(), rospy.Duration(60.0))
        target_base_pose = self.tf_listener.transformPoint(
            'base_footprint', target_camera_point)
        target_base_pose.point.z = target_base_pose.point.z  # -0.25
        print('target_camera_point:',target_camera_point.point)
        print('target_base_pose:',target_base_pose.point)

        # Publish the target pose in the rviz
        for i in range(10):
            self.target_pose_pub.publish(target_camera_pose)
            rospy.sleep(0.01)
        
        # 判断解决方式
        self.arm_solution_mode = 1

        # get the target_arm_joint_0_point
        self.tf_listener.waitForTransform(
            'base_link', 'base_footprint', rospy.Time(), rospy.Duration(60.0))
        target_arm_joint_0_point = self.tf_listener.transformPoint(
            'base_link', target_base_pose)
        target_arm_joint_0_point_ready = self.tf_listener.transformPoint(
            'base_link', target_base_pose)
        print('target_arm_joint_0_point:',target_arm_joint_0_point.point)     


        if self.arm_solution_mode == 1:
            # ready to compute
            open_goal_1 = get_gripper_or_plat_goal_from_position(gripper_open)
            self.gripper_client.send_goal(open_goal_1)
            self.gripper_client.wait_for_result(rospy.Duration(60.0))
            rospy.loginfo("...open the gripper")
            rospy.sleep(1)

            #  get the solution of pick
            # arm_length, arm_joint_0_angle = self.get_arm_length_and_arm_joint_0_angle(
            #     target_arm_joint_0_point)

            arm_solution, arm_solution_is_succeed =self.get_compute_result(target)
            print("arm_solution", arm_solution)

            arm_solution_angle = arm_solution[1:5]
            plat_solution = arm_solution[0]

            if arm_solution_is_succeed == True:
                rospy.logwarn("...finish computing the pick pose")
                # pick
                plat_goal = get_gripper_or_plat_goal_from_position(
                    plat_solution)
                self.plat_client.send_goal(plat_goal)
                self.plat_client.wait_for_result(rospy.Duration(60.0))
                rospy.loginfo("...plat down")
                time.sleep(4)

                pick_trajectory = get_singel_pos_trajectory(
                    arm_solution_angle, self.joint_names)
                print("pick_trajectory",pick_trajectory)
                pick_goal = get_goal_from_trajectory(pick_trajectory)
                print("pick_goal",pick_goal)
                self.arm_client.send_goal(pick_goal)
                self.arm_client.wait_for_result(rospy.Duration(60.0))
                rospy.loginfo("...pick_goal")
                time.sleep(4)


                close_goal = get_gripper_or_plat_goal_from_position(
                    gripper_close)
                self.gripper_client.send_goal(close_goal)
                self.gripper_client.wait_for_result(rospy.Duration(60.0))
                rospy.loginfo("...close the gripper")
                rospy.logwarn("...finish to pick")
                rospy.sleep(2)

                # back to the original pose

                ready_pick_trajectory = get_singel_pos_trajectory(
                    pick_pose, self.joint_names)
                ready_pick_goal = get_goal_from_trajectory(
                    ready_pick_trajectory)
                self.arm_client.send_goal(ready_pick_goal)
                self.arm_client.wait_for_result(rospy.Duration(60.0))

                plat_goal = get_gripper_or_plat_goal_from_position(
                    pick_plat)
                self.plat_client.send_goal(plat_goal)
                self.plat_client.wait_for_result(rospy.Duration(60.0))
                rospy.loginfo("...plat down")
                time.sleep(4)
                # # 这个高度是为了防止挡着摄像头
                # plat_goal_1 = get_gripper_or_plat_goal_from_position(plat_down)
                # self.plat_client.send_goal(plat_goal_1)
                # self.plat_client.wait_for_result(rospy.Duration(60.0))
                # rospy.logwarn("...back to the original pose")
                # rospy.loginfo("\033[1;36mArm Stack action succeed!\033[0m")
        else:
            pass

        # TODO check if get the object
        self.arm_stack_server.set_succeeded()
        rospy.loginfo("\033[1;36mArm Stack action end!\033[0m")

    # def get_pick_trajectory(self, arm_solution_angle, joint_names):

    #     trajectory = JointTrajectory()
    #     trajectory.joint_names = joint_names

    #     pos_demo = JointTrajectoryPoint()
    #     pos_demo.positions = [0.0 for i in joint_names]
    #     pos_demo.velocities = [0.0 for i in joint_names]
    #     pos_demo.accelerations = [0.0 for i in joint_names]
    #     pos_demo.time_from_start = rospy.Duration(5.0) 

    #     if self.arm_solution_mode == 1:
    #         # pos_1  end_pos,no joint_4
    #         pos_1 = deepcopy(pos_demo)
    #         pos_1.positions = [arm_solution_angle[0],
    #                            arm_solution_angle[1], arm_solution_angle[2]]
    #         trajectory.points.append(pos_1)

    #         # pos_2  end_pos, put down the joint_4
    #         pos_2 = deepcopy(pos_demo)
    #         pos_2.positions = arm_solution_angle
    #         trajectory.points.append(pos_2)           
    #     else:
    #         rospy.logerr("no mode match")
            
    #     return trajectory


if __name__ == "__main__":
    ArmStack()