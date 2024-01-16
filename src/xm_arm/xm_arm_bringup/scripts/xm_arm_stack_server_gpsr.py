#!/usr/bin/env python
# encoding:utf8


import rospy
import sys
import select
import termios
import time
import tty
import moveit_commander
import actionlib
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from sensor_msgs.msg import JointState
from control_msgs.msg import FollowJointTrajectoryAction, FollowJointTrajectoryGoal, GripperCommandAction, GripperCommandGoal
from copy import deepcopy
import math
from tf.transformations import euler_from_quaternion, quaternion_from_euler
import tf
from geometry_msgs.msg import *
from std_msgs.msg import *
from xm_msgs.msg import *
from xm_msgs.srv import *

PI = 3.1415926535
init_pose = [0, 0, 0, 0]
straight = [-1.57,2.11,-1.23,-0.9]
# pick_pose = [0,-1.57,3.14,0,-0.1,0]
# pick_pose = [1.57,2.11,-1.23,-0.9]
pick_pose = [0,-1.06,-2.20,-1.12]
take_back_pose_1 = [0, 3.14, -3.14, -0.16]

gripper_open = 0.03
gripper_close = -0.03

plat_lift = -0.1864
plat_down = -0.1
# center_offset = 0.00551
center_offset = 0
plat_up =-0.1864
plat_down = 0.1544
# plat_up=-1
# plat_down=1
e0 = 0.13
e1 = 0.23
e2 = 0.33
e3 = 0.235-0.1

es = 0.1
# 升降台高度限制 0.1520 -0.2344 正为向下，负为向上
plat_limit= [0.1520,-0.2344]

e3_doramen = 0.228  # 0.0655
e4 = 0.005
arm_0_1_h = 0.204
arm_base_0_h = 0.55
arm_0_1_offset = 0.014
arm_4_5_offset = 0.018

height = 0.95
distance = 0.70  # 70
depth = 0.04


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

        self.joint_names = ["arm_joint_0", "arm_joint_1",
                            "arm_joint_2", "arm_joint_3"]

        rospy.loginfo("Waiting for arm_controller...")
        self.arm_client = actionlib.SimpleActionClient(
            "arm_controller/follow_joint_trajectory", FollowJointTrajectoryAction)
        self.arm_client.wait_for_server()
        rospy.loginfo("arm_controller...connected.")

        rospy.loginfo("Waiting for plat_controller...")
        self.plat_client = actionlib.SimpleActionClient(
            "plat_controller/follow_joint_trajectory", GripperCommandAction)
        self.plat_client.wait_for_server()
        rospy.loginfo("plat_controller...connected.")

        rospy.loginfo("Waiting for gripper_controller...")
        self.gripper_client = actionlib.SimpleActionClient(
            "gripper_controller/follow_joint_trajectory", GripperCommandAction)
        self.gripper_client.wait_for_server()
        rospy.loginfo("gripper_controller...connected.")

        self.tf_listener = tf.TransformListener()

        # Create a publisher for displaying target poses
        self.target_pose_pub = rospy.Publisher(
            'target_pose', PoseStamped, queue_size=10)

        #************arm stack**********************#
        rospy.loginfo("start the arm_stack...")
        self.arm_stack_server = actionlib.SimpleActionServer(
            '/xm_arm/arm_stack', xm_ArmStackAction, self.action_cb, False)
        self.arm_stack_server.start()
        rospy.loginfo("\033[1;32mArm Stack Start!\033[0m")

        rospy.spin()

    def action_cb(self, goal):
        rospy.loginfo("\033[1;33mxm arm stack get the goal\033[0m")
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

            # 这个高度是为了防止挡着摄像头
            # plat_goal_1 = get_gripper_or_plat_goal_from_position(plat_down)
            # self.plat_client.send_goal(plat_goal_1)
            # self.plat_client.wait_for_result(rospy.Duration(60.0))
            # rospy.loginfo("...plat down")
            # rospy.sleep(1)

            # ready_pick_trajectory = get_singel_pos_trajectory(
            #     pick_pose, self.joint_names)
            # ready_pick_goal = get_goal_from_trajectory(ready_pick_trajectory)
            # self.arm_client.send_goal(ready_pick_goal)
            # self.arm_client.wait_for_result(rospy.Duration(60.0))
            # rospy.loginfo("...start to compute")
            # rospy.sleep(1)

            # get the solution of ready to pick
            target_arm_joint_0_point_ready.point.x = target_arm_joint_0_point.point.x
            target_arm_joint_0_point.point.z = target_arm_joint_0_point.point.z -0.3
            arm_length_ready, arm_joint_0_angle_ready = self.get_arm_length_and_arm_joint_0_angle(
                target_arm_joint_0_point_ready)
            print("arm_joint_0_angle_ready",arm_joint_0_angle_ready)
            print("arm_length_ready",arm_length_ready)

            arm_solution_ready, arm_solution_ready_is_succeed = self.get_arm_solution(
                arm_length_ready, target_arm_joint_0_point_ready, arm_joint_0_angle_ready, target_arm_joint_0_point.point.z)
            # rospy.logwarn(arm_solution_ready)
            print('arm_solution_ready:,arm_solution_ready_is_succeed',arm_solution_ready,arm_solution_ready_is_succeed)

            if arm_solution_ready_is_succeed == True:

                # rospy.loginfo(arm_solution_ready)
                # arm_solution_angle_ready = arm_solution_ready[1:5]
                # plat_solution_ready = arm_solution_ready[0]
                # rospy.logwarn("...finish computing the ready pose")

                # # ready to pick
                # lift_goal_ready = get_gripper_or_plat_goal_from_position(
                #     plat_solution_ready)
                # self.plat_client.send_goal(lift_goal_ready)
                # self.plat_client.wait_for_result(rospy.Duration(60.0))

                # pick_trajectory_ready = get_singel_pos_trajectory(
                #     arm_solution_angle_ready, self.joint_names)
                # pick_goal_ready = get_goal_from_trajectory(
                #     pick_trajectory_ready)
                # self.arm_client.send_goal(pick_goal_ready)
                # self.arm_client.wait_for_result(rospy.Duration(60.0))

                # 中间停顿一下
                # rospy.sleep(10)

                # get the solution of pick
                target_arm_joint_0_point.point.x = target_arm_joint_0_point.point.x
                arm_length, arm_joint_0_angle = self.get_arm_length_and_arm_joint_0_angle(
                    target_arm_joint_0_point)
                arm_solution, arm_solution_is_succeed = self.get_arm_solution(
                    arm_length, target_arm_joint_0_point, arm_joint_0_angle, target_arm_joint_0_point.point.z)
                # rospy.loginfo(arm_solution)
                print("arm_solution",arm_solution)
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
                    time.sleep(3)

                    # arm_solution_angle_ready =arm_solution_angle
                    # arm_solution_angle_ready[2] = -0.1
                    # arm_solution_angle_ready[3] = -0.1

                    # pick_trajectory_ready = get_singel_pos_trajectory(
                    #     arm_solution_angle_ready, self.joint_names)
                    # print("pick_trajectory",pick_trajectory_ready)
                    # pick_goal = get_goal_from_trajectory(pick_trajectory_ready)
                    # print("pick_goal",pick_goal)
                    # self.arm_client.send_goal(pick_goal)
                    # self.arm_client.wait_for_result(rospy.Duration(60.0))
                    # rospy.loginfo("...pick_goal_ready")

                    pick_trajectory = get_singel_pos_trajectory(
                        arm_solution_angle, self.joint_names)
                    print("pick_trajectory",pick_trajectory)
                    pick_goal = get_goal_from_trajectory(pick_trajectory)
                    print("pick_goal",pick_goal)
                    self.arm_client.send_goal(pick_goal)
                    self.arm_client.wait_for_result(rospy.Duration(60.0))
                    rospy.loginfo("...pick_goal")

                    time.sleep(3)

                    close_goal = get_gripper_or_plat_goal_from_position(
                        gripper_close)
                    self.gripper_client.send_goal(close_goal)
                    self.gripper_client.wait_for_result(rospy.Duration(60.0))
                    rospy.loginfo("...close the gripper")
                    rospy.logwarn("...finish to pick")
                    rospy.sleep(1)

                    # back to the pose of ready to pick
                    # rospy.loginfo(arm_solution_angle_ready)
                    # arm_back = arm_solution_angle_ready

                    # # arm_back[4] = -0.1
                    # rospy.loginfo(arm_back)
                    # pick_trajectory_back = get_singel_pos_trajectory(
                    #     arm_back, self.joint_names)
                    # pick_goal_back = get_goal_from_trajectory(
                    #     pick_trajectory_back)
                    # self.arm_client.send_goal(pick_goal_back)
                    # self.arm_client.wait_for_result(rospy.Duration(60.0))
                    # rospy.logwarn("...back to the ready pose")
                    # rospy.sleep(15)
                    # back to the original pose
                    ready_pick_trajectory = get_singel_pos_trajectory(
                        pick_pose, self.joint_names)
                    ready_pick_goal = get_goal_from_trajectory(
                        ready_pick_trajectory)
                    self.arm_client.send_goal(ready_pick_goal)
                    self.arm_client.wait_for_result(rospy.Duration(60.0))

                    # 这个高度是为了防止挡着摄像头
                    plat_goal_1 = get_gripper_or_plat_goal_from_position(0)
                    self.plat_client.send_goal(plat_goal_1)
                    self.plat_client.wait_for_result(rospy.Duration(60.0))
                    rospy.logwarn("...back to the original pose")
                    rospy.loginfo("\033[1;36mArm Stack action succeed!\033[0m")

        else:
            pass

        # TODO check if get the ojbect
        self.arm_stack_server.set_succeeded()
        rospy.loginfo("\033[1;36mArm Stack action end!\033[0m")

    def get_pick_trajectory(self, arm_solution_angle, joint_names):

        trajectory = JointTrajectory()
        trajectory.joint_names = joint_names

        pos_demo = JointTrajectoryPoint()
        pos_demo.positions = [0.0 for i in joint_names]
        pos_demo.velocities = [0.0 for i in joint_names]
        pos_demo.accelerations = [0.0 for i in joint_names]
        pos_demo.time_from_start = rospy.Duration(5.0)

        if self.arm_solution_mode == 1:
            # pos_1  end_pos,no joint_4
            pos_1 = deepcopy(pos_demo)
            pos_1.positions = [arm_solution_angle[0],
                               arm_solution_angle[1], arm_solution_angle[2]]
            trajectory.points.append(pos_1)

            # pos_2  end_pos, put down the joint_4
            pos_2 = deepcopy(pos_demo)
            pos_2.positions = arm_solution_angle
            trajectory.points.append(pos_2)
        else:
            rospy.logerr("no mode match")

        return trajectory

    def get_arm_length_and_arm_joint_0_angle(self, target_arm_joint_0_point):
        '''
        INPUT : target_arm_joint_0_point\n
        OUTPUT : arm_length,arm_joint_0_angle
        '''

        if self.arm_solution_mode == 0 or self.arm_solution_mode == 1 or self.arm_solution_mode == 2:
            gripper_length = math.sqrt(
                target_arm_joint_0_point.point.y**2+target_arm_joint_0_point.point.x**2)
            rospy.logwarn(gripper_length)
            arm_length = math.sqrt(gripper_length**2-center_offset**2)
            rospy.logwarn(arm_length)
            beta = math.asin(center_offset/gripper_length)

            gamma = math.atan(
                abs(target_arm_joint_0_point.point.x/target_arm_joint_0_point.point.y))

            alpha = PI/2 - beta - gamma

            arm_joint_0_angle = alpha

            #why--xpp arm_joint_0_angle是正是负要更具机械臂angle0的旋转方向来定
            if target_arm_joint_0_point.point.y > 0:
                arm_joint_0_angle = -arm_joint_0_angle

        elif self.arm_solution_mode == 3:
            arm_length = math.sqrt(
                target_arm_joint_0_point.point.y**2+target_arm_joint_0_point.point.x**2)
            arm_joint_0_angle = math.atan(
                abs(target_arm_joint_0_point.point.y/target_arm_joint_0_point.point.x))
            if target_arm_joint_0_point.point.y > 0:
                arm_joint_0_angle = -arm_joint_0_angle

        return arm_length, arm_joint_0_angle

    def get_arm_solution(self, arm_length, target_arm_joint_0_point, arm_joint_0_angle, target_base_heigh):
        '''
        INPUT : arm_length,target_arm_joint_0_point,arm_joint_0_angle,target_base_heigh\n
        OUTPUT : arm_solution
        '''
        # target_arm_joint_0_point need to be precise,we can not only use the tf change,some change maybe be down,especially for the moveing link

        if self.arm_solution_mode == 1:
            alpha = PI
            beta =0
            plat_position = 0
            get_solution = False  # to get the single
            solution0 =True
            solution1 =True
            solution2 =True
            rospy.logwarn("mode is 1")
            # get the alpha and beta
            while not get_solution:
                arm_length_max = e0+e1+e2+e3
                print("arm_length_max,arm_length,e0+e3",arm_length_max,arm_length,e0+e3)
                # 判断目标在什么位置，从而选择合适的解算方法
                if arm_length_max <= arm_length :
                    rospy.loginfo("\033[1;33mlose the grip potision\033[0m")
                    break
                elif e0 +e3 > arm_length :
                    rospy.loginfo("\033[1;33m the  target position is too close\033[0m")
                    break
                elif solution2:
                    # print(81123)
                    alpha = 5*PI/12
                    beta = PI/2
                    solution_test = False
                    while not solution_test:
                        if alpha >PI/2:
                            rospy.loginfo("\033[1;33m no solution 7\033[0m")
                            break
                        arm_length_test = e0+e2*math.sin(beta)+e3+e1*math.sin(alpha)
                        if  arm_length_test > arm_length:   
                            # print(4234234)
                            while arm_length_test > arm_length :
                                beta -= 0.005
                                # print(beta)
                                if  beta <0 :
                                    beta = PI/2
                                    alpha +=0.05
                                    # print(6)
                                    break
                                else :
                                    arm_length_test = e0+e2*math.sin(beta)+e3+e1*math.sin(alpha)
                                    if arm_length_test-arm_length < es :
                                        # print(7)
                                        # 两个锐角
                                        arm_high = arm_0_1_h-e1*math.cos(alpha)+e2*math.cos(beta)
                                        plat_position = arm_high - (target_base_heigh-arm_base_0_h)
                                        # print(plat_position,arm_high,e1*math.cos(alpha),e2*math.cos(beta),beta)
                                        # rospy.loginfo("\033[1;33m no solution 1\033[0m")
                                        # time.sleep(0.1)

                                        if  plat_position<plat_limit[0] and plat_position > plat_limit[1]:
                                            solution_test = True
                                            get_solution = True
                                            rospy.loginfo("\033[1;33m no solution 1\033[0m")
                                            break
                                    if arm_length_test-arm_length < es and not solution_test:
                                        # print(7)

                                        # alpha 钝角 beta 锐角
                                        rospy.loginfo("\033[1;33m no solution 2\033[0m")

                                        arm_high = arm_0_1_h+e1*math.cos(alpha)+e2*math.cos(beta)
                                        plat_position = arm_high - (target_base_heigh-arm_base_0_h)
                                        print(plat_position)
                                        if  plat_position<plat_limit[0] and plat_position > plat_limit[1]:
                                            solution_test = True
                                            print("plat_position",plat_position)
                                            get_solution = True
                                            rospy.loginfo("\033[1;33m no solution 2\033[0m")

                                            alpha += PI/2
                                            break
                                    if arm_length_test-arm_length < es and  not solution_test:
                                        # print(7)

                                        # alpha 锐角 beta 钝角
                                        arm_high = arm_0_1_h-e1*math.cos(alpha)-e2*math.cos(beta)
                                        plat_position = arm_high - (target_base_heigh-arm_base_0_h)
                                        if  plat_position<plat_limit[0] and plat_position > plat_limit[1]:
                                            solution_test = True
                                            get_solution = True
                                            rospy.loginfo("\033[1;33m no solution 3\033[0m")


                                            beta += PI/2
                                            break
                                    if arm_length_test-arm_length < es and  not solution_test:
                                        # print(7)

                                        # alpha beta 钝角  
                                        arm_high = arm_0_1_h+e1*math.cos(alpha)-e2*math.cos(beta)
                                        plat_position = arm_high - (target_base_heigh-arm_base_0_h)
                                        if  plat_position<plat_limit[0] and plat_position > plat_limit[1]:
                                            solution_test = True
                                            get_solution = True
                                            rospy.loginfo("\033[1;33m no solution 4\033[0m")


                                            alpha += PI/2
                                            beta += PI/2
                                            break
                                    else :
                                        break

                                
                        else :
                            alpha +=0.005
                            beta = PI/2
                            # print("alpha")
                            if alpha >PI/2:
                                # 全部算法无解
                                rospy.loginfo("\033[1;33m no solution 5\033[0m")
                                break
                            else :
                                # print(4)
                                continue
                    if  not solution_test :
                        # 此算法无解
                        solution2 = False
                        rospy.loginfo("\033[1;33m no sulotipn !\033[0m")
                        break
                    else :
                        break
                elif arm_length_max - e2 <arm_length and  solution0:
                    alpha = PI/2
                    beta = PI/2
                    solution_test = False
                    while True:
                        arm_length_test = e0+e1+e3+e2*math.sin(beta)
                        if arm_length_test-arm_length < es :
                            arm_high = arm_0_1_h+e2*math.cos(beta)
                            plat_position = arm_high - (target_base_heigh-arm_base_0_h)
                            if  plat_position<plat_limit[0] and plat_position > plat_limit[1]:
                                print(1)
                                solution_test = True
                                get_solution = True
                                break
                            else:
                                arm_high = arm_0_1_h-e2*math.cos(beta)
                                plat_position = arm_high - (target_base_heigh-arm_base_0_h)
                                if  plat_position<plat_limit[0] and plat_position > plat_limit[1]:
                                    solution_test = True
                                    get_solution = True
                                    print(2)

                                    beta += PI/2
                                    break
                                else :
                                    beta -= 0.005
                                    if  beta < 0 :
                                        # print(beta)
                                        break
                                    else :
                                        continue
                        else :
                            beta -= 0.005
                            if  beta < 0 :
                                break
                            else :
                                continue
                    if  not solution_test :
                        # 此算法无解
                        solution0 = False
                        continue
                    else :
                        break
                elif arm_length_max - e1 <arm_length and solution1:
                    alpha = PI/2
                    beta = PI/2
                    solution_test = False
                    while True:
                        arm_length_test = e0+e2+e3+e1*math.sin(alpha)
                        if arm_length_test-arm_length < es :
                            arm_high = arm_0_1_h+e1*math.cos(alpha)
                            plat_position = arm_high - (target_base_heigh-arm_base_0_h)
                            if  plat_position<plat_limit[0] and plat_position > plat_limit[1]:
                                solution_test = True
                                get_solution = True
                                print(3)

                                alpha+= PI/2
                                break
                            else:
                                arm_high = arm_0_1_h-e1*math.cos(alpha)
                                plat_position = arm_high - (target_base_heigh-arm_base_0_h)
                                if  plat_position<plat_limit[0] and plat_position > plat_limit[1]:
                                    solution_test = True
                                    get_solution = True
                                    print(4)

                                    break
                                else :
                                    alpha -= 0.005
                                    if  alpha < 0 :
                                        break
                                    else :
                                        continue
                        else :
                            alpha -= 0.005
                            if  alpha < 0 :
                                break
                            else :
                                continue
                    if  not solution_test :
                        # 此算法无解
                        solution1 = False
                        continue
                    else :
                        break
                
            arm_joint_1_angle = alpha - PI/2
            arm_joint_2_angle =alpha + beta -PI
            arm_joint_3_angle =-(PI/2 - beta)


        arm_solution = list()
        arm_solution.append(plat_position)
        arm_joint_0_angle = 0
        arm_solution.append(arm_joint_0_angle)
        arm_solution.append(arm_joint_1_angle)
        arm_solution.append(arm_joint_2_angle)
        arm_solution.append(arm_joint_3_angle)


        print("arm_joint_0_angle",arm_solution)

        return arm_solution, get_solution


if __name__ == "__main__":
    ArmStack()
