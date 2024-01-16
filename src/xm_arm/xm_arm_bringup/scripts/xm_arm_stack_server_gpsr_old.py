#!/usr/bin/env python
# encoding:utf8


import rospy
import sys
import select
import termios
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
pick_pose = [0,0,0,0]
take_back_pose_1 = [0, 3.14, -3.14, -0.16]

gripper_open = 0.03
gripper_close = -0.03

plat_lift = -0.1864
plat_down = 0.1544
# center_offset = 0.00551
center_offset = 0
plat_up =-0.1864
plat_down = 0.1544
# plat_up=-1
# plat_down=1
e0 = 0.13
e1 = 0.23
e2 = 0.33
e3 = 0.235
e3_doramen = 0.228  # 0.0655
e4 = 0.05
arm_0_1_h = 0.204
arm_base_0_h = 0.8395
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
        target_size_l = goal.target_size_l
        target_size_w = goal.target_size_w
        target_size_h = goal.target_size_h
        target_camera_point = goal.target_camera_point
        target_camera_point.header.frame_id='camera_link'
        table_depth = goal.table_depth

        # easy transfer the target_camera_point to the target_camera_pose
        target_camera_pose = PoseStamped()
        target_camera_pose.header.frame_id = "camera_link"
        target_camera_pose.pose.position.x = target_camera_point.point.x
        target_camera_pose.pose.position.y = target_camera_point.point.y
        target_camera_pose.pose.position.z = target_camera_point.point.z

        # get the target_base_pose
        self.tf_listener.waitForTransform(
            'base_footprint', 'camera_link', rospy.Time(), rospy.Duration(60.0))
        target_base_pose = self.tf_listener.transformPoint(
            'base_footprint', target_camera_point)
        target_base_pose.point.z = target_base_pose.point.z  # -0.25
        print('target_camera_point:','\n',target_camera_point.point)
        print('target_base_pose:','\n',target_base_pose.point)
        
        # Publish the target pose in the rviz
        for i in range(10):
            self.target_pose_pub.publish(target_camera_pose)
            rospy.sleep(0.01)

        # 判断解决方式
        self.arm_solution_mode = 1

        # get the target_arm_joint_0_point
        self.tf_listener.waitForTransform(
            'arm_link_0', 'base_footprint', rospy.Time(), rospy.Duration(60.0))
        target_arm_joint_0_point = self.tf_listener.transformPoint(
            'arm_link_0', target_base_pose)
        target_arm_joint_0_point_ready = self.tf_listener.transformPoint(
            'arm_link_0', target_base_pose)
        print('target_arm_joint_0_point:',target_arm_joint_0_point.point)

        if self.arm_solution_mode == 1:

            # ready to compute
            open_goal_1 = get_gripper_or_plat_goal_from_position(gripper_open)
            self.gripper_client.send_goal(open_goal_1)
            self.gripper_client.wait_for_result(rospy.Duration(60.0))
            rospy.loginfo("...open the gripper")
            rospy.sleep(1)

            # # 这个高度是为了防止挡着摄像头
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
            rospy.loginfo("...start to compute")
            rospy.sleep(1)

            # get the solution of ready to pick
            target_arm_joint_0_point_ready.point.x = target_arm_joint_0_point.point.x
            arm_length_ready, arm_joint_0_angle_ready = self.get_arm_length_and_arm_joint_0_angle(
                target_arm_joint_0_point_ready)
            # print("arm_joint_0_angle_ready",arm_joint_0_angle_ready)
            # print("arm_length_ready",arm_length_ready)

            arm_solution_ready, arm_solution_ready_is_succeed = self.get_arm_solution(
                arm_length_ready, target_arm_joint_0_point_ready, arm_joint_0_angle_ready, target_arm_joint_0_point.point.z)
            # rospy.logwarn(arm_solution_ready)
            # print('arm_solution_ready:',arm_solution_ready)

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
                rospy.loginfo(arm_solution)
                arm_solution_angle = arm_solution[1:5]
                plat_solution = arm_solution[0]

                if arm_solution_is_succeed == True:
                    rospy.logwarn("...finish computing the pick pose")
                    # pick
                    pick_trajectory = get_singel_pos_trajectory(
                        arm_solution_angle, self.joint_names)
                    pick_goal = get_goal_from_trajectory(pick_trajectory)
                    self.arm_client.send_goal(pick_goal)
                    self.arm_client.wait_for_result(rospy.Duration(60.0))
                    rospy.loginfo("...pick_goal")

                    plat_goal = get_gripper_or_plat_goal_from_position(
                        plat_solution)
                    self.plat_client.send_goal(plat_goal)
                    self.plat_client.wait_for_result(rospy.Duration(60.0))
                    rospy.loginfo("...plat down")
                    rospy.sleep(10)


                    close_goal = get_gripper_or_plat_goal_from_position(
                        gripper_close)
                    self.gripper_client.send_goal(close_goal)
                    self.gripper_client.wait_for_result(rospy.Duration(60.0))
                    rospy.loginfo("...close the gripper")
                    rospy.logwarn("...finish to pick")

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
            # some problem--xpp
            beta = math.asin(center_offset/gripper_length)

            gamma = math.atan(
                abs(target_arm_joint_0_point.point.x/target_arm_joint_0_point.point.y))

            alpha = PI/2 - beta - gamma

            arm_joint_0_angle = alpha

            #why--xpp
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
            rospy.logwarn("mode is 1")
            # get the alpha and beta
            while not get_solution:
                e2_length = (arm_length - e0 - e3 - e1 * math.sin(alpha))
                if e2_length >= e2:
                    alpha -= 0.005
                    if alpha > PI/2:
                        continue
                    else:
                        print('no arm solution ')
                        break
                # print("e2_length, e2",e2_length,e2)
                beta = math.asin(e2_length / e2)
                # print(math.asin(e2_length / e2),e2_length / e2)
                plat_position = target_base_heigh - (arm_0_1_h -e1 *math.cos(alpha) + e2 * math.cos(beta))
                # print(plat_position,"\n",target_base_heigh,"\n",arm_0_1_h,e1 *math.cos(alpha),"\n",e2 * math.cos(beta))
                if alpha > PI/2 and beta < PI/2 and plat_position > plat_up and plat_position < plat_down:
                    get_solution = True
                    rospy.logwarn("##########")
                    break
                else:
                    # if not alpha > PI/2:
                    #     print('alpha <= PI/2',alpha)
                    # if not beta < PI/2:
                    #     print('not beta < PI/2',beta)
                    # if not plat_position < plat_up:
                    #     print('not plat_position < plat_up',plat_position)
                    # if not plat_position > plat_down:
                    #     print('plat_position > plat_down',plat_position)
                    # print('alpha:',alpha,'beta:',beta,'plat_position:',plat_position)
                    alpha -= 0.005
                if alpha < PI/2:
                    alpha = PI
                    beta = 0
                    plat_position = 0
                    print('no arm solution ')
                    break
                # rospy.loginfo(alpha)
                # rospy.loginfo(beta)
                # rospy.logwarn(plat_position)
                # rospy.logwarn(get_solution)
            # while not get_solution:
            #     e2_length = (arm_length - e0 - e3 - e1 * math.sin(alpha))
            #     if e2_length > e2:
            #         alpha -= 0.05
            #         continue
            #     beta = math.asin(e2_length / e2)
            #     plat_position = target_base_heigh - arm_0_1_h + e1 * \
            #         math.cos(alpha) - e2 * math.cos(beta) - arm_base_0_h
            #     if alpha > PI/2 and beta < PI/2 and plat_position < plat_up and plat_position > plat_down:
            #         get_solution = True
            #         rospy.logwarn("################")
            #     else:
            #         alpha -= 0.05
            #     if alpha < PI/2:
            #         break
            #     rospy.loginfo(alpha)
            #     rospy.loginfo(beta)
            #     rospy.logwarn(plat_position)
                # rospy.logwarn(get_solution)

            arm_joint_1_angle = PI-0.2790-alpha
            arm_joint_2_angle = 0.5*PI - (PI-alpha + beta)-1.1550
            arm_joint_3_angle = -(0.5*PI - beta+0.0290)



        elif self.arm_solution_mode == 0:

            rospy.logwarn("mode is 0")

            alpha = 0
            beta = 0
            plat_position = 0
            get_solution = False  # to get the single
            # get the alpha and beta
            while not get_solution:
                e2_length = (arm_length - e0 - e3_doramen -
                             e1 * math.sin(alpha))
                print(e2_length)
                if alpha > PI:
                    rospy.logerr("too far")
                    return list()
                if e2_length > e2:
                    alpha += 0.1
                    continue
                beta = math.asin(e2_length / e2)
                plat_position = target_base_heigh - arm_0_1_h - e1 * \
                    math.cos(alpha) + e2 * math.cos(beta) - arm_base_0_h + e4
                print("***********")
                print(alpha)
                print(plat_position)
                if alpha + beta >= PI/2 and plat_position < plat_up and plat_position > plat_down:
                    get_solution = True
                else:
                    alpha += 0.1

            arm_joint_1_angle = PI/2 - alpha
            arm_joint_2_angle = alpha + beta - PI/2
            arm_joint_4_angle = beta

            arm_joint_3_angle = 0
            arm_joint_5_angle = PI

        elif self.arm_solution_mode == 2:
            rospy.logwarn("mode is 2")
            alpha = 0
            beta = 0
            plat_position = 0
            get_solution = False  # to get the single

            # get the alpha and beta
            while not get_solution:
                e2_length = (arm_length - e0 - e3 - e1 * math.sin(alpha))
                print(e2_length)
                if alpha > PI:
                    rospy.logerr("too far")
                    return list()
                if e2_length > e2:
                    alpha += 0.1
                    continue
                beta = math.asin(e2_length / e2)
                plat_position = target_base_heigh - arm_0_1_h - e1 * \
                    math.cos(alpha) + e2 * math.cos(beta) - arm_base_0_h
                print("***********")
                print(alpha)
                print(plat_position)
                if alpha + beta >= PI/2 and plat_position < plat_up and plat_position > plat_down:
                    get_solution = True
                else:
                    alpha += 0.1

            arm_joint_1_angle = PI/2 - alpha
            arm_joint_2_angle = alpha + beta - PI/2
            arm_joint_4_angle = beta

            arm_joint_3_angle = 0
            arm_joint_5_angle = 0

        elif self.arm_solution_mode == 3:
            # 这里假设alpha 与 beta 互余，但这不是一般情况
            alpha = 0
            beta = 0
            gamma = 0
            plat_position = 0
            get_solution = False  # to get the single
            rospy.logwarn("mode is 3")
            # get the alpha and beta

            while not get_solution:

                y = e3 * math.sin(gamma) - arm_0_1_offset
                x = math.sqrt(arm_length**2-y**2)

                tmp_x = x - e0 - e3 * math.cos(gamma)

                # 辅助角公式
                a = e2
                b = e1 - arm_4_5_offset
                tmp_angle = math.atan(b/a)

                sin_ = tmp_x / math.sqrt(a**2+b**2)
                if sin_ >= 1:
                    gamma += 0.1
                    continue

                beta = math.asin(tmp_x / math.sqrt(a**2+b**2)) - tmp_angle

                plat_position = target_arm_joint_0_point.point.z - \
                    arm_0_1_h - e1 * math.cos(alpha) + e2 * math.cos(beta)

                if plat_position < plat_up and plat_position > plat_down:
                    get_solution = True
                else:
                    gamma += 0.1

            alpha = PI/2 - beta
            arm_joint_0_angle -= math.atan(y/x)
            arm_joint_1_angle = PI/2 - alpha
            arm_joint_2_angle = alpha + beta - PI/2
            arm_joint_3_angle = gamma - PI/2
            arm_joint_4_angle = 0
            arm_joint_5_angle = -beta
        else:
            pass

        arm_solution = list()
        arm_solution.append(-plat_position)
        arm_solution.append(arm_joint_0_angle)
        arm_solution.append(arm_joint_1_angle)
        arm_solution.append(arm_joint_2_angle)
        arm_solution.append(arm_joint_3_angle)


        print(arm_joint_0_angle)

        return arm_solution, get_solution


if __name__ == "__main__":
    ArmStack()
