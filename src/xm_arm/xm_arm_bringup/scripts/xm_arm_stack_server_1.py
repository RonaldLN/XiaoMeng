#!/usr/bin/env python
# encoding:utf8


import rospy, sys, select, termios, tty
import moveit_commander
import actionlib
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from sensor_msgs.msg import JointState
from control_msgs.msg import FollowJointTrajectoryAction, FollowJointTrajectoryGoal , GripperCommandAction , GripperCommandGoal
from copy import deepcopy
import math
from tf.transformations import euler_from_quaternion, quaternion_from_euler
import tf
from geometry_msgs.msg import *
from std_msgs.msg import *
from xm_msgs.msg import *
from xm_msgs.srv import *

PI = 3.1415926535
init_pose = [0,0,0,0,0,0]
straight = [0,0,1.57,0,1.57,0]
pick_pose = [0,1.55,-0.75,0,0,0]
pick_pose_lifting = -0.15#pick_pose还要把lifting角度改成-0.15
take_back_pose_1 = [0,1.4,-0.6,0,0,0]

gripper_open = 0.03
gripper_close = -0.03

plat_lift = 0.089
plat_down = -0.2949
center_offset = 0.00551
plat_up = 0.086
plat_down = -0.3195

e0 = 0.115
e1 = 0.23
e2 = 0.256
e3 = 0.228
e3_doramen = 0.228#0.0655
e4 = 0.05
arm_0_1_h = 0.204
arm_base_0_h = 0.8395
arm_0_1_offset = 0.014
arm_4_5_offset = 0.018

height = 0.95
distance = 0.70 #70
depth = 0.04


def get_singel_pos_trajectory(arm_pose,joint_names):
    
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

def get_goal_from_pos(arm_pose,joint_names):
    
    goal = FollowJointTrajectoryGoal()
    goal.trajectory = get_singel_pos_trajectory(arm_pose,joint_names)
    goal.goal_time_tolerance = rospy.Duration(0.0)

    return goal

def get_gripper_or_plat_goal_from_position(position):
    
    goal = GripperCommandGoal()
    goal.command.position = position

    return goal 

#TODO raise exception to get the err single

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

        self.joint_names = ["arm_joint_0", "arm_joint_1", "arm_joint_2","arm_joint_3", "arm_joint_4", "arm_joint_5"]	

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
        self.target_pose_pub = rospy.Publisher('target_pose', PoseStamped,queue_size=10)

        #************arm stack**********************#
        rospy.loginfo("start the arm_stack...")
        self.arm_stack_server = actionlib.SimpleActionServer('/xm_arm/arm_stack',xm_ArmStackAction,self.action_cb,False)
        self.arm_stack_server.start()
        rospy.loginfo("\033[1;32mArm Stack Start!\033[0m")

        rospy.spin()

    def action_cb(self,goal):
        rospy.loginfo("\033[1;33mxm arm stack get the goal\033[0m")
        #rospy.logwarn(goal)
        target_size_l = goal.target_size_l
        target_size_w = goal.target_size_w
        target_size_h = goal.target_size_h
        target_camera_point = goal.target_camera_point
        table_depth = goal.table_depth
        
        #easy transfer the target_camera_point to the target_camera_pose
        target_camera_pose = PoseStamped()
        target_camera_pose.header.frame_id = "kinect2_rgb_link"
        target_camera_pose.pose.position.x = target_camera_point.point.x
        target_camera_pose.pose.position.y = target_camera_point.point.y
        target_camera_pose.pose.position.z = target_camera_point.point.z

        #get the target_base_pose
        self.tf_listener.waitForTransform('base_footprint','kinect2_rgb_link',rospy.Time(),rospy.Duration(60.0))
        target_base_pose = self.tf_listener.transformPoint('base_footprint',target_camera_point)
        #Publish the target pose in the rviz
        for i in range(10):
            self.target_pose_pub.publish(target_camera_pose)
            rospy.sleep(0.01)

        #判断解决方式
        self.arm_solution_mode = 1

        #get the target_arm_joint_0_point
        self.tf_listener.waitForTransform('arm_link_0','base_footprint',rospy.Time(),rospy.Duration(60.0))
        target_arm_joint_0_point = self.tf_listener.transformPoint('arm_link_0',target_base_pose)

        if self.arm_solution_mode == 1:

            #open the gripper
            open_goal = get_gripper_or_plat_goal_from_position(gripper_open)
            self.gripper_client.send_goal(open_goal)
            self.gripper_client.wait_for_result(rospy.Duration(60.0))
            rospy.loginfo("...open the gripper")
	    
	    #put the plat down
	    plat_down = get_gripper_or_plat_goal_from_position(pick_pose_lifting)
	    self.plat_client.send_goal(plat_down)
	    self.gripper_client.wait_for_result(rospy.Duration(60.0))
	    rospy.loginfo("...put the plat down")

            #ready to pick  (准备姿势一)
            ready_pick_trajectory = get_singel_pos_trajectory(pick_pose,self.joint_names)
            ready_pick_goal = get_goal_from_trajectory(ready_pick_trajectory)
            self.arm_client.send_goal(ready_pick_goal)
            self.arm_client.wait_for_result(rospy.Duration(60.0))
            rospy.loginfo("...ready to pick") 
	    rospy.sleep(10)

            rospy.loginfo("start compute")
	    #get the solution of ready to pick(偏移多少合适呢？)（准备姿势二）
	    target_arm_joint_0_point_ready = target_arm_joint_0_point
	    target_arm_joint_0_point_ready.point.x = target_arm_joint_0_point.point.x+0.1#多少合适？
	    target_arm_joint_0_point_ready.point.z = target_arm_joint_0_point.point.z+0.02 #z的偏移没什么用，但是为了好理解先写上
	    arm_length_ready,arm_joint_0_angle_ready = self.get_arm_length_and_arm_joint_0_angle(target_arm_joint_0_point_ready)
	    rospy.loginfo(arm_length_ready)
	    rospy.loginfo("sss")
            arm_solution_ready = self.get_arm_solution(arm_length_ready,target_arm_joint_0_point_ready,arm_joint_0_angle_ready,target_base_pose.point.z+0.05)#多少合适？
            rospy.logwarn("...finish the compute the ready pose")
            rospy.loginfo(arm_solution_ready)
            arm_solution_angle_ready = arm_solution_ready[1:8]
            plat_solution_ready = arm_solution_ready[0]
	    #运动到准备姿势二
            lift_goal_ready = get_gripper_or_plat_goal_from_position(plat_solution_ready)
            self.plat_client.send_goal(lift_goal_ready)
            self.plat_client.wait_for_result(rospy.Duration(60.0))

	    pick_trajectory_ready = self.get_pick_trajectory(arm_solution_angle_ready,self.joint_names)
            pick_goal_ready = get_goal_from_trajectory(pick_trajectory_ready)
            self.arm_client.send_goal(pick_goal_ready)
            self.arm_client.wait_for_result(rospy.Duration(60.0))
	    #中间停顿作一下观察
	    rospy.sleep(10)

            #get the solution
            arm_length,arm_joint_0_angle = self.get_arm_length_and_arm_joint_0_angle(target_arm_joint_0_point)
            rospy.logwarn("SSS")
            arm_solution = self.get_arm_solution(arm_length,target_arm_joint_0_point,arm_joint_0_angle,target_base_pose.point.z)
            rospy.logwarn("...finish the compute")
            rospy.loginfo(arm_solution)
            arm_solution_angle = arm_solution[1:8]
            plat_solution = arm_solution[0]
            
            #plat lift,抓取之前到底应该升起多少合适？
            #lift_goal = get_gripper_or_plat_goal_from_position(plat_solution+0.03)
            #self.plat_client.send_goal(lift_goal)
            #self.plat_client.wait_for_result(rospy.Duration(60.0))
            #rospy.loginfo("...plat lift")

            #pick_goal
            pick_trajectory = get_singel_pos_trajectory(arm_solution_angle,self.joint_names)
            pick_goal = get_goal_from_trajectory(pick_trajectory)
            self.arm_client.send_goal(pick_goal)
            self.arm_client.wait_for_result(rospy.Duration(60.0))
            rospy.loginfo("...pick_goal")
	    rospy.sleep(5)
            #plat down(最后调整plat位置)
            plat_goal = get_gripper_or_plat_goal_from_position(plat_solution)
            self.plat_client.send_goal(plat_goal)
            self.plat_client.wait_for_result(rospy.Duration(60.0))
            rospy.loginfo("...plat down")
	    rospy.sleep(5)
            #close the gripper
            close_goal = get_gripper_or_plat_goal_from_position(gripper_close)
            self.gripper_client.send_goal(close_goal)
            self.gripper_client.wait_for_result(rospy.Duration(60.0))
            rospy.loginfo("...close the gripper")
	    rospy.sleep(5)
	    #重新回到准备姿态二
            self.plat_client.send_goal(lift_goal_ready)
            self.plat_client.wait_for_result(rospy.Duration(60.0))

            pick_goal_ready = get_goal_from_trajectory(pick_trajectory_ready)
            self.arm_client.send_goal(pick_goal)
            self.arm_client.wait_for_result(rospy.Duration(60.0))
	    rospy.sleep(5)
	    #下降plat以防挡着摄像头
	    plat_down = get_gripper_or_plat_goal_from_position(pick_pose_lifting)
	    self.plat_client.send_goal(plat_down)
	    self.gripper_client.wait_for_result(rospy.Duration(60.0))
	    rospy.loginfo("...put the plat down")

            #take back the arm
            take_back_goal = get_goal_from_pos(take_back_pose_1,self.joint_names)
            self.arm_client.send_goal(take_back_goal)
            self.arm_client.wait_for_result(rospy.Duration(60.0))
            rospy.loginfo("...take back the arm")

        else:
            pass
        
        


        #TODO check if get the ojbect

               
        self.arm_stack_server.set_succeeded()
        rospy.loginfo("\033[1;36mArm Stack action successful!\033[0m")



    def get_pick_trajectory(self,arm_solution_angle,joint_names):
        
        trajectory = JointTrajectory()
        trajectory.joint_names = joint_names

        pos_demo = JointTrajectoryPoint()
        pos_demo.positions = [0.0 for i in joint_names]
        pos_demo.velocities = [0.0 for i in joint_names]
        pos_demo.accelerations = [0.0 for i in joint_names]
        pos_demo.time_from_start = rospy.Duration(5.0)

        if self.arm_solution_mode ==1:
            #pos_1  end_pos,no joint_4
            pos_1 = deepcopy(pos_demo)
            pos_1.positions = [arm_solution_angle[0],arm_solution_angle[1],arm_solution_angle[2],0,0,0]
            trajectory.points.append(pos_1)

            #pos_2  end_pos, put down the joint_4
            pos_2 = deepcopy(pos_demo)
            pos_2.positions = arm_solution_angle
            trajectory.points.append(pos_2)
        else:
            rospy.logerr("no mode match")

        return trajectory

    def get_arm_length_and_arm_joint_0_angle(self,target_arm_joint_0_point):
        '''
        INPUT : target_arm_joint_0_point\n
        OUTPUT : arm_length,arm_joint_0_angle
        '''

        if self.arm_solution_mode == 0 or self.arm_solution_mode == 1 or self.arm_solution_mode == 2:
            gripper_length = math.sqrt(target_arm_joint_0_point.point.y**2+target_arm_joint_0_point.point.x**2)
            rospy.logwarn(gripper_length)
            arm_length = math.sqrt(gripper_length**2-center_offset**2)
            rospy.logwarn(arm_length)
            beta = math.asin(center_offset/gripper_length)

            gamma = math.atan(abs(target_arm_joint_0_point.point.x/target_arm_joint_0_point.point.y))

            alpha = PI/2 - beta - gamma

            arm_joint_0_angle = alpha
            
            if target_arm_joint_0_point.point.y > 0:
                arm_joint_0_angle = -arm_joint_0_angle

            
        elif self.arm_solution_mode ==3:
            arm_length = math.sqrt(target_arm_joint_0_point.point.y**2+target_arm_joint_0_point.point.x**2)
            arm_joint_0_angle = math.atan(abs(target_arm_joint_0_point.point.y/target_arm_joint_0_point.point.x))
            if target_arm_joint_0_point.point.y > 0:
                arm_joint_0_angle = -arm_joint_0_angle
        
        return arm_length,arm_joint_0_angle
            
        
    def get_arm_solution(self,arm_length,target_arm_joint_0_point,arm_joint_0_angle,target_base_heigh):
        '''
        INPUT ： arm_length，target_arm_joint_0_point,arm_joint_0_angle,target_base_heigh\n
        OUTPUT ： arm_solution
        '''
        #target_arm_joint_0_point need to be precise,we can not only use the tf change,some change maybe be down,especially for the moveing link


        if self.arm_solution_mode == 1:
            alpha = 0
            beta = 0
            plat_position = 0
            get_solution = False #to get the single 
            rospy.logwarn("mode is 1")
            #get the alpha and beta
            while not get_solution:
                e2_length = (arm_length - e0- e3- e1 * math.sin(alpha))
                if e2_length > e2:
                    alpha +=0.1
                    continue
                beta = math.asin( e2_length / e2 )
                plat_position =  target_base_heigh - arm_0_1_h - e1 * math.cos(alpha) + e2 * math.cos(beta) -arm_base_0_h
                #print("EEEEEEEEEEE")
                #print(alpha)
                #print(beta)
                #print(plat_position)
                #there is some limit here
	        #alpha + beta >= PI/2 ?????待定
                if alpha + beta >= PI/2 and plat_position < plat_up and plat_position > plat_down:
                    get_solution = True
                else:
                    alpha +=0.1


            arm_joint_1_angle =  PI/2 - alpha
            arm_joint_2_angle = alpha + beta - PI/2
            arm_joint_4_angle = beta
            
            arm_joint_3_angle = 0
            arm_joint_5_angle = 0

        elif self.arm_solution_mode == 0:
            
            rospy.logwarn("mode is 0")

            alpha = 0
            beta = 0
            plat_position = 0
            get_solution = False #to get the single 
            #get the alpha and beta
            while not get_solution:
                e2_length = (arm_length - e0- e3_doramen- e1 * math.sin(alpha))
                print(e2_length)
                if alpha > PI:
                    rospy.logerr("too far")
                    return list()
                if e2_length > e2:
                    alpha +=0.1
                    continue
                beta = math.asin( e2_length / e2 )
                plat_position =  target_base_heigh - arm_0_1_h - e1 * math.cos(alpha) + e2 * math.cos(beta) -arm_base_0_h + e4
                print("***********")
                print(alpha)
                print(plat_position)
                if alpha + beta >= PI/2 and plat_position < plat_up and plat_position > plat_down:
                    get_solution = True
                else:
                    alpha +=0.1


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
            get_solution = False #to get the single 
            
            #get the alpha and beta
            while not get_solution:
                e2_length = (arm_length - e0- e3- e1 * math.sin(alpha))
                print(e2_length)
                if alpha > PI:
                    rospy.logerr("too far")
                    return list()
                if e2_length > e2:
                    alpha +=0.1
                    continue
                beta = math.asin( e2_length / e2 )
                plat_position =  target_base_heigh - arm_0_1_h - e1 * math.cos(alpha) + e2 * math.cos(beta) -arm_base_0_h
                print("***********")
                print(alpha)
                print(plat_position)
                if alpha + beta >= PI/2 and plat_position < plat_up and plat_position > plat_down:
                    get_solution = True
                else:
                    alpha +=0.1


            arm_joint_1_angle = PI/2 - alpha
            arm_joint_2_angle = alpha + beta - PI/2
            arm_joint_4_angle = beta
            
            arm_joint_3_angle = 0
            arm_joint_5_angle = 0
        
        elif self.arm_solution_mode == 3:
            #这里假设alpha 与 beta 互余，但这不是一般情况
            alpha = 0
            beta = 0
            gamma = 0
            plat_position = 0
            get_solution = False #to get the single 
            rospy.logwarn("mode is 3")
            #get the alpha and beta
        
            while not get_solution:

                y = e3 *math.sin(gamma) - arm_0_1_offset
                x = math.sqrt(arm_length**2-y**2)
                
                tmp_x = x - e0 -e3 * math.cos(gamma)
                
                #辅助角公式
                a = e2
                b = e1 - arm_4_5_offset
                tmp_angle = math.atan(b/a)
                
                sin_ = tmp_x / math.sqrt(a**2+b**2)
                if sin_ >= 1:
                    gamma += 0.1
                    continue
                
                beta = math.asin(tmp_x / math.sqrt(a**2+b**2)) - tmp_angle
                
                plat_position =  target_arm_joint_0_point.point.z - arm_0_1_h - e1 *math.cos(alpha) + e2 * math.cos(beta)

                if  plat_position < plat_up and plat_position > plat_down:
                    get_solution = True
                else:
                    gamma +=0.1

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
        arm_solution.append(plat_position)
        arm_solution.append(arm_joint_0_angle)
        arm_solution.append(arm_joint_1_angle)
        arm_solution.append(arm_joint_2_angle)
        arm_solution.append(arm_joint_3_angle)
        arm_solution.append(arm_joint_4_angle)
        arm_solution.append(arm_joint_5_angle)
        
        print(arm_joint_0_angle)

        return arm_solution



if __name__ == "__main__":
    ArmStack()

    
    
