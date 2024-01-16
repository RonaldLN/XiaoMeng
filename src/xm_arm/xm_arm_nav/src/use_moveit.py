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

GRIPPER_OPEN = [0.025]
GRIPPER_CLOSED = [-0.015]
GRIPPER_NEUTRAL = [0.00]

GRIPPER_JOINT_NAMES = ['active_finger_joint']

GRIPPER_EFFORT = [1.0]


class MoveItDemo:
    def __init__(self): 
        # 初始化move_group的API
        moveit_commander.roscpp_initialize(sys.argv)

        # 初始化ROS节点
        rospy.init_node('moveit_demo')
	
	# 初始化场景对象
        scene = PlanningSceneInterface()
        rospy.sleep(1)

        # Create a scene publisher to push changes to the scene
        self.scene_pub = rospy.Publisher('planning_scene', PlanningScene)
        
        # Create a publisher for displaying gripper poses
        self.gripper_pose_pub = rospy.Publisher('gripper_pose', PoseStamped)

	# 初始化需要使用move group控制的机械臂中的right_arm
        arm = MoveGroupCommander('arm')

	# 初始化move group中的right_gripper
        gripper = MoveGroupCommander('gripper')

	# 获取终端link的名称
        end_effector_link = arm.get_end_effector_link()
                
	# Create a dictionary to hold object colors
        self.colors = dict()

	# 设置位置（单位：米）和姿态（单位：弧度）的允许误差
        arm.set_goal_position_tolerance(0.05)
        arm.set_goal_orientation_tolerance(0.01)

	# 运动失败后允许重新规划（允许次数：10）
        arm.allow_replanning(True)
        arm.set_planning_time(5)

        # Set a limit on the number of pick attempts before bailing
        max_pick_attempts = 10
        
        # Set a limit on the number of place attempts
        max_place_attempts = 10

        rospy.sleep(2)

	# 移除场景中之前运行遗留的物体
	#scene.remove_attached_object(end_effector_link,'target')
	#scene.remove_attached_object('fake_Link', 'target')
        scene.remove_world_object('table')
        scene.remove_world_object('box1')
        scene.remove_world_object('box2')
        scene.remove_world_object('target')
        #scene.remove_world_object('tool')

        rospy.sleep(1)
	# 控制机器人回到默认位姿
        # arm.set_named_target('resting')
        arm.set_named_target('rest')
        arm.go()

        # Open the gripper to the neutral position
        gripper.set_joint_value_target(GRIPPER_NEUTRAL)
        gripper.go()


        rospy.sleep(1)
	# 设置桌面高度
        table_ground = 0.7

	# 设置table，box和tool的尺寸
        table_size = [0.2, 0.7, 0.01]
        box1_size = [0.05, 0.05, 0.05]
        box2_size = [0.05, 0.05, 0.15]
        target_size = [0.02, 0.01, 0.12]
	# 将table和box等加入场景中
        table_pose = PoseStamped()
        table_pose.header.frame_id = 'base_footprint'
        table_pose.pose.position.x = 0.8
        table_pose.pose.position.y = 0
        table_pose.pose.position.z = table_ground + table_size[2] / 2.0
        table_pose.pose.orientation.w = 1.0
        scene.add_box('table', table_pose, table_size)
        rospy.sleep(1)
        box1_pose = PoseStamped()
        box1_pose.header.frame_id = 'base_footprint'
        box1_pose.pose.position.x = 0.8
        box1_pose.pose.position.y = 0.1
        box1_pose.pose.position.z = table_ground + table_size[2] + box1_size[2] / 2.0
        box1_pose.pose.orientation.w = 1.0   
        #scene.add_box('box1', box1_pose, box1_size)
        rospy.sleep(1)
        box2_pose = PoseStamped()
        box2_pose.header.frame_id = 'base_footprint'
        box2_pose.pose.position.x = 0.8
        box2_pose.pose.position.y = -0.15
        box2_pose.pose.position.z = table_ground + table_size[2] + box2_size[2] / 2.0
        box2_pose.pose.orientation.w = 1.0   
        #scene.add_box('box2', box2_pose, box2_size)       
        rospy.sleep(1)
        # 放置target
        target_pose = PoseStamped()
        target_pose.header.frame_id = 'base_footprint'
        target_pose.pose.position.x = 0.8
        target_pose.pose.position.y = 0
        target_pose.pose.position.z = 0.8
        # target_pose.pose.position.z = table_ground + table_size[2] + target_size[2] / 2.0
	#target_pose.pose.orientation.x = 9.23133136722e-05
	#target_pose.pose.orientation.y =-6.38831626047e-05
	#target_pose.pose.orientation.z =4.15955675938e-05
        target_pose.pose.orientation.w =1.0
        scene.add_box('target', target_pose, target_size)
        rospy.sleep(1)

	# Specify a pose to place the target after being picked up
        place_pose = PoseStamped()
        place_pose.header.frame_id = 'base_footprint'
        place_pose.pose.position.x = 0.0
        place_pose.pose.position.y = -0.2
        place_pose.pose.position.z = table_ground + table_size[2] + target_size[2] / 2.0
        place_pose.pose.orientation.w = 1.0

	# Make the table red and the boxes orange
        self.setColor('table', 0.8, 0, 0, 1.0)
        self.setColor('box1', 0.8, 0.4, 0, 1.0)
        self.setColor('box2', 0.8, 0.4, 0, 1.0)
        
        # Make the target yellow
        self.setColor('target', 0.9, 0.9, 0, 1.0)
        
        # Send the colors to the planning scene
        self.sendColors()
	# Set the start state to the current state
        #right_arm.set_start_state_to_current_state()
        
        # Set the goal pose of the end effector to the stored pose
        #right_arm.set_pose_target(target_pose, end_effector_link)
	#right_arm.go()
	# Set the support surface name to the table object
        arm.set_support_surface_name('table')

        # Initialize the grasp pose to the target pose
        grasp_pose = target_pose
                
        # Generate a list of grasps
        grasps = self.make_grasps(grasp_pose, ['target'])

        # Publish the grasp poses so they can be viewed in RViz
        for grasp in grasps:
           self.gripper_pose_pub.publish(grasp.grasp_pose)
           rospy.sleep(0.2)

	# Track success/failure and number of attempts for pick operation
        result = None
        n_attempts = 0
        
        # Repeat until we succeed or run out of attempts
        while result != MoveItErrorCodes.SUCCESS and n_attempts < max_pick_attempts:
            n_attempts += 1
            rospy.loginfo("Pick attempt: " +  str(n_attempts))
            result = arm.pick('target', grasps)
            rospy.sleep(0.2)
        
        # If the pick was successful, attempt the place operation   
        if result == MoveItErrorCodes.SUCCESS:
                print('nice')
                result = None
                n_attempts = 0

 	    # Generate valid place poses
                places = self.make_places(place_pose)
            
            # Repeat until we succeed or run out of attempts
        while result != MoveItErrorCodes.SUCCESS and n_attempts < max_place_attempts:
                n_attempts += 1
                rospy.loginfo("Place attempt: " +  str(n_attempts))
                for place in places:
                    result = arm.place('target', place)
                    if result == MoveItErrorCodes.SUCCESS:
                        break
                rospy.sleep(0.2)
                
                if result != MoveItErrorCodes.SUCCESS:
                        rospy.loginfo("Place operation failed after " + str(n_attempts) + " attempts.")
        else:
            rospy.loginfo("Pick operation failed after " + str(n_attempts) + " attempts.")
                
	# 用attach来抓取物体
	# scene.attach_box('fake_Link', 'target', target_pose,target_size)

        # Return the arm to the "resting" pose stored in the SRDF file
        # arm.set_named_target('resting')
        arm.set_named_target('rest')
        arm.go()

        # Shut down MoveIt cleanly
        moveit_commander.roscpp_shutdown()
        
        # Exit the script
        moveit_commander.os._exit(0)


    # Get the gripper posture as a JointTrajectory
    def make_gripper_posture(self, joint_positions):
        #Initialize the joint trajectory for the gripper joints
        t = JointTrajectory()
        
        # Set the joint names to the gripper joint names
        t.joint_names = GRIPPER_JOINT_NAMES
        
        # Initialize a joint trajectory point to represent the goal
        tp = JointTrajectoryPoint()
        
        # Assign the trajectory joint positions to the input positions
        tp.positions = joint_positions
        
        # Set the gripper effort
        tp.effort = GRIPPER_EFFORT
        
        tp.time_from_start = rospy.Duration(1.0)
        
        # Append the goal point to the trajectory points
        t.points.append(tp)
        
        # Return the joint trajectory
        return t
    
    # Generate a gripper translation in the direction given by vector
    def make_gripper_translation(self, min_dist, desired, vector):
        # Initialize the gripper translation object
        g = GripperTranslation()
        
        # Set the direction vector components to the input
        g.direction.vector.x = vector[0]
        g.direction.vector.y = vector[1]
        g.direction.vector.z = vector[2]
        
        # The vector is relative to the gripper frame
        g.direction.header.frame_id ='gripper_ink'
        
        # Assign the min and desired distances from the input
        g.min_distance = min_dist
        g.desired_distance = desired
        
        return g


    # Generate a list of possible grasps
    def make_grasps(self, initial_pose_stamped, allowed_touch_objects):
        # Initialize the grasp object
        g = Grasp()
        
        # Set the pre-grasp and grasp postures appropriately
        g.pre_grasp_posture = self.make_gripper_posture(GRIPPER_OPEN)
        g.grasp_posture = self.make_gripper_posture(GRIPPER_CLOSED)
                
        # Set the approach and retreat parameters as desired
        g.pre_grasp_approach = self.make_gripper_translation(0.01, 0.1, [1.0, 0.0, 0.0])
        g.post_grasp_retreat = self.make_gripper_translation(0.01, 0.15, [1.0, 0.0, 0.0])
        
        # Set the first grasp pose to the input pose
        g.grasp_pose = initial_pose_stamped
    
        # Pitch angles to try
        pitch_vals = [0, 0.05, -0.05, 0.1, -0.1, 0.15, -0.15, 0.2, -0.2, 0.25, -0.25, 0.3, -0.3]
        
        # Yaw angles to try
        yaw_vals = [0]

        # A list to hold the grasps
        grasps = []

        # Generate a grasp for each pitch and yaw angle
        for y in yaw_vals:
            for p in pitch_vals:
                # Create a quaternion from the Euler angles
                q = quaternion_from_euler(0, p, y)
                
                # Set the grasp pose orientation accordingly
                g.grasp_pose.pose.orientation.x = q[0]
                g.grasp_pose.pose.orientation.y = q[1]
                g.grasp_pose.pose.orientation.z = q[2]
                g.grasp_pose.pose.orientation.w = q[3]
                
                # Set and id for this grasp (simply needs to be unique)
                g.id = str(len(grasps))
                
                # Set the allowed touch objects to the input list
                g.allowed_touch_objects = allowed_touch_objects
                
                # Don't restrict contact force
                g.max_contact_force = 0
                
                # Degrade grasp quality for increasing pitch angles
                g.grasp_quality = 1.0 - abs(p)
                
                # Append the grasp to the list
                grasps.append(deepcopy(g))
                
        # Return the list
        return grasps

    def make_places(self, init_pose):
        # Initialize the place location as a PoseStamped message
        place = PoseStamped()
        
        # Start with the input place pose
        place = init_pose
        
        # A list of x shifts (meters) to try
        x_vals = [0, 0.0025, 0.005, 0.0075, 0.01, 0.0125, 0.015,-0.0025, -0.005, -0.0075, -0.01, -0.0125, -0.015]
        
        # A list of y shifts (meters) to try
        y_vals = [0, 0.0025, 0.005, 0.0075, 0.01, 0.0125, 0.015,-0.0025, -0.005, -0.0075, -0.01, -0.0125, -0.015]
        
        pitch_vals = [0]
        
        # A list of yaw angles to try
        yaw_vals = [0]

        # A list to hold the places
        places = []
        
        # Generate a place pose for each angle and translation
        for y in yaw_vals:
            for p in pitch_vals:
                for y in y_vals:
                    for x in x_vals:
                        place.pose.position.x = init_pose.pose.position.x + x
                        place.pose.position.y = init_pose.pose.position.y + y
                        
                        # Create a quaternion from the Euler angles
                        q = quaternion_from_euler(0, p, y)
                        
                        # Set the place pose orientation accordingly
                        place.pose.orientation.x = q[0]
                        place.pose.orientation.y = q[1]
                        place.pose.orientation.z = q[2]
                        place.pose.orientation.w = q[3]
                        
                        # Append this place pose to the list
                        places.append(deepcopy(place))
        
        # Return the list
        return places

    # Set the color of an object
    def setColor(self, name, r, g, b, a = 0.9):
        # Initialize a MoveIt color object
        color = ObjectColor()
        
        # Set the id to the name given as an argument
        color.id = name
        
        # Set the rgb and alpha values given as input
        color.color.r = r
        color.color.g = g
        color.color.b = b
        color.color.a = a
        
        # Update the global color dictionary
        self.colors[name] = color

    # Actually send the colors to MoveIt!
    def sendColors(self):
        # Initialize a planning scene object
        p = PlanningScene()

        # Need to publish a planning scene diff        
        p.is_diff = True
        
        # Append the colors from the global color dictionary 
        for color in self.colors.values():
            p.object_colors.append(color)
        
        # Publish the scene diff
        self.scene_pub.publish(p)

    
if __name__ == "__main__":
        MoveItDemo()


