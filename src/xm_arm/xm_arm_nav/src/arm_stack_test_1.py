#!/usr/bin/env python

"""
    moveit_pick_and_place_demo.py - Version 0.1 2014-01-14
    
    Command the gripper to grasp a target object and move it to a new location, all
    while avoiding simulated obstacles.
    
    Created for the Pi Robot Project: http://www.pirobot.org
    Copyright (c) 2014 Patrick Goebel.  All rights reserved.

    This program is free software; you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation; either version 2 of the License, or
    (at your option) any later version.5
    
    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details at:
    
    http://www.gnu.org/licenses/gpl.html
"""

import rospy, sys
import moveit_commander
from geometry_msgs.msg import PoseStamped, Pose
from moveit_commander import MoveGroupCommander, PlanningSceneInterface
from moveit_msgs.msg import PlanningScene, ObjectColor
from moveit_msgs.msg import Grasp, GripperTranslation, MoveItErrorCodes

from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from tf.transformations import quaternion_from_euler
from copy import deepcopy

GROUP_NAME_ARM = 'arm'
REFERENCE_FRAME = 'base_footprint'

class MoveItDemo:
    def __init__(self):
        # Initialize the move_group API
        moveit_commander.roscpp_initialize(sys.argv)
        
        rospy.init_node('moveit_demo')
        
        # Use the planning scene object to add or remove objects
        scene = PlanningSceneInterface()
        
        # Create a scene publisher to push changes to the scene
        self.scene_pub = rospy.Publisher('planning_scene', PlanningScene)
        
        # Create a publisher for displaying gripper poses
        #self.gripper_pose_pub = rospy.Publisher('gripper_pose', PoseStamped)
        
        # Create a dictionary to hold object colors
        self.colors = dict()
                        
        # Initialize the move group for the right arm
        right_arm = MoveGroupCommander(GROUP_NAME_ARM)
        
        # Initialize the move group for the right gripper
        #right_gripper = MoveGroupCommander(GROUP_NAME_GRIPPER)
        
        # Get the name of the end-effector link
        end_effector_link = right_arm.get_end_effector_link()
 
        # Allow some leeway in position (meters) and orientation (radians)
        #right_arm.set_goal_position_tolerance(0.05)
        #right_arm.set_goal_orientation_tolerance(0.1)

        # Allow replanning to increase the odds of a solution
        right_arm.allow_replanning(True)
        
        # Set the right arm reference frame
        right_arm.set_pose_reference_frame(REFERENCE_FRAME)
        
        # Allow 5 seconds per planning attempt
        right_arm.set_planning_time(5)
        

        # Remove any attached objects from a previous session

        
        # Give the scene a chance to catch up    
        
        # Start the arm in the "resting" pose stored in the SRDF file
        right_arm.set_named_target('rest')
        right_arm.go()
        rospy.sleep(1)
	right_arm.set_named_target('wave')
        right_arm.go()
        rospy.sleep(2)
	right_arm.set_named_target('rest')
        right_arm.go()
        rospy.sleep(1)
	right_arm.set_named_target('watch')
        right_arm.go()
        rospy.sleep(2)
	right_arm.set_named_target('rest')
        right_arm.go()
        rospy.sleep(1)
	right_arm.set_named_target('grap')
        right_arm.go()
        rospy.sleep(2)
	right_arm.set_named_target('rest')
        right_arm.go()
        rospy.sleep(1)
        
  

if __name__ == "__main__":
    MoveItDemo()

    
