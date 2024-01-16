#! /usr/bin/env python
# encoding:utf8

import rospy
from pi_trees_lib.pi_trees_lib import *
from pi_trees_ros.pi_trees_ros import *
from pi_trees_ros.xm_tree_lib import *
from xm_msgs.srv import *
from xm_smach.target_gpsr import gpsr_target
#import xm_speech.srv
from xm_msgs.msg import *
from geometry_msgs.msg import *
from time import sleep
from math import *
import tf
from tf.transformations import quaternion_from_euler, euler_from_quaternion
import subprocess
import actionlib
from actionlib_msgs.msg import GoalStatus
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal, MoveBaseActionFeedback
from std_msgs.msg import Bool, Header
from std_srvs.srv import Empty, EmptyRequest, EmptyResponse
import os
from xm_arm_nav.xm_arm_controller_level import arm_controller_level, lifting_controller_level, gripper_service_level


'''
GPSR
Date:2020/1/17
'''

class GPSR():
    def __init__(self):
        rospy.init_node('Gpsr_Tree')
        #rospy.on_shutdown(self.shutdown)
        rospy.logwarn('Welcome to Gpsr!!!')
        self.turn=3

        #顶层行为树-INIT-ENTER->LOOP->LEAVE
        GPSR=Iterator("GPSR")
        
        #INIT=Init("INIT")

        #ENTER:DOORDETECT->MOVE_IN
        ENTER=Sequence("ENTER")
        #DOORDETECT=DoorDetect("DOORDETECE")
        self.start_point  = gpsr_target['speaker']['pos']
        MOVE_IN=Nav("MOVE_IN",self.start_point)

        #ENTER.add_child(DOORDETECT)
        ENTER.add_child(MOVE_IN)
        
        #LEAVE
        self.leave_point  = gpsr_target['init_pose']['pos']
        LEAVE=Nav("LEAVE")

        #LOOP
        LOOP=Loop("LOOP",iterations=self.turn)

        #GPSR
        #GPSR.add_child(INIT)
        GPSR.add_child(ENTER)
        GPSR.add_child(LOOP)
        GPSR.add_child(LEAVE)

        #LOOP
        MISSION=Iterator("MISSION")

        LOOP.add_child(MISSION)

        #MISSION:
        self.sentence_gm = 'give me the mission please'
        SPEAK_GM=Speak("SPEAK_GM",self.sentence_gm)
        RECEIVE=GetTask("RECEVE")
          

        DO_LOOP=Loop("DO_LOOP",iterations=-1)
        speaker_point  = gpsr_target['speaker']['pos']
        BACK=Nav("BACK",speaker_point)

        MISSION.add_child(SPEAK_GM)
        MISSION.add_child(RECEIVE)
        MISSION.add_child(DO_LOOP)
        MISSION.add_child(BACK)

        DO=Sequence("DO")

        DO_LOOP.add_child(DO)

        #DO
        ACTION=Selector("ACTION")
        GETTARGET=GetTarget("GETTARGET",1)

        DO.add_child(GETTARGET)
        DO.add_child(ACTION)

        #ACTION
        
        #NAV
        NAV=Sequence("NAV")
            
        CHECK_IF_GO=Check_Action("CHECK_IF_GO",'go')
         
        GO=Nav("GO")
        self.sentence_ar='I have arrive here'
        SPEAK_AR=Speak("SPEAK_AR",self.sentence_ar)
        
        NAV.add_child(CHECK_IF_GO)
        NAV.add_child(GO)
        NAV.add_child(SPEAK_AR)

        #TALK
        TALK=Sequence("TALK")

        CHECK_IF_TALK=Check_Action("CHECK_IF_TALK",'talk')
        SPEAK_TALK=Speak_S("SPEAK_TALK")

        TALK.add_child(CHECK_IF_TALK)
        TALK.add_child(SPEAK_TALK)

        #PICK_DOWN
        PICK_DOWN=Sequence("PUT_DOWN")

        CHECK_IF_PD=Check_Action("CHECK_IF_PD",'place')
        PD=ArmCmd("PD",2)

        PICK_DOWN.add_child(CHECK_IF_PD)
        PICK_DOWN.add_child(PD)

        #PICK_UP
        PICK_UP=Sequence("PICK_UP")

        CHECK_IF_PU=Check_Action("CHECK_IF_PU",'pick')
        RUNNODE_PU_1=RunNode("RUNNODE_PU_1",'image_test &')
        FIND_FIRST=FindObject("FIND_FIRST")
        POS_JUSTFY=PosJustfy("POS_JUSTFY")
        NAV_PU_X=Nav("NAV_PU_X")
        RUNNODE_PU_2=RunNode("RUNNODE_PU_2",'image_test &')
        FIND_SECOND=FindObject("FIND_SECOND")
        PICK_JUS=PickJustfy("PICK_JUS")
        PICK=ArmCmd("PICK",1)

        PICK_UP.add_child(CHECK_IF_PU)
        PICK_UP.add_child(RUNNODE_PU_1)
        PICK_UP.add_child(FIND_FIRST)
        PICK_UP.add_child(POS_JUSTFY)
        PICK_UP.add_child(NAV_PU_X)
        PICK_UP.add_child(RUNNODE_PU_2)
        PICK_UP.add_child(FIND_SECOND)
        PICK_UP.add_child(PICK_JUS)
        PICK_UP.add_child(PICK)


        #...

        #ACTION
        ACTION.add_child(NAV)
        ACTION.add_child(TALK)
        ACTION.add_child(PICK_DOWN)
        ACTION.add_child(PICK_UP)


        #ACTION.add_child(SUCCESS)
        
        print "GPSR Tree\n"
        print_tree(GPSR)

        rospy.loginfo("GPSR test")
            
        # Run the tree
        while not rospy.is_shutdown():
            GPSR.run()
            rospy.sleep(0.1)
    
if __name__ == '__main__':
    try:
        GPSR()
    except rospy.ROSInterruptException:
        rospy.loginfo("GPSR test finished.")