#!/usr/bin/env python
# encoding:utf8
'''
Author: jacy
Date: 2020-09-06 16:15:37
LastEditTime: 2020-09-08 22:06:51
LastEditors: Please set LastEditors
Description: In User Settings Edit
FilePath: /undefined/home/jacy/gazebo_test_ws/src/xm_smach/xm_smach/tests/jacy/Test/CameraTest/follow.py
'''

import rospy
from smach import StateMachine
from smach_ros import IntrospectionServer
from smach_common.common import *
from xm_smach.target_gpsr import gpsr_target
from geometry_msgs.msg import *
import math
import subprocess
from control_msgs.msg import *
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

#------------------------------for camera-------------------------------------------#
class NAV():
    def __init__(self):
        
        rospy.init_node('NAV')
        rospy.on_shutdown(self.shutdown)
        self.smach_bool = False
        

        self.nav = StateMachine(outcomes =['succeeded','aborted','error'])
        self.nav.userdata.pos_1 = gpsr_target['livingroom']['pos']
        self.nav.userdata.pos_2 = gpsr_target['livingroom_endtable']['pos']
        self.nav.userdata.pos_3 = gpsr_target['livingroom_couch']['pos']
        self.nav.userdata.pos_4 = gpsr_target['living_room_bookcase']['pos']
        self.nav.userdata.pos_5 = gpsr_target['receptionist_pose']['pos']

        self.nav.userdata.pos_6 = gpsr_target['bedroom']['pos']
        self.nav.userdata.pos_7 = gpsr_target['bedroom_bed']['pos']
        self.nav.userdata.pos_8 = gpsr_target['bedroom_desk']['pos']
        self.nav.userdata.pos_9 = gpsr_target['bedroom_side_table']['pos']

        self.nav.userdata.pos_10 = gpsr_target['kitchen']['pos']
        self.nav.userdata.pos_11 = gpsr_target['kitchen_storage_table']['pos']
        self.nav.userdata.pos_12 = gpsr_target['kitchen_counter']['pos']
        self.nav.userdata.pos_13 = gpsr_target['kitchen_sink']['pos']
        self.nav.userdata.pos_14 = gpsr_target['kitchen_dishwasher']['pos']
        self.nav.userdata.pos_15 = gpsr_target['kitchen_cupboard']['pos']

        self.nav.userdata.pos_16 = gpsr_target['diningroom']['pos']
        self.nav.userdata.pos_17 = gpsr_target['exit_pos']['pos']
        #self.nav.userdata.pos_5 = gpsr_target['put_down']['pos']
        
        with self.nav:
            StateMachine.add('NAV1',NavStack(),
                             transitions={'succeeded': 'NAV2','aborted':'NAV1','error': 'error'},
                             remapping={'pos_xm':'pos_1'})                            
            StateMachine.add('NAV2',NavStack(),
                             transitions={'succeeded': 'NAV3','aborted':'NAV2','error': 'error'},
                             remapping={'pos_xm':'pos_2'}) 
            StateMachine.add('NAV3',NavStack(),
                             transitions={'succeeded': 'NAV5','aborted':'NAV3','error': 'error'},
                             remapping={'pos_xm':'pos_3'}) 
            StateMachine.add('NAV4',NavStack(),
                             transitions={'succeeded': 'NAV5','aborted':'NAV4','error': 'error'},
                             remapping={'pos_xm':'pos_4'}) 
            StateMachine.add('NAV5',NavStack(),
                             transitions={'succeeded': 'NAV6','aborted':'NAV4','error': 'error'},
                             remapping={'pos_xm':'pos_5'}) 
            StateMachine.add('NAV6',NavStack(),
                             transitions={'succeeded': 'NAV7','aborted':'NAV4','error': 'error'},
                             remapping={'pos_xm':'pos_6'}) 
            StateMachine.add('NAV7',NavStack(),
                             transitions={'succeeded': 'NAV8','aborted':'NAV4','error': 'error'},
                             remapping={'pos_xm':'pos_7'}) 
            StateMachine.add('NAV8',NavStack(),
                             transitions={'succeeded': 'NAV9','aborted':'NAV4','error': 'error'},
                             remapping={'pos_xm':'pos_8'}) 
            StateMachine.add('NAV9',NavStack(),
                             transitions={'succeeded': 'NAV10','aborted':'NAV4','error': 'error'},
                             remapping={'pos_xm':'pos_9'}) 
            StateMachine.add('NAV10',NavStack(),
                             transitions={'succeeded': 'NAV11','aborted':'NAV4','error': 'error'},
                             remapping={'pos_xm':'pos_10'}) 
            StateMachine.add('NAV11',NavStack(),
                             transitions={'succeeded': 'NAV12','aborted':'NAV4','error': 'error'},
                             remapping={'pos_xm':'pos_11'}) 
            StateMachine.add('NAV12',NavStack(),
                             transitions={'succeeded': 'NAV13','aborted':'NAV4','error': 'error'},
                             remapping={'pos_xm':'pos_12'}) 
            StateMachine.add('NAV13',NavStack(),
                             transitions={'succeeded': 'NAV14','aborted':'NAV4','error': 'error'},
                             remapping={'pos_xm':'pos_13'}) 
            StateMachine.add('NAV14',NavStack(),
                             transitions={'succeeded': 'NAV15','aborted':'NAV4','error': 'error'},
                             remapping={'pos_xm':'pos_14'}) 
            StateMachine.add('NAV15',NavStack(),
                             transitions={'succeeded': 'NAV16','aborted':'NAV4','error': 'error'},
                             remapping={'pos_xm':'pos_15'}) 
            StateMachine.add('NAV16',NavStack(),
                             transitions={'succeeded': 'NAV17','aborted':'NAV4','error': 'error'},
                             remapping={'pos_xm':'pos_16'}) 
            StateMachine.add('NAV17',NavStack(),
                             transitions={'succeeded': 'succeeded','aborted':'NAV4','error': 'error'},
                             remapping={'pos_xm':'pos_17'}) 
            

        intro_server = IntrospectionServer('sm_nav' , self.nav , '/SM_ROOT')
        intro_server.start()
        out = self.nav.execute()
        print out
        intro_server.stop()
        self.smach_bool =True

    def shutdown(self):
        if self.smach_bool == True:
            rospy.logwarn("DONE")
        else:
            rospy.logwarn('FUCK THE ERROE')


if __name__ == '__main__':
    try:
	    NAV()
    except Exception , e:
        rospy.logerr(e)
