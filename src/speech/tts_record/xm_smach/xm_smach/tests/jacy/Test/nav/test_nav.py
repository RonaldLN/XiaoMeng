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
        self.nav.userdata.pos_1 = gpsr_target['receptionist']['pos']
        self.nav.userdata.pos_2 = gpsr_target['dining_room']['pos']
        self.nav.userdata.pos_3 = gpsr_target['kitchen']['pos']
        self.nav.userdata.pos_4 = gpsr_target['exit_pos']['pos']
        with self.nav:
            StateMachine.add('NAV1',NavStack(),
                             transitions={'succeeded': 'NAV2','aborted':'aborted','error': 'error'},
                             remapping={'pos_xm':'pos_1'})
            StateMachine.add('NAV2',NavStack(),
                             transitions={'succeeded': 'NAV3','aborted':'aborted','error': 'error'},
                             remapping={'pos_xm':'pos_2'})
            StateMachine.add('NAV3',NavStack(),
                             transitions={'succeeded': 'NAV4','aborted':'aborted','error': 'error'},
                             remapping={'pos_xm':'pos_3'})
            StateMachine.add('NAV4',NavStack(),
                             transitions={'succeeded': 'succeeded','aborted':'aborted','error': 'error'},
                             remapping={'pos_xm':'pos_4'})                             

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