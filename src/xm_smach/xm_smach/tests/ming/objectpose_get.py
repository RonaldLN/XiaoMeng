#!/usr/bin/env python
# encoding:utf8
from os import WIFSTOPPED
import os
from signal import set_wakeup_fd
from telnetlib import SE
import rospy
from smach import StateMachine
from smach_ros import IntrospectionServer
from smach_special.gpsr import *
from smach_special.garbage import *
from smach_special.HandMeThat_lib3 import *
from smach_compose.compose import *
from smach_common.common import *
from xm_smach.target_gpsr import gpsr_target

from geometry_msgs.msg import *
import math
import subprocess
from std_srvs.srv import Empty, EmptyRequest, EmptyResponse


class object_get():

    def __init__(self):
        rospy.init_node("Gpsr_Smach")
        rospy.on_shutdown(self.shutdown)
        rospy.logerr("Welcome to object_get!")
        # empty pid file
        dir_path = '/home/xm/vision_pid/'
        for file in os.listdir(dir_path):
            with open(os.path.join(dir_path, file), 'w') as f:
                f.write('')
        print('Empty PID Files Success!')
        self.smach_bool = False

        self.xm_HandMeTHat = StateMachine(outcomes=['succeeded', 'aborted', 'error'])
        with self.xm_HandMeTHat:
            self.xm_HandMeTHat.userdata.name ='person'
            StateMachine.add('FIND_2',
                             GetObjectPosition(),
                             transitions={'succeeded': 'succeeded', 'error': 'error'},
                             remapping={'target':'name'})

        intro_server = IntrospectionServer('xm_HandMeThat', self.xm_HandMeTHat, '/XM_ROOT')
        intro_server.start()
        out = self.xm_HandMeTHat.execute()
        intro_server.stop()
        if out == 'succeeded':
            self.smach_bool = True

    def shutdown(self):
        if self.smach_bool == True:
            rospy.loginfo('smach succeeded')
        else:
            rospy.loginfo('smach error')

if __name__ == "__main__":
    try:
        object_get()
    except:
        rospy.logerr(e)