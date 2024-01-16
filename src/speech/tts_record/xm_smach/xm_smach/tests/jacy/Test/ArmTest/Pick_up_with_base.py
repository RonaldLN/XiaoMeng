#!/usr/bin/env python
# encoding:utf8
'''
Author: jacy
Date: 2020-08-10 20:15:44
LastEditTime: 2020-09-20 09:43:08
LastEditors: Please set LastEditors
Description: 底盘不移动，联合摄像头进行机械臂抓取,假设已经移动到适合抓取的位置
FilePath: /undefined/home/jacy/catkin_ws/src/xm_smach/tests/jacy/ArmTest/Pick_up_without_base.py
'''
from smach_common.common import *
import rospy
from smach import *
from tf.transformations import euler_from_quaternion, quaternion_from_euler
import math
import subprocess
from smach_ros import *
import actionlib
from xm_msgs.srv import *
from xm_msgs.msg import *
from geometry_msgs.msg import *
import tf
from xm_smach.target_gpsr import gpsr_target



class Pick_up():
    def __init__(self):
        rospy.init_node('Pick_up_without_move')
        rospy.on_shutdown(self.shutdown)
        self.smach_bool = False
        

        self.Pick = StateMachine(outcomes =['succeeded','aborted','error'])
        
        with self.Pick:
            self.Pick.userdata.target_name ='cola'  
            self.Pick.userdata.table_depth = 0.04
            self.Pick.userdata.traget_size = [0.04, 0.065, 0.105]
            self.Pick.userdata.observe_pose = gpsr_target['bookcase']['pos']
            self.Pick.userdata.distance = 0.9
            self.Pick.userdata.pick_pose = [ [0,-1.3,1.57,0,-1.3,0]  ]
            self.Pick.userdata.take_back_pose = [ [0,-1.3,1.57,0,-1.3,0] ]
            self.Pick.userdata.back_distance = -0.05
            """
            StateMachine.add('NAVTOOBSERVEPOS',
                                NavStack(),
                                transitions ={'succeeded':'PICK_POS','aborted':'NAVTOOBSERVEPOS','error':'error'},
                                remapping ={"pos_xm":'observe_pose'})

            StateMachine.add('PICK_POS',ArmTrajectory(),
                             transitions={'succeeded': 'FIND_1', 'error': 'error'},
                             remapping={'arm_waypoints':'pick_pose'}) 
            """
            StateMachine.add('FIND_1',GetObjectPosition(),
                             transitions={'succeeded': 'GETPICKPOSE', 'error': 'error'})

            StateMachine.add('GETPICKPOSE',
                                GetPickPos(),
                                transitions ={'succeeded':'NAVTOPICKPOS','error':'error'})       
            
            StateMachine.add('NAVTOPICKPOS',
                                NavStack(),
                                transitions ={'succeeded':'FIND_2','aborted':'NAVTOPICKPOS','error':'error'},
                                remapping ={"pos_xm":'pick_pos'})

            StateMachine.add('FIND_2',GetObjectPosition(),
                             transitions={'succeeded': 'TARGET_POINT_JUSFY', 'error': 'error'})   
                                                               
            StateMachine.add('TARGET_POINT_JUSFY',TargerPosReload(),
                             transitions={'succeeded': 'PICK', 'error': 'error'})

            StateMachine.add('PICK', ArmStack(),
                             transitions={'succeeded': 'NAV_BACK','error': 'error'})
            
            #TODO back to the observer pose
            StateMachine.add('NAV_BACK', GoAhead(),
                             transitions={'succeeded': 'BACK_POS','error': 'error'},
                             remapping={'move_len':'back_distance'})
            #TODO arm back pose
            StateMachine.add('BACK_POS',ArmTrajectory(),
                             transitions={'succeeded': 'succeeded', 'error': 'error'},
                             remapping={'arm_waypoints':'take_back_pose'}) 

        intro_server = IntrospectionServer('sm_Pick' , self.Pick , '/SM_ROOT')
        intro_server.start()
        out = self.Pick.execute()
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
	    Pick_up()
    except Exception , e:
        rospy.logerr(e)
