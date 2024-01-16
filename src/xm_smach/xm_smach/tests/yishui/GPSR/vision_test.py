#!/usr/bin/env python
# encoding:utf8
from os import WIFSTOPPED
import os
import rospy
from smach import StateMachine
from smach_ros import IntrospectionServer
from geometry_msgs.msg import *
import math
import subprocess
from std_srvs.srv import Empty, EmptyRequest, EmptyResponse

'''
Author: yishui
Date: 2022-06-28
LastEditTime: 2022-06-28 11:59:27
LastEditors: yishui
Description: Codes for GPSR
FilePath: xm_smach\tests\yishui
'''
        
class GetObjectNum(State):
    def __init__(self):
        State.__init__(self,
                       outcomes=['succeeded', 'aborted', 'error'],
                       input_keys=['target','current_task'],
                       output_keys=['count_num'])
        #self.speak_client = rospy.ServiceProxy("tts", xm_Speech_tts)
        #rospy.wait_for_service('handGesture')
        # self.nav_client = actionlib.SimpleActionClient(
        #     "move_base", MoveBaseAction)

        
        # self.cmd_vel = rospy.Publisher('/mobile_base/mobile_base_controller/cmd_vel',Twist,queue_size=1)
        # self.twist_xm = Twist()
        
        self.countObject_client = rospy.ServiceProxy('countObject', xm_count_object)
        

    def execute(self, userdata):
        try:
        #     subprocess.call(r"xterm -e python3 /home/xm/catkin_ws/src/xm_vision/src/scripts/mrsupw_detect_object.py",shell=True)
        #     rospy.sleep(2.0)
            a = subprocess.Popen(['python3',r'/home/xm/catkin_ws/src/xm_vision/src/scripts/GPSR/mrsupw_count_object.py','-d','true'] ,shell =False)
            with open(r"/home/xm/vision_pid/people_tracking.txt",'w+') as f:
                if f.read() == '':
                    print('-------------------')
                    print('pid',a.pid)
                    print('pid',a.pid)
                    print('pid',a.pid)
                    print('pid',a.pid)
                    print('pid',a.pid)
                    print('pid',a.pid)
                    print('-------------------')
                    f.write(str(a.pid))
                    print('hhh')
            rospy.sleep(2.0)
            print a.poll()
            if a.returncode != None:
                a.wait()
                return 'aborted'
        except:
             rospy.logerr('No param specifieWd')
             return 'error'
        
        rospy.loginfo('.................Count Object ^_^..........\n')
        try:
            req = xm_count_objectRequest()
            # obj=str(userdata.target)
            # L=obj.split("_")
            # if "three_" in obj:
            #     req.name = "_".join(L[2:])
            #     req.adj = "three_"+L[1]
            # elif L[0] in ["biggest","largest","thinnest","smallest","heaviest","lightest"]:
            #     req.name = "_".join(L[1:])
            #     req.adj = L[0]
            req.des = userdata.target[userdata.current_task]
            rospy.logwarn(req)
                
            self.countObject_client.wait_for_service(timeout=10.0)
            res = self.countObject_client.call(req)
            rospy.logwarn(res)
            userdata.count_num = res.num

            pid = get_pid("people_tracking")
            subprocess.Popen(['kill','-9',pid],shell=False)
            with open(r"/home/xm/vision_pid/people_tracking.txt",'w') as f:
                 f.write('')

            return 'succeeded'
            
        except Exception:
            rospy.logerr('e')
            return 'aborted'



def shutdown(self):
    if self.smach_bool ==True:
        rospy.loginfo('smach succeeded')
    else:
        rospy.loginfo('smach error')


if __name__ == "__main__":
    rospy.init_node("Gpsr_Smach")
    rospy.on_shutdown(shutdown)
    rospy.logerr("Welcome to GPSR!")
    smach_bool = False
    intro_server = IntrospectionServer('xm_gpsr',xm_GPSR,'/XM_ROOT')
    intro_server.start()
    out = xm_GPSR.execute()
    intro_server.stop()

     #顶层状态机
    xm_GPSR = StateMachine(outcomes = ['succeeded','aborted','error'])
    with xm_GPSR:
        xm_GPSR.userdata.current_task = -1 
        xm_GPSR.userdata.gesture = ''
        xm_GPSR.userdata.count_num = 0

        StateMachine.add('GET_OBJECT_NUM',
                                    GetObjectNum(),
                                    transitions ={'succeeded':'succeeded','aborted':'aborted','error':'error'},
                                    remapping = {'count_num':'count_num','gesture':'gesture','current_task':'current_task'}
                                    )
