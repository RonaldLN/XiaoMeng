#!/usr/bin/env python
# encoding:utf8

# 用于语音、人物识别所用的类
# 用于对语音与人脸识别项目的辅助状态
# 创建于2018年3月2日 yzy
import rospy
from xm_msgs.msg import *
from xm_msgs.srv import *
from smach import State, StateMachine, UserData, Concurrence, Iterator
from smach_ros import IntrospectionServer
from xm_smach.gpsr_lib import *
from xm_smach.common_lib import *
from subprocess import *
from geometry_msgs.msg import *
from xm_smach.target_gpsr import gpsr_target
import math
import os


# 将识别数据转化成一句话
class GetSentences(State):
    def __init__(self):
        State.__init__(self,outcomes = ['succeeded','error'],
                            input_keys = ['people_condition'],
                            io_keys = ['sentences'])
    def execute(self,userdata):
        try:
            getattr(userdata, 'people_condition')
        except:
            rospy.logerr('there is no param transiformed')
            return 'error'
        else:
                    
            userdata.sentences = 'There are %d people , %d gentlemen and %d ladies.'%(userdata.people_condition['All'] , 
                                                                                        userdata.people_condition['Male'], 
                                                                                        userdata.people_condition['Female'])
            print userdata.sentences
            return 'succeeded'

#　收集识别数据记录在字典中
class CountPeople(State):
    def __init__(self):
        State.__init__(self,outcomes=['succeeded','aborted','error'],
                        io_keys=['people_condition'])

    def execute(self,userdata):
       # pid = get_pid('gender')
       # subprocess.call('kill '+str(pid[0]),shell=True)
        #subprocess.call('rosnode kill gender &',shell = True)
           
        ####subprocess.call('xterm -e rosrun xm_vision gender  &',shell = True)
        #os.sys('rosrun xm_vision gender')
        rospy.sleep(1.0)
        ret = os.access('/home/domestic/Vision/data/gender_and_pose/result.txt',os.R_OK)
        if ret == True:
            result = open("/home/domestic/Vision/data/gender_and_pose/result.txt","r")
                
            userdata.people_condition['All'] = int(result.readline())
            userdata.people_condition['Male'] = int(result.readline())
            userdata.people_condition['Female'] = int(result.readline())
            userdata.people_condition['seated'] = int(result.readline())
            userdata.people_condition['standing'] = int(result.readline())
            userdata.people_condition['wave'] = int(result.readline())
            # userdata.people_condition['seatedF'] = int(result.readline()[8:])
            # userdata.people_condition['standM'] = int(result.readline()[7:])
            # userdata.people_condition['standF'] = int(result.readline()[7:])
            # userdata.people_condition['seated_elder'] = int(result.readline()[13:])
            # userdata.people_condition['seated_adult'] = int(result.readline()[13:])
            # userdata.people_condition['seated_younger'] = int(result.readline()[15:])
            # userdata.people_condition['stand_elder'] = int(result.readline()[12:])
            # userdata.people_condition['stand_adult'] = int(result.readline()[12:])
            # userdata.people_condition['stand_younger'] = int(result.readline()[14:])

            print userdata.people_condition
            result.close()
            return 'succeeded'
        else:
            rospy.logerr('the file can not be opened')
            return 'aborted'

# class Get_angle_sql(State):
#     def __init__(self):
#         State.__init__(self, outcomes = ['succeeded','aborted'],
#                         output_keys=['angle'])
#         self.sql_proxy = rospy.ServiceProxy('xm_spl',xm_Spl)

#     def execute(self,userdata):
#         try:
            
#             self.angle = self.sql_proxy.call(True)
#             rospy.logwarn(self.angle.angle)
#             if self.angle.angle >180:
#                 angle = (360-self.angle.angle) * 3.1415926/180
#             else:
#                 angle = -self.angle.angle * 3.1415926/180
#             userdata.angle = angle
#             rospy.logwarn(angle)
#             return 'succeeded'
#         except Exception,e:
#             rospy.logerr(e)
#             return 'aborted'
class RunNode_Num(State):
    def __init__(self):
        State.__init__(self,outcomes=['succeeded','aborted'])
    
    def execute(self,userdata):
        try:
            #subprocess.Popen('xterm -e rosrun xm_vision gender  &',shell =True)
            #subprocess.Popen('./kinect2/build/main &' , shell = True)
            #os.system('./kinect2/build/main')
            #os.system('./kinect2/build/main')
            rospy.sleep(5.0)
        except Exception, e:
            rospy.logerr('people_tracking node error')
            rospy.logerr(e)
            return 'aborted'
        return 'succeeded'
