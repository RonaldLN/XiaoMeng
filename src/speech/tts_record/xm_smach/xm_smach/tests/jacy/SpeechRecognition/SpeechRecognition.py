#!/usr/bin/env python
# encoding:utf8

import rospy
from xm_msgs.msg import *
from xm_msgs.srv import *
from smach import *
from smach_ros import *
from subprocess import *
from geometry_msgs.msg import *
#from xm_smach.gpsr_lib import *
#from xm_smach.common_lib import *
#from xm_smach.speech_reco_lib import *

from smach_common.speech import *
from smach_common.nav import *
from xm_smach.target_gpsr import gpsr_target
import math
import linecache
import datetime
import tf   
'''
开机->导航到指定位置/直接开机，地图修正(保证周围一圈没有障碍物)>  
->并发（听问题，记录+听音辩位 ）
->得到角度，旋转-》（图像矫正）
-》回答
-》重复五次
'''

#存储物体的属性
ojbect_list = {
    'milk':{'type':'drink','location':'bedroom'},
    'cola':{'type':'drink','location':'bedroom'},
    'sofa':{'type':'furniture','location':'livingroom'},
    'shelf':{'type':'furniture','location':'bedroom'}
}


class CheckTurn(State):
    '''
    用于判断循环的一个状态
    '''
    def __init__(self):
        State.__init__(self,outcomes =['succeeded','continue','error'],
                        io_keys =['num'])

    def execute(self,userdata):
        try:
            getattr(userdata, 'num')
        except:
            rospy.logerr('no param')
            return 'error'
        else :
            if userdata.num <=5:
                userdata.num +=1
                return 'continue'
            elif userdata.num ==6:
                return 'succeeded'
            else:
                return 'error'


class GetAngle(State):
    '''
    用于获取听音辨位获取的角度
    '''
    def __init__(self):
        State.__init__(self,outcomes =['succeeded','preempt','aborted'],
                        output_keys =['turn_pose'])
        self.client = rospy.ServiceProxy('/mobile_base/SoundSource',SoundSource)
        self.tf_listener = tf.TransformListener()
        self.angle =0.0
    def execute(self,userdata):
        try:
            self.client.wait_for_service(timeout=10.0)
            res = self.client.call("s")
            self.angle  = res.angle
            
            if abs(self.angle-404) <0.1:
                return 'aborted'

            if self.angle < 180:
                self.angle = -self.angle
            else:
                self.angle = 360-self.angle
            
            self.angle = self.angle* 3.1415926535/180

            #从角度得到四元数
            qs = QuaternionStamped()
            qs.header.frame_id = 'base_link'
            qs.quaternion = Quaternion(*quaternion_from_euler(0,0,self.angle))

            point = PointStamped()
            point.header.frame_id = 'base_link'
            
            #等待tf的信息
            self.tf_listener.waitForTransform('map', 'base_link',rospy.Time(), rospy.Duration(1))
        
            
            point = self.tf_listener.transformPoint('map',point)
            qs =self.tf_listener.transformQuaternion('map',qs)
            userdata.turn_pose = Pose(point.point,qs.quaternion)
            
            return 'succeeded'
        except Exception, e:
            rospy.logerr(e)
            return 'aborted'



class PushQuestion(State):
    '''
    将问题假如问题列表中    \n
    input_keys:     question    \n
    io_key:         question_list   \n
    '''
    def __init__(self):
        State.__init__(self,outcomes =['succeeded','aborted'],
                        input_keys =['question'],
                        io_keys = ['question_list'])

    def execute(self,userdata):
        try:
            userdata.question_list.append(userdata.question)
            rospy.logwarn(userdata.question_list)
            return 'succeeded'
        except Exception, e:
            rospy.logerr(e)
            return 'aborted'


# action    0 1         
# target    type        number
# name      target
#object     index       
#answer     question    


class GetQuestion(State):
    '''
    获取问题并返回答案
    '''
    def __init__(self):
        State.__init__(self,outcomes =['succeeded','aborted','error'],
                        output_keys =['answer'])

        self.client = rospy.ServiceProxy('xm_speech_meaning',xm_Speech_meaning)
        self.answer = ""
    def execute(self,userdata):
        try:
            self.client.wait_for_service(timeout=10)
        except:
            rospy.logerr('xm_speech_meaning service is error')
            return 'aborted'
        else :
            res = self.client.call(command=4)
            try:
                self.answer = res.target[0]
                rospy.logwarn(self.answer)
                userdata.answer = self.answer
                return 'succeeded'
            except Exception, e:
                rospy.logerr(e)
                return 'aborted'
            return 'succeeded'


'''
为了测试，将GetQuestion()改为调用7服务
'''
class SpeechRecognition():
    def __init__(self):
        rospy.init_node('speech_recognition')
        rospy.on_shutdown(self.shutdown)
        rospy.logwarn('Welcome to SpeechRecognition!!!')
        self.smach_bool = False

        #顶层状态机
        self.SPEECHRECOGNITION = StateMachine(outcomes = ['succeeded','aborted','error'])
        with self.SPEECHRECOGNITION:
            self.SPEECHRECOGNITION.userdata.sentences_ask = '/home/domistic/askme.wav'
            self.SPEECHRECOGNITION.userdata.sentences_game = '/home/domistic/play.wav'
            self.SPEECHRECOGNITION.userdata.num = 2
            StateMachine.add('START',
                                    SpeakWAV(),
                                    transitions={'succeeded':'GETANGLE','aborted':'aborted','error':'error'},
                                    remapping = {'filename':'sentences_game'}
                                    )
            
            StateMachine.add('GETANGLE',
                                    GetAngle(),
                                    transitions={'succeeded':'TURN','aborted':'GETANGLE','preempt':'error'},
                                    remapping = {'turn_pose':'turn_pose'}
                                    )                                
            StateMachine.add('TURN',
                                    NavStack(),
                                    transitions={'succeeded':'ANSWER_GAME','aborted':'aborted','error':'error'},
                                    remapping = {'pos_xm':'turn_pose'}
                                    )  
            StateMachine.add('ANSWER_GAME',
                                    SpeakWAV(),
                                    transitions={'succeeded':'GETQUESTION','aborted':'aborted','error':'error'},
                                    remapping = {'filename':'sentences_ask'}
                                    )  

            StateMachine.add('GETQUESTION',
                                        GetQuestion(),
                                        transitions={'succeeded':'ANSWER','aborted':'aborted','error':'error'},
                                        remapping = {'answer':'answer'}
                                        )  
            
            StateMachine.add('ANSWER',
                                    Speak(),
                                    transitions={'succeeded':'CHECKTURN','aborted':'aborted','error':'error'},
                                    remapping = {'sentences':'answer'}
                                    )     
            StateMachine.add('CHECKTURN',
                                        CheckTurn(),
                                        transitions={'succeeded':'succeeded','continue':'START','error':'error'},
                                        remapping = {'num':'num'}
                                        )  
        
        intro_server = IntrospectionServer('SPEECHRECOGNITION',self.SPEECHRECOGNITION,'/SPEECHRECOGNITION')
        intro_server.start()
        out = self.SPEECHRECOGNITION.execute()
        intro_server.stop()

    def shutdown(self):  
        if self.smach_bool ==True:
            rospy.loginfo('smach succeeded')
        else:
            rospy.loginfo('smach error')

if __name__ == "__main__":
    try:
        SpeechRecognition()
    except:
        rospy.logerr('test wrong!')



