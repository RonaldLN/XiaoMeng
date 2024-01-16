#!/usr/bin/env python
# encoding:utf8
import rospy
from smach import StateMachine
from smach_ros import IntrospectionServer
from xm_smach.common_lib import *
from xm_smach.gpsr_lib import * 
from xm_smach.help_me_carry_lib import *
from xm_smach.target_gpsr import gpsr_target
from geometry_msgs.msg import *
import math
import subprocess
from xm_smach.pick_turn import PickTurn , IsTurn
from xm_smach.store_lib import *
'''
Where is This
规则理解：
        语音教机器人做奶昔，需要倒牛奶和糖在碗里，同时按顺序将3种水果放置
状态机:
        导航到柜台-》御银获取物体列表
语音：
        获取含有3种水果的列表（内含顺序）

问题:
        机械臂落点问题

'''
#存放水果相对位置
relative_distance_list = {
    'apple':{'x':0,'y':0},
    'banna':{'x':0,'y':0},
    'fork':{'x':0,'y':0},
    'bowl':{'x':0,'y':0},
}

class GetObject(State):
    '''
    用于获取物体清单里的物体并且删除
    判断任务是否完成的关键
    '''
    def __init__(self):
        State.__init__(self,outcomes =['succeeded','continue','error'],
                        io_keys =['object_list'],
                        output_keys= ['object','x','y'])

    def execute(self,userdata):
        try:
            getattr(userdata, 'object_list')
        except:
            rospy.logerr('no param')
            return 'error'
        else :
            if len(userdata.object_list) == 0:
                return 'succeeded'
            else:
                userdata.object = userdata.object_list.pop(0)
                userdata.x = relative_distance_list[userdata.object]['x']
                userdata.y = relative_distance_list[userdata.object]['y']
                return 'continue'



class GetFruits(State):
    '''
    语音获得水果列表的状态
    '''
    def __init__(self):
        State.__init__(self,outcomes =['succeeded','aborted','error'],
                        output_keys =['fruits'])
        self.client = rospy.ServiceProxy('xm_speech_meaning',xm_Speech_meaning)
    def execute(self,userdata):
        try:
            self.client.wait_for_service(timeout=10)
        except:
            rospy.logerr('xm_speech_meaning service is error')
            return 'aborted'
        else :
            res = self.client.call(command=3)
            rospy.logwarn(res)
            userdata.fruits = res.object
            return 'succeeded'
            

class SmoothieChef():
    def __init__(self):
        rospy.init_node('SmoothieChef_Smach')
        rospy.on_shutdown(self.shutdown)
        rospy.logerr('Welcome to SmoothieChef!!!')
        self.smach_bool = False

        self.sm_SetTable = StateMachine(outcomes =['succeeded','aborted','error'],
                                    input_keys =['object','x','y','pos_cupboard','pos_table']) 
        with self.sm_SetTable:

            self.meta_Pick = StateMachine(outcomes=['succeeded','aborted','error'],
                                                input_keys = ['object'])
            with self.meta_Pick:
                self.meta_Pick.userdata.target_mode =0
                self.meta_Pick.userdata.objmode = -1
                self.meta_Pick.userdata.rec = 1.0
                StateMachine.add('RUNNODE_IMG',
                                    RunNode_img(),
                                    transitions = {'succeeded':'WAIT','aborted':'aborted'})
                StateMachine.add('WAIT',
                                    Wait(),
                                    transitions = {'succeeded':'FIND_OBJECT','error':'error'})
                
                self.meta_Pick.userdata.object_pos = PointStamped()
                self.meta_Pick.userdata.objmode = -1
                StateMachine.add('FIND_OBJECT',
                                    FindObject(),
                                    transitions ={'succeeded':'POS_JUSTFY','aborted':'aborted','error':'SPEAK'},
                                    remapping ={'name':'object','object_pos':'object_pos','objmode':'objmode'})
                
                #making the xm foreward the object may make the grasping task easier  
                self.meta_Pick.userdata.pose = Pose()
                StateMachine.add('POS_JUSTFY',
                                    PosJustfy(),
                                    remapping={'object_pos':'object_pos','pose':'pose'},
                                    transitions={'succeeded':'NAV_TO','aborted':'aborted','error':'error'})
                StateMachine.add('NAV_TO',
                                    NavStack(),
                                    transitions ={'succeeded':'RUNNODE_IMG2','aborted':'NAV_TO','error':'error'},
                                    remapping ={"pos_xm":'pose'})
                StateMachine.add('RUNNODE_IMG2',
                                    RunNode_img(),
                                    transitions = {'succeeded':'FIND_AGAIN','aborted':'aborted'})                    
                StateMachine.add('FIND_AGAIN',
                                    FindObject(),
                                    transitions ={'succeeded':'PICK','aborted':'aborted','error':'SPEAK'},
                                    remapping ={'name':'object','object_pos':'object_pos','objmode':'objmode'})
                self.meta_Pick.userdata.arm_mode_1 =3
                StateMachine.add('PICK',
                                    ArmCmd(),
                                    transitions ={'succeeded':'succeeded','aborted':'aborted','error':'error'},
                                    remapping ={'arm_ps':'object_pos','mode':'arm_mode_1'})
                self.meta_Pick.userdata.sentences = 'xiao meng can not find the thing'
                StateMachine.add('SPEAK',
                                    Speak(),
                                    transitions={'succeeded':'succeeded','aborted':'aborted','error':'error'})
            
            self.meta_Place = StateMachine(outcomes=['succeeded','aborted','error'],
                                            input_keys = ['x','y'])
            with self.meta_Place:
                self.meta_Place.userdata.name = "plate"
                self.meta_Place.userdata.mode = 2
                self.meta_Place.userdata.objmode = 2
                StateMachine.add('FIND_OBJECT',
                                    FindObject(),
                                    transitions ={'succeeded':'POS_JUSTFY','aborted':'succeeded','error':'error'},
                                    remapping ={'name':'name','object_pos':'object_pos','objmode':'objmode'})
                self.meta_Place.userdata.pose = Pose()
                StateMachine.add('POS_JUSTFY',
                                    PosJustfy(),
                                    remapping={'object_pos':'object_pos','pose':'pose'},
                                    transitions={'succeeded':'NAV_TO','aborted':'aborted','error':'error'})
                StateMachine.add('NAV_TO',
                                    NavStack(),
                                    transitions ={'succeeded':'RUNNODE_IMG2','aborted':'NAV_TO','error':'error'},
                                    remapping ={"pos_xm":'pose'})
                StateMachine.add('RUNNODE_IMG2',
                                    RunNode_img(),
                                    transitions = {'succeeded':'FIND_AGAIN','aborted':'RUNNODE_IMG2'})                    
                StateMachine.add('FIND_AGAIN',
                                    FindObject(),
                                    transitions ={'succeeded':'PLACE','aborted':'succeeded','error':'error'},
                                    remapping ={'name':'name','object_pos':'object_pos','objmode':'objmode'})                                         
                StateMachine.add('PLACE',
                                    ArmCmd(),
                                    transitions = {'succeeded':'succeeded','aborted':'aborted','error':'error'})





            StateMachine.add('NAV_CUPBOARD',
                                NavStack(),
                                transitions ={'succeeded':'PICK','aborted':'NAV_CUPBOARD','error':'error'},
                                remapping ={"pos_xm":'pos_cupboard'})
            StateMachine.add('PICK',
                                self.meta_Pick,
                                transitions = {'succeeded':'NAV_TABLE','aborted':'PICK','error':'error'},
                                remapping = {'object':'object'}) 
            StateMachine.add('NAV_TABLE',
                                NavStack(),
                                transitions ={'succeeded':'PUT','aborted':'NAV_TABLE','error':'error'},
                                remapping ={"pos_xm":'pos_table'})        
            StateMachine.add('PUT',
                                self.meta_Place,
                                transitions = {'succeeded':'succeeded','aborted':'PUT','error':'error'},
                                remapping = {'x':'x','y':'y'})         
       #顶层状态机
        self.SMOOTHIECHEF = StateMachine(outcomes = ['succeeded','aborted','error'])
        with self.SMOOTHIECHEF:
            self.SMOOTHIECHEF.userdata.fruits = list()
            self.SMOOTHIECHEF.userdata.init_pos = gpsr_target['speaker']['pos']
            self.SMOOTHIECHEF.userdata.sentences_ask = 'What is the order of the three fruits?'
            self.SMOOTHIECHEF.userdata.pos_cupboard = gpsr_target['speaker']['pos']
            self.SMOOTHIECHEF.userdata.pos_table = gpsr_target['speaker']['pos']  
            
            StateMachine.add('NAV_INIT',
                                NavStack(),
                                transitions = {'succeeded':'ASK','aborted':'NAV_INIT','error':'error'},
                                remapping = {'pos_xm':'init_pos'})   
            StateMachine.add('ASK',
                                Speak(),
                                transitions={'succeeded':'GETFRUITS','aborted':'aborted','error':'error'},
                                remapping = {'sentences':'sentences_ask'}
                                )
            StateMachine.add('GETFRUITS',
                                GetFruits(),
                                transitions={'succeeded':'GETOBJECT','aborted':'aborted','error':'error'},
                                remapping = {'fruits':'fruits'}
                                )
            StateMachine.add('GETOBJECT',
                                GetObject(),
                                transitions = {'succeeded':'succeeded','continue':'SET','error':'error'},
                                remapping = {'object_lsit':'fruits','object':'object','x':'x','y':'y'}) 
            StateMachine.add('SET',
                                self.sm_SetTable,
                                transitions = {'succeeded':'GETOBJECT','aborted':'SET','error':'error'},
                                remapping = {'object':'object','x':'x','y':'y','pos_cupboard':'pos_cupboard','pos_table':'pos_table'})                                   






                                                    

        intro_server = IntrospectionServer('SMOOTHIECHEF',self.SMOOTHIECHEF,'/SMOOTHIECHEF')
        intro_server.start()
        out = self.SMOOTHIECHEF.execute()
        intro_server.stop()

  
    def shutdown(self):
        if self.smach_bool ==True:
            rospy.loginfo('smach succeeded')
        else:
            rospy.loginfo('smach error')

if __name__ == "__main__":
    try:
        SmoothieChef()
    except Exception,e:
        rospy.logerr(e)   
