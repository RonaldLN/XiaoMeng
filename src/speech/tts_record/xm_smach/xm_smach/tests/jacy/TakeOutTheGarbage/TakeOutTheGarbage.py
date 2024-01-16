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
Take Out The Garbage
规则理解：
        到两个垃圾箱中取出垃圾袋，然后到达指定区域，一次性拾取两个垃圾袋
状态机:
        依次导航到两个垃圾桶的位置，摄像头观察，精确抓取，到达指定位置，两个动作扔下垃圾袋
图像：
        识别垃圾袋
问题:
        导航精度不够，单靠硬编程无法实现抓取，需要俯视，××摄像头需要云台××
        至于是否要抓取垃圾盖，暂不考虑
        机械抓将抓着第二个垃圾袋导航一段距离，要求垃圾袋不会在中途掉，如果有危险，可适当降低移动速度
        
'''


class PlaceBag(State):
    '''
    用于放置第一个垃圾袋
    '''
    def __init__(self):
        State.__init__(self, outcomes=['succeeded', 'aborted'])

    def execute(self, userdata):

        try:
            joints2 = [-0.6, -1.1387, 1.191, -1.3563, -1.57]
            lifting_controller_level(0.0789094045758)
            arm_controller_level(joints2)
            rospy.sleep(2)
            gripper_service_level(True)
            rospy.sleep(2)

            #导航姿态
            joints3 = [-2.49428339885e-05, -1.44365561008, -
                       0.0446576103568, 1, -1.57]
            lifting_controller_level(-0.0403053089976)
            arm_controller_level(joints3)
            return 'succeeded'
        except Exception, e:
            rospy.logerr(e)
            return 'aborted'

class Put_G2(State):
    '''
    用于放置第二个垃圾袋，在机械爪中
    '''
    def __init__(self):
        State.__init__(self, outcomes=['succeeded', 'aborted'])

    def execute(self, userdata):
        try:
            gripper_service_level(True)
            rospy.sleep(2)
            return 'succeeded'
        except Exception, e:
            rospy.logerr(e)
            return 'aborted'

class Put_G1(State):
    '''
    用于放置第一个垃圾袋
    '''
    def __init__(self):
        State.__init__(self, outcomes=['succeeded', 'aborted'])

    def execute(self, userdata):

        try:
            #向右转
            joints2 = [1.5177, -1.5700, -1.5700, 1.5472, 0]
            arm_controller_level(joints2)
            rospy.sleep(2)
            #下降
            lifting_controller_level(0.025)
            rospy.sleep(2)
            #向左转
            joints2 = [-1.5700, -1.5700, -1.5700, 1.5472, 0]
            arm_controller_level(joints2)
            rospy.sleep(2)
            return 'succeeded'
        except Exception, e:
            rospy.logerr(e)
            return 'aborted'


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
            if useradta.num <=3:
                userdata.num +=1
                return 'continue'
            elif userdata.num ==4:
                return 'succeeded'
            else:
                return 'error'


class TakeOutTheGarbage():
    def __init__(self):
        rospy.init_node('TakeOutTheGarbage_Smach')
        rospy.on_shutdown(self.shutdown)
        rospy.logerr('Welcome to TakeOutTheGarbage!!!')
        self.smach_bool = False

        self.sm_Pick = StateMachine(outcomes=['succeeded','aborted','error'],
                                            input_keys = ['current_obj'])
        with self.sm_Pick:
            self.sm_Pick.userdata.target_mode =0
            self.sm_Pick.userdata.objmode = -1
            self.sm_Pick.userdata.rec = 1.0
            StateMachine.add('RUNNODE_IMG',
                                RunNode_img(),
                                transitions = {'succeeded':'WAIT','aborted':'aborted'})
            StateMachine.add('WAIT',
                                Wait(),
                                transitions = {'succeeded':'FIND_OBJECT','error':'error'})
            
            self.sm_Pick.userdata.object_pos = PointStamped()
            self.sm_Pick.userdata.objmode = -1
            StateMachine.add('FIND_OBJECT',
                                FindObject(),
                                transitions ={'succeeded':'POS_JUSTFY','aborted':'aborted','error':'SPEAK'},
                                remapping ={'name':'current_obj','object_pos':'object_pos','objmode':'objmode'})
            
            #making the xm foreward the object may make the grasping task easier  
            self.sm_Pick.userdata.pose = Pose()
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
                                remapping ={'name':'current_obj','object_pos':'object_pos','objmode':'objmode'})
            self.sm_Pick.userdata.arm_mode_1 =3
            StateMachine.add('PICK',
                                ArmCmd(),
                                transitions ={'succeeded':'succeeded','aborted':'aborted','error':'error'},
                                remapping ={'arm_ps':'object_pos','mode':'arm_mode_1'})
            self.sm_Pick.userdata.sentences = 'xiao meng can not find the thing'
            StateMachine.add('SPEAK',
                                Speak(),
                                transitions={'succeeded':'succeeded','aborted':'aborted','error':'error'})


        #顶层状态机
        self.TAKEOUTTHEGARBAGE = StateMachine(outcomes = ['succeeded','aborted','error'])
        with self.TAKEOUTTHEGARBAGE:
            self.TAKEOUTTHEGARBAGE.userdata.pos_g1 = gpsr_target['speaker']['pos']#垃圾袋1号位置
            self.TAKEOUTTHEGARBAGE.userdata.pos_g2 = gpsr_target['speaker']['pos']#垃圾袋2号位置
            self.TAKEOUTTHEGARBAGE.userdata.pos_p = gpsr_target['speaker']['pos']#垃圾袋放置位置
            self.TAKEOUTTHEGARBAGE.userdata.object_name = 'garbage'
            
            StateMachine.add('NAV_G1',
                                NavStack(),
                                transitions = {'succeeded':'PICK_G1','aborted':'NAV_G1','error':'error'},
                                remapping = {'pos_xm':'pos_g1'})   
            StateMachine.add('PICK_G1',
                                self.sm_Pick,
                                transitions = {'succeeded':'PLACEBAG','aborted':'PICK_G1','error':'error'},
                                remapping = {'current_obj':'object_name'})                 
            StateMachine.add('PLACEBAG',
                                PlaceBag(),
                                transitions = {'succeeded':'NAV_G2','aborted':'PLACEBAG'})       
            StateMachine.add('NAV_G2',
                                NavStack(),
                                transitions = {'succeeded':'PICK_G2','aborted':'NAV_G2','error':'error'},
                                remapping = {'pos_xm':'pos_g2'})   
            StateMachine.add('PICK_G2',
                                self.sm_Pick,
                                transitions = {'succeeded':'NAV_P','aborted':'PICK_G2','error':'error'},
                                remapping = {'current_obj':'object_name'})   
            StateMachine.add('NAV_P',
                                NavStack(),
                                transitions = {'succeeded':'PUT_G2','aborted':'NAV_P','error':'error'},
                                remapping = {'pos_xm':'pos_p'})   
            StateMachine.add('PUT_G2',
                                Put_G2(),
                                transitions = {'succeeded':'PUT_G1','aborted':'PUT_G2'}) 
            StateMachine.add('PUT_G1',
                                Put_G1(),
                                transitions = {'succeeded':'succeeded','aborted':'PUT_G1'})  
        intro_server = IntrospectionServer('TAKEOUTTHEGARBAGE',self.TAKEOUTTHEGARBAGE,'/TAKEOUTTHEGARBAGE')
        intro_server.start()
        out = self.TAKEOUTTHEGARBAGE.execute()
        intro_server.stop()

  
    def shutdown(self):
        if self.smach_bool ==True:
            rospy.loginfo('smach succeeded')
        else:
            rospy.loginfo('smach error')

if __name__ == "__main__":
    try:
        TakeOutTheGarbage()
    except Exception,e:
        rospy.logerr(e)   
