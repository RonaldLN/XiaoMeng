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
Set The Table
规则理解：
        整齐的摆放餐具
        
状态机:
        1.打开抽屉
        2.抓取盘子放置桌子中间
        3.抓取餐巾放置在盘子左边
        4.抓取两种银器在盘子右边
        5.抓取碗或杯子在盘子上面
        6.关闭抽屉
        由于抓取盘子，碗等大物体过程中容易发生危险从而终止比赛，故在终止之前尽量得到最多的分数。
        根据效果可调整顺序
图像：
        各餐具的精确位置是要根据盘子的位置来定位的，故要准确识别并返回盘子的正中央位置，
        *************此外，多开一个服务类型，当调用这个服务时，返回摄像头画面中正中央的深度数据，******************
        用以精确控制开门或者开橱柜的抽屉时机器人与把手的相对距离
问题:
        根据调试以及各餐具的规格，事先在程序中确定其他餐具放置点与盘子中央的相对位置  
        铺设铺垫可能会有难度，暂时先不考虑
        同样需要摄像头俯视

'''

#用于记录各餐具与盘子的相对位置
relative_distance_list = {
    'napkin':{'x':0,'y':0},
    'knife':{'x':0,'y':0},
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


class SetTheTable():
    def __init__(self):
        rospy.init_node('SetTheTable_Smach')
        rospy.on_shutdown(self.shutdown)
        rospy.logerr('Welcome to SetTheTable!!!')
        self.smach_bool = False
        
        self.sm_OpenDrawer = StateMachine(outcomes =['succeeded','aborted','error'],
                                    input_keys =['name','position'])    
        with self.sm_OpenDrawer: 
            self.sm_OpenDrawer.userdata.name =''
            self.sm_OpenDrawer.userdata.target_mode =0
            self.sm_OpenDrawer.userdata.objmode = -1
            StateMachine.add('NAV',
                                NavStack(),
                                transitions ={'succeeded':'RUNNODE_IMG','aborted':'NAV','error':'error'},
                                remapping ={"pos_xm":'position'})
            StateMachine.add('RUNNODE_IMG',
                                RunNode_img(),
                                transitions = {'succeeded':'FIND_OBJECT','aborted':'RUNNODE_IMG'})

            self.sm_OpenDrawer.userdata.object_pos = PointStamped()
            StateMachine.add('FIND_OBJECT',
                                FindObject(),
                                transitions ={'succeeded':'POS_JUSTFY','aborted':'succeeded','error':'SPEAK'},
                                remapping ={'name':'name','object_pos':'object_pos','objmode':'objmode'})
            self.sm_OpenDrawer.userdata.pose = Pose()
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
                                transitions ={'succeeded':'PICK_JUS','aborted':'succeeded','error':'error'},
                                remapping ={'name':'name','object_pos':'object_pos','objmode':'objmode'})
            StateMachine.add('PICK_JUS' , 
                                PickJustfy(),
                                transitions = {'succeeded':'PICK','error':'error'},
                                remapping = {'name':'name',
                                            'object_pos':'object_pos'})
            self.sm_OpenDrawer.userdata.arm_mode_1 =1
            StateMachine.add('PICK',
                                ArmCmd(),
                                transitions ={'succeeded':'succeeded','aborted':'aborted','error':'error'},
                                remapping ={'arm_ps':'object_pos','mode':'arm_mode_1'})
            self.sm_OpenDrawer.userdata.sentences = 'xiao meng can not find the thing'
            StateMachine.add('SPEAK',
                                Speak(),
                                transitions={'succeeded':'succeeded','aborted':'aborted','error':'error'})


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
                self.sm_OpenDrawer.userdata.pose = Pose()
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
        self.SETTHETABLE = StateMachine(outcomes = ['succeeded','aborted','error'])
        with self.SETTHETABLE:
            self.SETTHETABLE.userdata.object_list = ['plate','napkin','knife','fork','bowl']
            self.SETTHETABLE.userdata.pos_cupboard = gpsr_target['speaker']['pos']
            self.SETTHETABLE.userdata.pos_table = gpsr_target['speaker']['pos']  
            self.SETTHETABLE.userdata.drawer = "drawer"

            StateMachine.add('OPENDRAWER',
                                self.sm_OpenDrawer,
                                transitions = {'succeeded':'GETOBJECT','aborted':'OPENDRAWER','error':'error'},
                                remapping = {'position':'pos_cupboard','name':'drawer'})  
            StateMachine.add('GETOBJECT',
                                GetObject(),
                                transitions = {'succeeded':'succeeded','continue':'SET','error':'error'},
                                remapping = {'object_lsit':'object_lsit','object':'object','x':'x','y':'y'})  
            StateMachine.add('SET',
                                self.sm_SetTable,
                                transitions = {'succeeded':'GETOBJECT','aborted':'SET','error':'error'},
                                remapping = {'object':'object','x':'x','y':'y','pos_cupboard':'pos_cupboard','pos_table':'pos_table'})                                   
                                                    

        intro_server = IntrospectionServer('SETTHETABLE',self.SETTHETABLE,'/SETTHETABLE')
        intro_server.start()
        out = self.SETTHETABLE.execute()
        intro_server.stop()

  
    def shutdown(self):
        if self.smach_bool ==True:
            rospy.loginfo('smach succeeded')
        else:
            rospy.loginfo('smach error')

if __name__ == "__main__":
    try:
        SetTheTable()
    except Exception,e:
        rospy.logerr(e)   
