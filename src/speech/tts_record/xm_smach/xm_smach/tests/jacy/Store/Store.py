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
Store
        
状态机:
        导航到架子-》拉开门-》面对架子-》分析架子中的物体，并按顺序返回每个类别的代表物体的位置
        -》导航到桌子-》图像识别最左边第一个物体的类别，返回类别和位置-》放到架子上-》   X5
        
图像节点:
        分析架子中的物体，并按顺序返回每个类别的代表物体的位置
        代表物体是指可以直接返回每个类别的物体中最靠右的，如果太靠右，则最靠左的为代表物体

问题:
    图像那边返回可以放置的物体的点是一个问题


'''


class Get_position(State):
    '''
    返回放置位置
    '''

    def __init__(self):
        State.__init__(self,outcomes=['succeeded','error'],
                        input_keys = ['position_list','current_obj'],
                        output_keys = ['position'])
    
    def execute(self,userdata):
        try:
            getattr(userdata,'position_list')
            self.name = userdata.current_obj
        except:
            rospy.logerr('no params specified')
            return 'error'
        else:
            if self.name =='juice':
                inedx = 0
            else:
                index = 1
            userdata.position = userdata.position_list[index]
            return 'succeeded'


class Scan(State):
    '''
    分析架子中的物体，并按顺序返回每个类别的代表物体的位置
    目前假设返回可以放置的所有位置
    '''
    def __init__(self):
        State.__init__(self,
                       outcomes=['succeeded', 'aborted', 'error'],
                       output_keys=['position_list'])
        self.xm_findobject = rospy.ServiceProxy(
            '/get_position', xm_ObjectDetect)
        self.tf_listener = tf.TransformListener()

    def execute(self, userdata):
        goal = Point()
        try:
            subprocess.Popen(
                "xterm -e rosrun xm_vision object_detect.py &", shell=True)
        except:
            rospy.logerr('No param specified')
            return 'error'
        for i in range(5):
            try:
                self.xm_findobject.wait_for_service(timeout=30.0)
                req = xm_ObjectDetectRequest()
                res = self.xm_findobject.call(req)
            except Exception, e:
                rospy.logerr(e)
                return 'aborted'
        rospy.logwarn(res.object[0])
        if i == 2:
            rospy.logerr('result wrong')
            return 'aborted'
        
        for i in range(len(res.object)):
        #   object_pos 是 PointStamped 类型
            object_pos = PointStamped()
            object_pos.point.x = res.object[i].pos.point.z - 0.11
            object_pos.point.y = res.object[i].pos.point.x - 0.125
            object_pos.point.z = 0.917-res.object[i].pos.point.y
            object_pos.header.frame_id = 'base_link'
            userdata.position_list.append(object_pos)
       # 将物品的位置信息传输到userdata
        
        try:
            pid_str = subprocess.check_output('ps -aux | grep object_detect.py' , shell= True)
            pid_str1 = pid_str.splitlines()[0].split()[0]
            rospy.logwarn(pid_str1)
            subprocess.call('kill '+pid_str1 , shell = True)


        except Exception,e:
            rospy.logerr('No such process ')
            return 'succeeded'
        return 'succeeded'


class Store():
    def __init__(self):
        rospy.init_node('Stroe_Smach')
        rospy.on_shutdown(self.shutdown)
        rospy.logerr('Welcome to Stroe!!!')
        self.smach_bool = False


        self.table_rec = StateMachine(outcomes=['succeeded','aborted','error'],
                                            output_keys=['object_list'])                             
        with self.table_rec:
            self.table_rec.userdata.rec = 25.0
            self.table_rec.userdata.pos_table = gpsr_target['speaker']['pos']
            StateMachine.add('NAV',
                                NavStack(),
                                transitions ={'succeeded':'RUNNODE','aborted':'NAV','error':'error'},
                                remapping ={"pos_xm":'pos_table'})
            StateMachine.add('RUNNODE',
                                RunNode_obj(),
                                transitions = {'succeeded':'WAIT','aborted':'aborted'})
            StateMachine.add('WAIT',
                                Wait(),
                                transitions = {'succeeded':'GETLIST','error':'error'})
            StateMachine.add('GETLIST',
                                GetObject_list(),
                                transitions = {'succeeded':'succeeded','aborted':'aborted'},
                                remapping = {'object_list':'object_list'})


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
        
        self.sm_Place = StateMachine(outcomes=['succeeded','aborted','error'],
                                        output_keys = ['position_list'],
                                        input_keys = ['current_obj','position_list'])
        with self.sm_Place:
            self.sm_Place.userdata.mode = 2
            self.sm_Place.userdata.objmode = 2
            StateMachine.add('GET_POSITION',
                                Get_position(),
                                transitions = {'succeeded':'POS_JUSTFY','error':'error'},
                                remapping = {'position':'position'})
            StateMachine.add('POS_JUSTFY',
                                PosJustfy(),
                                remapping={'position':'position','pose':'pose'},
                                transitions={'succeeded':'NAV_TO','aborted':'aborted','error':'error'})
            StateMachine.add('NAV_TO',
                                NavStack(),
                                transitions ={'succeeded':'PLACE','aborted':'NAV_TO','error':'error'},
                                remapping ={"pos_xm":'pose'})                                
            
            StateMachine.add('PLACE',
                                ArmCmd(),
                                transitions = {'succeeded':'succeeded','aborted':'aborted','error':'error'})

        self.grasp_from_list = StateMachine(outcomes=['succeeded','aborted','error'],
                                                input_keys = ['object_list'])
        with self.grasp_from_list:
            self.grasp_from_list.userdata.current_task = 0
            self.grasp_from_list.userdata.degree = 3.1415
            self.grasp_from_list.userdata.pos_xm_place = gpsr_target['speaker']['pos'] #放置位置
            self.grasp_from_list.userdata.pos_xm_pick = gpsr_target['speaker']['pos']  #抓取位置
            self.grasp_from_list.userdata.pos_xm_place = Pose(Point(1.65, 1.34 , 0),Quaternion(0,0,0,1))
            self.grasp_from_list.userdata.position_list = {1:PointStamped(Header(frame_id = 'base_link'),Point( -0.300 , 0.750 , 0.670 )),
                                                           2:PointStamped(Header(frame_id = 'base_link'),Point( -0.300 , 0.750 , 0.355 )),
                                                           3:PointStamped(Header(frame_id = 'base_link'),Point( -0.300 , 0.750 , 0.040 )),
                                                           4:PointStamped(Header(frame_id = 'base_link'),Point( -0.300 , 0.750 , 0.000 ))}
            StateMachine.add('GETOBJ',
                                Get_object(),
                                transitions = {'succeeded':'GRASP','finish':'succeeded','error':'error'},
                                remapping = {'current_obj':'current_obj'})
            StateMachine.add('GRASP',
                                self.sm_Pick,
                                transitions={'succeeded':'MOVE_PLACE','aborted':'MOVE_PICK2','error':'error'})
            StateMachine.add('MOVE_PLACE',
                                NavStack(),
                                transitions={'succeeded':'PLACE','aborted':'MOVE_PLACE','error':'error'},
                                remapping = {'pos_xm':'pos_xm_place'})
            StateMachine.add('PLACE',
                                self.sm_Place,
                                transitions={'succeeded':'MOVE_PICK2','aborted':'MOVE_PICK2'})

            StateMachine.add('MOVE_PICK2',
                                NavStack(),
                                transitions={'succeeded':'GETOBJ','aborted':'MOVE_PICK2','error':'error'},
                                remapping = {'pos_xm':'pos_xm_pick'})


        self.sm_OpenDoor = StateMachine(outcomes =['succeeded','aborted','error'],
                                    input_keys =['name','position'])    
        with self.sm_OpenDoor: 
            self.sm_OpenDoor.userdata.name =''
            self.sm_OpenDoor.userdata.target_mode =0
            self.sm_OpenDoor.userdata.objmode = -1
            StateMachine.add('NAV',
                                NavStack(),
                                transitions ={'succeeded':'RUNNODE_IMG','aborted':'NAV','error':'error'},
                                remapping ={"pos_xm":'position'})
            StateMachine.add('RUNNODE_IMG',
                                RunNode_img(),
                                transitions = {'succeeded':'FIND_OBJECT','aborted':'RUNNODE_IMG'})

            self.sm_OpenDoor.userdata.object_pos = PointStamped()
            StateMachine.add('FIND_OBJECT',
                                FindObject(),
                                transitions ={'succeeded':'POS_JUSTFY','aborted':'succeeded','error':'SPEAK'},
                                remapping ={'name':'name','object_pos':'object_pos','objmode':'objmode'})
            self.sm_OpenDoor.userdata.pose = Pose()
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
            self.sm_OpenDoor.userdata.arm_mode_1 =1
            StateMachine.add('PICK',
                                ArmCmd(),
                                transitions ={'succeeded':'succeeded','aborted':'aborted','error':'error'},
                                remapping ={'arm_ps':'object_pos','mode':'arm_mode_1'})
            self.sm_OpenDoor.userdata.sentences = 'xiao meng can not find the thing'
            StateMachine.add('SPEAK',
                                Speak(),
                                transitions={'succeeded':'succeeded','aborted':'aborted','error':'error'})


        #顶层状态机
        self.STORE = StateMachine(outcomes = ['succeeded','aborted','error'])
        with self.STORE:
            self.STORE.userdata.pos_shelf = gpsr_target['speaker']['pos']#抓取架子的位置
            self.STORE.userdata.shelf = "shelf"
            self.STORE.userdata.object_pos_list = list()
            StateMachine.add('OPENSHELF',
                                self.sm_OpenDoor,
                                transitions={'succeeded':'SCAN','aborted':'aborted','error':'error'},
                                remapping ={'name':'shelf','position':'pos_shelf'}
                                )
            StateMachine.add('SCAN',
                                Scan(),
                                transitions={'succeeded':'RECO','aborted':'aborted','error':'error'},
                                remapping ={'position_list':'position_list'}
                                )            
            StateMachine.add('RECO',
                                self.table_rec,
                                transitions={'succeeded':'GRASP_FROM_LIST','aborted':'aborted','error':'error'},
                                remapping={'object_list':'object_list'} )     
            StateMachine.add('GRASP_FROM_LIST',
                                self.grasp_from_list,
                                transitions={'succeeded':'succeeded','aborted':'aborted','error':'error'},
                                remapping={'object_list':'object_list'})

            
        intro_server = IntrospectionServer('STORE',self.STORE,'/SM_ROOT')
        intro_server.start()
        out = self.STORE.execute()
        intro_server.stop()

    def shutdown(self):
        if self.smach_bool ==True:
            rospy.loginfo('smach succeeded')
        else:
            rospy.loginfo('smach error')


if __name__ == "__main__":
    try:
        Store()
    except Exception,e:
        rospy.logerr(e)   
