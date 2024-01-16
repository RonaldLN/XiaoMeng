#!/usr/bin/env python
# encoding:utf8
import rospy
from smach import *
from smach_ros import IntrospectionServer
#from xm_smach.common_lib import *
#from xm_smach.gpsr_lib import * 
#from xm_smach.help_me_carry_lib import *
from xm_smach.target_gpsr import gpsr_target
from geometry_msgs.msg import *
import math
import subprocess
from xm_smach.pick_turn import PickTurn , IsTurn

from new_lib.basic_vision import *
from new_lib.basic_voice import *
from new_lib.basic_move import *
from new_lib.basic_pick import *
from new_lib.special import *
from new_lib.basic_voice import *

#对于顶层状态机GPSR，
#SPEAK-->LISTEN-->GET_TASK-->ACTION-->BACK-->CHECK_TURN-->BYE
#ACTION-->GET_TASK
#CHECK_TURN-->LISTEN
#通过speak进行交互，然后listen接受一组命令，在这一组命令中，通过GET_TASK分解任务，分解给各种action执行
#一组命令结束后，回到讲话人附近，判断是否完成所有循环，如果完成，BYEBYE

#PLACE=GO+PICK_UP+GO+PICK_DOWN
#PICK=GO+PICK_UP
#GO:
#TALK
#FIND:前提是find的目标在附近
#FOLLOW：前提是follow的目标在前方

class FindPeople():
    def __init__(self):
        self.find_people_ = MonitorState('follow',
                                        xm_FollowPerson,
                                        self.people_cb,
                                        max_checks =5,
                                        output_keys=['pos_xm'])

        self.tf_listener = tf.TransformListener()

# 如果相机找到人,这个状态将会返回False
# 相反,如果在五个循环里相机都没找到人,将会返回True
# msg传入主题的数据
    def people_cb(self,userdata,msg):
        if msg is not None:
            try:
                self.tmp_pos = msg.position
                rospy.logwarn(self.tmp_pos)
            #如果得到人的坐标信息返回移动的位置
                if self.get_distance(self.tmp_pos) >= 0.5 and self.get_distance(self.tmp_pos)<=2.0 :
                    rospy.loginfo('i will move')
                    ps =self.data_deal(self.tmp_pos)
                    userdata.pos_xm = ps
                    return False
                elif self.get_distance(self.tmp_pos) < 0.5:
                    rospy.loginfo('i will not move')
                    ps = self.data_deal_turn(self.tmp_pos)
                    userdata.pos_xm = ps
                    return False
                else:
                    rospy.logerr('the person is out of the range')
                    return True
            except:
                rospy.logerr(e)
                return False
            
        else:
            raise Exception('MsgNotFind')
    def get_distance(self,pos_xm):
        person_x = pos_xm.point.z
        person_y = pos_xm.point.x

        return  hypot(person_x,person_y)
    def data_deal_turn(self,pos_xm):
        #图像到导航的坐标转换
        person_x = pos_xm.point.z
        person_y = pos_xm.point.x
        #计算人和xm连线与视线正前方夹角
        angle = atan2(person_y,person_x)
        #初始化xm现在的位置用于之后得到base_link在全局坐标系中的位置
        pos_xm.point.x = 0
        pos_xm.point.y = 0
        pos_xm.point.z = 0
        new_header = Header()
        new_header.frame_id = 'base_link'
        pos_xm.header = new_header
        #从角度得到四元数
        q_angle = quaternion_from_euler(0, 0, angle)
        self.q = Quaternion(*q_angle)
        qs = QuaternionStamped()
        qs.header = pos_xm.header
        qs.quaternion = self.q
        rospy.logwarn(qs)
        #等待tf的信息
        self.tf_listener.waitForTransform('map', 'base_link',rospy.Time(), rospy.Duration(60.0))
        rospy.logwarn('get the tf message')
        #利用tf信息转化坐标
        pos_xm = self.tf_listener.transformPoint('map',pos_xm)

        rospy.logwarn('tf point succeeded ')
        #qs是一个四元数
        qs =self.tf_listener.transformQuaternion('map',qs)

        rospy.logwarn('tf quaternion succeeded ')

        ps = Pose(pos_xm.point,qs.quaternion)
        return ps
#返回xm经过处理后的Pose()
    def data_deal(self,pos_xm):
        # 这个方法简单地处理来自cv的数据,将数据从PointStmp()类型转换到Pose()类型
        # 由于我们改变了camera_link 的坐标,所以数据处理可能没有跟着改变
        person_x = pos_xm.point.z
        person_y = pos_xm.point.x
        
        #计算方位角
        angle = atan2(person_y, person_x)
        #这里是为了到人的面前进行问题回答
#        person_x = person_x - (hypot(person_x,person_y)-0.2)*cos(angle)
#        person_y = person_y - (hypot(person_x,person_y)-0.2)*sin(angle)
        person_x = person_x - 0.6*cos(angle)
        person_y = person_y - 0.6*sin(angle)
        pos_xm.point.x = person_x
        pos_xm.point.y =person_y
        pos_xm.point.z =0

        # init the stamped of the Header
        # 初始化Header
        new_header =Header()
        new_header.frame_id = 'base_link'
        pos_xm.header = new_header

        #一个四元数描述了旋转轴和旋转角度
        #绕z轴旋转angle角度
        #这个函数从欧拉旋转（绕x轴旋转角，绕y轴旋转角，绕z轴旋转角）变换到四元数表示旋转
        #对给定的旋转轴(a,b,c)和一个角度theta对应四元数
        #q = (a*sin(theta/2), b*sin(theta/2), c*sin(theta/2), cos(theta))
        q_angle = quaternion_from_euler(0, 0, angle)
        self.q = Quaternion(*q_angle)
        rospy.loginfo(self.q)
        qs = QuaternionStamped()
        qs.header = pos_xm.header
        qs.quaternion = self.q
        self.tf_listener.waitForTransform('map','base_link',rospy.Time(),rospy.Duration(60.0))    
    
        rospy.logwarn('wait for tf succeeded ')    

        #pos_xm是一个Point()
        pos_xm = self.tf_listener.transformPoint('map',pos_xm)

        rospy.logwarn('tf point succeeded ')    

        #qs是一个四元数
        qs =self.tf_listener.transformQuaternion('map',qs)

        rospy.logwarn('tf quaternion succeeded ')

        ps = Pose(pos_xm.point,qs.quaternion)
        return ps

class Anwser(State):
    def __init__(self):
        State.__init__(self, outcomes=['succeeded', 'aborted'],
                       input_keys=['people_condition'])
        self.anwser_client = rospy.ServiceProxy(
            'xm_speech_meaning', xm_Speech_meaning)
        self.speak_client = rospy.ServiceProxy("tts", xm_Speech_tts)

    def execute(self, usedata):
        # try:

        #     subprocess.call('xterm -e rosrun xm_speech xm_speech_client_18_4.py &', shell=True)
        #     # res =self.anwser_client.call(command =1)
        # except:cmd
        #     rospy.logerr('call the client error')
        #     return 'aborted'
        # else:

        self.anwser_client.wait_for_service(timeout=10)
        # gpsr 1  help_me 5    speech 4  who 3
        self.rec = self.anwser_client.call(command=1)
        self.string_ = ''
        self.peo_con = usedata.people_condition
        if self.rec.num > 1:
            # if self.rec.num == 2:
            #     self.string_ = str(self.peo_con['seatedM'])
            # elif self.rec.num == 3:
            #     self.string_ = str(self.peo_con['seatedF'])
            # elif self.rec.num == 4:
            #     self.string_ = str(self.peo_con['standM'])
            # elif self.rec.num == 5:
            #     self.string_ = str(self.peo_con['standF'])
            # elif self.rec.num == 6:
            #     self.string_ = str(self.peo_con['standF']+self.peo_con['standM'])
            # elif self.rec.num == 7:
            #     self.string_ = str(self.peo_con['seatedF']+self.peo_con['seatedM'])
            if self.rec.num == 2:
                self.string_ = str(self.peo_con['Male'])
            elif self.rec.num == 3:
                self.string_ = str(self.peo_con['Female'])
            elif self.rec.num == 4:
                self.string_ = str(self.peo_con['All'])
            # if self.rec.num == 2:
            #     if self.peo_con['seatedM']+self.peo_con['seatedF'] == 1 :
            #         if self.peo_con['seatedM'] ==1:
            #             self.string_ = 'the sitting person is a male'
            #         else:
            #             self.string_ = 'the sitting person is a female'
            #     else:
            #         self.string_ = 'the sitting people is not one'
            # elif self.rec.num == 3:
            #     if self.peo_con['Female'] <= 1:
            #         self.string_ = 'There is %d woman'%(self.peo_con['Female'])
            #     else:
            #         self.string_ = 'There are %d women'%(self.peo_con['Female'])
            # elif self.rec.num ==4 :
            #     if self.peo_con['seatedF'] == 1 and self.peo_con['seatedM']+self.peo_con['seatedF'] ==1:
            #         self.string_ = 'Yes ,the sitting person is woman'
            #     elif self.peo_con['seatedM']+self.peo_con['seatedF'] >1:
            #         self.string_ = 'No,the sitting people are more than one'
            #     elif self.peo_con['seatedM']+self.peo_con['seatedF'] == 0 :
            #         self.string_ = 'No,There is no sitting people'
            #     else:
            #         self.string_ = 'The sitting people is a man'
            # elif self.rec.num == 5:
            #     if self.peo_con['seated_elder']+self.peo_con['stand_elder'] >0:
            #         self.string_ = 'There is %d elder '%(self.peo_con['seated_elder']+self.peo_con['stand_elder'])
            #     else:
            #         self.string_ = 'There is no elder ,you are all yonger'
            # elif self.rec.num == 6:
            #     if self.peo_con['seatedM']+self.peo_con['seatedF'] == 1 :
            #         if self.peo_con['seatedM'] ==1:
            #             self.string_ = 'the sitting person is a boy'
            #         else:
            #             self.string_ = 'the sitting person is a girl'
            #     else:
            #         self.string_ = 'the sitting people is not one'
            # elif self.rec.num == 7:
            #     if self.peo_con['Male'] <= 1:
            #         self.string_ = 'There is %d man'%(self.peo_con['Male'])
            #     else:
            #         self.string_ = 'There are %d men'%(self.peo_con['Male'])
            else:
                self.string_ = 'sorry,please ask again'
            try:
                self.speak_client.wait_for_service(timeout=10.0)
                speech_bool = self.speak_client.call(self.string_)
                if speech_bool.flag == 1:
                    subprocess.call(["play", "tts_sample.wav"])
                elif speech_bool.flag == 0:
                    subprocess.call("espeak -vf5 -s 100 '%(a)s'" %
                                    {'a': str(self.string_)}, shell=True)
                else:
                    rospy.logerr('the response error')
                    return 'aborted'
            except Exception, e:
                rospy.logerr(e)
            rospy.sleep(2.5)
            return 'succeeded'
        else:
            return 'succeeded'

class Gpsr():
    def __init__(self):
        rospy.init_node('Gpsr_Smach')
        rospy.on_shutdown(self.shutdown)
        rospy.logerr('Welcome to Gpsr!!!')
        self.smach_bool = False
        #add the function_smach here!
        #action状态机
        #---------------#
        #----GO-----#
        self.sm_Nav = StateMachine(outcomes = ['succeeded','aborted','error'],
                                input_keys = ['target','current_task'])
        with self.sm_Nav:
            self.sm_Nav.userdata.pos_xm = Pose()
            self.sm_Nav.userdata.turn_pose = Pose()
            self.sm_Nav.userdata.target_mode = 1
            
            StateMachine.add('GETTARGET',
                                GetTarget(),
                                transitions = {'succeeded':'GO','aborted':'aborted','error':'error'},
                                remapping = {'target':'target','current_task':'current_task','current_target':'pos_xm','mode':'target_mode'}
                            )
            StateMachine.add('GO',
                                NavStack(),
                                transitions = {'succeeded':'SPEAK','aborted':'GO','error':'error'},
                                remapping = {'pos_xm':'pos_xm'}
                            )
            self.sm_Nav.userdata.sentences = 'I have arrive here'
            StateMachine.add('SPEAK',
                                SpeakSentence(),
                                transitions = {'succeeded':'succeeded','error':'error'},
                                remapping = {'sentences':'sentences'}
                            )               class CheckTurn(State):
    def __init__(self):
        State.__init__(self, outcomes=['succeeded','continue','error'],
                        io_keys=['current_turn','task_num','current_task'],
                        input_keys=['turn'])
    def execute(self,userdata):
        try:
            self.current_turn = userdata.current_turn
            self.turn = userdata.turn
            rospy.logwarn(self.current_turn)
            rospy.logwarn(self.turn)
        except Exception,e:
            rospy.logerr(e)
            return 'error'
        if self.current_turn > self.turn:
            rospy.logwarn('xm finish the turn')
            return 'succeeded'
        else:
            rospy.logwarn('finish one turn')
            userdata.current_turn += 1
            userdata.task_num = 0
            userdata.current_task = -1
            return 'continue'
        #----FOLLOW-----#
        #sm_Follow包括两个并发状态，FOLLOW和STOP
        #STOP用于语音获取stop信号
        #FOLLOW包装了FindPeople().find_people_监视器和meta_nav
        #当找到人需要移动时，状态转移到meta_nav
        #meta_nav后,再继续进入监视状态
        #meta_nav是一个并发状态机，包装了NAV和WAIT
        #WAIT用于记录时间引发time_over
        #nav成功或中止后，与time_over一起进入继续找人状态
        self.sm_Follow = Concurrence(outcomes=['succeeded','aborted'],
                                        default_outcome = 'succeeded',
                                        outcome_map = {'succeeded':{'STOP':'stop'},
                                                        'aborted':{'FOLLOW':'aborted'}},
                                        child_termination_cb = self.child_cb)
        with self.sm_Follow:
            self.meta_follow = StateMachine(['succeeded','aborted','preempted'])
            with self.meta_follow:
                StateMachine.add('FIND',
                                    FindPeople().find_people_,
                                    transitions = {'invalid':'META_NAV','valid':'FIND','preempted':'preempted'},
                                    remapping = {'pos_xm':'pos_xm'}
                                )
                self.meta_nav = Concurrence(outcomes = ['time_over','get_pos','aborted'],
                                                default_outcome = 'aborted',
                                                outcome_map = {'time_over':{'WAIT':'succeeded'},
                                                               'get_pos':{'NAV':'succeeded'},
                                                               'aborted':{'NAV':'aborted'}},
                                                child_termination_cb=self.nav_child_cb,
                                                input_keys=['pos_xm']
                                            )
                with self.meta_nav:
                    Concurrence.add('NAV',NavStack(),remapping={'pos_xm':'pos_xm'})
                    Concurrence.add('WAIT',Wait_trace())
                StateMachine.add('META_NAV',
                                    self.meta_nav,
                                    transitions={'get_pos':'FIND','time_over':'FIND','aborted':'FIND'})
            Concurrence.add('FOLLOW',self.meta_follow)
            Concurrence.add('STOP',CheckStop())
            Concurrence.add('RUNNODE',RunNode())
        
        #----TALK----#
        #说出房间中人的状况
        self.sm_Talk = StateMachine(outcomes =['succeeded','aborted','error'])
        with self.sm_Talk:
            self.sm_Talk.userdata.people_condition = list()
            StateMachine.add('SPEAK',
                                Anwser(),
                                transitions ={'succeeded':'succeeded','aborted':'aborted'})      
        #----FIND----#
        #包含三个状态，由PERSON_OR_POS决定进入PERSON or POS
        #------<PERSON>-----#
        #PERSON：RUNNODE-->WAIT-->GET_PEOPLE_POS-->NAV-->SPEAK-->CLOSE_KINECT
        #运行图像节点，等待图像可用，然后利用find_people_监视器，nav到需要找的人面前，然后进行speak反馈，最后关闭摄像头
        #------<POS>------#
        #POS:GET_POS-->NAV-->GET_TARGET-->FIND_OBJECT-->GO-->FIND_OBJECT-->SPEAK
        #前一个任务的位置和需要find的小物体位置一样，得到位置后，nav到具体地点，然后获取物体名字，
        #打开摄像头寻找物体，获取具体信息后go，然后再找一个，speak反馈
        self.sm_Find = StateMachine(outcomes = ['succeeded','aborted','error'],
                                input_keys = ['target','current_task'])
        
        with self.sm_Find:
            #sm_Person--RUNNODE-->WAIT-->GET_PEOPLE_POS-->NAV-->SPEAK-->CLOSE_KINECT
            self.sm_Person = StateMachine(outcomes = ['succeeded','aborted','error'])
            with self.sm_Person:
                self.sm_Person.userdata.rec = 2.0
                #运行跟随人的图像节点
                StateMachine.add('RUNNODE',
                                    RunNode(),
                                    transitions={'succeeded':'WAIT','aborted':'RUNNODE'})#On simulation , We need to skip RunNode ;So aborted -> Wait Or aborted->RunNode 
                #等待保证能够找到人
                StateMachine.add('WAIT',
                                    Wait(),
                                    transitions = {'succeeded':'GET_PEOPLE_POS','error':'error'},
                                    remapping ={'rec':'rec'})
                #the data should be PointStamped() 
                self.sm_Person.userdata.pos_xm  =Pose()
                #用FindPeople().find_people_监视器找人的位置
                StateMachine.add('GET_PEOPLE_POS',
                                    FindPeople().find_people_,
                                    transitions ={'invalid':'NAV_PEOPLE','valid':'GET_PEOPLE_POS','preempted':'aborted'},
                                    remapping = {'pos_xm':'pos_xm'}
                                    )      
                StateMachine.add('NAV_PEOPLE',
                                    NavStack(),
                                    transitions = {'succeeded':'SPEAK','aborted':'NAV_PEOPLE','error':'error'},
                                    remapping = {'pos_xm':'pos_xm'})
                self.sm_Person.userdata.sentences = 'I find you'
                StateMachine.add('SPEAK',
                                    SpeakSentence(),
                                    transitions = {'succeeded':'CLOSEKINECT','error':'error'},
                                    remapping = {'sentences':'sentences'})

                # close the KinectV2
                StateMachine.add('CLOSEKINECT',
                                    CloseKinect(),
                                    transitions ={'succeeded':'succeeded','aborted':'aborted'})
 

            #sm_Pos--GET_POS-->NAV-->GET_TARGET-->FIND_OBJECT-->GO-->FIND_OBJECT-->SPEAK-->CHECK_TURN-->TURN-->NAV
            self.sm_Pos = StateMachine(outcomes=['succeeded','aborted','error'],
                                    input_keys =['target','current_task'])
            with self.sm_Pos:
                self.sm_Pos.userdata.pose = Pose()
                self.sm_Pos.userdata.mode_1 =1
                self.sm_Pos.userdata.nav_pos = gpsr_target['speaker']['pos']                                     
                StateMachine.add('GET_POS',
                                    GetPos(),
                                    remapping ={'target':'target','current_task':'current_task','pose':'pose','mode':'mode_1'},
                                    transitions={'succeeded':'NAV_HEHE','aborted':'aborted','error':'error'})

                StateMachine.add('NAV_HEHE',
                                    NavStack(),
                                    remapping ={'pos_xm':'pose'},
                                    transitions ={'succeeded':'GET_TARGET','aborted':'NAV_HEHE','error':'error'})

                self.sm_Pos.userdata.target_mode = 0
                self.sm_Pos.userdata.name = ''
                StateMachine.add('GET_TARGET',
                                    GetTarget(),
                                    remapping ={'target':'target','current_task':'current_task','mode':'target_mode','current_target':'name'},
                                    transitions ={'succeeded':'FIND_OBJECT','aborted':'aborted','error':'error'})
                
                self.sm_Pos.userdata.object_pos = PointStamped()
                StateMachine.add('FIND_OBJECT',
                                FindObject(),
                                transitions ={'succeeded':'SPEAK','aborted':'SPEAK','error':'error'},
                                remapping ={'name':'name','object_pos':'object_pos'}) 
                
                StateMachine.add('GO' , 
                                    GoAhead(),
                                    transitions = {'succeeded':'FIND_OBJECT_1' , 'aborted':'FIND_OBJECT_1' , 'error':'error'})

                self.sm_Pos.userdata.object_pos = PointStamped()
                StateMachine.add('FIND_OBJECT_1',
                                FindObject(),
                                transitions ={'succeeded':'SPEAK','aborted':'SPEAK','error':'error'},
                                remapping ={'name':'name','object_pos':'object_pos'}) 

                self.sm_Pos.userdata.sentences = 'I find it'
                self.sm_Pos.userdata.back_target = 'speaker'
                StateMachine.add('SPEAK',
                                    SpeakSentence(),
                                    transitions = {'succeeded':'succeeded','error':'error'},
                                    remapping ={'sentences':'sentences'})

    
            #顶层状态机装配
            StateMachine.add('PERSON_OR_POS',
                                PersonOrPosition(),
                                transitions ={'person':'PERSON','position':'POS','error':'error'},
                                remapping = {'target':'target','current_task':'current_task'})
            StateMachine.add('PERSON',
                                self.sm_Person,
                                transitions = {'succeeded':'succeeded','aborted':'aborted','error':'error'})
            StateMachine.add('POS',
                                self.sm_Pos,
                                transitions = {'succeeded':'succeeded','aborted':'aborted','error':'error'},
                                remapping = {'target':'target','current_task':'current_task'})
        #-------PICK_UP-------#
        #RUNNODE_IMG-->GETNAME-->GET_POS-->NAV-->FIND-->POS_JUS-->NAV-->RUNNODE_IMG-->FIND-->POS_JUS-->PICK-->SPEAK
        #运行图像节点，获取物体名字和大物体位置后，nav到大物体位置，然后用图像找物体，然后用pos_jus得出适宜抓取的坐标，nav后继续循环一次
        #在此之后，进行抓取
        #如果上面的图像没有找到物体，则通过speak进行错误反馈
        self.sm_Pick_up = StateMachine(outcomes =['succeeded','aborted','error'],
                                    input_keys =['target','current_task'])    
        with self.sm_Pick_up: 
            self.sm_Pick_up.userdata.target_pos = PointStamped()
            self.sm_Pick_up.userdata.pick_pos = PointStamped()
            self.sm_Pick_up.userdata.object_state = 1
            self.sm_Pick_up.userdata.target_mode = 1
            self.sm_Pick_up.userdata.objmode = 1
            self.sm_Pick_up.userdata.newik_id = 0
 
            StateMachine.add('FIND', FindObject(),
                             transitions={'succeeded': 'DISTANCE',
                                          'aborted': 'aborted', 'error': 'error'},
                             remapping = {'name':'target' ,
                                            'object_pos':'target_pos',
                                            'object_map_point':'object_map_point'})
            StateMachine.add('DISTANCE' , CBState(self.PickDistance,outcomes=['succeeded','error'],input_keys=['name'],output_keys=['distance']),
                                transitions={'succeeded':'POS_JUS',
                                                'error':'POS_JUS'},
                                remapping={'distance':'distance',
                                            'name':'target'})
            StateMachine.add('POS_JUS', PosJustfy(),
                             transitions={'succeeded': 'NAV',
                                          'aborted': 'aborted',
                                          'error': 'error'},
                             remapping={'pose': 'nav_pos',
                                        'distance': 'distance',
                                        'object_pos': 'target_pos'})

            StateMachine.add('NAV', NavStack(),
                             transitions={'succeeded': 'FIND_AGAIN',
                                          'aborted': 'NAV',
                                          'error': 'error',
                                          'preempted':'NAV'},
                             remapping={'pos_xm': 'nav_pos'})
            
            StateMachine.add('PICK2', ArmCmdForTf(),
                             transitions={'succeeded': 'succeeded',
                                          'error': 'error',
                                          'aborted': 'aborted'},
                             remapping={'arm_ps': 'object_map_point', 'mode': 'objmode'})

            StateMachine.add('FIND_AGAIN', FindObject(),
                             transitions={'succeeded': 'ARM_PS_JUS',
                                          'aborted': 'PICK2', 'error': 'error'},
                             remapping={'object_pos': 'pick_pos',
                                        'name': 'target',
                                        'object_state':'object_state'})
            
            StateMachine.add('ARM_PS_JUS' , CBState(self.Arm_psJus , outcomes=['succeeded','error'],input_keys=['name'],io_keys = ['pick_pos']),
                                transitions = {'succeeded':'PICK',
                                                'error':'PICK'},
                                remapping = {'pick_pos':'pick_pos',
                                            'name':'target'})

            StateMachine.add('PICK', ArmCmd(),
                             transitions={'succeeded': 'succeeded',
                                          'error': 'error',
                                          'aborted': 'aborted'},
                             remapping={'arm_ps': 'pick_pos', 'mode': 'objmode','object_state':'object_state'})
            self.sm_Pick_up.userdata.sentences = 'xiao meng can not find the thing'
            StateMachine.add('SPEAK',
                                SpeakSentence(),
                                transitions={'succeeded':'succeeded','aborted':'aborted','error':'error'})

        

        #-------Pick_down---------#
        self.sm_Put_down = StateMachine(outcomes =['succeeded','aborted','error'])
        with self.sm_Put_down:
            # place_ps please specified due to the scene
            self.sm_Put_down.userdata.place_ps = PointStamped()
            self.sm_Put_down.userdata.place_ps.header.frame_id ='base_link'
            self.sm_Put_down.userdata.place_ps.point.x =0.8
            self.sm_Put_down.userdata.place_ps.point.y =0.0
            self.sm_Put_down.userdata.place_ps.point.z =0.6 
            self.sm_Put_down.userdata.objmode = 2
            # without moveit, if is just place it in a open space
            self.sm_Put_down.userdata.arm_mode_1 =2         
            StateMachine.add('PLACE',
                                ArmCmd(),
                                transitions ={'succeeded':'succeeded','aborted':'PLACE','error':'error'},
                                remapping ={'arm_ps':'place_ps','mode':'arm_mode_1'})
        

        #顶层状态机
        self.sm_GPSR = StateMachine(outcomes = ['succeeded','aborted','error'])
        with self.sm_GPSR:
            self.sm_GPSR.userdata.target = list()
            self.sm_GPSR.userdata.action = list()
            self.sm_GPSR.userdata.task_num = 0
            self.sm_GPSR.userdata.current_task = -1
            self.sm_GPSR.userdata.current_turn = -1
            self.sm_GPSR.userdata.turn = 3              #the max_num of the mission
            self.sm_GPSR.userdata.pos_xm_door = gpsr_target['speaker']['pos']
            self.sm_GPSR.userdata.sentences = 'give me the mission please' #for the SpeakSentence() state
            
            StateMachine.add('SPEAK_RESTART',
                            SpeakSentence(),
                            transitions = {'succeeded':'RECEIVE_TASKS','aborted':'aborted','error':'error'},
                            remapping = {'sentences':'sentences'}
                            )

            StateMachine.add('RECEIVE_TASKS',
                            GetTask(),
                            transitions = {'succeeded':'GET_NEXT_TASK','aborted':'RECEIVE_TASKS','error':'error'},
                            remapping = {'target':'target','action':'action','task_num':'task_num'}
                            )
            
            StateMachine.add('GET_NEXT_TASK',
                                NextDo(),
                                transitions = {'succeeded':'BACK_DOOR',
                                                'aborted':'aborted',
                                                'error':'error',
                                                'go':'GO',
                                                'find':'FIND',
                                                'talk':'TALK',
                                                'follow':'FOLLOW',
                                                'pick':'PICK_UP',
                                                'place':'PUT_DOWN'},
                                remapping = {'action':'action',
                                              'current_task':'current_task',
                                              'task_num':'task_num'}
                            )
            StateMachine.add('BACK_DOOR',
                                NavStack(),
                                transitions = {'succeeded':'CHECK_TURN','aborted':'BACK_DOOR','error':'error'},
                                remapping = {'pos_xm':'pos_xm_door'}
                            )

            StateMachine.add('CHECK_TURN',
                                CheckTurn(),
                                transitions = {'succeeded':'GO_OUT','continue':'SPEAK_RESTART','error':'error'}
                            )
            StateMachine.add('GO_OUT',
                                NavStack(),
                                transitions ={'succeeded':'succeeded','aborted':'aborted','error':'error'},
                                remapping ={"pos_xm":'waypoint'}
                            )
        
            #add all the task smach
            #-----------------#
            StateMachine.add('TALK',
                                self.sm_Talk,
                                transitions={'succeeded':'GET_NEXT_TASK','aborted':'TALK'})
            StateMachine.add('GO',
                                self.sm_Nav,
                                remapping ={'target':'target','current_task':'current_task'},
                                transitions={'succeeded':'GET_NEXT_TASK','aborted':'GO'})
            StateMachine.add('CLOSE',
                                CloseKinect(),
                                transitions={'succeeded':'GET_NEXT_TASK','aborted':'GET_NEXT_TASK'})
            StateMachine.add('FIND',
                                self.sm_Find,
                                remapping ={'target':'target','current_task':'current_task'},
                                transitions={'succeeded':'GET_NEXT_TASK','aborted':'FIND'})
            StateMachine.add('FOLLOW',
                                self.sm_Follow,
                                transitions={'succeeded':'CLOSE','aborted':'FOLLOW'})
            StateMachine.add('PICK_UP',
                                self.sm_Pick_up,
                                remapping ={'target':'target','current_task':'current_task'},
                                transitions={'succeeded':'GET_NEXT_TASK','aborted':'PICK_UP'})
            StateMachine.add('PUT_DOWN',
                                self.sm_Put_down,
                                transitions={'succeeded':'GET_NEXT_TASK','aborted':'PUT_DOWN'}) 
        
        intro_server = IntrospectionServer('sm_gpsr',self.sm_GPSR,'/SM_ROOT')
        intro_server.start()
        out = self.sm_GPSR.execute()
        intro_server.stop()

    def shutdown(self):
        if self.smach_bool ==True:
            rospy.loginfo('smach succeeded')
        else:
            rospy.loginfo('smach error')

    #return True down
    def child_cb(self,outcome_map):
        if outcome_map['STOP'] == 'stop':
            rospy.logwarn('---------get the signal of stop,stop tracing---------')
            pip = get_pid("people_tracking")
            subprocess.call('kill'+str(pip[0]),shell=True)
            return True
        elif outcome_map['STOP'] == 'aborted':
            rospy.logerr('the stop state meet error!')
            return True
            
        if outcome_map['FOLLOW']:
            rospy.logerr('the follow state meet error!')
            return True
        return False

    def nav_child_cb(self,outcome_map):
        if outcome_map['WAIT'] == 'succeeded':
            rospy.logwarn('get the pos again')
            return True
        elif outcome_map['NAV'] == 'succeeded':
            return True
        elif outcome_map['NAV'] == 'aborted':
            return True
        else:
            print outcome_map
            return False
        

    
    def PickDistance(self, userdata):
        try:
            if(userdata.name == 'ice_tea' or userdata.name == 'water' or userdata.name=='tea'):
                userdata.distance = 0.92
            elif userdata.name == 'milk' or userdata.name == 'redbull' or userdata.name == 'orange_juice' or userdata.name == 'grape_juice' or userdata.name == 'Green_tea':
                userdata.distance =0.90
           
            else:
                userdata.distance = 0.8
            return 'succeeded'

        except Exception ,e:
            rospy.logerr(e)
            return 'error'

    
    def Arm_psJus(self , userdata):
        try:
            if userdata.name == 'ice_tea' :
                userdata.pick_pos.point.x +=-0.02
                userdata.pick_pos.point.y+= 0.02
                userdata.pick_pos.point.z+=0.04
            elif userdata.name == 'tea':

                userdata.pick_pos.point.x +=-0.05
                userdata.pick_pos.point.y+= -0.02
                userdata.pick_pos.point.z+=0.04
	    elif userdata.name == 'Green_tea':
		userdata.pick_pos.point.z+=0.1
            elif userdata.name == 'cola' or userdata.name == 'herbal_tea' or userdata.name == 'fanta' or userdata.name=='porridge' or userdata.name == 'redbull':
                userdata.pick_pos.point.z -= 0.05
            elif userdata.name == 'Oreo':
                userdata.pick_pos.point.x+=0.04
	    elif userdata.name == 'orange_juice':
                userdata.pick_pos.point.x +=-0.1
                userdata.pick_pos.point.y+= 0.00
                #userdata.pick_pos.point.z+=0.4
		rospy.logerr('gao!!!')
            rospy.logerr(userdata.pick_pos)
        
            return 'succeeded'

        except Exception,e:
            rospy.logerr(e)
            return 'error'

    # use for concurrence
if __name__ == "__main__":
    try:
        Gpsr()
    except Exception,e:
        rospy.logerr(e)   
