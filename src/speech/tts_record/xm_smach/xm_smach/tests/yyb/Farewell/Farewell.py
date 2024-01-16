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
'''
Farewell
    一些客人累了,所以他们叫机器人取回外套。 外面在下雨，只有把雨伞 所以机器人将客人一带到他们的驾驶室 ，然后带 着雨伞返回。
    由于要优先识别的想要离开的女性，这意味着在同一时刻想要离开的人可能不只一个，因此采取与F indMyMates同样的策略，
状态机:
        导航到客厅便于观察的位置->语音让想要离开的人挥手(这个可以改成听音辩位) ->旋转拍照(得到想要离开的人)
        (为了节约时间，上面这一流程只进行次，前提是认为在护送第一个客人的时候另个想要离开的人不会移动位置)
        >面向第一个想要离开的宾客>语音交互确认想要离开. >语音询问外套->抓取大衣->
        交给客人->抓取雨伞>交给客人>找到驾驶员->交接雨伞>-》导航到驾驶室-x》图像找并面向驾驶员
        >导航回带客厅>
        
        ->面向第2个想离开的宾客->语音交互确认想要离开->语自询问外套->交接雨伞-》抓取大衣>交给客人-》导航到驾驶室-》图像找并面向驾驶员
图像节点:
        类以FindMyMates
        在拍摄到的6个图片中,
        确定两个挥手的人，并且优先将女性排在列表前面
        
        服务有两种情况，command为0时， 单纯拍摄照片并存到指定文件
        
        command为1时，对指定文牛中的6张图片进行分析并确认哪两个想离开，并编号
        
        然后调用多人辨识里的，找特定人并面向

        识别特定颜色的大衣和雨伞(提供训练伞的目的应该是识别伞，但没有说伞放在哪里，可能是显眼位置，或者训练伞是为了训练撑伞的姿势)
语音:
        得出想要的衣服名字
        在理解xm_speach_meaning服务里，多开一个command，用于判断说的是yes还是no，并返回yes或no在res.answer里面，不需要am i right
问题:
        驾驶室的位置识别问题
        第二个想要离开的人会不会在护送第一个人的时候离开
        雨伞的细节
        可以不可以直接问想要的衣服，还是只能问名字，人名与衣服的对应关系事先得知
        雨伞在哪放着(应该是要抓取，不然为什么要训练，， 难道是训练如何享伞。。。?)
        状态机内容过多，可能会严重超时
        找司机的时候，还没有靠近


'''
class FacePeople(State):
    '''
    用于寻找特定id的人
    '''
    def __init__(self):
        State.__init__(self,outcomes =['succeeded','aborted','error','lost'],
                        input_keys =['people_id'])
        self.cmd_vel = rospy.Publisher('/mobile_base/mobile_base_controller/cmd_vel', Twist, queue_size=1)
        #主题可能有问题
        self.xm_findPeople = rospy.ServiceProxy('/get_position', xm_ObjectDetect)
    def execute(self,userdata):
        try:
            getattr(userdata, 'people_id')
        except:
            rospy.logerr('no param')
            return 'aborted'
        try:
            #打开找特定人的脚本,有问题
            subprocess.call("xterm -e rosrun xm_vision people_identify.py &",shell=True)
            
        except:
            rospy.logerr('meet wrong when open the py')
            return 'error'
        else:
            req.object_name ="person"
            req.people_id = userdata.people_id

            self.xm_findPeople.wait_for_service(10.0)
            res = self.xm_findPeople.call(req)

            try:
                #防止第二次打不开
                pid_str = subprocess.check_output('ps -aux | grep people_identify.py' , shell= True)
                pid_str1 = pid_str.splitlines()[0].split()[1]
                rospy.logwarn(pid_str1)
                subprocess.call('kill '+pid_str1 , shell = True)
            except Exception,e:
                rospy.logerr('No such process ')
            else:
                #图像最好能把太靠边缘的数据转换成-9，,-12这样的数据
                if res.object[0].pos.x == -10:
                    return 'lost'
                else:
                    self.twist_xm = self.data_deal_turn(res.object[0].pos)
                    self.cmd_vel.publish(self.twist_xm)
                    return 'succeeded'

    def data_deal_turn(self,pos_xm):
        person_x = pos_xm.point.z
        person_y = pos_xm.point.x

        distence = person_x
        angle = atan2(person_y,person_x)
        twist_xm = Twist()
        if  distence < 1.5 and (angle < -0.30 or angle > 0.30):
            twist_xm.angular.z = angle * 0.50    
        if distence > 0.7 and distence < 1.5 and (angle < 0.2 or angle > -0.2):
            twist_xm.angular.z = angle*0.60
        elif distence > 0.7 and distence < 1.5 and (angle >= 0.2 or angle <= -0.2):
            twist_xm.angular.z = 0.0
        else:
            twist_xm.angular.z = 0.0        
        return twist_xm



class GetClothes(State):
    '''
    获取衣服
    '''
    def __init__(self):
        State.__init__(self,outcomes =['succeeded','aborted','error'],
                        output_keys =['clothes'])

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
            userdata.clothes = res.answer
            return 'succeeded'


class Resure(State):
    '''
    语音交互确认
    '''
    def __init__(self):
        State.__init__(self,
                       outcomes=['yes', 'no', 'error'],
                       input_keys = ['sentences'])
        self.client = rospy.ServiceProxy('xm_speech_meaning',xm_Speech_meaning)
        self.speak_client = rospy.ServiceProxy("tts", xm_Speech_tts)
    def execute(self, userdata):
        try:
            getattr(userdata, 'sentences')
        except:
            rospy.logerr('No param')
            return 'error'
        else:
            try:
                self.string_ = userdata.sentences
                rospy.logwarn(self.string_)
                self.speak_client.wait_for_service(timeout=10.0)
                self.speak_client.call(self.string_)

                rospy.sleep(2.0)

                speech_bool = self.speak_client.call(self.string_)
                if speech_bool.flag == 1:
                    subprocess.call(["play", "tts_sample.wav"])
                elif speech_bool.flag == 0:
                    subprocess.call("espeak -vf5 -s 75 '%(a)s'" %
                                    {'a': str(self.string_)}, shell=True)
                else:
                    rospy.logerr('the response error')
                    return 'error'
            except:
                return 'error'
            else:
                try:
                    self.client.wait_for_service(timeout=10)
                except:
                    rospy.logerr('xm_speech_meaning service is error')
                    return 'error'
                else:
                    res = self.client.call(command=3)
                    if res == 'yes':
                        return 'yes'
                    elif res == 'no':
                        return 'no'
                    else:
                        return 'error'
class Scan(State):
    '''
    旋转拍6张照片，之后再调用图像服务得到两个想要离开的人
    '''

    def __init__(self):
        State.__init__(self, outcomes=['succeeded', 'aborted', 'error'])
        self.cmd_vel = rospy.Publisher(
            '/mobile_base/mobile_base_controller/cmd_vel', Twist, queue_size=1)
        # 暂定
        self.scan_client = rospy.ServiceProxy('get_position', xm_ObjectDetect)
        #self.scan_client = rospy.ServiceProxy('get_information',xm_Person_Information)
        self.degree = 3.1415926535/6
        self.turn = Twist()
        self.turn.angular.z = self.degree

    def execute(self, userdata):
        try:
            for i in range(6):
                self.cmd_vel.publish(self.turn)
                # 等待摄像头稳定
                rospy.sleep(3)
                # 拍照片
                self.scan_client.call(command=0)
        except:
            rospy.logerr('meet wrong when turn and take photos')
            return 'error'
        else:
            self.scan_client.call(command=1)
            return 'succeeded'



class Farewell():
    def __init__(self):
        rospy.init_node('Farewell_Smach')
        rospy.on_shutdown(self.shutdown)
        rospy.logerr('Welcome to Farewell!!!')
        self.smach_bool = False

        self.sm_GiveMeUmbreall = StateMachine(outcomes = ['succeeded','aborted','error'])
        with self.sm_GiveMeUmbreall:
            self.sm_GiveMeUmbreall.userdata.sentences_give_me_umbreall = 'give me the umbreall'
            self.sm_GiveMeUmbreall.userdata.sentences_resure = 'can i close my finger'
            
            StateMachine.add('SPEAK_GMU',
                                Speak(),
                                transitions={'succeeded':'RELEASE','aborted':'aborted','error':'error'},
                                remapping ={'sentences':'sentences_give_me_umbreall'}
                                )   
            StateMachine.add('RELEASE',
                                Release(),
                                transitions={'succeeded':'RESURE_U1','aborted':'aborted'})                                  

            StateMachine.add('RESURE_U1',
                                Resure(),
                                transitions={'yes':'CLOSE','no':'aborted','error':'error'},
                                remapping ={'sentences':'sentences_resure'})  
            
            StateMachine.add('CLOSE',
                                Close(),
                                transitions={'succeeded':'succeeded','aborted':'aborted'},
                                remapping ={'sentences':'sentences_resure2'})                                  
        self.sm_GetClothes = StateMachine(outcomes = ['succeeded','aborted','error'],
                                    output_keys =['name'])
        with self.sm_GetClothes:
            self.sm_GetClothes.userdata.name = ""
            self.sm_GetClothes.userdata.sentences_what_clothes = 'which clothes is yours'
            StateMachine.add('SPEAK',
                                Speak(),
                                transitions={'succeeded':'GETCLOTHES','aborted':'aborted','error':'error'},
                                remapping ={'sentences':'sentences_what_clothes'}
                                ) 
            StateMachine.add('GETCLOTHES',
                                GetClothes(),
                                transitions={'succeeded':'succeeded','aborted':'aborted','error':'error'},
                                remapping ={'clothes':'name'}
                                )     
        self.sm_Pick_up = StateMachine(outcomes =['succeeded','aborted','error'],
                                    input_keys =['name','position'])    
        with self.sm_Pick_up: 
            self.sm_Pick_up.userdata.name =''
            self.sm_Pick_up.userdata.target_mode =0
            self.sm_Pick_up.userdata.objmode = -1
            StateMachine.add('NAV',
                                NavStack(),
                                transitions ={'succeeded':'RUNNODE_IMG','aborted':'NAV','error':'error'},
                                remapping ={"pos_xm":'position'})
            StateMachine.add('RUNNODE_IMG',
                                RunNode_img(),
                                transitions = {'succeeded':'FIND_OBJECT','aborted':'RUNNODE_IMG'})

            self.sm_Pick_up.userdata.object_pos = PointStamped()
            StateMachine.add('FIND_OBJECT',
                                FindObject(),
                                transitions ={'succeeded':'POS_JUSTFY','aborted':'succeeded','error':'SPEAK'},
                                remapping ={'name':'name','object_pos':'object_pos','objmode':'objmode'})
            self.sm_Pick_up.userdata.pose = Pose()
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
            self.sm_Pick_up.userdata.arm_mode_1 =1
            StateMachine.add('PICK',
                                ArmCmd(),
                                transitions ={'succeeded':'succeeded','aborted':'aborted','error':'error'},
                                remapping ={'arm_ps':'object_pos','mode':'arm_mode_1'})
            self.sm_Pick_up.userdata.sentences = 'xiao meng can not find the thing'
            StateMachine.add('SPEAK',
                                Speak(),
                                transitions={'succeeded':'succeeded','aborted':'aborted','error':'error'})

 

        self.sm_Face = StateMachine(outcomes = ['succeeded','aborted','error'],
                                        input_keys = ['people_id'])
        with self.sm_Face:
            self.sm_Face.userdata.degree = 3.1415926535/8
            StateMachine.add('FINDPEOPLE_1',
                                FacePeople(),
                                transitions ={'succeeded':'FINDPEOPLE_2','aborted':'aborted','error':'error','lost':'TURN'},
                                remapping = {'people_id':'people_id'})
            StateMachine.add('FINDPEOPLE_2',
                                FacePeople(),
                                transitions ={'succeeded':'CLOSEKINECT','aborted':'aborted','error':'error','lost':'TURN'},
                                remapping = {'people_id':'people_id'})
            StateMachine.add('TURN',
                                TurnDegree(),
                                transitions = {'succeeded':'FINDPEOPLE_1','aborted':'aborted','error':'error'},
                                remapping = {'degree':'degree'})
            StateMachine.add('CLOSEKINECT',
                                CloseKinect(),
                                transitions ={'succeeded':'succeeded','aborted':'aborted'})            

        self.sm_Find = StateMachine(outcomes = ['succeeded','aborted','error'])
        with self.sm_Find:

            self.sm_Find.userdata.rec = 2.0
            #运行跟随人的图像节点
            StateMachine.add('RUNNODE',
                                RunNode(),
                                transitions={'succeeded':'WAIT','aborted':'RUNNODE'})
            #等待保证能够找到人
            StateMachine.add('WAIT',
                                Wait(),
                                transitions = {'succeeded':'GET_PEOPLE_POS','error':'error'},
                                remapping ={'rec':'rec'})
            #the data should be PointStamped() 
            self.sm_Find.userdata.pos_xm  =Pose()
            #用FindPeople().find_people_监视器找人的位置
            StateMachine.add('GET_PEOPLE_POS',
                                FindPeople().find_people_,
                                transitions ={'invalid':'NAV_PEOPLE','valid':'GET_PEOPLE_POS','preempted':'aborted'},
                                remapping = {'pos_xm':'pos_xm'}
                                )      
            StateMachine.add('NAV_PEOPLE',
                                NavStack(),
                                transitions = {'succeeded':'CLOSEKINECT','aborted':'NAV_PEOPLE','error':'error'},
                                remapping = {'pos_xm':'pos_xm'})

            StateMachine.add('CLOSEKINECT',
                                CloseKinect(),
                                transitions ={'succeeded':'succeeded','aborted':'aborted'})
                


                

        #顶层状态机
        self.FAREWELL = StateMachine(outcomes = ['succeeded','aborted','error'])
        with self.FAREWELL:
            self.FAREWELL.userdata.people_id0 = 0
            self.FAREWELL.userdata.people_id1 = 1
            self.FAREWELL.userdata.people_id2 = 2 #司机
            self.FAREWELL.userdata.pos_umbreall = gpsr_target['speaker']['pos']#抓取雨伞的位置
            self.FAREWELL.userdata.pos_bus = gpsr_target['speaker']['pos']#驾驶室的大概位置
            self.FAREWELL.userdata.pos_give = gpsr_target['speaker']['pos']#递还的位置
            self.FAREWELL.userdata.pos_clothes = gpsr_target['speaker']['pos']#易于识别大衣的位置
            self.FAREWELL.userdata.pos_living_room = gpsr_target['speaker']['pos']#导航到便于观察的位置 
            self.FAREWELL.userdata.sentences_raise_hand = 'who want to leave,please raise your hands,I may not ask you again'
            self.FAREWELL.userdata.sentences_resure1 = 'do you want to leave'
            self.FAREWELL.userdata.sentences_resure2 = 'have you take out your clothes'
            self.FAREWELL.userdata.sentences_resure3 = 'are you ready to take out the umbreall'

            StateMachine.add('NAVLIVING1',
                                NavStack(),
                                transitions={'succeeded':'SPEAK_RH','aborted':'aborted','error':'error'},
                                remapping ={'pos_xm':'pos_living_room'}
                                )
            StateMachine.add('SPEAK_RH',
                                Speak(),
                                transitions={'succeeded':'SCAN','aborted':'aborted','error':'error'},
                                remapping ={'sentences':'sentences_raise_hand'}
                                )
            StateMachine.add('SCAN',
                                Scan(),
                                transitions={'succeeded':'FACEGUEST1','aborted':'aborted','error':'error'},
                                remapping ={'sentences':'sentences_raise_hand'}
                                )
            StateMachine.add('FACEGUEST1',
                                self.sm_Face,
                                transitions={'succeeded':'RESURE_LEAVE1','aborted':'aborted','error':'error'},
                                remapping ={'people_id':'people_id0'}
                                )            
            StateMachine.add('RESURE_LEAVE1',
                                Resure(),
                                transitions={'yes':'GETCLOTHES1','no':'aborted','error':'error'},
                                remapping ={'sentences':'sentences_resure1'})
            StateMachine.add('GETCLOTHES1',
                                self.sm_GetClothes,
                                transitions={'succeeded':'PICKCLOTHES1','aborted':'aborted','error':'error'},
                                remapping ={'name':'name'})                           
            StateMachine.add('PICKCLOTHES1',
                                self.sm_Pick_up,
                                transitions={'succeeded':'NAVGIVE1_0','aborted':'aborted','error':'error'},
                                remapping ={'name':'name','position':'pos_clothes'}
                                ) 
            StateMachine.add('NAVGIVE1_0',
                                NavStack(),
                                transitions={'succeeded':'RELEASE_C1','aborted':'aborted','error':'error'},
                                remapping ={'pos_xm':'pos_give'}
                                )      
            StateMachine.add('RELEASE_C1',
                                Release(),
                                transitions={'succeeded':'RESURE_TO1_0','aborted':'aborted'}
                                )               
            StateMachine.add('RESURE_TO1_0',
                                Resure(),
                                transitions={'yes':'PICKUMBREALL1','no':'aborted','error':'error'},
                                remapping ={'sentences':'sentences_resure2'})
            StateMachine.add('PICKUMBREALL1',
                                self.sm_Pick_up,
                                transitions={'succeeded':'NAVGIVE1_1','aborted':'aborted','error':'error'},
                                remapping ={'name':'name','position':'pos_umbreall'}
                                )             
            StateMachine.add('NAVGIVE1_1',
                                NavStack(),
                                transitions={'succeeded':'RESURE_U1','aborted':'aborted','error':'error'},
                                remapping ={'pos_xm':'pos_give'}
                                )   
            StateMachine.add('RESURE_U1',
                                Resure(),
                                transitions={'yes':'RELEASE_U1','no':'aborted','error':'error'},
                                remapping ={'sentences':'sentences_resure2'})                                   
            StateMachine.add('RELEASE_U1',
                                Release(),
                                transitions={'succeeded':'NAVBUS1','aborted':'aborted'}
                                )    
            StateMachine.add('NAVBUS1',
                                NavStack(),
                                transitions={'succeeded':'FACEDRIVER1','aborted':'aborted','error':'error'},
                                remapping ={'pos_xm':'pos_bus'}
                                )                
            StateMachine.add('FACEDRIVER1',
                                self.sm_Face,
                                transitions={'succeeded':'GIVEMEUMBREALL','aborted':'aborted','error':'error'},
                                remapping ={'people_id':'people_id2'}
                                )                   
            StateMachine.add('GIVEMEUMBREALL',
                                self.sm_GiveMeUmbreall,
                                transitions={'succeeded':'NAVLIVING2','aborted':'aborted','error':'error'}
                                )      
            #护送第一个客人完成

            StateMachine.add('NAVLIVING2',
                                NavStack(),
                                transitions={'succeeded':'FACEGUEST2','aborted':'aborted','error':'error'},
                                remapping ={'pos_xm':'pos_living_room'}
                                )
            StateMachine.add('FACEGUEST2',
                                self.sm_Face,
                                transitions={'succeeded':'RESURE_LEAVE2','aborted':'aborted','error':'error'},
                                remapping ={'people_id':'people_id1'}
                                )            
            StateMachine.add('RESURE_LEAVE2',
                                Resure(),
                                transitions={'yes':'GETCLOTHES2','no':'aborted','error':'error'},
                                remapping ={'sentences':'sentences_resure1'})
            StateMachine.add('GETCLOTHES2',
                                self.sm_GetClothes,
                                transitions={'succeeded':'NAVGIVE2_0','aborted':'aborted','error':'error'},
                                remapping ={'name':'name'})  
            
            StateMachine.add('NAVGIVE2_0',
                                NavStack(),
                                transitions={'succeeded':'RESURE_U2','aborted':'aborted','error':'error'},
                                remapping ={'pos_xm':'pos_give'}
                                )   
            StateMachine.add('RESURE_U2',
                                Resure(),
                                transitions={'yes':'RELEASE_U1','no':'aborted','error':'error'},
                                remapping ={'sentences':'sentences_resure2'})                                   
            StateMachine.add('RELEASE_U2',
                                Release(),
                                transitions={'succeeded':'PICKCLOTHES2','aborted':'aborted'}
                                )    
            StateMachine.add('PICKCLOTHES2',
                                self.sm_Pick_up,
                                transitions={'succeeded':'NAVGIVE2_1','aborted':'aborted','error':'error'},
                                remapping ={'name':'name','position':'pos_clothes'}
                                ) 
            StateMachine.add('NAVGIVE2_1',
                                NavStack(),
                                transitions={'succeeded':'RELEASE_C2','aborted':'aborted','error':'error'},
                                remapping ={'pos_xm':'pos_give'}
                                )      
            StateMachine.add('RELEASE_C2',
                                Release(),
                                transitions={'succeeded':'RESURE_TO2_0','aborted':'aborted'}
                                )               
            StateMachine.add('RESURE_TO2_0',
                                Resure(),
                                transitions={'yes':'NAVBUS2','no':'aborted','error':'error'},
                                remapping ={'sentences':'sentences_resure2'})
          

            StateMachine.add('NAVBUS2',
                                NavStack(),
                                transitions={'succeeded':'FACEDRIVER2','aborted':'aborted','error':'error'},
                                remapping ={'pos_xm':'pos_bus'}
                                )                
            StateMachine.add('FACEDRIVER2',
                                self.sm_Face,
                                transitions={'succeeded':'succeeded','aborted':'aborted','error':'error'},
                                remapping ={'people_id':'people_id2'}
                                )                   
            #StateMachine.add('GIVEMEUMBREALL',
            #                    self.sm_GiveMeUmbreall,
            #                    transitions={'succeeded':'SPEAK1','aborted':'aborted','error':'error'}
            #                    )      
            
        intro_server = IntrospectionServer('FAREWELL',self.FAREWELL,'/SM_ROOT')
        intro_server.start()
        out = self.FAREWELL.execute()
        intro_server.stop()

    def shutdown(self):
        if self.smach_bool ==True:
            rospy.loginfo('smach succeeded')
        else:
            rospy.loginfo('smach error')


if __name__ == "__main__":
    try:
        Farewell()
    except Exception,e:
        rospy.logerr(e)   
