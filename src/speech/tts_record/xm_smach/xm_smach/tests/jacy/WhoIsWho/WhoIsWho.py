#!/usr/bin/env python
# encoding:utf8
import rospy
from smach import *
from smach_ros import *
from xm_smach.common_lib import *
from xm_smach.gpsr_lib import * 
from xm_smach.help_me_carry_lib import *
from xm_smach.target_gpsr import gpsr_target
from geometry_msgs.msg import *
import math
import subprocess
from xm_smach.pick_turn import PickTurn , IsTurn
from xm_smach.store_lib import *
from xm_msgs.srv import *
from xm_msgs.msg import *
import tf
'''
Who Is Who
1.进入房间:等待开门-》进入房间-》介绍自己
2.打开摄像头，获取三个人的位置
3.面对视野中第一个人，获取信息


'''

class FaceReco(State):
    def __init__(self):
        State.__init__(self, outcomes =['succeeded','aborted','error','again','train_error','turn_l','turn_r'],
                        input_keys = ['distance','name_id'],
                        io_keys = ['position'],
                        output_keys = ['num_list'])
        #service name should be specified when used 
        self.face_reco_client = rospy.ServiceProxy('get_position',xm_ObjectDetect)      #图像获取物体信息
        self.tf_listener =tf.TransformListener()
        self.ps = Pose()
        self.position =list()
        self.cmd_vel = rospy.Publisher('/mobile_base/mobile_base_controller/cmd_vel',Twist,queue_size=1)

    def execute(self,userdata):
        rospy.loginfo('face recongize begin')
        req  = xm_ObjectDetectRequest()
        self.distance = userdata.distance

        req.object_name ="person"
        req.people_id = userdata.name_id
        name_id = userdata.name_id
        
        #请求图像服务
        try:
            self.face_reco_client.wait_for_service(10.0)
            res= self.face_reco_client.call(req)
            print res

        except Exception,e:
            rospy.logerr('can not call!!!')
            rospy.logerr(e)
            rospy.sleep(5.0)
            return 'aborted'
        else:
            
            if len(res.object)==0 :
                return 'aborted'
            res.object.sort(key = lambda obj:obj.pos.point.x)  #按照位置从左到右排序

            #第一个人的状态正确
            if res.object[0].state ==0:
                self.true_person = list()
                self.error_num_l = list()
                self.error_num_r = list()
                
                #如果是找所有人，人数小于三，后退
                if(name_id == -1  and len(res.object)<3):
                    rospy.logerr('people<3')
                    self.ahead_justice()
                    return 'aborted'
                
                #对视野中所有的人进行处理
                for i in range(len(res.object)):
                    print res.object[i].pos.point.x
                    #第一个人的位置信息合格
                    if res.object[i].pos.point.x >= -11.0 and res.object[i].pos.point.x <= 11.0 :
                    ######################这个地方处理很有问题目前先这么决定，以后一定要改########################
                        #向右
                        if abs(res.object[i].pos.point.x - 10.0) < 0.00001 and abs(res.object[i].pos.point.y - 10.0 < 0.00001) and abs(res.object[i].pos.point.z - 10.0 < 0.00001):
                            #修正obj的位置
                            rospy.logerr("right")
                            obj = self.right_justice(res.object[i])
                            self.data_deal(obj.pos ,self.distance)
                            self.true_person.append(obj)
                        #向左true
                        elif abs(res.object[i].pos.point.x + 10.0 < 0.00001) and abs(res.object[i].pos.point.y + 10.0 < 0.00001) and abs(res.object[i].pos.point.z + 10.0 < 0.00001):
                            rospy.logerr("left")
                            obj = self.left_justice(res.object[i])
                            self.data_deal(obj.pos,self.distance)
                            self.true_person.append(obj)
                        else:
                            rospy.logerr("ok")
                            rospy.logwarn("pos of the person have been found")
                            rospy.logwarn(i)
                            rospy.logwarn(res.object[i].pos)
                            
                            #最后一个人
                            if(name_id == -1 and res.object[i].name == 2):
                                rospy.logerr('name_id = 0?')
                                rospy.logerr(res.object[i].name)
                                #修正最后一个人的位置
                                self.data_deal(res.object[i].pos,self.distance-0.2)
                            else:
                                #修正第一个和第二个人的位置
                                self.data_deal(res.object[i].pos ,self.distance)

                            #将正确的人的信息插入到 true_person    
                            self.true_person.append(res.object[i])
                            self.position.append(self.ps)
                            rospy.logwarn(self.ps) 

                #人的信息列表为空
                if len(self.true_person)==0:
                    return 'aborted'

                #寻找特定人，弹出人的位置
                if name_id !=-1:  
                    userdata.position =self.position.pop()
                    rospy.logerr(name_id)
                
                #寻找所有人
                else:
                    # out_list: 0 1 2
                    self.out_list = [int(obj.name) for obj in self.true_person]
                    print self.out_list
                    userdata.num_list = self.out_list
                    rospy.logerr(self.out_list)
                    userdata.position.extend(self.position)
                
                return 'succeeded'
            elif res.object[0].state ==-1:
                rospy.logwarn("I will recognize again")
                self.ahead_justice()
                return 'again'
            elif res.object[0].state ==-2:
                rospy.logwarn("the train may cause error")
                return 'train_error'
            elif res.object[0].state ==-3:
                rospy.logwarn("the position is not fit,turn left")
                return 'turn_l'
            elif res.object[0].state ==-4:
                rospy.logwarn("the position is not fit, turn right")
                return 'turn_r'
            else :
                return 'aborted'
    def left_justice(self,obj):
        '''
        向左边旋转，直到视野中人的信息准确
        '''
        sub_req = xm_ObjectDetectRequest()
        sub_req.object_name ="person"
        sub_req.people_id = obj.name

        self.turn(3.1415926/8)
    
        self.face_reco_client.wait_for_service(10.0)
        sub_res= self.face_reco_client.call(sub_req)
        if sub_res.object[0].state == 0:
            return sub_res.object
        elif sub_res.object[0].state== -3:
            return self.left_justice(obj)
        elif sub_res.object[0].state == -4:
            return self.right_justice(obj)
        else:
            raise Exception("xm meet wrong when justice")
   
    def right_justice(self,obj):
        '''
        向右边旋转，直到视野中人的信息准确
        '''
        sub_req = xm_ObjectDetectRequest()
        sub_req.object_name ="person"
        sub_req.people_id = obj.name
        
        #向右转22.5度    
        self.turn(-3.1415926/8)
        
        self.face_reco_client.wait_for_service(10.0)
        sub_res= self.face_reco_client.call(sub_req)
        if sub_res.object[0].state == 0:
            return sub_res.object
        elif sub_res.object[0].state== -3:
            return left_justice(obj)
        elif sub_res.object[0].state == -4:
            return right_justice(obj)
        else:
            raise Exception("xm meet wrong when justice")

    def ahead_justice(self):
        '''
        太近了->后退
        '''
        angular_speed = 0.1
        self.turn = Twist()
        self.turn.linear.x = 0.1
        self.turn.linear.y = 0.0
        self.turn.linear.z = 0.0
        self.turn.angular.x = 0.0
        self.turn.angular.y = 0.0
        self.turn.angular.z = 0.0

        goal_angle = 0.1
        angular_duration = goal_angle/angular_speed
        #发布频率
        rate = 50
        r = rospy.Rate(rate)
        ticks = int(goal_angle*rate)+5
        for i in range(ticks):
            self.cmd_vel.publish(self.turn)
            r.sleep()        
         
    def data_deal(self,pos_xm,distance):
        '''
        获得距离distance距离的位置，修改pos_xm和self.ps
        '''
        # the simple deal for data from the cv
        person_x = pos_xm.point.z
        person_y = pos_xm.point.x
        angle = math.atan2(person_y, person_x)
        person_x = person_x - distance*math.cos(angle)
        person_y = person_y -distance*math.sin(angle)
        pos_xm.point.x = person_x
        pos_xm.point.y =person_y
        pos_xm.point.z =0
        new_header =Header()
        new_header.frame_id = 'base_link'
        pos_xm.header = new_header
        # change 
        q_angle = quaternion_from_euler(0,0,angle)
        self.q = Quaternion(*q_angle)
        qs = QuaternionStamped()
        qs.header  =pos_xm.header
        qs.quaternion = self.q
      
   
        self.tf_listener.waitForTransform('map','base_link',rospy.Time(),rospy.Duration(60.0))    
    
        rospy.logwarn('wait for tf succeeded ')    
        

        pos_xm =self.tf_listener.transformPoint('map',pos_xm)
        rospy.logwarn('tf point succeeded ')    

        #修改self.ps
        qs =self.tf_listener.transformQuaternion('map',qs)
        rospy.logwarn('tf quaternion succeeded ')    
        self.ps = Pose(pos_xm.point,qs.quaternion)
        
    def turn(self,goal_angle):
        '''
        旋转
        '''
        angular_speed = 1.0
        self.turn = Twist()
        self.turn.linear.x = 0.0
        self.turn.linear.y = 0.0
        self.turn.linear.z = 0.0
        self.turn.angular.x = 0.0
        self.turn.angular.y = 0.0
        self.turn.angular.z = angular_speed

        angular_duration = goal_angle/angular_speed
        #发布频率
        rate = 50
        r = rospy.Rate(rate)
        ticks = int(goal_angle*rate)+5
        for i in range(ticks):
            self.cmd_vel.publish(self.turn)
            r.sleep()        

class GetValue(State):
    def __init__(self):
        State.__init__(self, outcomes =['succeeded','aborted','error'],
                        input_keys =['element_list'],
                        output_keys =['element'])
        self.element_list = list()
        self.mode = True
    def execute(self, userdata):
        try:
            getattr(userdata,'element_list')
        except:
            rospy.logerr('No params specified')
            return 'error'
        else:
            rospy.logwarn(userdata.element_list)
            #只有在第一次的时候，初始化element_list*
            if self.mode ==True:
                self.element_list = userdata.element_list
                self.mode = not self.mode
            try:
                userdata.element = self.element_list.pop()
            except:
                rospy.logerr('pop from empty list')
                return 'aborted'
            return 'succeeded'

class NameAndThing(State):
    def __init__(self):
        State.__init__(self,outcomes =['succeeded','aborted','error'],
                        output_keys =['name','target'])

        self.client = rospy.ServiceProxy('xm_speech_meaning',xm_Speech_meaning)
        self.speak_client = rospy.ServiceProxy("tts", xm_Speech_tts)
    def execute(self,userdata):
        try:
            a=1
        except:
            rospy.logerr('ros is error, please buy a new computer')
            return 'error'
        # name and target recognize
        try:
            self.client.wait_for_service(timeout=10)
        except:
            rospy.logerr('xm_speech_meaning service is error')
            return 'aborted'
        else :
            #example :  I am Tom and I want ice tea.
            res = self.client.call(command=3)
            rospy.logwarn(res)
            # the name from the speech_node is the form of list
            name = res.name.pop()
            rospy.logerr(name)
            target = res.object.pop()
            userdata.name = name
            userdata.target = target
            self.string_ ='I know that you are '+str(name)+'and you want '+str(target)
            print self.string_
            self.speak_client.wait_for_service(timeout=10.0)
            # self.speak_client.call(self.string_)
            # rospy.sleep(2.0)

            speech_bool = self.speak_client.call(self.string_)
            if speech_bool.flag == 1:
                subprocess.call(["play","tts_sample.wav"])
            elif speech_bool.flag == 0:
                subprocess.call("espeak -vf5 -s 100 '%(a)s'"%{'a':str(self.string_)} , shell = True)
            else:
                rospy.logerr('the response error')
                return 'error'
            return 'succeeded'

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
            if userdata.num <=3:
                userdata.num +=1
                return 'continue'
            elif userdata.num ==4:
                return 'succeeded'
            else:
                return 'error'

class NameInList(State):
    '''
    将名字和想要的物体信息插入到列表中,顺序进行了翻转
    '''
    def __init__(self):
        State.__init__(self,outcomes =['succeeded','aborted','error'],
                        input_keys=['name','target'],
                        io_keys=['name_list','target_list'])
        self.name_list = list()
        self.target_list = list()
    def execute(self, userdata):
        try:
            getattr(userdata,'name')
            getattr(userdata,'target')
            self.name_list = userdata.name_list
            self.target_list = userdata.target_list
        except Exception,e:
            rospy.logerr('No params specified')
            rospy.logerr(e)
            return 'error'
        else:
            self.name_list.append(userdata.name)            
            self.target_list.append(userdata.target)
            userdata.name_list =self.name_list
            userdata.target_list =self.target_list
            print self.target_list
            print self.name_list
            return 'succeeded'
        # oh hehe 
        if False:
            return 'aborted'

class GetId(State):
    def __init__(self):
        State.__init__(self,outcomes=['succeeded','aborted','error'],
                        input_keys =['input_list'],
                        output_keys =['output_id'],
                        io_keys = ['num_list'])
        self.input_list = list()
        self.mode = True
    
    def execute(self,userdata):
        try:
            getattr(userdata,'input_list')
            getattr(userdata,'num_list')
        except:
            rospy.logerr('No params specified')
            return 'error'
        else:
            print userdata.input_list
            print userdata.num_list
            #第一次识别
            if self.mode ==True:
                self.mode =False
                self.input_list = deepcopy(userdata.input_list)
            else:
                pass
            try:
                num = userdata.num_list.pop()
                print num
                userdata.output_id = int(num)
                return 'succeeded'
            except :
                rospy.logerr('pop from empty list')
                return 'aborted'
                
class GenerateInformation(State):
    def __init__(self):
        State.__init__(self,outcomes =["succeeded",'aborted','error'],
                            input_keys =['name_id','target_name','target_list'],
                            output_keys =['sentences'])
    
    def execute(self,userdata):
        try:
            getattr(userdata,'name_id')
            getattr(userdata,'name_list')
            getattr(userdata,'target_list')
            
        except:
            rospy.logerr("No params specified")
            return 'error'
        else:
            print userdata.name_list
            print userdata.name_id
            sentences = 'your name is '+userdata.name_list.pop() +' and' +'you want ' +userdata.target_list.pop()
            userdata.sentences = sentences
            print sentences
            return 'succeeded'
        if False:
            return 'aborted'

class WhoIsWho():
    def __init__(self):
        rospy.init_node('WhoIsWho_Smach')
        rospy.on_shutdown(self.shutdown)
        rospy.logerr('Welcome to WhoIsWho!!!')
        self.smach_bool = False
        
        self.waypoints = list()
        self.waypoints.append(gpsr_target['speaker']['pos'])     #找人的点
        self.waypoints.append(gpsr_target['shelf']['pos'])     #抓东西的点
        self.waypoints.append(gpsr_target['init_pose']['pos'])     #结束出门的点

        self.sm_EnterRoom = StateMachine(outcomes = ['succeeded','aborted','error'])
        with self.sm_EnterRoom:
            self.sm_EnterRoom.userdata.sentences = 'I am robot xiaomeng'
            self.sm_EnterRoom.userdata.start_waypoint = self.waypoints[0]
            StateMachine.add('NAV',
                                NavStack(),
                                transitions={'succeeded':'SELF_INTRO','aborted':'NAV','error':'error'},
                                remapping = {'pos_xm':'start_waypoint'})
            
            StateMachine.add('SELF_INTRO',
                                Speak(),
                                remapping ={'sentences':'sentences'},
                                transitions ={'succeeded':'succeeded','aborted':'SELF_INTRO','error':'error'})
        
        self.sm_FaceDetect = StateMachine(outcomes = ['succeeded','aborted','error'],
                                            output_keys = ['people_position','num_list'])
        with self.sm_FaceDetect:
            self.sm_FaceDetect.userdata.people_position =list()
            self.sm_FaceDetect.userdata.name_id =-1
            self.sm_FaceDetect.userdata.num_list = list()
            self.sm_FaceDetect.userdata.sentences = 'please look at me'
            self.sm_FaceDetect.userdata.turn_point_1 = Point(0.0,0.0,3.1415926535/8)  
            self.sm_FaceDetect.userdata.turn_point_2 = Point(0.0,0.0,-3.1415926535/8)
            StateMachine.add('SPEAK',
                                Speak(),
                                remapping = {'sentences':"sentences"},
                                transitions = {'succeeded':'GET_POSITION','aborted':'aborted','error':'error'})

            StateMachine.add('GET_POSITION',
                                FaceReco(),
                                remapping  ={'name_id':'name_id','position':'people_position','num_list':'num_list'},
                                transitions ={'succeeded':'succeeded',
                                              'again':'GET_POSITION',
                                              'aborted':'GET_POSITION',
                                              'error':'error',
                                              'turn_l':'TURN_L',
                                              'turn_r':'TURN_R',
                                              'train_error':'aborted'})                 
            StateMachine.add('TURN_L',
                                SimpleMove_move(),
                                transitions={'succeeded':'SPEAK','error':'error'},
                                remapping ={'point':'turn_point_1'})
            
            
            StateMachine.add('TURN_R',
                                SimpleMove_move(),
                                remapping ={'point':'turn_point_2'},
                                transitions ={'succeeded':'SPEAK','error':'error'})
   

        self.sm_Remember = StateMachine(outcomes =['succeeded','aborted','error'],
                                        input_keys =['person_position'],
                                        output_keys =['name','target']
                                        )
        with self.sm_Remember:  
            self.sm_Remember.userdata.sentences = "what is your name and what do you want？"   
            StateMachine.add('NAV',
                                NavStack(),
                                remapping ={'pos_xm':'person_position'},
                                transitions ={'succeeded':'TALK','aborted':'NAV_GO','error':'error'}
                                )
                                
            
            StateMachine.add('TALK',
                                Speak(),
                                remapping ={'sentences':'sentences'},
                                transitions ={'succeeded':'GETINFORMATION','aborted':'TALK','error':'error'})           
            StateMachine.add('GETINFORMATION',
                                NameAndThing(),
                                remapping ={'name':'name','target':'target'},
                                transitions ={'succeeded':'succeeded','aborted':'GETINFORMATION','error':'error'})

        self.sm_GetTarget = StateMachine(outcomes =['succeeded','aborted','error'],
                                            input_keys =['target'])#the target is a string.....
        with self.sm_GetTarget:

            # because xm is nav to pose in the nav_pose 
            self.sm_GetTarget.userdata.nav_ps = self.waypoints[1]
            # this smach code donnot to grasp ,so this part is useless
            StateMachine.add('NAV_TARGET',
                                NavStack(),
                                remapping ={'pos_xm':'nav_ps'},
                                transitions ={'succeeded':'FIND_OBJECT','aborted':'NAV_TARGET','error':'error'})
            self.sm_GetTarget.userdata.object_pos = PointStamped()
            StateMachine.add('FIND_OBJECT',
                                new_vision.FindObject(),
                                remapping ={'name':'target','object_pos':'object_pos'},
                                transitions ={'succeeded':'TALK','aborted':'FIND_OBJECT','error':'error'})
            self.sm_GetTarget.userdata.sentences = 'I find the target'
            StateMachine.add('TALK',
                                Speak(),
                                remapping ={'sentences':'sentences'},
                                transitions ={'succeeded':'PICK','aborted':'TALK','error':'error'})
            self.sm_GetTarget.userdata.arm_mode_1 =1
            StateMachine.add('PICK',
                                ArmCmd(),
                                remapping ={'mode':'arm_mode_1','arm_ps':'object_pos'},
                                transitions ={'succeeded':'succeeded','aborted':'succeeded','error':'error'})
            self.sm_GetTarget.userdata.arm_mode_2 = 0
            self.sm_GetTarget.userdata.arm_ps_2 = PointStamped()
            StateMachine.add('PUT',
                                PlaceBag(),
                                transitions ={'succeeded':'succeeded','aborted':'aborted'})

        self.sm_GiveBack = StateMachine(outcomes =['succeeded','aborted','error'],
                                            input_keys =['name_id','name_list','target_list'])# the name is a string
        with self.sm_GiveBack:
            self.sm_GiveBack.userdata.sentences = "please look at me"
            self.sm_GiveBack.userdata.turn_point_1 = Point(0.0,0.0,pi/8) 
            self.sm_GiveBack.userdata.turn_point_2 = Point(0.0,0.0,-pi/8)   
            self.sm_GiveBack.userdata.rec =5.0
            StateMachine.add('SPEAK',
                                Speak(),
                                remapping ={'sentences':'sentences'},
                                transitions ={'succeeded':'WAIT','aborted':'aborted','error':'error'})
            
            StateMachine.add('WAIT',
                                Wait(),
                                remapping ={'rec':'rec'},
                                transitions ={'succeeded':'FACE_RECO','error':'error'})  
            StateMachine.add('FACE_RECO',
                                FaceReco(),
                                remapping ={'position':'person_position','name_id':'name_id'},
                                transitions ={'succeeded':'NAV_GO',
                                              'again':'FACE_RECO',
                                              'aborted':'FACE_RECO',
                                              'error':'error',
                                              'turn_l':'TURN_L',
                                              'turn_r':'TURN_R',
                                              'train_error':'aborted'}
                                )
                
            StateMachine.add('TURN_L',
                                SimpleMove_move(),
                                transitions={'succeeded':'SPEAK_2','error':'error'},
                                remapping ={'point':'turn_point_1'})
            StateMachine.add('TURN_R',
                                SimpleMove_move(),
                                remapping ={'point':'turn_point_2'},
                                transitions ={'succeeded':'SPEAK_2','error':'error'})
            StateMachine.add('SPEAK_2',
                                Speak(),
                                remapping ={'sentences':'sentences'},
                                transitions ={'succeeded':'FACE_RECO','aborted':'aborted','error':'error'})  
            StateMachine.add('NAV_GO',
                                NavStack(),
                                remapping ={'pos_xm':'person_position'},
                                transitions ={'succeeded':'GET_NAME','aborted':'NAV_GO','error':'error'})
            StateMachine.add("GET_NAME",
                                GenerateInformation(),
                                remapping ={'name_list':'name_list','name_id':'name_id','target_list':'target_list', 'sentences':'sentences'},
                                transitions ={'succeeded':'TALK','aborted':'aborted','error':'error'})
            
            StateMachine.add('TALK',
                                Speak(),
                                remapping ={'sentences':"sentences"},
                                transitions ={'succeeded':'PUT','aborted':"PUT",'error':'error'})

            StateMachine.add('PUT' , PlaceBag(),
                                transitions = {'succeeded':'succeeded',
                                                'aborted':'succeeded'})

        #顶层状态机
        self.WHOISWHO = StateMachine(outcomes = ['succeeded','aborted','error'])
        with self.WHOISWHO:
            self.WHOISWHO.userdata.people_position =list()
            self.WHOISWHO.userdata.num_list = list()
            self.WHOISWHO.userdata.person_position = PointStamped()
            self.WHOISWHO.userdata.name_list =list()
            self.WHOISWHO.userdata.target_list =list()
            self.WHOISWHO.userdata.nav_ps = self.waypoints[0]

            StateMachine.add('ENTERROOM',
                                self.sm_EnterRoom,
                                transitions ={'succeeded':'FACEDETECT','aborted':'aborted','error':'error'})

            StateMachine.add('FACEDETECT',
                                self.sm_FaceDetect,
                                remapping ={'people_position':'people_position','num_list':'num_list'},
                                transitions = {'succeeded':'GETPERSON','aborted':'aborted','error':'error'}
                                )
            #people_position:0 1 2

            StateMachine.add('GETPERSON',
                                GetValue(),
                                remapping ={'element_list':'people_position','element':'person_position'},
                                transitions ={'succeeded':'REMEMBER','aborted':"GETTARGET",'error':'error'}
                                )
            #people_position:2->1->0
            
            StateMachine.add('REMEMBER',
                                self.sm_Remember,
                                remapping ={'person_position':'person_position','name':'name','target':'target'},
                                transitions ={'succeeded':'NAMEINLIST','aborted':'aborted','error':'error'}
                                )
            StateMachine.add('NAMEINLIST',
                                NameInList(),
                                remapping ={'name':'name','target':'target','name_list':'name_list','target_list':'target_list'},
                                transitions ={'succeeded':'GETPERSON','aborted':'aborted','error':'error'}
                                )
            #name_list,target_list:0 1 2
            #num_list:0 1 2
            #到此为止，我们已经获取了三个人的名字和物体列表
            #获取物体信息->2->1->0
            StateMachine.add('GETTARGET',
                                GetValue(),
                                remapping ={'element_list':'target_list','element':'target'},
                                transitions ={'succeeded':'CATCHTARGET','aborted':'ENDTASK','error':'error'})
            StateMachine.add('CATCHTARGET',
                                self.sm_GetTarget,
                                transitions= {'succeeded':'NAV_ROOM','aborted':'NAV_ROOM','error':'error'},
                                remapping = {'target':'target'})
            
            
            # self.WHOISWHO.userdata.nav_mode_1 =0
            StateMachine.add('NAV_ROOM',
                                NavStack(),
                                remapping ={'pos_xm':'nav_ps'},
                                transitions ={'succeeded':'GETID','aborted':'GETID','error':'error'})



            #开始识别
            self.WHOISWHO.userdata.sentences_1 = 'please change the order'
            StateMachine.add('SPEAK_1',
                                Speak(),
                                remapping ={'sentences':'sentences_1'},
                                transitions ={'succeeded':'WAIT_HEHE','aborted':'WAIT_HEHE','error':'error'})
            self.WHOISWHO.userdata.rec_hehe =10.0
            StateMachine.add('WAIT_HEHE',
                                Wait(),
                                remapping ={'rec':'rec_hehe'},
                                transitions ={'succeeded':'SPEAK_2','error':'error'})
            self.WHOISWHO.userdata.sentences_2 = 'I will make the recongization task'
            StateMachine.add('SPEAK_2',
                                Speak(),
                                transitions ={'succeeded':'GETID','aborted':'GETID','error':'error'},
                                remapping ={'sentences':'sentences_2'})

            self.WHOISWHO.userdata.name_id = 0   
            #GETID name-id :2 1 0
            StateMachine.add('GETID',
                                GetId(),
                                remapping ={'output_id':'name_id','input_list':'name_list','num_list':'num_list'},
                                transitions ={'succeeded':'GIVEBACK','aborted':'ENDTASK','error':'error'}
                                )
            #GIVE BACK :2 1 0
            StateMachine.add('GIVEBACK',
                                self.sm_GiveBack,
                                remapping ={'name_id':'name_id','name_list':'name_list','target_list':'target_list'},
                                transitions ={'succeeded':'CHECKFINISH','aborted':'CHECKFINISH','error':'error'}
                                )

            StateMachine.add('CHECKFINISH',
                                CBState(self.checkfinish,outcomes=['finish','continue'],input_keys=['num_list']),
                                transitions={'finish':'ENDTASK',
                                             'continue':'GETTARGET'},
                                remapping={'num_list':'num_list'})
            StateMachine.add('ENDTASK',
                                self.sm_EndTask,
                                transitions ={'succeeded':'succeeded','aborted':'aborted','error':'error'})

                                                    

        intro_server = IntrospectionServer('WHOISWHO',self.WHOISWHO,'/WHOISWHO')
        intro_server.start()
        out = self.WHOISWHO.execute()
        intro_server.stop()

  
    def shutdown(self):
        if self.smach_bool ==True:
            rospy.loginfo('smach succeeded')
        else:
            rospy.loginfo('smach error')

    def checkfinish(self,ud):
        print ud.num_list
        if len(ud.num_list) == 0:
            return 'finish'
        else:
            return 'continue'

if __name__ == "__main__":
    try:
        WhoIsWho()
    except Exception,e:
        rospy.logerr(e)   
