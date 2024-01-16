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
import copy
'''
Where is This
规则理解：
        从信息点出发引导一个人到某位置 X3
        要求以导游的形式
状态机:
        由于导航的时候直接规划出路径，本项目要求以导游的形式进行引导，在导航的过程中，只能每走到一个小位置，走的走的过程中说出标志性语句，
        汇总一下，最后走到目的地
        所以导航的策略发生变化，比如要从信息点到A点，先查询A点属于哪个房间，
        再从事先规划的路径表里查询，这个表事先存储了从A房间到B房间需要经过的房间，
        到达下一个房间时，因为语音描述和导航时同步的，这就要求开始说时几乎已经离开上一个房间了
        假设每个房间只有一个门，这样每个房间的导航点则为房间内靠近这个门，在路径点列表中找到，同时，含有一句话描述本房间信息,
        同时，路劲规划表里要注意合理性，在每走一个地方会说话
语音：
        获取目的地，详见GetDestination的定义
        ×××提供每次互动的录音和笔录，加分很多×××

问题:
        对途中突然出现的障碍物没有一点办法
        可能出现过程不流畅的现象
        路径规划表并不是规划出来的，而是事先存储的，配合每个房间导航时说的的information，在设置时需要花一点功夫，
        考虑到到时候项目较多，可能会委托别人事先设计路线和information
        对回头客的理解不清

'''

#用于查询物体属于的房间和每个房间的路径点和描述
TargetList={
    
    'livingroom_i': {'pos': Pose(Point(0.97924, -0.10189, 0), Quaternion(0, 0, -0.016398, 0.99987)), 'information': "the table and sofa" ,'room':'livingroom'},
    'livingroom_o': {'pos': Pose(Point(0.94983, -0.39899, 0), Quaternion(0, 0, 0.0012617, 1)), 'information': "the table and sofa" ,'room':'livingroom'},
    'diningroom_i': {'pos': Pose(Point(4.6709, -1.3557, 0), Quaternion(0, 0, 0.042216, 0.99911)), 'information': "three people" ,'room':'diningroom'},
    'diningroom_o': {'pos': Pose(Point(3.2848, -1.5118, 0), Quaternion(0, 0, 0.93583, -0.35246)), 'information': "three people" ,'room':'diningroom'},
    'bedroom_i': {'pos': Pose(Point(4.2013, -3.7768, 0), Quaternion(0, 0, 0.005402, 0.99999)), 'information': "the shelf and the bed" ,'room':'bedroom'},
    'bedroom_o': {'pos': Pose(Point(3.2867, -2.9962, 0), Quaternion(0, 0, 0.99771, 0.067644)), 'information': "" ,'room':'bedroom'},
    'sofa': {'pos': Pose(), 'information': "" ,'room':'livingroom'},
    'shelf': {'pos': Pose(Point(4.4566, -3.9481, 0) , Quaternion(0, 0, 0.01656, 0.99986)), 'information': "" ,'room':'bedroom'},
    'kitchen_i': {'pos': Pose(Point(2.1492, -4.8791, 0), Quaternion(0, 0, -0.81615, 0.57784)), 'information': "my brother" ,'room':'kitchen'},
    'kitchen_o': {'pos': Pose(Point(2.336, -4.1979, 0), Quaternion(0, 0, 0.51875, 0.85492)), 'information': "my brother" ,'room':'kitchen'},
    'speaker': {'pos': Pose(Point(1.3478, -0.024805, 0), Quaternion(0, 0, 0.011599, 0.99993)), 'information': "" ,'room':'bedroom'},
}
#信号点
start_room = "livingroom_i"

#路径点没有自身到自身，information里不能出现This is room之类的描述，而且前面加you will see  不会显得有语法错误
PathList = {
    'livingroom':{'diningroom':['livingroom_o','diningroom_i'],
                'bedroom':['livingroom_o','bedroom_i'],
                'kitchen':['livingroom_o','kitchen_i'],
                'livingroom':[]},
    'diningroom':{'livingroom':['livingroom_o','diningroom_i'],
                'bedroom':[],
                'kitchen':[]},
    'bedroom':{'diningroom':['livingroom_o','diningroom_i'],
                'livingroom':[],
                'kitchen':[]},
    'kitchen':{'diningroom':['livingroom_o','diningroom_i'],
                'bedroom':[],
                'livingrooom':[]},
}

class GetPath(State):
    '''
    获取路径点和引导描述(总)
    '''
    def __init__(self):
        State.__init__(self,outcomes =['succeeded','aborted','error'],
                        input_keys =['start_room','end_room','destination'],
                        output_keys =['waypoint_list','path_description','index'])        
    def execute(self,userdata):
        try:
            getattr(userdata,'start_room')
            print(userdata.start_room)
            getattr(userdata,'end_room')
            print(userdata.end_room)
            print(userdata,'destination')
        except:
            rospy.logerr('no param')
            return 'aborted'
        else:
            self.description ="Ok，I will guide you to there，Here is "+userdata.start_room+". "
            self.description +="In the "+userdata.start_room+". You will see "+TargetList[userdata.start_room]['information']+". "
            
            s_room = userdata.start_room.replace("_i","")
            print(s_room)
            waypoints = copy.deepcopy(PathList[s_room][userdata.end_room])
            print(waypoints)
            if TargetList[userdata.destination]['pos'] is Pose():
                waypoints.append(end_room+"_i") 
            else:
                waypoints.append(userdata.destination)
            userdata.waypoint_list = waypoints
            print(waypoints)
            for i in range(len(waypoints)-2):
                if i!=1:
                    self.description +="Then, "
                self.description += "You should go to the"+waypoints[i+1]+". "
                self.description += "In the "+waypoints[i+1]+", "
                self.description += "You will see "+TargetList[waypoints[i+1]]['information']+". "
            self.description +="Then you will see the " +str(userdata.destination)+". "
            description_1 = self.description.replace("_i","") 
            description_2 = description_1.replace("_o","")     
            userdata.path_description = description_2
            rospy.logwarn(description_2)
            userdata.index = 0
            
            return 'succeeded'

class GetDestination(State):
    '''
    获取目的地和目的房间
    '''
    def __init__(self):
        State.__init__(self,outcomes =['succeeded','aborted','error'],
                        output_keys =['destination','end_room'])

        self.client = rospy.ServiceProxy('xm_speech_meaning',xm_Speech_meaning)
    def execute(self,userdata):
        try:
            self.client.wait_for_service(timeout=10)
        except:
            rospy.logerr('xm_speech_meaning service is error')
            return 'aborted'
        else :
            res = self.client.call(command=7)
            try:
                
                try:
                    userdata.destination = res.answer
                    userdata.end_room = TargetList[str(res.answer)]['room']
                except:
                    if res.answer =="living room":
                        res.answer = "livingroom"
                    userdata.destination = res.answer+"_i"
                    userdata.end_room = res.answer

            except:
                rospy.logerr('can not find the room of destination ')
                return 'aborted'

            return 'succeeded'


class GetRoomAndWord(State):
    '''
    获取单个路径点的目的地和描述语句
    '''
    def __init__(self):
        State.__init__(self,outcomes =['continue','finish','error'],
                        input_keys =['waypoint_list'],
                        output_keys = ['target_room','sentences','goal'],
                        io_keys = ['index'])

    def execute(self,userdata):
        try:
            if userdata.index <len(userdata.waypoint_list):
                sentences = ""
                if userdata.index ==len(userdata.waypoint_list)-1:
                    print("####")
                    target_room = userdata.waypoint_list[userdata.index].replace("_i","").replace("_o","") 
                    sentences +="Now, you can see the"+target_room+". "

                else:
                    target_room = userdata.waypoint_list[userdata.index]
                    sentences = "Here is the "+target_room+". And you can see "+TargetList[target_room]['information']+". "
                
                userdata.goal = gpsr_target[target_room]['pos']

                sentences_tmp = sentences.replace("_i","") 
                userdata.sentences = sentences_tmp.replace("_o","") 
                userdata.target_room = target_room
                rospy.logwarn(userdata.index)
                rospy.logwarn(userdata.waypoint_list)
                rospy.logwarn(target_room)
                rospy.logwarn(sentences)
                userdata.index +=1
                return 'continue'
            else:
                return 'finish'
        except Exception,e:
            rospy.logerr(e)
            return 'error'


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


class WhereIsThis():
    def __init__(self):
        rospy.init_node('WhereIsThis_Smach')
        rospy.on_shutdown(self.shutdown)
        rospy.logwarn('Welcome to WhereIsThis!!!')
        self.smach_bool = False

        self.sm_Guide = StateMachine(outcomes = ['succeeded','aborted','error'],
                                    input_keys = ['waypoint_list','index'])  
        with self.sm_Guide:
            self.sm_Guide.userdata.index = 0

            self.meta_NavAndSpeak = Concurrence(outcomes=['succeeded','aborted'],
                                        default_outcome = 'succeeded',
                                        outcome_map = {'succeeded':{'NAV':'succeeded','SPEAK':'succeeded'},
                                                        'aborted':{'NAV':'aborted'}},
                                        input_keys=['target_room','sentences','goal'])
            with self.meta_NavAndSpeak:
                '''
                self.RepeatNav = StateMachine(outcomes = ['succeeded','aborted','error'],
                                    input_keys = ['pos_xm'])  
                with self.RepeatNav:
                    StateMachine.add('NAV',
                                        NavStack(),
                                        transitions = {'succeeded':'succeeded','aborted':'NAV','error':'error'},
                                        remapping = {'pos_xm':'pos_xm'}) 
                '''                       
                Concurrence.add('NAV',NavStack(),remapping={'pos_xm':'goal'})
                Concurrence.add('SPEAK',Speak(),remapping={'sentences':'sentences'})

        
            
            StateMachine.add('GETROOMANDWORD',
                    GetRoomAndWord(),
                    transitions={'continue':'NAVANDSPEAK','finish':'succeeded','error':'error'},
                    remapping = {'target_room':'target_room','sentences':'sentences','index':'index','waypoint_list':'waypoint_list','goal':'goal'}
                    )
            StateMachine.add('NAVANDSPEAK',
                    self.meta_NavAndSpeak,
                    transitions={'succeeded':'GETROOMANDWORD','aborted':'aborted'},
                    remapping = {'target_room':'target_room','sentences':'sentences','goal':'goal'}
                    )
             

        #顶层状态机
        self.WHEREISTHIS = StateMachine(outcomes = ['succeeded','aborted','error'])
        with self.WHEREISTHIS:
            self.WHEREISTHIS.userdata.start_room = start_room
            self.WHEREISTHIS.userdata.init_pos = gpsr_target[start_room]['pos']
            self.WHEREISTHIS.userdata.num = 1
            self.WHEREISTHIS.userdata.sentences_ask = 'Where do you want to go'
            
            StateMachine.add('NAV_INIT',
                                NavStack(),
                                transitions = {'succeeded':'ASK','aborted':'NAV_INIT','error':'error'},
                                remapping = {'pos_xm':'init_pos'})   
            StateMachine.add('ASK',
                                Speak(),
                                transitions={'succeeded':'GETDESTINATION','aborted':'aborted','error':'error'},
                                remapping = {'sentences':'sentences_ask'}
                                )
            StateMachine.add('GETDESTINATION',
                                GetDestination(),
                                transitions={'succeeded':'GETPATH','aborted':'ASK','error':'ASK'},
                                remapping = {'destination':'destination','end_room':'end_room'}
                                )
            StateMachine.add('GETPATH',
                                GetPath(),
                                transitions={'succeeded':'TELLWAY','aborted':'aborted','error':'error'},
                                remapping = {'start_room':'start_room','end_room':'end_room','waypoint_list':'waypoint_list','path_description':'path_description','index':'index'}
                                )
            StateMachine.add('TELLWAY',
                                Speak(),
                                transitions={'succeeded':'GUIDE','aborted':'aborted','error':'error'},
                                remapping = {'sentences':'path_description'}
                                )   
            StateMachine.add('GUIDE',
                                self.sm_Guide,
                                transitions={'succeeded':'CHECKTURN','aborted':'aborted','error':'error'},
                                remapping = {'waypoint_list':'waypoint_list'}
                                )   
            StateMachine.add('CHECKTURN',
                                CheckTurn(),
                                transitions={'succeeded':'succeeded','continue':'NAV_INIT','error':'error'},
                                remapping = {'num':'num'}
                                )   
                                                    

        intro_server = IntrospectionServer('WHEREISTHIS',self.WHEREISTHIS,'/WHEREISTHIS')
        intro_server.start()
        out = self.WHEREISTHIS.execute()
        intro_server.stop()

  
    def shutdown(self):
        if self.smach_bool ==True:
            rospy.loginfo('smach succeeded')
        else:
            rospy.loginfo('smach error')

if __name__ == "__main__":
    try:
        WhereIsThis()
    except Exception,e:
        rospy.logerr(e)   
