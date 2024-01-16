#!/usr/bin/env python
# encoding:utf8
import rospy
from smach import *
from smach_ros import *
from smach_special.gpsr import * 
from smach_compose.compose import * 
from smach_common.common import * 
from xm_smach.target_gpsr import gpsr_target
from geometry_msgs.msg import *
import math
import subprocess
from control_msgs.msg import *
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from std_srvs.srv import Empty, EmptyRequest, EmptyResponse

'''
导航到客厅中预定开始地点->获取要拿起的袋子->抓取->找到并面对操作员->交互告诉操作员开始跟随->跟随操作员（记忆）->将袋子递交操作员-》等待感谢-》返回竞机场
                                                                                ->获取终止信号 
                                                                                ->丢失叫操作员
'''
class ClearMap(State):
    def __init__(self):
        State.__init__(self,outcomes =['succeeded','aborted'])
    
    def execute(self,userdata):
        try:
            # subprocess.call(["rosservice","call","/move_base/clear_costmaps"])
            subprocess.call('rosservice call /move_base/clear_costmaps "{}" ' , shell = True)
            
        except:
            return 'aborted'
        return 'succeeded'

class Carry_Platdown(State):
    def __init__(self):
        State.__init__(self,outcomes=['succeeded', 'error'])
        self.plat_client = actionlib.SimpleActionClient("/plat_controller/follow_joint_trajectory", GripperCommandAction)
    
    def execute(self,userdata):
        goal = GripperCommandGoal()
        goal.command.position = -0.05
        rospy.logwarn("^^^^^^^^^^^^^^")
        self.plat_client.wait_for_server()
        self.plat_client.send_goal(goal)
        rospy.logwarn("**************8")
        self.plat_client.wait_for_result(rospy.Duration(10.0))
        rospy.loginfo("...set the plat") 

        return 'succeeded'

class Carry_Before(State):
    '''
    before carry
    '''
    def __init__(self):
        State.__init__(self, outcomes=['succeeded', 'aborted','error'],
                        input_keys = ['angle'])
        self.arm_client = actionlib.SimpleActionClient("/arm_controller/follow_joint_trajectory", FollowJointTrajectoryAction)
    def execute(self,userdata):
        
        trajectory = JointTrajectory()
        joint_names = ["arm_joint_0", "arm_joint_1", "arm_joint_2","arm_joint_3", "arm_joint_4", "arm_joint_5"]	
        trajectory.joint_names = joint_names

        pos_demo = JointTrajectoryPoint()
        pos_demo.positions = [0.0 for i in joint_names]
        pos_demo.velocities = [0.0 for i in joint_names]
        pos_demo.accelerations = [0.0 for i in joint_names]
        pos_demo.time_from_start = rospy.Duration(5.0)

        arm_waypoints = list()
        joints = [userdata.angle,-1.4,3.14,0,0,0]

        pos = deepcopy(pos_demo)
        pos.positions = joints
        trajectory.points.append(pos)

        goal = FollowJointTrajectoryGoal()
        goal.trajectory = trajectory
        goal.goal_time_tolerance = rospy.Duration(0.0)

        self.arm_client.wait_for_server()

        rospy.logwarn("^^^^^^^^^^^^^^")
        self.arm_client.send_goal(goal)
        rospy.logwarn("&&&&&&&&&&&&&&&")
        self.arm_client.wait_for_result(rospy.Duration(60.0))

        rospy.sleep(5.0)
        return 'succeeded'

class Carry(State):
    '''
    carry
    '''
    def __init__(self):
        State.__init__(self, outcomes=['succeeded', 'aborted','error'],
                        input_keys = ['angle'])
        self.arm_client = actionlib.SimpleActionClient("/arm_controller/follow_joint_trajectory", FollowJointTrajectoryAction)
    def execute(self,userdata):

        self.arm_client.wait_for_server()
   
        trajectory = JointTrajectory()
        joint_names = ["arm_joint_0", "arm_joint_1", "arm_joint_2","arm_joint_3", "arm_joint_4", "arm_joint_5"]	
        trajectory.joint_names = joint_names

        pos_demo = JointTrajectoryPoint()
        pos_demo.positions = [0.0 for i in joint_names]
        pos_demo.velocities = [0.0 for i in joint_names]
        pos_demo.accelerations = [0.0 for i in joint_names]
        pos_demo.time_from_start = rospy.Duration(5.0)

        arm_waypoints = list()
        joints = [userdata.angle,0,0,0,0,0]

        pos = deepcopy(pos_demo)
        pos.positions = joints
        trajectory.points.append(pos)

        goal = FollowJointTrajectoryGoal()
        goal.trajectory = trajectory
        goal.goal_time_tolerance = rospy.Duration(0.0)

        self.arm_client.send_goal(goal)
        self.arm_client.wait_for_result(rospy.Duration(60.0))

        rospy.sleep(8.0)
        return 'succeeded'
class Put_Down(State):
    '''
    put package down
    '''
    def __init__(self):
        State.__init__(self, outcomes=['succeeded', 'aborted','error'],
                        input_keys = ['angle'])
        self.arm_client = actionlib.SimpleActionClient("/arm_controller/follow_joint_trajectory", FollowJointTrajectoryAction)
    def execute(self,userdata):
        
        self.arm_client.wait_for_server()

        trajectory = JointTrajectory()
        joint_names = ["arm_joint_0", "arm_joint_1", "arm_joint_2","arm_joint_3", "arm_joint_4", "arm_joint_5"]	
        trajectory.joint_names = joint_names

        pos_demo = JointTrajectoryPoint()
        pos_demo.positions = [0.0 for i in joint_names]
        pos_demo.velocities = [0.0 for i in joint_names]
        pos_demo.accelerations = [0.0 for i in joint_names]
        pos_demo.time_from_start = rospy.Duration(5.0)

        arm_waypoints = list()
        joints = [userdata.angle,0,0,0,1.2,0]

        pos = deepcopy(pos_demo)
        pos.positions = joints
        trajectory.points.append(pos)

        goal = FollowJointTrajectoryGoal()
        goal.trajectory = trajectory
        goal.goal_time_tolerance = rospy.Duration(0.0)

        self.arm_client.send_goal(goal)
        self.arm_client.wait_for_result(rospy.Duration(60.0))

        rospy.sleep(1.0)
        return 'succeeded'


class CarryMyLuggage():
    def __init__(self):
        rospy.init_node('CarryMyLuggage_Smach')
        rospy.on_shutdown(self.shutdown)
        rospy.logerr('Welcome to CarryMyLuggage!!!')
        self.smach_bool = False
        #add the function_smach here!
       
        self.xm_Find = StateMachine(outcomes = ['succeeded','aborted','error'])        
        with self.xm_Find:
            self.xm_Find.userdata.rec = 2.0
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
            self.xm_Find.userdata.pos_xm  =Pose()
            #用FindPeople().find_people_监视器找人的位置
            StateMachine.add('GET_PEOPLE_POS',
                                FindPeople().find_people_,
                                transitions ={'invalid':'CLOSECAMERA','valid':'GET_PEOPLE_POS','preempted':'aborted'},
                                remapping = {'pos_xm':'pos_xm'}
                                )      
            StateMachine.add('NAV_PEOPLE',
                                NavStack(),
                                transitions = {'succeeded':'SPEAK','aborted':'NAV_PEOPLE','error':'error'},
                                remapping = {'pos_xm':'pos_xm'})
            self.xm_Find.userdata.sentences = 'I find you'
            StateMachine.add('SPEAK',
                                Speak(),
                                transitions = {'succeeded':'succeeded','aborted':'aborted','error':'error'},
                                remapping = {'sentences':'sentences'})

            # close the KinectV2
            StateMachine.add('CLOSECAMERA',
                                    CloseCamera(),
                                    transitions ={'succeeded':'NAV_PEOPLE','aborted':'aborted'})           
        #----FOLLOW-----#
        #xm_Follow包括两个并发状态，FOLLOW和STOP
        #STOP用于语音获取stop信号
        #FOLLOW包装了FindPeople().find_people_监视器和meta_nav
        #当找到人需要移动时，状态转移到meta_nav
        #meta_nav后,再继续进入监视状态
        #meta_nav是一个并发状态机，包装了NAV和WAIT
        #WAIT用于记录时间引发time_over
        #nav成功或中止后，与time_over一起进入继续找人状态

        self.xm_Follow = Concurrence(outcomes = ['succeeded','aborted'],
                                 default_outcome = 'aborted',
                                 outcome_map={'succeeded':{'STOP':'stop'},
                                              'aborted':{'FOLLOW':'aborted'},
                                              'aborted':{'FOLLOW':'preempted'}},
                                 child_termination_cb = self.child_cb)   
        with self.xm_Follow:

            self.meta_follow = StateMachine(['succeeded','aborted','preempted'])

            with self.meta_follow:

                self.meta_follow.userdata.pos_xm = Pose()
                self.meta_follow.userdata.rec = 0.2

                StateMachine.add('FIND',
                                    LegTracker0().tracker,#摄像头给订阅的话题发消息来跟踪人，在follow中
                                    transitions = {'invalid':'WAIT','valid':'FIND','preempted':'preempted'},
                            
                                    remapping={'pos_xm':'pos_xm'})
                StateMachine.add('WAIT' , 
                                    Wait(),#位于tool中，给rec赋值，沉睡0s
                                    transitions={'succeeded':'META_NAV' , 'error':'META_NAV'})
                
                self.meta_nav = Concurrence(outcomes=['time_over','get_pos','aborted'],
                                                default_outcome  = 'aborted',
                                                outcome_map={'time_over':{'WAIT':'succeeded'},
                                                             'get_pos':{'NAV':'succeeded'},
                                                             'aborted':{'NAV':'aborted'}},
                                                child_termination_cb=self.nav_child_cb,
                                                input_keys=['pos_xm'])
                with self.meta_nav:
                    Concurrence.add('NAV',NavStack0(),remapping={'pos_xm':'pos_xm'})#跟随人
                    Concurrence.add('WAIT',Wait_trace())#以一定频率发移动指令
                StateMachine.add('META_NAV',
                                    self.meta_nav,
                                    transitions={'get_pos':'FIND','time_over':'FIND','aborted':'FIND'})
            Concurrence.add('FOLLOW',self.meta_follow)
            Concurrence.add('STOP',CheckStop())
        
        
        #-------PICK_UP-------#
        #RUNNODE_IMG-->GETNAME-->GET_POS-->NAV-->FIND-->POS_JUS-->NAV-->RUNNODE_IMG-->FIND-->POS_JUS-->PICK-->SPEAK
        #运行图像节点，获取物体名字和大物体位置后，nav到大物体位置，然后用图像找物体，然后用pos_jus得出适宜抓取的坐标，nav后继续循环一次
        #在此之后，进行抓取
        #如果上面的图像没有找到物体，则通过speak进行错误反馈
        self.xm_Pick_up = StateMachine(outcomes =['succeeded','aborted','error'])    
        with self.xm_Pick_up: 
            self.xm_Pick_up.userdata.sentences = 'Please hang the bag up to the claw'

            StateMachine.add('SPEAK_BEFORE_PICK',
                                Speak(),
                                transitions = {'succeeded':'succeeded','aborted':'aborted','error':'error'},
                                remapping = {'sentences':'sentences'}
                            )  

            # self.xm_Pick_up.userdata.sentences = 'Now I will follow you and help you carry the luggage'

            # StateMachine.add('SPEAK_AFTER_PICK',
            #                     Speak(),
            #                     transitions = {'succeeded':'succeeded','aborted':'aborted','error':'error'},
            #                     remapping = {'sentences':'sentences'}
            #                 )       
        
        '''
        上层为GPSR通用状态机
        '''
        #顶层状态机
        #HELP
        self.xm_HELP = StateMachine(outcomes = ['succeeded','aborted','error'])
        with self.xm_HELP:
            self.xm_HELP.userdata.start_pos = gpsr_target['livingroom']['pos']#xm开始的预定位置
            #self.xm_HELP.userdata.start_pos = gpsr_target['test_pos']['pos']
            self.xm_HELP.userdata.finish_pos = gpsr_target['dining_room_exit']['pos']
            self.xm_HELP.userdata.sentences = 'which luggage you want' #for the Speak() state
            self.xm_HELP.userdata.start_follow = 'Now I will follow you and help you carry the luggage.We can go to the car.'#for the Speak() state
            self.xm_HELP.userdata.finish = 'Please take your luggage out of my claw'
            self.xm_HELP.userdata.current_task = -1
            self.xm_HELP.userdata.current_turn = -1
            self.xm_HELP.userdata.turn = 0
            self.xm_HELP.userdata.straight_angle = 0

            # StateMachine.add('PLAT_COMMAND',
            #                 Carry_Platdown(),
            #                 transitions={'succeeded': 'CARRY_BEFORE', 'error': 'error'}
            #                 )

            # StateMachine.add('CARRY_BEFORE',
            #                 Carry_Before(),
            #                 transitions={'succeeded':'START','aborted':'aborted','error':'error'},
            #                 remapping={'angle':'straight_angle'}#attention the angle
            #                 )


            StateMachine.add('START',
                            NavStack(),
                            transitions ={'succeeded':'FIND_PEOPLE','aborted':'aborted','error':'error'},
                            remapping ={"pos_xm":'start_pos'}
                            )

            StateMachine.add('FIND_PEOPLE',
                            self.xm_Find,
                            transitions ={'succeeded':'SPEAK','aborted':'aborted','error':'error'},
                            remapping ={"pos_xm":'start_pos'}
                            )   

            # StateMachine.add('FOLLOW_PEOPLE',
            #                 self.xm_Follow,
            #                 transitions={'succeeded':'SPEAK','aborted':'FOLLOW_PEOPLE'}) 

            StateMachine.add('SPEAK',
                            Speak(),
                            transitions = {'succeeded':'PICK_UP','aborted':'aborted','error':'error'},
                            remapping = {'sentences':'sentences'}
                            )

            # StateMachine.add('PICK',
            #                 Carry(),
            #                 transitions={'succeeded':'PICK_UP','aborted':'aborted','error':'error'},
            #                 remapping={'angle':'straight_angle'}#attention the angle
            #                 )

            StateMachine.add('PICK_UP',
                            self.xm_Pick_up,
                            transitions={'succeeded':'RUNNODE','aborted':'PICK_UP','error':'error'})
            
            StateMachine.add('RUNNODE',
                                RunNode(),
                                transitions={'succeeded':'SPEAK_AFTER_PICK','aborted':'RUNNODE'})
            
            StateMachine.add('SPEAK_AFTER_PICK',
                            Speak(),
                            transitions = {'succeeded':'FOLLOW','aborted':'aborted','error':'error'},
                            remapping = {'sentences':'start_follow'}
                            )                
            StateMachine.add('FOLLOW',
                                self.xm_Follow,
                                transitions={'succeeded':'RELEASE','aborted':'FOLLOW'})

            StateMachine.add('RELEASE',
                                Speak(),
                                transitions={'succeeded':'CLEAN','aborted':'RELEASE'},
                                remapping = {'sentences':'finish'})
            StateMachine.add('PUTDOWN',
                                Put_Down(),
                                transitions={'succeeded':'CLEAN','aborted':'aborted','error':'error'},
                                remapping={'angle':'straight_angle'}#attention the angle
                                )
            StateMachine.add('CLEAN',
                                ClearMap(),
                                transitions={'succeeded':'BACK','aborted':'BACK'})
            StateMachine.add('BACK',
                            NavStack(),
                            transitions ={'succeeded':'succeeded','aborted':'aborted','error':'error'},
                            remapping ={"pos_xm":'finish_pos'}
                            )   

            StateMachine.add('CHECK_TURN',
                CheckTurn(),
                transitions = {'succeeded':'GO_OUT','continue':'START','error':'error'}
            )   

            StateMachine.add('GO_OUT',
                                NavStack(),
                                transitions ={'succeeded':'succeeded','aborted':'aborted','error':'error'},
                                remapping ={"pos_xm":'start_pos'}
                            )      
        
        #内部检测服务器，用于检测状态及状态   
        intro_server = IntrospectionServer('xm_HELP',self.xm_HELP,'/XM_ROOT')
        intro_server.start()

        #运行顶层状态机
        out = self.xm_HELP.execute()
        intro_server.stop()

    #关闭状态机
    def shutdown(self):
        if self.smach_bool ==True:
            rospy.loginfo('smach succeeded')
        else:
            rospy.loginfo('smach error')

    #return True down
    def child_cb(self,outcome_map):
        if outcome_map['STOP'] == 'stop':
            rospy.logwarn('---------get the signal of stop,stop tracing---------')
            pid = get_pid("people_tracking")
            subprocess.Popen(['kill','-9',pid],shell=True)
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
            print(outcome_map)
            return False
        
    # use for concurrence
if __name__ == "__main__":
    try:
        CarryMyLuggage()
    except:
        rospy.logerr(e)   
