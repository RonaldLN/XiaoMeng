from turtle import pos
import rospy
from smach import *
from smach_ros import *
from xm_msgs.srv import *
from xm_msgs.msg import *
from geometry_msgs.msg import *
import actionlib
from actionlib_msgs.msg import GoalStatus
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal, MoveBaseActionFeedback
import tf
from tf.transformations import quaternion_from_euler, euler_from_quaternion
from math import *
from std_srvs.srv import Empty, EmptyRequest, EmptyResponse
from geometry_msgs.msg import *

pubulisher = rospy.Publisher(
    '/move_base_simple/goal', PointStamped, queue_size=1)

rospy.init_node("move_base_simple")
pose = PointStamped()




pubulisher.publish(pose)
