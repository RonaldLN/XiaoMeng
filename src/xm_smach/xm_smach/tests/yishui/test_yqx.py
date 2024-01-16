from ctypes.wintypes import POINT
import rospy

from xm_msgs.msg import *





publisher = rospy.Publisher(
    '/move_base_simple/goal' , POINT ,queue_size=1
)

point = POINT()

publisher.publish(point)
