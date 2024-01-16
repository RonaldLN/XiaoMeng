#!/usr/bin/env python 
#encoding:utf8 

"""

	it is a stuff list for GPSR 
	it contains the pose of all the stuff
        
"""
from geometry_msgs.msg import *
from geometry_msgs.msg._Pose import Pose


# mode = 1 在桌子上
# mode = 2 在架子上
gpsr_target={

#############################################################
###########################通用以A为入口###############################
# kitchen
# # 储物柜
# 'room_object_pos1': {'pos': Pose(Point(8.2833, 0.97924, 0), Quaternion(0, 0, 0.27641, 0.96104)), 'mode': 1 },
# 'room_object_pos2': {'pos': Pose(Point(8.6519, 0.58662, 0), Quaternion(0, 0, 0.3897, 0.92094)), 'mode': 1 },

# # living_room
# # 茶几 
# 'room_object_pos1': {'pos': Pose(Point(1.7373, 0.9539, 0), Quaternion(0, 0, 0.42352, 0.90589)), 'mode': 1 },
# 'room_object_pos2': {'pos': Pose(Point(2.7522, 0.68115, 0), Quaternion(0, 0, 0.65226, 0.75799)), 'mode': 1 },
# 电视柜
'room_object_pos1': {'pos': Pose(Point(2.3529, 0.96717, 0), Quaternion(0, 0, -0.63064, 0.77607)), 'mode': 1 },
'room_object_pos2': {'pos': Pose(Point(3.0907, 0.92901, 0), Quaternion(0, 0, -0.66196, 0.74954)), 'mode': 1 },


# bedroom
# 梳妆台
# 'room_object_pos1': {'pos': Pose(Point(4.5861, 4.6758, 0), Quaternion(0, 0, 0.72281, 0.69105)), 'mode': 1 },
# 'room_object_pos2': {'pos': Pose(Point(4.0998, 4.783, 0), Quaternion(0, 0, 0.61308, 0.79002)), 'mode': 1 },

# # 学习卓
# 'room_object_pos1': {'pos': Pose(Point(4.2757, 4.9711, 0), Quaternion(0, 0, -0.68753, 0.72616)), 'mode': 1 },
# 'room_object_pos2': {'pos': Pose(Point(3.5197, 4.9212, 0), Quaternion(0, 0, -0.59548, 0.80337)), 'mode': 1 },

# normal
'start_pos':{'pos':Pose(Point(6.417, 5.0378, 0), Quaternion(0, 0, -0.31509, 0.94906)), 'mode':1 },
'exit_pos':{'pos':Pose(Point(10.904, 6.4459, 0), Quaternion(0, 0, 0.65368, 0.75677)), 'mode':1 },


#############################################################
###########################通用以B为入口###############################
# kitchen
# 储物柜
# 'room_object_pos1': {'pos': Pose(Point(6.3049, -1.1735, 0), Quaternion(0, 0, 0.99899, 0.045009)), 'mode': 1 },
# 'room_object_pos2': {'pos': Pose(Point(5.7831, -2.3902, 0), Quaternion(0, 0, 0.88106, 0.473)), 'mode': 1 },

# # bedroom
# # 梳妆台
# 'room_object_pos1': {'pos': Pose(Point(2.2942, -6.536, 0), Quaternion(0, 0, 0.99329, 0.11565)), 'mode': 1 },
# 'room_object_pos2': {'pos': Pose(Point(1.9157, -7.2327, 0), Quaternion(0, 0, 0.87748, 0.47961)), 'mode': 1 },
# # 学习桌
# 'room_object_pos1': {'pos': Pose(Point(2.1439, -7.1704, 0), Quaternion(0, 0, 0.024534, 0.9997)), 'mode': 1 },
# 'room_object_pos2': {'pos': Pose(Point(2.3116, -6.4353, 0), Quaternion(0, 0, -0.092313, 0.99573)), 'mode': 1 },

# # dining_room
# # 储物柜
# 'room_object_pos1': {'pos': Pose(Point(1.634, -1.1122, 0), Quaternion(0, 0, 0.8736, -0.48664)), 'mode': 1 },
# 'room_object_pos2': {'pos': Pose(Point(1.77, -3.4478, 0), Quaternion(0, 0, 0.9094, 0.41592)), 'mode': 1 },

# # 


# # normal
# 'start_pos':{'pos':Pose(Point(6.6317, -6.8338, 0), Quaternion(0, 0, 0.94568, -0.3251)), 'mode':1 },
# 'exit_pos':{'pos':Pose(Point(7.5336, -10.092, 0), Quaternion(0, 0, -0.73408, 0.67906)), 'mode':1 }

#############################################################


}
