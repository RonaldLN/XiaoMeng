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
'room_object_pos1': {'pos': Pose(Point(2.8003, 2.5387, 0), Quaternion(0, 0, -0.0415, 0.9999)), 'mode': 1 },
'room_object_pos2': {'pos': Pose(Point(5.2984, 1.0299, 0), Quaternion(0, 0, 0.95544, 0.29381)), 'mode': 1 },

# # living_room
# 茶几 
# 'room_object_pos1': {'pos': Pose(Point(1.7373, 0.9539, 0), Quaternion(0, 0, 0.42352, 0.90589)), 'mode': 1 },
# 'room_object_pos2': {'pos': Pose(Point(2.7522, 0.68115, 0), Quaternion(0, 0, 0.65226, 0.75799)), 'mode': 1 },
# 电视柜
# 'room_object_pos1': {'pos': Pose(Point(4.1317, 1.0371, 0), Quaternion(0, 0, -0.85673, 0.51576)), 'mode': 1 },
# 'room_object_pos2': {'pos': Pose(Point(3.0907, 0.92901, 0), Quaternion(0, 0, -0.66196, 0.74954)), 'mode': 1 },

#kitchen 
#灶台
# 'room_object_pos2': {'pos': Pose(Point(7.2285, 0.4494, 0), Quaternion(0, 0, -0.39197, 0.91998)), 'mode': 1 },
# 'room_object_pos1': {'pos': Pose(Point(7.9941, 0.5225, 0), Quaternion(0, 0, -0.67211, 0.74045)), 'mode': 1 },

# # 清晰台9.5943, 0.31776, 0
# 'room_object_pos2': {'pos': Pose(Point(9.5943, 0.31776, 0), Quaternion(0, 0, -0.055022, 0.99849)), 'mode': 1 },
# 'room_object_pos1': {'pos': Pose(Point(9.5943, 0.31776, 0), Quaternion(0, 0, -0.055022, 0.99849)), 'mode': 1 },


# bedroom
# 梳妆台
# 'room_object_pos1': {'pos': Pose(Point(4.5861, 4.6758, 0), Quaternion(0, 0, 0.72281, 0.69105)), 'mode': 1 },
# 'room_object_pos2': {'pos': Pose(Point(4.0998, 4.783, 0), Quaternion(0, 0, 0.61308, 0.79002)), 'mode': 1 },

# # 学习卓
# 'room_object_pos1': {'pos': Pose(Point(4.2757, 4.9711, 0), Quaternion(0, 0, -0.68753, 0.72616)), 'mode': 1 },
# 'room_object_pos2': {'pos': Pose(Point(3.5197, 4.9212, 0), Quaternion(0, 0, -0.59548, 0.80337)), 'mode': 1 },

# normal NO
'start_pos':{'pos':Pose(Point(3.3289, 0.61974, 0), Quaternion(0, 0, 0.95702, 0.29003)), 'mode':1 },
'exit_pos':{'pos':Pose(Point(3.3289, 0.61974, 0), Quaternion(0, 0, 0.95702, 0.29003)), 'mode':1 },

# normal
# 'start_pos':{'pos':Pose(Point(4.7037, 0.83823, 0), Quaternion(0, 0, 0.9621, 0.2727)), 'mode':1 },
# 'exit_pos':{'pos':Pose(Point(10.904, 6.4459, 0), Quaternion(0, 0, 0.65368, 0.75677)), 'mode':1 },


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
# # # 储物柜
# 'room_object_pos1': {'pos': Pose(Point(1.634, -1.1122, 0), Quaternion(0, 0, 0.8736, -0.48664)), 'mode': 1 },
# 'room_object_pos2': {'pos': Pose(Point(1.77, -3.4478, 0), Quaternion(0, 0, 0.9094, 0.41592)), 'mode': 1 },

# 参作
# 'room_object_pos1': {'pos': Pose(Point(4.0605, -2.9085, 0), Quaternion(0, 0, 0.9895, 0.14452)), 'mode': 1 },
# 'room_object_pos2': {'pos': Pose(Point(4.0605, -2.9085, 0), Quaternion(0, 0, 0.9895, 0.14452)), 'mode': 1 },

# # # 
#kitchen 
# #灶台
# 'room_object_pos2': {'pos': Pose(Point(6.2992, -3.0348, 0), Quaternion(0, 0, 0.35829, 0.93361)), 'mode': 1 },
# 'room_object_pos1': {'pos': Pose(Point(6.1946, -2.5294, 0), Quaternion(0, 0, 0.059621, 0.99822)), 'mode': 1 },

# # 清晰台9.5943, 0.31776, 0
# 'room_object_pos2': {'pos': Pose(Point(6.0682, -0.94913, 0), Quaternion(0, 0, 0.6987, 0.71542)), 'mode': 1 },
# 'room_object_pos1': {'pos': Pose(Point(6.0682, -0.94913, 0), Quaternion(0, 0, 0.6987, 0.71542)), 'mode': 1 },




# # # # normal NO 
# 'start_pos':{'pos':Pose(Point(6.0992, -5.9771, 0), Quaternion(0, 0, -0.81439, 0.58031)), 'mode':1 },
# 'exit_pos':{'pos':Pose(Point(7.5336, -10.092, 0), Quaternion(0, 0, -0.73408, 0.67906)), 'mode':1 }

# # # normal
# 'start_pos':{'pos':Pose(Point(1.8963, -4.4564, 0), Quaternion(0, 0, 0.55939, 0.82891)), 'mode':1 },
# 'exit_pos':{'pos':Pose(Point(7.5336, -10.092, 0), Quaternion(0, 0, -0.73408, 0.67906)), 'mode':1 }

#############################################################


}
