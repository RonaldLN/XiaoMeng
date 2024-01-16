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
'bottle': {'pos': Pose(Point(2.5917, 0.24196, 0), Quaternion(0, 0, -0.29153, 0.95656)), 'mode': 1 },
'food': {'pos': Pose(Point(2.3532, -0.14993, 0), Quaternion(0, 0, -0.71698, 0.6971)), 'mode': 1 },
'clean': {'pos': Pose(Point(1.4634, -0.17947, 0), Quaternion(0, 0, -0.70384, 0.71036)), 'mode': 1 },
'test_pos': {'pos': Pose(Point(1,0,0), Quaternion(0, 0, 1, 0)), 'mode': 1 },
'pick_up': {'pos': Pose(Point(0,0,0), Quaternion(0, 0, 0, 1)), 'mode': 1 },
'put_down': {'pos': Pose(Point(0,0,0), Quaternion(0, 0, 0, 1)), 'mode': 1 },
'open_door_pose_test': {'pos': Pose(Point(0,0,0), Quaternion(0, 0, 0, 1)), 'mode': 1 },
'open_door_pose': {'pos': Pose(Point(2.6402,-0.062398,0), Quaternion(0, 0, 1, 0.0087739)), 'mode': 1 },
'instruction_pos': {'pos': Pose(Point(4.1764, -0.1139, 0), Quaternion(0, 0, 0.2558, 0.9667)), 'mode': 1 },
'who_observe_1': {'pos': Pose(Point(4.2227, 1.8187, 0), Quaternion(0, 0, -0.026148, 0.99966)), 'mode': 1 },
'who_observe_2': {'pos': Pose(Point(4.1674, 1.7413, 0), Quaternion(0, 0, 0.99772, -0.067511)), 'mode': 1 },
'who_observe_3': {'pos': Pose(Point(7.2871, 5.5477, 0), Quaternion(0, 0, 0.037944, 0.99928)), 'mode': 1 },
'receptionist_pose_test': {'pos': Pose(Point(2, 0, 0), Quaternion(0, 0, 0, 1)), 'mode': 1 },
'receptionist_pose': {'pos': Pose(Point(5.0922, -0.16216, 0), Quaternion(0, 0, 0.67005, 0.74232)), 'mode': 1 },
'point_pose': {'pos': Pose(Point(1.5268, -1.6197, 0), Quaternion(0, 0, 0.36128, 0.93246)), 'mode': 1 },
'speaker': {'pos': Pose(Point(1.4242, -1.7244, 0), Quaternion(0, 0, 0.37287, 0.92788)), 'mode': 1 },
#######################living_room##########
'livingroom':{'pos':Pose(Point(4.226, -0.15923, 0), Quaternion(0, 0, 0, 1)), 'mode':1 },
'livingroom_end_table': {'pos': Pose(Point(5.1925, 0.092607, 0),Quaternion(0, 0, 0.7059, 0.70831)), 'mode':1 },
'left_short_sofa': {'pos': Pose(Point(3.355, 2.5128, 0),Quaternion(0, 0, 0.66233, 0.74921)), 'mode':1 },
'left_end_table': {'pos': Pose(Point(3.8759, 2.5006, 0),Quaternion(0, 0, 0.72489, 0.68887)), 'mode':1 },
'right_short_sofa': {'pos': Pose(Point(4.7528, 2.496, 0),Quaternion(0, 0, 0.74078, 0.67174)), 'mode':1 },
'right_end_table': {'pos': Pose(Point(4.2713, 1.7869, 0),Quaternion(0, 0, -0.6544, 0.75615)), 'mode':1 },
'living_room_couch': {'pos': Pose(Point(6.5356, -0.40736, 0),Quaternion(0, 0, 0.68955, 0.72424)), 'mode':1 },
'living_room_entrance': {'pos': Pose(Point(2.042, 0, 0),Quaternion(0, 0, 0.286, 0.958)), 'mode':1 },
'living_room_bookcase': {'pos': Pose(Point(2.4757, 1.2046, 0),Quaternion(0, 0, 0.69365, 0.72031)), 'mode':1 },
#######################bedroom##########
'bedroom':{'pos':Pose(Point(0.52701, 0.42515, 0), Quaternion(0, 0, 0.72987, 0.68359)), 'mode': 1 },
'bedroom_1':{'pos':Pose(Point(1.7052, 0.63668, 0), Quaternion(0, 0, 0.71289, 0.70128)), 'mode': 1 },
'bedroom_2':{'pos':Pose(Point(3.1395, 1.722, 0), Quaternion(0, 0, 0.70073, 0.71342)), 'mode': 1 },
'bedroom_desk':{'pos':Pose(Point(10.959, -0.17989, 0), Quaternion(0, 0, -0.73439, 0.67873)), 'mode':1 },
'bedroom_side_table':{'pos':Pose(Point(10.035, -0.3939, 0), Quaternion(0, 0, -0.73439, 0.67873)), 'mode':1 },
'bedroom_bookcase':{'pos':Pose(Point(10.543, 0.61378, 0), Quaternion(0, 0, 0.70734, 0.707)), 'mode':1 },
'bedroom_bed':{'pos':Pose(Point(10.7931, 0.4807, 0), Quaternion(0, 0, 0, 1)), 'mode':1 },
'dresser':{'pos':Pose(Point(8.7844, 1.2972, 0), Quaternion(0, 0, 0.78174, 0.62361)), 'mode':1 },
#######################diningroom############
'diningroom': {'pos': Pose(Point(8.68, 4.387, 0), Quaternion(0, 0, 0, 1)), 'mode': 1 },
'diningroom_dinning_table': {'pos': Pose(Point(8.88, 4.387, 0), Quaternion(0, 0, 0, 1)), 'mode': 1 },
'vase': {'pos': Pose(Point(6.651, 7.4104, 0), Quaternion(0, 0, 0.89151, 0.45301)), 'mode': 1 },
'food_table': {'pos': Pose(Point(7.7869, 6.9745, 0), Quaternion(0, 0, 0.70757, 0.70664)), 'mode': 1 },
'trash_can': {'pos': Pose(Point(7.2871, 5.5477, 0), Quaternion(0, 0, 0.037944, 0.99928)), 'mode': 1 },
'shelf': {'pos': Pose(Point(10.654, 3.5322, 0), Quaternion(0, 0, -0.714, 0.7)), 'mode': 1 },
'dining_room_exit':{'pos' : Pose(Point(0, 0, 0), Quaternion(0, 0, 0, 0)) , 'mode' : 1},

######################kitchen######################
'kitchen': {'pos': Pose(Point(4.3469, 4.7377, 0), Quaternion(0, 0, 0.9998, 0.018661)), 'mode': 1 },
'kitchen_dishwasher': {'pos': Pose(Point(5.3050, 5.35, 0), Quaternion(0, 0, 0.6795, 0.7337)), 'mode': 1 },
'kitchen_counter': {'pos': Pose(Point(4.1381, 4.594, 0), Quaternion(0, 0, -0.7174, 0.6967)), 'mode': 1 },
'kitchen_sink': {'pos': Pose(Point(4.3526, 5.3636, 0), Quaternion(0, 0, 0.6755, 0.7374)), 'mode': 1 },
'right_cooking_bench': {'pos': Pose(Point(6.0458, 5.45, 0), Quaternion(0, 0, 0.68816, 0.71274)), 'mode': 1 },
'kitchen_storage_table': {'pos': Pose(Point(2.1581, 4.7448, 0), Quaternion(0, 0, -0.7355, 0.6775)), 'mode': 1 },
'kitchen_cupboard': {'pos': Pose(Point(5.903, 5.4143, 0), Quaternion(0, 0, 0.5849, 0.811)), 'mode': 1 },
#########################################################################################
'sprite': {'pos': Pose(), 'mode': 1 },
'red-bull': {'pos': Pose(), 'mode': 1 },
'corridor': {'pos': Pose(Point(7.8492, 2.3128, 0), Quaternion(0, 0, 0.7286, 0.6849)), 'mode': 1 },


'tea': {'pos': Pose(), 'mode': 1 },
'juice': {'pos': Pose(), 'mode': 1 },
'coffee': {'pos': Pose(), 'mode': 1 },
'biscuit': {'pos': Pose(), 'mode': 1 },
'chips': {'pos': Pose(), 'mode': 1 },
'roll-paper': {'pos': Pose(), 'mode': 1 },
'toothpaste': {'pos': Pose(), 'mode': 1 },

'Gray': {'pos': Pose(), 'mode': 1 },
'David': {'pos': Pose(), 'mode': 1 },
'Daniel': {'pos': Pose(), 'mode': 1 },
'Jack': {'pos': Pose(), 'mode': 1 },
'Jenny': {'pos': Pose(), 'mode': 1 },
'Michael': {'pos': Pose(), 'mode': 1 },
'Lucy': {'pos': Pose(), 'mode': 1 },
'Peter': {'pos': Pose(), 'mode': 1 },
'Tom': {'pos': Pose(), 'mode': 1 },
'Jordan': {'pos': Pose(), 'mode': 1 },

'soap':{'pos': Pose(), 'mode': 1 },
'red bull':{'pos': Pose(), 'mode': 2 },
'coconut':{'pos': Pose(), 'mode': 2 },
'green bean':{'pos': Pose(), 'mode': 2 },
'cola':{'pos': Pose(), 'mode': 2 },
'tooth brush':{'pos': Pose(), 'mode': 1 },
'milk tea':{'pos': Pose(), 'mode': 2 },
'paper':{'pos': Pose(), 'mode': 1 }

}
