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
'drink': {'pos': Pose(Point(3.4577, 1.4577, 0), Quaternion(0, 0, -0.78206, 0.6232)), 'mode': 1 },
'food': {'pos': Pose(Point(2.1256, 1.021, 0), Quaternion(0, 0, 0.99505, 0.099381)), 'mode': 1 },
'clean': {'pos': Pose(Point(4.4669, 1.9477, 0), Quaternion(0, 0, -0.011094, 0.99994)), 'mode': 1 },


'test_pos': {'pos': Pose(Point(1,0,0), Quaternion(0, 0, 1, 0)), 'mode': 1 },
'pick_up': {'pos': Pose(Point(0.37587, -0.54148, 0), Quaternion(0, 0, -0.70501, 0.7092)), 'mode': 1 },
'put_down': {'pos': Pose(Point(1.677, -0.99258, 0), Quaternion(0, 0, -0.70871, 0.7055)), 'mode': 1 },
'open_door_pose_test': {'pos': Pose(Point(0,0,0), Quaternion(0, 0, 0, 1)), 'mode': 1 },
'open_door_pose': {'pos': Pose(Point(0.0050598, -0.0085862, 0), Quaternion(0, 0, -0.32845, 0.94452)), 'mode': 1 },
'exit_pos': {'pos': Pose(Point(0.0050598, -0.0085862, 0), Quaternion(0, 0, -0.32845, 0.94452)), 'mode': 1 },
'instruction_pos': {'pos': Pose(Point(2.3746, 1.2307, 0), Quaternion(0, 0, 0.2558, 0.9667)), 'mode': 1 },

'who_observe_1': {'pos': Pose(Point(4.2227, 1.8187, 0), Quaternion(0, 0, -0.026148, 0.99966)), 'mode': 1 },
'who_observe_2': {'pos': Pose(Point(4.1674, 1.7413, 0), Quaternion(0, 0, 0.99772, -0.067511)), 'mode': 1 },
'who_observe_3': {'pos': Pose(Point(7.2871, 5.5477, 0), Quaternion(0, 0, 0.037944, 0.99928)), 'mode': 1 },
'receptionist_pose_test': {'pos': Pose(Point(2, 0, 0), Quaternion(0, 0, 0, 1)), 'mode': 1 },
'receptionist_pose': {'pos': Pose(Point(-0.7983, -2.5627, 0), Quaternion(0, 0, -0.022434, 0.99975)), 'mode': 1 },
'point_pose': {'pos': Pose(Point(0.63953, -0.76623, 0), Quaternion(0, 0, 0.97676, 0.21436)), 'mode': 1 },
'speaker': {'pos': Pose(Point(1.4242, -1.7244, 0), Quaternion(0, 0, 0.37287, 0.92788)), 'mode': 1 },
#######################living_room##########
'livingroom':{'pos':Pose(Point(-0.010213, -0.044544, 0), Quaternion(0, 0, 0.0058168, 0.99998)), 'mode':1 },
'livingroom_end_table': {'pos': Pose(Point(5.1925, 0.092607, 0),Quaternion(0, 0, 0.7059, 0.70831)), 'mode':1 },
'left_short_sofa': {'pos': Pose(Point(3.355, 2.5128, 0),Quaternion(0, 0, 0.66233, 0.74921)), 'mode':1 },
'left_end_table': {'pos': Pose(Point(3.8759, 2.5006, 0),Quaternion(0, 0, 0.72489, 0.68887)), 'mode':1 },
'right_short_sofa': {'pos': Pose(Point(4.7528, 2.496, 0),Quaternion(0, 0, 0.74078, 0.67174)), 'mode':1 },
'right_end_table': {'pos': Pose(Point(4.2713, 1.7869, 0),Quaternion(0, 0, -0.6544, 0.75615)), 'mode':1 },
'living_room_couch': {'pos': Pose(Point(6.5356, -0.40736, 0),Quaternion(0, 0, 0.68955, 0.72424)), 'mode':1 },
'living_room_entrance': {'pos': Pose(Point(2.042, 0, 0),Quaternion(0, 0, 0.286, 0.958)), 'mode':1 },
'living_room_bookcase': {'pos': Pose(Point(-0.51974, -2.6731, 0), Quaternion(0, 0, -0.07896, 0.99688)), 'mode':1 },
#######################bedroom##########
'bedroom':{'pos':Pose(Point(0.92182, -0.091826, 0), Quaternion(0, 0, 0.013846, 0.9999)), 'mode': 1 },
'bedroom_1':{'pos':Pose(Point(3.952, 4.2913, 0), Quaternion(0, 0, 0.43024, 0.90272)), 'mode': 1 },
'bedroom_2':{'pos':Pose(Point(4.6161, 5.4275, 0), Quaternion(0, 0, 0.45085, 0.8926)), 'mode': 1 },

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
'kitchen': {'pos': Pose(Point(1.4361, 3.0826, 0), Quaternion(0, 0, 0.80494, 0.59335)), 'mode': 1 },
'kitchen_1': {'pos': Pose(Point(0.47087, 2.9635, 0), Quaternion(0, 0, 0.85207, 0.52343)), 'mode': 1 },
'kitchen_2': {'pos': Pose(Point(1.6552, 6.1217, 0), Quaternion(0, 0, 0.55246, 0.83354)), 'mode': 1 },
'kitchen_dishwasher': {'pos': Pose(Point(5.3050, 5.35, 0), Quaternion(0, 0, 0.6795, 0.7337)), 'mode': 1 },
'kitchen_counter': {'pos': Pose(Point(1.669, -0.89225, 0), Quaternion(0, 0, -0.75776, 0.65254)), 'mode': 1 },
'kitchen_sink': {'pos': Pose(Point(4.3526, 5.3636, 0), Quaternion(0, 0, 0.6755, 0.7374)), 'mode': 1 },
'right_cooking_bench': {'pos': Pose(Point(6.0458, 5.45, 0), Quaternion(0, 0, 0.68816, 0.71274)), 'mode': 1 },
'kitchen_storage_table': {'pos': Pose(Point(2.1581, 4.7448, 0), Quaternion(0, 0, -0.7355, 0.6775)), 'mode': 1 },
'kitchen_cupboard': {'pos': Pose(Point(5.903, 5.4143, 0), Quaternion(0, 0, 0.5849, 0.811)), 'mode': 1 },
#################################StoringGroceries###########################################
'information_pos': {'pos': Pose(Point(-0.00039673, -0.021202, 0), Quaternion(0, 0, -0.003477, 0.99999)), 'mode': 1 },
############################################3333
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
