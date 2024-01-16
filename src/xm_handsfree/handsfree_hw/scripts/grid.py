#! /usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
from nav_msgs.msg import Path
from move_base_msgs.msg import MoveBaseActionGoal
import math
import tf.transformations as tf_trans


# from nav_msgs.msg import Odometry
# import actionlib
from geometry_msgs.msg import PoseWithCovarianceStamped


# 全局变量
processing_alternative = False
path_received = False
path_valid = True
tried_goals = []
pathPoses = None
external_goal = True
current_pose = None


def path_callback(msg):
    global path_received, path_valid, pathPoses
    path_received = True
    pathPoses = msg.poses
    path_valid = len(msg.poses) > 0


def find_alternative_goal(init_x, init_y, quaternion, step=0.5, max_iterations=100):
    global processing_alternative
    processing_alternative = True

    rospy.loginfo("Entered find_alternative_goal function.")

    # 从四元数获取yaw
    goal_yaw = quaternion_to_yaw(quaternion.x, quaternion.y, quaternion.z, quaternion.w)
    # 计算反方向
    reverse_yaw = goal_yaw + math.pi
    if reverse_yaw > math.pi:
        reverse_yaw -= 2 * math.pi

    dx = step * math.cos(reverse_yaw)
    dy = step * math.sin(reverse_yaw)

    for i in range(max_iterations):
        new_goal_x = init_x + i * dx
        new_goal_y = init_y + i * dy
        new_goal = (new_goal_x, new_goal_y)

        if new_goal not in tried_goals:
            tried_goals.append(new_goal)
            processing_alternative = False
            return new_goal

    processing_alternative = False
    return None, None


def goal_callback(data):
    global processing_alternative, path_received, path_valid, tried_goals, external_goal

    if processing_alternative:
        return

    if not external_goal:
        external_goal = True
        return
    rospy.loginfo("Received a goal!")
    # 初始化目标点
    init_x = data.goal.target_pose.pose.position.x
    init_y = data.goal.target_pose.pose.position.y
    quaternion = data.goal.target_pose.pose.orientation

    # 计算新的目标点，该点位于人前的指定距离，并朝向人
    distance = 1.0  # 自定义
    new_goal_x, new_goal_y, quaternion_towards_person = compute_new_goal(
        init_x, init_y, quaternion, distance
    )

    print(path_valid)
    # 如果已经有一个有效的路径，直接使用初始的目标点
    if path_valid:
        publish_goal(init_x, init_y, quaternion_towards_person)
        return

    max_attempts = 1000
    attempts = 0
    rospy.loginfo("path_valid = %d", path_valid)
    while not path_valid and attempts < max_attempts:
        path_valid = False
        alt_x, alt_y = find_alternative_goal(new_goal_x, new_goal_y, quaternion)
        if alt_x and alt_y:
            publish_goal(alt_x, alt_y, quaternion_towards_person)
            rospy.sleep(0.3)  # 等待一段时间以获取路径消息
            if path_valid:
                tried_goals = []  # 清空tried_goals
                break  # 退出循环
        attempts += 1

    if attempts == max_attempts:
        rospy.logerr("Failed to find a valid goal after 1000 attempts.")


# 通过四元数来计算 yaw 偏航角
def quaternion_to_yaw(x, y, z, w):
    # 将四元数转换为欧拉角。
    euler = tf_trans.euler_from_quaternion([x, y, z, w])
    return euler[2]  # 返回yaw


# 计算新的目标点，该点位于人前的指定距离，并朝向人。
def compute_new_goal(goal_x, goal_y, quaternion, distance):
    # 从四元数获取yaw
    goal_yaw = quaternion_to_yaw(quaternion.x, quaternion.y, quaternion.z, quaternion.w)

    # 计算人的坐标
    person_x = goal_x + distance * math.cos(goal_yaw)
    person_y = goal_y + distance * math.sin(goal_yaw)

    # 计算新的目标点的坐标，该点位于人前的指定距离
    new_goal_x = person_x - distance * math.cos(goal_yaw)
    new_goal_y = person_y - distance * math.sin(goal_yaw)

    # 计算新的目标点的朝向，使其朝向人
    yaw_towards_person = math.atan2(person_y - new_goal_y, person_x - new_goal_x)
    quaternion_towards_person = tf_trans.quaternion_from_euler(0, 0, yaw_towards_person)

    return new_goal_x, new_goal_y, quaternion_towards_person


def pose_callback(msg):
    global current_pose
    current_pose = msg.pose.pose


def is_goal_reached(goal_pose):
    global current_pose
    if not current_pose:
        return False

    position_tolerance = 0.2  # 20cm
    orientation_tolerance = 0.15  # 0.15 radians

    dx = goal_pose.position.x - current_pose.position.x
    dy = goal_pose.position.y - current_pose.position.y
    distance = math.sqrt(dx * dx + dy * dy)

    # For simplicity, we'll just compare the orientation's z component and w component
    d_orientation = abs(goal_pose.orientation.z - current_pose.orientation.z) + abs(
        goal_pose.orientation.w - current_pose.orientation.w
    )

    return distance < position_tolerance and d_orientation < orientation_tolerance


def publish_goal(x, y, quaternion):
    global external_goal
    external_goal = False

    # 使用传入的四元数设置机器人的朝向
    goal_msg = MoveBaseActionGoal()
    goal_msg.goal.target_pose.header.frame_id = "map"
    goal_msg.header.stamp = rospy.Time.now()
    goal_msg.goal.target_pose.header.stamp = rospy.Time.now()
    goal_msg.goal_id.stamp = rospy.Time.now()
    goal_msg.goal.target_pose.pose.position.x = x
    goal_msg.goal.target_pose.pose.position.y = y
    goal_msg.goal.target_pose.pose.orientation.x = quaternion[0]
    goal_msg.goal.target_pose.pose.orientation.y = quaternion[1]
    goal_msg.goal.target_pose.pose.orientation.z = quaternion[2]
    goal_msg.goal.target_pose.pose.orientation.w = quaternion[3]

    # rospy.logerr(nav_client.get_goal_status_text())
    goal_pub.publish(goal_msg)
    


if __name__ == "__main__":
    rospy.init_node("alternative_goal_finder")
    rospy.sleep(1)
    rospy.loginfo("Ready!")

    rospy.Subscriber("move_base/GlobalPlanner/plan", Path, path_callback)  # 订阅路径消息
    # /move_base/GlobalPlanner/plan

    rospy.Subscriber("move_base/goal", MoveBaseActionGoal, goal_callback)
    # "/move_base/goal"

    rospy.Subscriber("/amcl_pose", PoseWithCovarianceStamped, pose_callback)

    goal_pub = rospy.Publisher("move_base/goal", MoveBaseActionGoal, queue_size=1)

    rospy.spin()



