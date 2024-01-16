cmake_minimum_required(VERSION 2.8.3)
project(sim_interface)
 
set(LINK_DIR /usr/lib)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  genmsg 
  actionlib_msgs 
  actionlib
)



catkin_package(
  CATKIN_DEPENDS actionlib_msgs
)

include_directories(
        include/xm_arm_sim_hw
  ${catkin_INCLUDE_DIRS}
)


