/***********************************************************************************************************************
* Copyright (c) Hands Free Team. All rights reserved.
* FileName: hf_link.cpp
* Contact:  QQ Exchange Group -- 521037187
* Version:  V2.0
*
* LICENSING TERMS:
* The Hands Free is licensed generally under a permissive 3-clause BSD license.
* Contributions are required to be made under the same license.
*
* History:
* <author>      <time>      <version>      <desc>
* luke liao       2016.4.1   V1.0           creat this file
*
* Description: handsfree ros ros_control framework
***********************************************************************************************************************/

#ifndef HF_HW_ROS_
#define HF_HW_ROS_

#include <vector>

#include <handsfree_hw/base_cmd_interface.h>
//#include <handsfree_hw/base_state_interface.h>被包含在<handsfree_hw/base_cmd_interface.h>了
#include <hardware_interface/joint_state_interface.h>
#include <hardware_interface/joint_command_interface.h>
#include <handsfree_hw/head_servo_state_interface.h>
#include <handsfree_hw/head_servo_command_interface.h>
#include <handsfree_hw/gripper_state_interface.h>
#include <handsfree_hw/gripper_cmd_interface.h>
#include <hardware_interface/robot_hw.h>
#include <realtime_tools/realtime_publisher.h>
// #include <handsfree_msgs/robot_state.h>


//#include <xm_msgs/SoundSource.h>

#include<std_msgs/Float32MultiArray.h>
#include <sensor_msgs/Range.h>

#include <controller_manager/controller_manager.h>
// for ros headers
#include <ros/ros.h>
#include <ros/callback_queue.h>

// for hf link and transport
//#include <handsfree_hw/transport.h>包含在了<handsfree_hw/transport_serial.h>
//#include <handsfree_hw/transport_serial.h>
//#include <hf_link.h>包含在了<handsfree_hw/hf_hw.h>
#include <handsfree_hw/hf_hw.h>
#include <math.h>

//for tf
#include <tf/transform_broadcaster.h>

#include <ecl/threads.hpp>
namespace handsfree_hw
{

class HF_HW_ros : public hardware_interface::RobotHW
{

public:
    HF_HW_ros(ros::NodeHandle &nh, std::string url, std::string config_addr, bool use_sim_);

    double getFreq() const
    {
        return controller_freq_;
    }

    void mainloop();
   //bool SoundSourceCallBack(xm_msgs::SoundSource::Request &req, xm_msgs::SoundSource::Response &res);

   //void publishUltraSound();

private:
    //communication with embeded system
    boost::mutex read_mutex_3;

    HF_HW hf_hw_;
    ros::NodeHandle nh_;
    ros::CallbackQueue queue_;
    //int tempi;//
    // publish the robot state for diagnose system
    // ros::Publisher robot_state_publisher_;
    ros::ServiceServer server_;
    ros::ServiceServer getparam_srv_;
    ros::ServiceServer setparam_srv_;
    //声源定位的服务端，调用服务时，向电子组发送声源定位的开始指令，接受声源定位的结果，将其返回给客户端
    //ros::ServiceServer SoundSource_srv_;


tf::TransformBroadcaster ultraTransform;
    //ros::Publisher ultrasound_pub; //发布电子组的激光、超色波数据
    //sensor_msgs::Range ultraMSG[15];
    //ultra相对于base_link的y坐标
    //float ultra_y[14]={0.1485,0.1215,0.0945,0.0675,0.0405,0.0135,-0.0135,-0.0405,-0.0675,-0.0945,-0.1215,-0.1485};

    //parameter list 在launch文件中声明了with_arm_、sim_xm_的同名参数(除去后面的下划线)并赋值
    std::string base_mode_;
    double controller_freq_;
    bool sim_xm_;
    //hardware resource
    // handsfree_msgs::robot_state robot_state;


    //char endEffort_command_;

    //float sound_angle_;
    //bool sound_start_;

    //float ultrasound[15]; //电子组读到的数据首先读到这里

    double x_, y_, theta_, x_cmd_, y_cmd_, theta_cmd_;
    double x_vel_, y_vel_, theta_vel_; //机器人当前速度和转角

    double head_servo_yaw_pos_, head_servo_pitch_pos_;
    double head_servo_yaw_cmd_, head_servo_pitch_cmd_;


    /*hardware_interface::HeadServoStateInterface head_servo_state_interface_;
    hardware_interface::HeadServoInterface head_servo_cmd_interface ;*/

    hardware_interface::BaseStateInterface base_state_interface_;
    hardware_interface::BaseVelocityInterface base_velocity_interface_;

    // //ecl::Thread write_update_thread_;
    // ecl::Thread read_update_thread_;

    inline void writeBufferUpdate()
    { //将机器人期望状态发送给下位机

        /*
        hf_hw_.getRobotAbstract()->expect_motor_speed.servo1 = wheel_cmd_[0];
        hf_hw_.getRobotAbstract()->expect_motor_speed.servo2 = wheel_cmd_[1];
        hf_hw_.getRobotAbstract()->expect_motor_speed.servo3 = wheel_cmd_[2];
        */
        hf_hw_.getRobotAbstract()->expect_robot_speed.x = x_cmd_;
        hf_hw_.getRobotAbstract()->expect_robot_speed.y = y_cmd_;
        hf_hw_.getRobotAbstract()->expect_robot_speed.z = theta_cmd_;

        //hf_hw_.getRobotAbstract()->sound_start = sound_start_;
        // the servo num is differentROS_ERRORmd_;
        /*
        hf_hw_.getRobotAbstract()->expect_head2_state.pitch  = head2_servo1_cmd_;
        hf_hw_.getRobotAbstract()->expect_head2_state.yaw  = head2_servo2_cmd_;
        */
    }

    inline void readBufferUpdate()//读取机器人当前测量值
    {
        boost::mutex::scoped_lock lock(read_mutex_3);

        x_ = hf_hw_.getRobotAbstract()->measure_global_coordinate.x;
        y_ = hf_hw_.getRobotAbstract()->measure_global_coordinate.y;
        theta_ = hf_hw_.getRobotAbstract()->measure_global_coordinate.z;
        // if(tempi%30==0)
        //ROS_ERROR("measure_global_coordinate:         x_:%f ;y_:%f ;theta_:%f",x_,y_,theta_);

        x_vel_ = hf_hw_.getRobotAbstract()->measure_robot_speed.x;
        y_vel_ = hf_hw_.getRobotAbstract()->measure_robot_speed.y;
        theta_vel_ = hf_hw_.getRobotAbstract()->measure_robot_speed.z;
       // ROS_ERROR("measure_global_coordinate:         x_vel_:%f ;y_vel_:%f ;theta_vel_:%f",x_vel_,y_vel_,theta_vel_);

                //sound_angle_ = hf_hw_.getRobotAbstract()->sound_angle;
        //ROS_ERROR("sound_angle_:   :%f",sound_angle_);
/*
        for (int i = 0; i < 12; i++)
            ultrasound[i] = hf_hw_.getRobotAbstract()->ultra_sound[i];
*/
        // if(tempi%50==0)
        //ROS_ERROR("measure_robot_speed: x_vel_:%f ;y_vel_:%f ;theta_vel_:%f",x_vel_,y_vel_,theta_vel_);
        // tempi++;
        /*
        wheel_pos_[0] = hf_hw_.getRobotAbstract()->measure_motor_mileage.servo1;
        wheel_pos_[1] = hf_hw_.getRobotAbstract()->measure_motor_mileage.servo2;
        wheel_pos_[2] = hf_hw_.getRobotAbstract()->measure_motor_mileage.servo3;
        */
        // robot_state.battery_voltage = hf_hw_.getRobotAbstract()->robot_parameters.robot_body_radius;
        // robot_state.cpu_temperature = hf_hw_.getRobotAbstract()->robot_system_info.cpu_temperature;
        // robot_state.cpu_usage = hf_hw_.getRobotAbstract()->robot_system_info.cpu_usage;
        // robot_state.system_time = hf_hw_.getRobotAbstract()->robot_system_info.system_time;


        /*head_servo_yaw_pos_ = hf_hw_.getRobotAbstract()->measure_head1_state.yaw;//暂无卵用
        head_servo_pitch_pos_ = hf_hw_.getRobotAbstract()->measure_head1_state.pitch;*/
        /*
        wheel_vel_[0] = hf_hw_.getRobotAbstract()->measure_motor_speed.servo1;
        wheel_vel_[1] = hf_hw_.getRobotAbstract()->measure_motor_speed.servo2;
        wheel_vel_[2] = hf_hw_.getRobotAbstract()->measure_motor_speed.servo3;

        head1_servo1_pos_ = hf_hw_.getRobotAbstract()->measure_head1_state.pitch ;
        head1_servo1_vel_ = 0 ;
        head1_servo1_eff_ = 0 ;

        head1_servo2_pos_ = hf_hw_.getRobotAbstract()->measure_head1_state.yaw ;
        head1_servo2_vel_ = 0 ;
        head1_servo2_eff_ = 0 ;

        head2_servo1_pos_ = hf_hw_.getRobotAbstract()->measure_head2_state.pitch ;
        head2_servo1_vel_ = 0 ;
        head2_servo1_eff_ = 0 ;

        head2_servo2_pos_ = hf_hw_.getRobotAbstract()->measure_head2_state.yaw ;
        head2_servo2_vel_ = 0 ;
        head2_servo2_eff_ = 0 ;
        */
    }
};

} // namespace handsfree_hw

#endif
