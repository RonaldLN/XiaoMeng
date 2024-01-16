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
/********************


                *******************/

#include <handsfree_hw/hf_hw_ros.h>

float armHeight = 0;

void callBackArm(const xm_msgs::plat_control& msg){
    armHeight = msg.height;
    //std::cout << armHeight << std::endl;
    //std::cout << "accept msg successfully!" << std::endl;;
}

typedef union 
{
    float FloatNum;
    int IntNum;
    /* data */
}FloatType;

unsigned char* FloatToByteArray(float f){
    unsigned char* Databuf = new unsigned char[4];
    FloatType Number;
    Number.FloatNum = f;
    Databuf[0] = (unsigned char)Number.IntNum;
    Databuf[1] = (unsigned char)(Number.IntNum >> 8);
    Databuf[2] = (unsigned char)(Number.IntNum >> 16);
    Databuf[3] = (unsigned char)(Number.IntNum >> 24);
    return Databuf;
}






namespace handsfree_hw
{
uint8_t plat_flag = 0;

HF_HW_ros::HF_HW_ros(ros::NodeHandle &nh, std::string url, std::string config_addr, bool use_sim_) :
    hf_hw_(url, config_addr, use_sim_),
    nh_(nh)
{
    sim_xm_ = use_sim_;//从参数服务器获取launch文件中设定的同名参数sim_xm
    if (!sim_xm_)
    {
        ROS_ERROR("Started xm_robot connection on port /dev/ttyUSB0");
    }
    else  ROS_ERROR("xm_robot being simulated.");

    nh_.setCallbackQueue(&queue_);

    //下面参数的值设置是有必要的，因为只有部分launch文件设置了这些值才能通过getparam获取
    base_mode_ = "2-wheel";
    controller_freq_ = 50; //100
    nh_.getParam("base_mode", base_mode_);
    nh_.getParam("freq", controller_freq_);
    // robot_state_publisher_ = nh_.advertise<handsfree_msgs::robot_state>("robot_state", 10);////////////////////

    //ultrasound_pub = nh_.advertise<sensor_msgs::Range>("UltraSoundPublisher", 10); //话题名

    x_ = y_ = theta_ = x_cmd_ = y_cmd_ = theta_cmd_ = 0.0;//设置x、y坐标和转动角度
    x_vel_ = y_vel_ = theta_vel_ = 0.0;//设置x、y的速度

        //超声波数据部分初始化
    //std::string ultra = "/ultrasonic";

/*
    for (int i = 0; i < 14; i++)
    {
        std_msgs::Header header;
        //header.stamp = ros::Time::now();
        //header.frame_id = ultra + std::to_string(i);
        //header.frame_id="/base_footprint";
        //ultraMSG[i].header = header;
        //ultraMSG[i].radiation_type = sensor_msgs::Range::ULTRASOUND;

        ultraMSG[i].field_of_view = 0.088; //约5度
        ultraMSG[i].min_range = 0;
        ultraMSG[i].max_range = 1.2; //1.2米的数据有效

        //ultraMSG[i].range = ultradound[i];
    }
    */

    /*
    head1_servo1_cmd_ = head1_servo2_cmd_  = head2_servo1_cmd_ = head2_servo2_cmd_ = 0.0;
    head1_servo1_pos_ = head1_servo2_pos_ = head1_servo1_vel_ = head2_servo1_vel_ = head1_servo1_eff_ = head2_servo1_eff_ = 0;
    */
    //real_theta = -theta_;real_theta_cmd = theta_cmd_;real_theta_vel = theta_vel_;
    //register the hardware interface on the robothw
    hardware_interface::BaseStateHandle base_state_handle("mobile_base", &x_, &y_, &theta_, &x_vel_, &y_vel_, &theta_vel_); //7.23电子组更改坐标系，为协调故在软件与电子交互层更改xy顺序
    base_state_interface_.registerHandle(base_state_handle);
    registerInterface(&base_state_interface_);
    hardware_interface::BaseVelocityHandle base_handle(base_state_handle, &x_cmd_, &y_cmd_, &theta_cmd_);
    base_velocity_interface_.registerHandle(base_handle);
    registerInterface(&base_velocity_interface_);

    /*hardware_interface::HeadServoStateHandle head_servo_state_handle("xm_head",&head_servo_yaw_pos_,&head_servo_pitch_pos_);
    head_servo_state_interface_.registerHandle(head_servo_state_handle);
    registerInterface(&head_servo_state_interface_);
    hardware_interface::HeadServoHandle head_servo_handle(head_servo_state_handle,&head_servo_yaw_cmd_,&head_servo_pitch_cmd_);
    head_servo_cmd_interface.registerHandle(head_servo_handle);
    
    registerInterface(&head_servo_cmd_interface);*/

    //write_update_thread_.start(&HF_HW_ros::writeBufferUpdate, *this);
    //read_update_thread_.start(&HF_HW_ros::readBufferUpdate, *this);



    if (!sim_xm_)
    {;
       std::cout<<hf_hw_.initialize_ok()<<std::endl;
       if (hf_hw_.initialize_ok())
        {
            ROS_INFO("system initialized succeed, ready for communication");
        }
        else
        {
            ROS_ERROR("hf link initialized failed, please check the hardware");
        }
    }
    else
        ROS_ERROR("hf link initialized is no need, take it easy");
}

/*
bool HF_HW_ros::SoundSourceCallBack(xm_msgs::SoundSource::Request &req, xm_msgs::SoundSource::Response &res)
{
    ros::Time begin = ros::Time::now();

    sound_angle_ = 0.0;
    hf_hw_.getRobotAbstract()->sound_angle = 0.0;
    sound_start_ = true;
    //std::cout<<"sound_start_=true"<<std::endl;

    //当10s后仍为收到数据时，终端等待，返回404
    while (sound_angle_ == 0.0)
        if (ros::Time::now().sec - begin.sec >= 10)
            break;
    if (ros::Time::now().sec - begin.sec >= 10)
        res.angle = 404;
    else
        res.angle = sound_angle_;

    sound_angle_ = 0.0;
    hf_hw_.getRobotAbstract()->sound_angle = 0.0;
    sound_start_ = false;
    return true;
}
*/

/*
void HF_HW_ros::publishUltraSound()
{
    for (int i = 0; i < 14; i++)
    {
        ros::Time begin =ros::Time::now();
       
        ultraMSG[i].header.stamp=begin;
        //ultraMSG[i].header.stamp=begin;
        
        //ultraMSG[i].range = ultrasound[i];      //ultrasound[i];
        ultraMSG[i].range=-1;

        if(ultraMSG[i].range>1.2)
            ultraMSG[i].range=1.2;
        ultraMSG[i].header.frame_id="ultrasonic"+std::to_string(i)+"_link";

        //ultrasound_pub.publish(ultraMSG[i]);

        //ultraTransform.sendTransform(
        //    tf::StampedTransform(
        //        tf::Transform(tf::Quaternion(0,0,0,1),tf::Vector3(0.2356,ultra_y[i],-0.05675)),
        //        begin,"base_link","ultrasonic"+std::to_string(i)+"_link"));
    }
}
*/




void HF_HW_ros::mainloop()
{
    ros::CallbackQueue cm_callback_queue;
    ros::NodeHandle cm_nh("mobile_base");
    cm_nh.setCallbackQueue(&cm_callback_queue);
    controller_manager::ControllerManager cm(this, cm_nh); //传入一个和controller同一工作空间的ros句柄

    ros::AsyncSpinner cm_spinner(1, &cm_callback_queue); //开一个1线程
    ros::AsyncSpinner hw_spinner(1, &queue_);

    //SoundSource_srv_ = cm_nh.advertiseService("SoundSource", &HF_HW_ros::SoundSourceCallBack, this);

    ros::Rate loop(controller_freq_);//设置回调频率
    cm_spinner.start();
    hw_spinner.start();

    int count = 0;
    ros::NodeHandle armNode;
    ros::Subscriber arm = armNode.subscribe("/plat_control" , 10 , callBackArm);
    


    ros::Time currentTime = ros::Time::now();//得到ros::time实例化的当前时间
    while (ros::ok())
    {
        if (true)//false,sim   true run
        {
            hf_hw_.checkHandshake();
            /*// if(!hf_hw_.updateCommand(READ_GLOBAL_COORDINATE, count))
                hf_hw_.updateCommand(READ_GLOBAL_COORDINATE, count);
            // if(!hf_hw_.updateCommand(READ_ROBOT_SPEED, count))
                hf_hw_.updateCommand(READ_ROBOT_SPEED, count);
           
            // hf_hw_.updateCommand(READ_HEAD_1, count);

            // if(!hf_hw_.updateCommand(SET_ROBOT_SPEED, count))
            /////  
            */
            readBufferUpdate();//读取当前测量值
            //ROS_ERROR("cmd %f,%f,%f",x_cmd_,y_cmd_,theta_cmd_);
            //ROS_ERROR("vel %f,%f,%f",x_vel_,y_vel_,theta_vel_);

            //std::cout<< "* *"<<std::endl;

            //std::cout<<x_cmd_<<" "<<y_cmd_<<" "<<theta_cmd_<<std::endl;
            //ROS_INFO("head1_servo1_cmd_ = %.4f  head1_servo2_cmd_=%.4f" , head1_servo1_cmd_ ,head1_servo2_cmd_);
            //std::cout<<std::endl<<

             //std::cout<<x_<<" "<<y_<<" "<<theta_<<std::endl;
            //仅当sound_start_为true时向电子组发送开始指令
            /*
            if (sound_start_ == true)
            {
                hf_hw_.updateWriteCommand(SET_SOUND_START, count);
                std::cout<<"send"<<std::endl;
                sound_start_=false;
            }
            */

           //RobotAbstract类中的变量看作缓冲区
           //hf_hw_.updateWriteCommand(SET_ROBOT_SPEED, count);//将数据按照一定频率写入串口，发送命令
                       //for a high cmd

            //来自2022年叶的修改，，，，，机械臂需要通过底盘发串口，特此更改
            //由于发串口藏的太深，更改了部分private为public 以便直接访问。
            // if((-0.6<=x_cmd_ && x_cmd_<=0.6) && (-0.6<=y_cmd_ && y_cmd_<=0.35 ) && (-0.7<=theta_cmd_ && theta_cmd_<=0.7)){
            //         hf_hw_.updateWriteCommand(SET_ROBOT_SPEED, count);//将数据按照一定频率写入串口，发送命令
            //         //std::cout<<"send"<<std::endl;
            //         //ROS_INFO("111111111111111");
            //         writeBufferUpdate();//将机器人的期望值发给下位机
            //     }
            //std::cout << "choose arm or nav" << armHeight << std::endl;
            // if(armHeight != -666){
            
            std::vector<uint8_t> Height;
            Height.push_back('\xff');//255
            Height.push_back('\xff');//255
            Height.push_back('\x01');//1
            Height.push_back('\x11');//17
            Height.push_back('\x00');//0
            Height.push_back('\x11');//17
            
            // Height.push_back('\x01');//1

            // 定义 cheek，第一次发送 0，保证与下位机握手成功，之后发送 01，正常进行数据交换
            static int cheek = 0;

            if(cheek == 0)
            {
                Height.push_back('\x00');//
                cheek = 1;
            }
            else
            {
                Height.push_back('\x01');//1
               
            }


            // std::cout << armHeight << std::endl;
            unsigned char* xPort = FloatToByteArray(x_cmd_);
            Height.push_back(xPort[0]);
            Height.push_back(xPort[1]);
            Height.push_back(xPort[2]);
            Height.push_back(xPort[3]);
            unsigned char* yPort = FloatToByteArray(y_cmd_);
            Height.push_back(yPort[0]);
            Height.push_back(yPort[1]);
            Height.push_back(yPort[2]);
            Height.push_back(yPort[3]);
            unsigned char* thetaPort = FloatToByteArray(theta_cmd_);
            Height.push_back(thetaPort[0]);
            Height.push_back(thetaPort[1]);
            Height.push_back(thetaPort[2]);
            Height.push_back(thetaPort[3]);
            unsigned char* heightArm = FloatToByteArray(armHeight);
            Height.push_back(heightArm[0]);
            Height.push_back(heightArm[1]);
            Height.push_back(heightArm[2]);
            Height.push_back(heightArm[3]);
            int num_count = 0;
            for(int iii = 0 ; iii < 23 ; iii++ ){
                num_count += Height[iii];
            }
            sleep(0.01);
            unsigned char Jiaoyanwei = num_count % 255;
            Height.push_back(Jiaoyanwei);
            // std::cout << "This is arm not navgation" << std::endl;
            hf_hw_.port_->writeBuffer(Height);

                // // std::cout << "send arm height, not navigation" << std::endl;
                // armHeight = -666;
                // hf_hw_.sendCommand()
            // }else{
            //     if((-0.6<=x_cmd_ && x_cmd_<=0.6) && (-0.6<=y_cmd_ && y_cmd_<=0.35 ) && (-0.7<=theta_cmd_ && theta_cmd_<=0.7)){
            //         hf_hw_.updateWriteCommand(SET_ROBOT_SPEED, count);//将数据按照一定频率写入串口，发送命令
            //         //std::cout<<"send"<<std::endl;
            //         //ROS_INFO("111111111111111");
            //         writeBufferUpdate();//将机器人的期望值发给下位机
            //     }
            // }
            
            //ROS_INFO("222222222222");
            //writeBufferUpdate();//将机器人的期望值发给下位机

//xiao
/*
            std_msgs::Float32MultiArray msg;
            msg.data.push_back(0);
            msg.data.push_back(0);
            msg.data.push_back(0);
            msg.data.push_back(0.2);
            msg.data.push_back(0);
            msg.data.push_back(0);
            //test_pub.publish(msg);
*/
            //publishUltraSound();

            cm.update(ros::Time::now(), ros::Duration(1 / controller_freq_));//ros Duration进行延时

            // hf_hw_.updateCommand(SET_HEAD_1, count);
        }
        else
        {
            //底盘数据的模拟更新
            x_ += x_vel_ * cos(theta_) / controller_freq_;
            y_ += x_vel_ * sin(theta_) / controller_freq_;
            theta_ += theta_cmd_ / controller_freq_;

            x_vel_ = x_cmd_;
            y_vel_ = y_cmd_;
            theta_vel_ = theta_cmd_;

            //publishUltraSound();

            cm.update(ros::Time::now(), ros::Duration(1 / controller_freq_));//ros Duration进行延时

            //hehe.sleep();注释于1.27如果不注释，仿真时每次更新完都会睡眠一次，显示在rviz上就会感觉机器人特别卡
        }

        loop.sleep();
        count++;
        ros::spinOnce();
    }

    cm_spinner.stop();
    //hw_spinner.stop();
}
}; // namespace handsfree_hw
