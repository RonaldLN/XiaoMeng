#include <handsfree_hw/hf_hw_ros.h>
int main(int argc, char** argv)
{
    ros::init(argc, argv, "robothw");//robothw：启动节点的名字
    ros::NodeHandle nh("mobile_base");//创建句柄nh,命名空间为robothwspace/mobile_base
    ros::NodeHandle nh_private("~");//命名空间为robothwspace/robothw
    std::string config_filename = "/config.txt" ;
    std::string config_filepath = CONFIG_PATH+config_filename ; 
    std::cerr<<"the configure file path is: "<<config_filepath<<std::endl;

    std::string robolink_config_file;
    nh_private.param<std::string>("robolink_config_file",robolink_config_file,"");
    std::cerr<<"the configure file path is: "<<robolink_config_file<<std::endl ; 

    std::string serial_port;
    nh_private.param<std::string>("serial_port", serial_port, "/dev/ttyUSB0"); //参数传递
    std::string serial_port_path="serial://" + serial_port;
     bool sim_xm_;
    nh.getParam("/handsfree_hw_node/sim_xm",sim_xm_);//优先获取是否仿真
    handsfree_hw::HF_HW_ros hf(nh, serial_port_path , config_filepath , sim_xm_);

    hf.mainloop();
    return 0;
   
   
   //以下可以在非仿真情况下使用看看，官方源码不包含仿真
    /* while (ros::ok())
    {
        std::string serial_port;
        nh_private.param<std::string>("serial_port", serial_port, "/dev/ttyUSB0");//参数传递

        if((access(serial_port.c_str(),0)) != -1)//前一个参数是文件路径，0为访问权限（只检查文件是否存在），如果文件具有指定的访问权限，则函数返回0；如果文件不存在或者不能访问指定的权限，则返回-1.
        {
            std::string serial_port_path="serial://" + serial_port;
            std::cerr<<"the serial_port is: "<<serial_port_path<<std::endl ;
            nh.getParam("/handsfree_hw_node/sim_xm",sim_xm_);//优先获取是否仿真
            handsfree_hw::HF_HW_ros hf(nh, serial_port_path , config_filepath , sim_xm_);
            hf.mainloop();
        }
        else
        {
            std::cerr<<"not find serial_port: "<<serial_port<<std::endl ;//查看源码后增添了串口错误处理
        }
        usleep(2000000);//usleep() 与sleep()类似，用于延迟挂起进程，单位为毫秒
    }
    return 0;
    */
}
