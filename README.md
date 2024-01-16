# XiaoMeng
Code repository for Xiaomeng

## 环境搭配步骤
### 设置sources.list
首先配置好ubuntu18的ros环境  
``sudo sh -c '. /etc/lsb-release && echo "deb http://mirrors.tuna.tsinghua.edu.cn/ros/ubuntu/ `lsb_release -cs` main" > /etc/apt/sources.list.d/ros-latest.list'
``
###  设置密钥
`sudo apt-key adv --keyserver 'hkp://keyserver.ubuntu.com:80' --recv-key C1CF6E31E6BADE8868B172B4F42ED6FBAB17C654
`
### 安装
`sudo apt update 
`     
`
sudo apt install ros-melodic-desktop-full   
`    
`sudo apt-get install python-rosdep
`
**不然你会经历报错的噩梦**      
### 设置环境
`
echo "source /opt/ros/melodic/setup.bash" >> ~/.bashrc
`    
`
source ~/.bashrc
`        
### 安装一些晓萌要用到的msgs等
`
sudo apt-get install libsdl1.2-dev libsdl-image1.2-dev ros-melodic-tf2-sensor-msgs ros-melodic-move-base-msgs ros-melodic-ecl-threads
`
