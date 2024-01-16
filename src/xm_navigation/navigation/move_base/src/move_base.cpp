/*********************************************************************
*
* Software License Agreement (BSD License)
*
*  Copyright (c) 2008, Willow Garage, Inc.
*  All rights reserved.
*
*  Redistribution and use in source and binary forms, with or without
*  modification, are permitted provided that the following conditions
*  are met:
*
*   * Redistributions of source code must retain the above copyright
*     notice, this list of conditions and the following disclaimer.
*   * Redistributions in binary form must reproduce the above
*     copyright notice, this list of conditions and the following
*     disclaimer in the documentation and/or other materials provided
*     with the distribution.
*   * Neither the name of the Willow Garage nor the names of its
*     contributors may be used to endorse or promote products derived
*     from this software without specific prior written permission.
*
*  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
*  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
*  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
*  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
*  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
*  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
*  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
*  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
*  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
*  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
*  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
*  POSSIBILITY OF SUCH DAMAGE.
*
* Author: Eitan Marder-Eppstein
*         Mike Phillips (put the planner in its own thread)
*********************************************************************/
#include <move_base/move_base.h>
#include <move_base_msgs/RecoveryStatus.h>
#include <cmath>

#include <boost/algorithm/string.hpp>
#include <boost/thread.hpp>

#include <geometry_msgs/Twist.h>

#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

namespace move_base {

  MoveBase::MoveBase(tf2_ros::Buffer& tf) ://创建侦听器
    tf_(tf),
    as_(NULL),//MoveBaseActionServer* 指针
    planner_costmap_ros_(NULL), controller_costmap_ros_(NULL),//建立两个代价地图，costmap的实例化指针，前为全局，后为全部
    bgp_loader_("nav_core", "nav_core::BaseGlobalPlanner"),
    blp_loader_("nav_core", "nav_core::BaseLocalPlanner"), 
    recovery_loader_("nav_core", "nav_core::RecoveryBehavior"),
    planner_plan_(NULL), latest_plan_(NULL), controller_plan_(NULL),//三种规划器，看触发哪种
    //参数配置
    runPlanner_(false), setup_(false), p_freq_change_(false), c_freq_change_(false), new_global_plan_(false) {

    turn_flag=0;
    //as_维护movebase的actionServer状态机，并且新建了一个executeCb线程。 
    //创建move_base action,绑定回调函数。定义一个名为move_base的SimpleActionServer。该服务器的Callback为moveBase::executeCb
    as_ = new MoveBaseActionServer(ros::NodeHandle(), "move_base", boost::bind(&MoveBase::executeCb, this, _1), false);

    ros::NodeHandle private_nh("~");
    ros::NodeHandle nh;

    recovery_trigger_ = PLANNING_R;

    //get some parameters that will be global to the move base node
    std::string global_planner, local_planner;
    //从private_nh中获取参数设定值
    private_nh.param("base_global_planner", global_planner, std::string("navfn/NavfnROS"));//全局规划器，用于规划最优路径
    private_nh.param("base_local_planner", local_planner, std::string("base_local_planner/TrajectoryPlannerROS"));//局部规划器，用于动态避障
    private_nh.param("global_costmap/robot_base_frame", robot_base_frame_, std::string("base_link"));//机器人底座的局部坐标系
    private_nh.param("global_costmap/global_frame", global_frame_, std::string("map"));//代价地图的世界坐标系
    private_nh.param("planner_frequency", planner_frequency_, 1.0);//全局规划器的执行频率，如果为0，则只有出现新的目标点，才能重新规划。
    private_nh.param("controller_frequency", controller_frequency_, 20.0);//向机器人底盘发送的控制速度的频率，这个速度由base_local_planner计算
    private_nh.param("planner_patience", planner_patience_, 5.0);//进行全局规划的时间间隔，如果超时则认为规划失败
    private_nh.param("controller_patience", controller_patience_, 15.0);//等待控制速度的时间间隔，如果控制速度的发布超过设置时间，则认为局部路径规划失败
    private_nh.param("max_planning_retries", max_planning_retries_, -1);  // 在执行恢复行为之前允许计划重试的次数，默认为-1，表示全局规划失败后立即执行恢复模块。

    private_nh.param("oscillation_timeout", oscillation_timeout_, 0.0);//在执行恢复行为之前允许震荡的时间（秒）0.0
    private_nh.param("oscillation_distance", oscillation_distance_, 0.5);//机器人必须移动多远（米）才能被视为不摆动
    //如果出现摆动则说明全局规划失败，那么将在超时后执行恢复模块

    // parameters of make_plan service
    //private_nh.param("make_plan_clear_costmap", make_plan_clear_costmap_, true);
    private_nh.param("make_plan_add_unreachable_goal", make_plan_add_unreachable_goal_, true);

    //set up plan triple buffer 这三个plan都是global_plan，最终由controller_plan_ 传给 local planner
    planner_plan_ = new std::vector<geometry_msgs::PoseStamped>();
    latest_plan_ = new std::vector<geometry_msgs::PoseStamped>();
    controller_plan_ = new std::vector<geometry_msgs::PoseStamped>();

    //set up the planner's thread 创建规划器线程，在该线程里运行planThread函数
    planner_thread_ = new boost::thread(boost::bind(&MoveBase::planThread, this));

    //for commanding the base，设置很多publish
    vel_pub_ = nh.advertise<geometry_msgs::Twist>("cmd_vel", 1);
    current_goal_pub_ = private_nh.advertise<geometry_msgs::PoseStamped>("current_goal", 0 );

    //发布MoveBaseActionGoal消息到/move_base/goal话题上 
    ros::NodeHandle action_nh("move_base");//movebase命名空间下的句柄
    action_goal_pub_ = action_nh.advertise<move_base_msgs::MoveBaseActionGoal>("goal", 1);
    recovery_status_pub_= action_nh.advertise<move_base_msgs::RecoveryStatus>("recovery_status", 1);

    //we'll provide a mechanism for some people to send goals as PoseStamped messages over a topic
    //they won't get any useful information back about its status, but this is useful for tools
    //like nav_view and rviz
    ros::NodeHandle simple_nh("move_base_simple");
    goal_sub_ = simple_nh.subscribe<geometry_msgs::PoseStamped>("goal", 1, boost::bind(&MoveBase::goalCB, this, _1));

    //we'll assume the radius of the robot to be consistent with what's specified for the costmaps加载代价地图的参数
    private_nh.param("local_costmap/inscribed_radius", inscribed_radius_, 0.325);//内切半径
    private_nh.param("local_costmap/circumscribed_radius", circumscribed_radius_, 0.46);//外切半径
    private_nh.param("clearing_radius", clearing_radius_, circumscribed_radius_);//清除半径
    private_nh.param("conservative_reset_dist", conservative_reset_dist_, 3.0);//在恢复模块执行后，重置的代价地图范围

   //一些标志位的设置
    private_nh.param("shutdown_costmaps", shutdown_costmaps_, false);//在move_base不活动时，是否关闭代价地图的加载
    private_nh.param("clearing_rotation_allowed", clearing_rotation_allowed_, true);//是否允许旋转恢复行为
    private_nh.param("recovery_behavior_enabled", recovery_behavior_enabled_, true);//是否使用恢复模块

    //create the ros wrapper for the planner's costmap... and initializer a pointer we'll use with the underlying map
    //实例化planner_costmap_ros_，controller_costmap_ros_
    planner_costmap_ros_ = new costmap_2d::Costmap2DROS("global_costmap", tf_);//实例化planner_costmap_ros_
    planner_costmap_ros_->pause();

    //initialize the global planner
    try {
      planner_ = bgp_loader_.createInstance(global_planner);
      planner_->initialize(bgp_loader_.getName(global_planner), planner_costmap_ros_);
    } catch (const pluginlib::PluginlibException& ex) {
      ROS_FATAL("Failed to create the %s planner, are you sure it is properly registered and that the containing library is built? Exception: %s", global_planner.c_str(), ex.what());
      exit(1);
    }

    //create the ros wrapper for the controller's costmap... and initializer a pointer we'll use with the underlying map
    controller_costmap_ros_ = new costmap_2d::Costmap2DROS("local_costmap", tf_);
    controller_costmap_ros_->pause();

    //create a local planner
    try {
      tc_ = blp_loader_.createInstance(local_planner);
      ROS_INFO("Created local_planner %s", local_planner.c_str());
      tc_->initialize(blp_loader_.getName(local_planner), &tf_, controller_costmap_ros_);
    } catch (const pluginlib::PluginlibException& ex) {
      ROS_FATAL("Failed to create the %s planner, are you sure it is properly registered and that the containing library is built? Exception: %s", local_planner.c_str(), ex.what());
      exit(1);
    }

    // Start actively updating costmaps based on sensor data 开启costmap基于传感器数据的更新，根据传感器数据动态更新全局和本地的代价地图
    planner_costmap_ros_->start();
    controller_costmap_ros_->start();

    //advertise a service for getting a plan
    make_plan_srv_ = private_nh.advertiseService("make_plan", &MoveBase::planService, this);

    //advertise a service for clearing the costmaps
    clear_costmaps_srv_ = private_nh.advertiseService("clear_costmaps", &MoveBase::clearCostmapsService, this);

    //if we shutdown our costmaps when we're deactivated... we'll do that now
    if(shutdown_costmaps_){
      ROS_DEBUG_NAMED("move_base","Stopping costmaps initially");
      planner_costmap_ros_->stop();
      controller_costmap_ros_->stop();
    }

    //load any user specified recovery behaviors, and if that fails load the defaults载入recovery behavior，这部分是状态机的设计问题
    if(!loadRecoveryBehaviors(private_nh)){
      loadDefaultRecoveryBehaviors();
    }

    //initially, we'll need to make a plan
    state_ = PLANNING;

    //we'll start executing recovery behaviors at the beginning of our list
    recovery_index_ = 0;

    //we're all set up now so we can start the action server调用action server的start函数，服务器启动
    as_->start();

    dsrv_ = new dynamic_reconfigure::Server<move_base::MoveBaseConfig>(ros::NodeHandle("~"));
    dynamic_reconfigure::Server<move_base::MoveBaseConfig>::CallbackType cb = boost::bind(&MoveBase::reconfigureCB, this, _1, _2);
    dsrv_->setCallback(cb);
  }

  void MoveBase::reconfigureCB(move_base::MoveBaseConfig &config, uint32_t level){
    boost::recursive_mutex::scoped_lock l(configuration_mutex_);

    //The first time we're called, we just want to make sure we have the
    //original configuration
    if(!setup_)
    {
      last_config_ = config;
      default_config_ = config;
      setup_ = true;
      return;
    }

    if(config.restore_defaults) {
      config = default_config_;
      //if someone sets restore defaults on the parameter server, prevent looping
      config.restore_defaults = false;
    }

    if(planner_frequency_ != config.planner_frequency)
    {
      planner_frequency_ = config.planner_frequency;
      p_freq_change_ = true;
    }

    if(controller_frequency_ != config.controller_frequency)
    {
      controller_frequency_ = config.controller_frequency;
      c_freq_change_ = true;
    }

    planner_patience_ = config.planner_patience;
    controller_patience_ = config.controller_patience;
    max_planning_retries_ = config.max_planning_retries;
    conservative_reset_dist_ = config.conservative_reset_dist;

    recovery_behavior_enabled_ = config.recovery_behavior_enabled;
    clearing_rotation_allowed_ = config.clearing_rotation_allowed;
    shutdown_costmaps_ = config.shutdown_costmaps;

    oscillation_timeout_ = config.oscillation_timeout;
    oscillation_distance_ = config.oscillation_distance;
    if(config.base_global_planner != last_config_.base_global_planner) {
      boost::shared_ptr<nav_core::BaseGlobalPlanner> old_planner = planner_;
      //initialize the global planner
      ROS_INFO("Loading global planner %s", config.base_global_planner.c_str());
      try {
        planner_ = bgp_loader_.createInstance(config.base_global_planner);

        // wait for the current planner to finish planning
        boost::unique_lock<boost::recursive_mutex> lock(planner_mutex_);

        // Clean up before initializing the new planner
        planner_plan_->clear();
        latest_plan_->clear();
        controller_plan_->clear();
        resetState();
        planner_->initialize(bgp_loader_.getName(config.base_global_planner), planner_costmap_ros_);

        lock.unlock();
      } catch (const pluginlib::PluginlibException& ex) {
        ROS_FATAL("Failed to create the %s planner, are you sure it is properly registered and that the \
                   containing library is built? Exception: %s", config.base_global_planner.c_str(), ex.what());
        planner_ = old_planner;
        config.base_global_planner = last_config_.base_global_planner;
      }
    }

    if(config.base_local_planner != last_config_.base_local_planner){
      boost::shared_ptr<nav_core::BaseLocalPlanner> old_planner = tc_;
      //create a local planner
      try {
        tc_ = blp_loader_.createInstance(config.base_local_planner);
        // Clean up before initializing the new planner
        planner_plan_->clear();
        latest_plan_->clear();
        controller_plan_->clear();
        resetState();
        tc_->initialize(blp_loader_.getName(config.base_local_planner), &tf_, controller_costmap_ros_);
      } catch (const pluginlib::PluginlibException& ex) {
        ROS_FATAL("Failed to create the %s planner, are you sure it is properly registered and that the \
                   containing library is built? Exception: %s", config.base_local_planner.c_str(), ex.what());
        tc_ = old_planner;
        config.base_local_planner = last_config_.base_local_planner;
      }
    }

//学长的代码中未出现先注释试试看
    //make_plan_clear_costmap_ = config.make_plan_clear_costmap;
    //make_plan_add_unreachable_goal_ = config.make_plan_add_unreachable_goal;

    last_config_ = config;
  }

  void MoveBase::goalCB(const geometry_msgs::PoseStamped::ConstPtr& goal){
    ROS_DEBUG_NAMED("move_base","In ROS goal callback, wrapping the PoseStamped in the action message and re-sending to the server.");
    move_base_msgs::MoveBaseActionGoal action_goal;
    action_goal.header.stamp = ros::Time::now();
    action_goal.goal.target_pose = *goal;

    action_goal_pub_.publish(action_goal);
  }

  void MoveBase::clearCostmapWindows(double size_x, double size_y){
    geometry_msgs::PoseStamped global_pose;

    //clear the planner's costmap
    getRobotPose(global_pose, planner_costmap_ros_);

    std::vector<geometry_msgs::Point> clear_poly;
    double x = global_pose.pose.position.x;
    double y = global_pose.pose.position.y;
    geometry_msgs::Point pt;

    pt.x = x - size_x / 2;
    pt.y = y - size_y / 2;
    clear_poly.push_back(pt);

    pt.x = x + size_x / 2;
    pt.y = y - size_y / 2;
    clear_poly.push_back(pt);

    pt.x = x + size_x / 2;
    pt.y = y + size_y / 2;
    clear_poly.push_back(pt);

    pt.x = x - size_x / 2;
    pt.y = y + size_y / 2;
    clear_poly.push_back(pt);

    planner_costmap_ros_->getCostmap()->setConvexPolygonCost(clear_poly, costmap_2d::FREE_SPACE);

    //clear the controller's costmap
    getRobotPose(global_pose, controller_costmap_ros_);

    clear_poly.clear();
    x = global_pose.pose.position.x;
    y = global_pose.pose.position.y;

    pt.x = x - size_x / 2;
    pt.y = y - size_y / 2;
    clear_poly.push_back(pt);

    pt.x = x + size_x / 2;
    pt.y = y - size_y / 2;
    clear_poly.push_back(pt);

    pt.x = x + size_x / 2;
    pt.y = y + size_y / 2;
    clear_poly.push_back(pt);

    pt.x = x - size_x / 2;
    pt.y = y + size_y / 2;
    clear_poly.push_back(pt);

    controller_costmap_ros_->getCostmap()->setConvexPolygonCost(clear_poly, costmap_2d::FREE_SPACE);
  }

//清除costmap
  bool MoveBase::clearCostmapsService(std_srvs::Empty::Request &req, std_srvs::Empty::Response &resp){

    boost::unique_lock<costmap_2d::Costmap2D::mutex_t> lock_controller(*(controller_costmap_ros_->getCostmap()->getMutex()));
    controller_costmap_ros_->resetLayers();  // 清除代价地图的调用
    planner_costmap_ros_->resetLayers();

    boost::unique_lock<costmap_2d::Costmap2D::mutex_t> lock_planner(*(planner_costmap_ros_->getCostmap()->getMutex()));
    planner_costmap_ros_->resetLayers();
    return true;
  }

//当action不活跃时，调用此函数，返回plan
  bool MoveBase::planService(nav_msgs::GetPlan::Request &req, nav_msgs::GetPlan::Response &resp){
    if(as_->isActive()){
      ROS_ERROR("move_base must be in an inactive state to make a plan for an external user");
      return false;
    }
    //make sure we have a costmap for our planner
    if(planner_costmap_ros_ == NULL){
      ROS_ERROR("move_base cannot make a plan for you because it doesn't have a costmap");
      return false;
    }

    geometry_msgs::PoseStamped start;
    //if the user does not specify a start pose, identified by an empty frame id, then use the robot's pose
    if(req.start.header.frame_id.empty())
    {
        geometry_msgs::PoseStamped global_pose;
        if(!getRobotPose(global_pose, planner_costmap_ros_)){
          ROS_ERROR("move_base cannot make a plan for you because it could not get the start pose of the robot");
          return false;
        }
        start = global_pose;
    }
    else
    {
        start = req.start;
    }

    if (make_plan_clear_costmap_) {
      //update the copy of the costmap the planner uses
      clearCostmapWindows(2 * clearing_radius_, 2 * clearing_radius_);
    }

    //first try to make a plan to the exact desired goal
    std::vector<geometry_msgs::PoseStamped> global_plan;
    if(!planner_->makePlan(start, req.goal, global_plan) || global_plan.empty()){
      ROS_DEBUG_NAMED("move_base","Failed to find a plan to exact goal of (%.2f, %.2f), searching for a feasible goal within tolerance", 
          req.goal.pose.position.x, req.goal.pose.position.y);

      //search outwards for a feasible goal within the specified tolerance
      geometry_msgs::PoseStamped p;
      p = req.goal;
      bool found_legal = false;
      float resolution = planner_costmap_ros_->getCostmap()->getResolution();
      float search_increment = resolution*3.0;
      if(req.tolerance > 0.0 && req.tolerance < search_increment) search_increment = req.tolerance;
      for(float max_offset = search_increment; max_offset <= req.tolerance && !found_legal; max_offset += search_increment) {
        for(float y_offset = 0; y_offset <= max_offset && !found_legal; y_offset += search_increment) {
          for(float x_offset = 0; x_offset <= max_offset && !found_legal; x_offset += search_increment) {

            //don't search again inside the current outer layer
            if(x_offset < max_offset-1e-9 && y_offset < max_offset-1e-9) continue;

            //search to both sides of the desired goal
            for(float y_mult = -1.0; y_mult <= 1.0 + 1e-9 && !found_legal; y_mult += 2.0) {

              //if one of the offsets is 0, -1*0 is still 0 (so get rid of one of the two)
              if(y_offset < 1e-9 && y_mult < -1.0 + 1e-9) continue;

              for(float x_mult = -1.0; x_mult <= 1.0 + 1e-9 && !found_legal; x_mult += 2.0) {
                if(x_offset < 1e-9 && x_mult < -1.0 + 1e-9) continue;

                p.pose.position.y = req.goal.pose.position.y + y_offset * y_mult;
                p.pose.position.x = req.goal.pose.position.x + x_offset * x_mult;

                if(planner_->makePlan(start, p, global_plan)){
                  if(!global_plan.empty()){

                    if (make_plan_add_unreachable_goal_) {
                      //adding the (unreachable) original goal to the end of the global plan, in case the local planner can get you there
                      //(the reachable goal should have been added by the global planner)
                      global_plan.push_back(req.goal);
                    }

                    found_legal = true;
                    ROS_DEBUG_NAMED("move_base", "Found a plan to point (%.2f, %.2f)", p.pose.position.x, p.pose.position.y);
                    break;
                  }
                }
                else{
                  ROS_DEBUG_NAMED("move_base","Failed to find a plan to point (%.2f, %.2f)", p.pose.position.x, p.pose.position.y);
                }
              }
            }
          }
        }
      }
    }

    //copy the plan into a message to send out
    resp.plan.poses.resize(global_plan.size());
    for(unsigned int i = 0; i < global_plan.size(); ++i){
      resp.plan.poses[i] = global_plan[i];
    }

    return true;
  }

  MoveBase::~MoveBase(){
    recovery_behaviors_.clear();

    delete dsrv_;

    if(as_ != NULL)
      delete as_;

    if(planner_costmap_ros_ != NULL)
      delete planner_costmap_ros_;

    if(controller_costmap_ros_ != NULL)
      delete controller_costmap_ros_;

    planner_thread_->interrupt();
    planner_thread_->join();

    delete planner_thread_;

    delete planner_plan_;
    delete latest_plan_;
    delete controller_plan_;

    planner_.reset();
    tc_.reset();
  }

//新的全局规划，goal 规划的目标点，plan 规划
//该函数先进行一些预备工作，如检查全局代价地图、起始位姿，然后将起始位姿的数据格式做转换
  bool MoveBase::makePlan(const geometry_msgs::PoseStamped& goal, std::vector<geometry_msgs::PoseStamped>& plan){
    boost::unique_lock<costmap_2d::Costmap2D::mutex_t> lock(*(planner_costmap_ros_->getCostmap()->getMutex()));

    //make sure to set the plan to be empty initially
    //初始化空plan
    plan.clear();

    //since this gets called on handle activate
    //如果没有全局代价地图，返回false，因为全局规划必须基于全局代价地图
    if(planner_costmap_ros_ == NULL) {
      ROS_ERROR("Planner costmap ROS is NULL, unable to create global plan");
      return false;
    }

    //get the starting pose of the robot
    //如果得不到机器人的起始位姿，返回false
    geometry_msgs::PoseStamped global_pose;
    if(!getRobotPose(global_pose, planner_costmap_ros_)) {
      ROS_WARN("Unable to get starting pose of robot, unable to create global plan");
      return false;
    }

    const geometry_msgs::PoseStamped& start = global_pose;

    //if the planner fails or returns a zero length plan, planning failed
    //调用BaseGlobalPlanner类的makePlan函数做全局规划
    //如果全局规划失败或者全局规划为空，返回false
    if(!planner_->makePlan(start, goal, plan) || plan.empty()){
      ROS_DEBUG_NAMED("move_base","Failed to find a  plan to point (%.2f, %.2f)", goal.pose.position.x, goal.pose.position.y);
      return false;
    }

    return true;
  }

//发布速度为0的指令
  void MoveBase::publishZeroVelocity(){
    geometry_msgs::Twist cmd_vel;
    cmd_vel.linear.x = 0.0;
    cmd_vel.linear.y = 0.0;
    cmd_vel.angular.z = 0.0;
    vel_pub_.publish(cmd_vel);
  }

  bool MoveBase::isQuaternionValid(const geometry_msgs::Quaternion& q){
    //first we need to check if the quaternion has nan's or infs
    //1.检查四元数是否元素完整
    if(!std::isfinite(q.x) || !std::isfinite(q.y) || !std::isfinite(q.z) || !std::isfinite(q.w)){
      ROS_ERROR("Quaternion has nans or infs... discarding as a navigation goal");
      return false;
    }

    tf2::Quaternion tf_q(q.x, q.y, q.z, q.w);

    //next, we need to check if the length of the quaternion is close to zero
   //2.检查四元数是否趋近于0
    if(tf_q.length2() < 1e-6){
      ROS_ERROR("Quaternion has length close to zero... discarding as navigation goal");
      return false;
    }

    //next, we'll normalize the quaternion and check that it transforms the vertical vector correctly
    //3.对四元数规范化，转化为vector
    tf_q.normalize();

    tf2::Vector3 up(0, 0, 1);

    double dot = up.dot(up.rotate(tf_q.getAxis(), tf_q.getAngle()));

    if(fabs(dot - 1) > 1e-3){
      ROS_ERROR("Quaternion is invalid... for navigation the z-axis of the quaternion must be close to vertical.");
      return false;
    }

    return true;
  }

  geometry_msgs::PoseStamped MoveBase::goalToGlobalFrame(const geometry_msgs::PoseStamped& goal_pose_msg){
  //完成导航点坐标系的转换
    std::string global_frame = planner_costmap_ros_->getGlobalFrameID();
    geometry_msgs::PoseStamped goal_pose, global_pose;
    goal_pose = goal_pose_msg;

    //just get the latest available transform... for accuracy they should send
    //goals in the frame of the planner
    goal_pose.header.stamp = ros::Time();

    try{
      tf_.transform(goal_pose_msg, global_pose, global_frame);
    }
    catch(tf2::TransformException& ex){
      ROS_WARN("Failed to transform the goal pose from %s into the %s frame: %s",
          goal_pose.header.frame_id.c_str(), global_frame.c_str(), ex.what());
      return goal_pose_msg;
    }
    geometry_msgs::PoseStamped global_pose_msg;
    global_pose_msg = global_pose;
    return global_pose_msg;
  }


  void MoveBase::wakePlanner(const ros::TimerEvent& event)
  {
    // we have slept long enough for rate
    planner_cond_.notify_one();
  }

 //全局规划线程，调用makePlan函数，该函数中实际进行全局规划
 //全局规划线程时刻等待被executeCB函数唤醒，当executeCB函数中唤醒planThread并将标志位runPlanner_设置为真，跳出内部的循环，继续进行下面部分。
  void MoveBase::planThread(){
    ROS_DEBUG_NAMED("move_base_plan_thread","Starting planner thread...");
    ros::NodeHandle n;
    ros::Timer timer;
    bool wait_for_wake = false;//标志位置为假，表示线程已唤醒
    boost::unique_lock<boost::recursive_mutex> lock(planner_mutex_);
    while(n.ok()){
      //check if we should run the planner (the mutex is locked)
      // 不断循环，直到wait_for_wake（上面行已置为假）和runPlanner_为真，跳出循环
      while(wait_for_wake || !runPlanner_){
        //if we should not be running the planner then suspend this thread
        //如果waitforwake是真或者runplanner是假，不断执行循环，wait()等待
        ROS_DEBUG_NAMED("move_base_plan_thread","Planner thread is suspending");
       //调用wait函数时，函数会自动调用lock.unlock()释放锁，使得其他被阻塞在锁竞争上的线程得以继续执行
        //比如executeCB中先把标志位runPlanner设为true，然后用notify，使这里的锁由unlock变成lock，竞争到了资源，并且这里的规划线程开始执行
        planner_cond_.wait(lock);
        wait_for_wake = false;
      }
      //start_time设为当前时间
      ros::Time start_time = ros::Time::now();

      //time to plan! get a copy of the goal and unlock the mutex
      //把全局中被更新的全局目标planner_goal存储为临时目标
      geometry_msgs::PoseStamped temp_goal = planner_goal_;
      lock.unlock();
      ROS_DEBUG_NAMED("move_base_plan_thread","Planning...");

      //run planner
      planner_plan_->clear();//全局规划初始化，清空
      //调用MoveBase类的makePlan函数，如果成功为临时目标制定全局规划planner_plan_，则返回true
      bool gotPlan = n.ok() && makePlan(temp_goal, *planner_plan_);

     //若成功为临时目标制定全局规划
      if(gotPlan){
        //输出成功制定全局规划，并打印规划路线上的点数
        ROS_DEBUG_NAMED("move_base_plan_thread","Got Plan with %zu points!", planner_plan_->size());
        //pointer swap the plans under mutex (the controller will pull from latest_plan_)
        //用指针交换planner_plan_和latest_plan_的值
        std::vector<geometry_msgs::PoseStamped>* temp_plan = planner_plan_;

        lock.lock();
        planner_plan_ = latest_plan_;
        latest_plan_ = temp_plan;
        //最近一次有效全局规划的时间设为当前时间
        last_valid_plan_ = ros::Time::now();
        planning_retries_ = 0;
        new_global_plan_ = true;

        ROS_DEBUG_NAMED("move_base_plan_thread","Generated a plan from the base_global_planner");

        //make sure we only start the controller if we still haven't reached the goal
        //确保只有在我们还没到达目标时才启动controller以局部规划
        //如果runPlanner_在调用此函数时被置为真，将MoveBase状态设置为CONTROLLING（局部规划中）
        if(runPlanner_)
          state_ = CONTROLLING;
          //如果规划频率小于0，runPlanner_置为假
        if(planner_frequency_ <= 0)
          runPlanner_ = false;
        lock.unlock();
      }
      //if we didn't get a plan and we are in the planning state (the robot isn't moving)
      //如果全局规划失败并且MoveBase还在planning状态，即机器人没有移动，则进入自转模式
      else if(state_==PLANNING){
        ROS_DEBUG_NAMED("move_base_plan_thread","No Plan...");
        //最迟制定出本次全局规划的时间=上次成功规划的时间+容忍时间
        ros::Time attempt_end = last_valid_plan_ + ros::Duration(planner_patience_);

        //check if we've tried to make a plan for over our time limit or our maximum number of retries
        //issue #496: we stop planning when one of the conditions is true, but if max_planning_retries_
        //is negative (the default), it is just ignored and we have the same behavior as ever
        //检查时间和次数是否超过限制，若其中一项不满足限制，停止全局规划
        lock.lock();
        //对同一目标的全局规划的次数记录+1
        planning_retries_++;
        //如果runplanner被置为真，且目前超时或超次数，进入恢复行为模式
        if(runPlanner_ &&
           (ros::Time::now() > attempt_end || planning_retries_ > uint32_t(max_planning_retries_))){
          //we'll move into our obstacle clearing mode
          //将MoveBase状态设置为恢复行为
          state_ = CLEARING;
          //全局规划标志位置为假
          runPlanner_ = false;  // proper solution for issue #523
          //发布0速度
          publishZeroVelocity();
          //恢复行为触发器状态设置为全局规划失败
          recovery_trigger_ = PLANNING_R;
        }

        lock.unlock();
      }

      //take the mutex for the next iteration
      lock.lock();

      //setup sleep interface if needed
      if(planner_frequency_ > 0){
        ros::Duration sleep_time = (start_time + ros::Duration(1.0/planner_frequency_)) - ros::Time::now();
        if (sleep_time > ros::Duration(0.0)){
          wait_for_wake = true;
          timer = n.createTimer(sleep_time, &MoveBase::wakePlanner, this);
        }
      }
    }
  }

 //MoveBase控制流的主体，调用了MoveBase内另外几个作为子部分的重要成员函数，先后完成了全局规划和局部规划
  void MoveBase::executeCb(const move_base_msgs::MoveBaseGoalConstPtr& move_base_goal)
  {
     //检测收到的目标位置的旋转四元数是否有效，若无效，直接返回
    if(!isQuaternionValid(move_base_goal->target_pose.pose.orientation)){
      as_->setAborted(move_base_msgs::MoveBaseResult(), "Aborting on goal because it was sent with an invalid quaternion");
      return;
    }

    //将目标位置转换到global坐标系下（geometry_msgs形式）
    geometry_msgs::PoseStamped goal = goalToGlobalFrame(move_base_goal->target_pose);
  
    turn_flag = 1;
    //publishZeroVelocity();
    //we have a goal so start the planner启动全局规划
    boost::unique_lock<boost::recursive_mutex> lock(planner_mutex_);
    //用接收到的目标goal来更新全局变量，即全局规划目标，这个值在planThread中会被用来做全局规划的当前目标
    planner_goal_ = goal;
    //全局规划标志位设为真
    runPlanner_ = true;
    //开启全局规划并于此处阻塞
    planner_cond_.notify_one();
    lock.unlock();

   //全局规划完成后，发布目标到current_goal话题上
    current_goal_pub_.publish(goal);
    //创建一个全局规划容器
    std::vector<geometry_msgs::PoseStamped> global_plan;

    //局部规划频率
    ros::Rate r(controller_frequency_);
    //如果代价地图是被关闭的，这里重启
    if(shutdown_costmaps_){
      ROS_DEBUG_NAMED("move_base","Starting up costmaps that were shut down previously");
      planner_costmap_ros_->start();
      controller_costmap_ros_->start();
    }

    //we want to make sure that we reset the last time we had a valid plan and control
    //上一次有效的局部规划时间设为现在
    last_valid_control_ = ros::Time::now();
    //上一次有效的全局规划时间设为现在
    last_valid_plan_ = ros::Time::now();
    //上一次震荡重置时间设为现在
    last_oscillation_reset_ = ros::Time::now();
    //对同一目标的全局规划次数记录归为0
    planning_retries_ = 0;

    ros::NodeHandle n;
    //全局规划完成，接下来循环调用executeCycle函数来控制机器人进行局部规划，完成相应跟随。
    while(n.ok())
    {
     //c_freq_change_被初始化为false
    //如果c_freq_change_即局部规划频率需要中途更改为真，用更改后的controller_frequency_来更新r值
      if(c_freq_change_)
      {
        ROS_INFO("Setting controller frequency to %.2f", controller_frequency_);
        r = ros::Rate(controller_frequency_);
        c_freq_change_ = false;
      }
      //如果action的服务器被抢占
      if(as_->isPreemptRequested()){
        //如果获得了新目标，接收并存储新目标，并将上述过程重新进行一遍
        if(as_->isNewGoalAvailable()){
          //if we're active and a new goal is available, we'll accept it, but we won't shut anything down
          move_base_msgs::MoveBaseGoal new_goal = *as_->acceptNewGoal();

          //检测四元数是否有效
          if(!isQuaternionValid(new_goal.target_pose.pose.orientation)){
            as_->setAborted(move_base_msgs::MoveBaseResult(), "Aborting on goal because it was sent with an invalid quaternion");
            return;
          }

        //将新目标坐标转换到全局坐标系（默认/map）下
          goal = goalToGlobalFrame(new_goal.target_pose);

          //we'll make sure that we reset our state for the next execution cycle
          //重设恢复行为索引位为0
          recovery_index_ = 0;
          //重设MoveBase状态为全局规划中
          state_ = PLANNING;

          //we have a new goal so make sure the planner is awake
          //重新调用planThread进行全局规划
          lock.lock();
          planner_goal_ = goal;
          runPlanner_ = true;
          planner_cond_.notify_one();
          lock.unlock();

          //publish the goal point to the visualizer
          //全局规划成功后，发布新目标到current_goal话题上
          ROS_DEBUG_NAMED("move_base","move_base has received a goal of x: %.2f, y: %.2f", goal.pose.position.x, goal.pose.position.y);
          current_goal_pub_.publish(goal);

          //make sure to reset our timeouts and counters
          last_valid_control_ = ros::Time::now();
          last_valid_plan_ = ros::Time::now();
          last_oscillation_reset_ = ros::Time::now();
          planning_retries_ = 0;
        }
        else {
          //if we've been preempted explicitly we need to shut things down
          //否则，服务器的抢占是由于收到了取消行动的命令
          //重置服务器状态
          resetState();
        
        //action服务器清除相关内容，并调用setPreempted()函数
          //notify the ActionServer that we've successfully preempted
          ROS_DEBUG_NAMED("move_base","Move base preempting the current goal");
          as_->setPreempted();

          //we'll actually return from execute after preempting
          //取消命令后，返回
          return;
        }
      }

      //we also want to check if we've changed global frames because we need to transform our goal pose
      //服务器接收到目标后，没有被新目标或取消命令抢占
      //检查目标是否被转换到全局坐标系（/map）下
      if(goal.header.frame_id != planner_costmap_ros_->getGlobalFrameID()){
        goal = goalToGlobalFrame(goal);

        //we want to go back to the planning state for the next execution cycle
        //恢复行为索引重置为0，MoveBase状态置为全局规划中
        recovery_index_ = 0;
        state_ = PLANNING;

        //we have a new goal so make sure the planner is awake
        lock.lock();
        planner_goal_ = goal;
        runPlanner_ = true;
        planner_cond_.notify_one();
        lock.unlock();

        //publish the goal point to the visualizer
        ROS_DEBUG_NAMED("move_base","The global frame for move_base has changed, new frame: %s, new goal position x: %.2f, y: %.2f", goal.header.frame_id.c_str(), goal.pose.position.x, goal.pose.position.y);
        current_goal_pub_.publish(goal);

        //make sure to reset our timeouts and counters
        last_valid_control_ = ros::Time::now();
        last_valid_plan_ = ros::Time::now();
        last_oscillation_reset_ = ros::Time::now();
        planning_retries_ = 0;
      }

      //for timing that gives real time even in simulation记录开始局部规划的时刻为当前时间
      ros::WallTime start = ros::WallTime::now();

      //the real work on pursuing a goal is done here调用executeCycle函数进行局部规划，传入目标和全局规划路线
      bool done = executeCycle(goal, global_plan);

      //if we're done, then we'll return from execute
      if(done)
        return;

      //check if execution of the goal has completed in some way
     
     //记录从局部规划开始到这时的时间差
      ros::WallDuration t_diff = ros::WallTime::now() - start;
      //打印用了多长时间完成操作
      ROS_DEBUG_NAMED("move_base","Full control cycle time: %.9f\n", t_diff.toSec());

     //用局部规划频率进行休眠
      r.sleep();
      //make sure to sleep for the remainder of our cycle time

       //cycleTime用来获取从r实例初始化到r实例被调用sleep函数的时间间隔
      //时间间隔超过了1/局部规划频率，且还在局部规划，打印“未达到实际要求，实际上时间是r.cycleTime().toSec()”
      if(r.cycleTime() > ros::Duration(1 / controller_frequency_) && state_ == CONTROLLING)
        ROS_WARN("Control loop missed its desired rate of %.4fHz... the loop actually took %.4f seconds", controller_frequency_, r.cycleTime().toSec());
    }

    //wake up the planner thread so that it can exit cleanly唤醒全局规划线程，以使它能够“干净地退出”
    lock.lock();
    runPlanner_ = true;
    planner_cond_.notify_one();
    lock.unlock();

    //if the node is killed then we'll abort and return如果节点被关闭了，那么Action服务器也关闭并返回
    as_->setAborted(move_base_msgs::MoveBaseResult(), "Aborting on the goal because the node has been killed");
    return;
  }

  double MoveBase::distance(const geometry_msgs::PoseStamped& p1, const geometry_msgs::PoseStamped& p2)
  {
    return hypot(p1.pose.position.x - p2.pose.position.x, p1.pose.position.y - p2.pose.position.y);
  }

   //局部规划
   //executeCycle函数的作用是进行局部规划，函数先声明了将要发布的速度，然后获取当前位姿并格式转换。
  bool MoveBase::executeCycle(geometry_msgs::PoseStamped& goal, std::vector<geometry_msgs::PoseStamped>& global_plan){
    boost::recursive_mutex::scoped_lock ecl(configuration_mutex_);
    //we need to be able to publish velocity commands
    //声明速度信息
    geometry_msgs::Twist cmd_vel;

    //update feedback to correspond to our curent position
    //声明全局姿态
    geometry_msgs::PoseStamped global_pose;
    //从全局代价地图上获取当前位姿
    getRobotPose(global_pose, planner_costmap_ros_);
    //把当前位姿转换为geometry_msgs::PoseStamped格式，存储在current_position中
    const geometry_msgs::PoseStamped& current_position = global_pose;

       double length1 = sqrt(pow((current_position.pose.position.x-planner_goal_.pose.position.x),2)+pow((current_position.pose.position.y-planner_goal_.pose.position.y),2));
    //ROS_ERROR("Length: %f" ,length1);
    if(length1 < 2.0)
      nearly = 1;
      else
      {
        nearly = 0;
      }

    //push the feedback out
    //feedback指的是从服务端周期反馈回客户端的信息，把当前位姿反馈给客户端
    move_base_msgs::MoveBaseFeedback feedback;
    feedback.base_position = current_position;
    as_->publishFeedback(feedback);

    //check to see if we've moved far enough to reset our oscillation timeout
    //如果长时间内移动距离没有超过震荡距离，那么认为机器人在震荡（长时间被困在一片小区域），进入恢复行为
    if(distance(current_position, oscillation_pose_) >= oscillation_distance_)
    {
      last_oscillation_reset_ = ros::Time::now();//把最新的振荡重置设置为当前时间
      oscillation_pose_ = current_position;//振荡位姿设为当前姿态

      //if our last recovery was caused by oscillation, we want to reset the recovery index 
      //如果我们上一次的恢复行为是由振荡引起，我们就重新设置恢复行为的索引
      if(recovery_trigger_ == OSCILLATION_R)
        recovery_index_ = 0;
    }

    //check that the observation buffers for the costmap are current, we don't want to drive blind
    //检查局部代价地图是否是当前的
    if(!controller_costmap_ros_->isCurrent()){
      ROS_WARN("[%s]:Sensor data is out of date, we're not going to allow commanding of the base for safety",ros::this_node::getName().c_str());
      publishZeroVelocity();
      return false;
    }

    //if we have a new plan then grab it and give it to the controller
    //new_global_plan_标志位在planThread中被置为真，表示生成了全局规划
    if(new_global_plan_){
      //make sure to set the new plan flag to false
      new_global_plan_ = false;//重置标志位

      ROS_DEBUG_NAMED("move_base","Got a new plan...swap pointers");

      //do a pointer swap under mutex
      std::vector<geometry_msgs::PoseStamped>* temp_plan = controller_plan_;

     //在指针的保护下，交换latest_plan和controller_plan的值
      boost::unique_lock<boost::recursive_mutex> lock(planner_mutex_);
      //controller_plan_存储“当前最新制定好的待到达的全局规划”
      controller_plan_ = latest_plan_;
      //使得全局规划制定好的planner_plan经由latest_plan一路传递到controller_plan供局部规划器使用
      latest_plan_ = temp_plan;
      lock.unlock();
      ROS_DEBUG_NAMED("move_base","pointers swapped!");

     //在实例tc_上调用局部规划器BaseLocalPlanner的类函数setPlan()， 把全局规划的结果传递给局部规划器，如果传递失败，退出并返回。
      if(!tc_->setPlan(*controller_plan_)){
        //ABORT and SHUTDOWN COSTMAPS
        ROS_ERROR("Failed to pass global plan to the controller, aborting.");
        resetState();

        //disable the planner thread停止全局规划线程
        lock.lock();
        runPlanner_ = false;
        lock.unlock();

       //停止Action服务器，打印“将全局规划传递至局部规划器控制失败”
        as_->setAborted(move_base_msgs::MoveBaseResult(), "Failed to pass global plan to the controller.");
        return true;
      }

      //make sure to reset recovery_index_ since we were able to find a valid plan
      //如果我们找到有效的局部规划路线，且恢复行为是“全局规划失败”，重置恢复行为索引
      if(recovery_trigger_ == PLANNING_R)
        recovery_index_ = 0;
    }

    //the move_base state machine, handles the control logic for navigation
    switch(state_){
      //if we are in a planning state, then we'll attempt to make a plan如果我们还在全局规划状态
      //全局规划还没完成，还没得到一个全局路线，那么唤醒一个全局规划线程去制定全局路线
      case PLANNING:
        {
          boost::recursive_mutex::scoped_lock lock(planner_mutex_);
          runPlanner_ = true;
          planner_cond_.notify_one();
        }
        ROS_DEBUG_NAMED("move_base","Waiting for plan, in the planning state.");
        break;

      //if we're controlling, we'll attempt to find valid velocity commands
      //全局规划成功，得到全局路线，这里进行真正的局部规划，进入CONTROLLING状态，就去找有效的速度控制
      case CONTROLLING:
        ROS_DEBUG_NAMED("move_base","In controlling state.");

        //check to see if we've reached our goal
        //查询是否到达终点，如果到达终点，结束
        if(tc_->isGoalReached()){
          ROS_DEBUG_NAMED("move_base","Goal reached!");
          resetState();

          //disable the planner thread
          //结束全局规划线程
          boost::unique_lock<boost::recursive_mutex> lock(planner_mutex_);
          //标志位置false
          runPlanner_ = false;
          ROS_INFO("daoledaole@@@!!!");
          //Action返回成功
          lock.unlock();

          as_->setSucceeded(move_base_msgs::MoveBaseResult(), "Goal reached.");
          
          ROS_INFO("woyoudaoledaole@@@!!!");

          return true;
        }

        //check for an oscillation condition
        //如果未到达终点，检查是否处于振荡状态
        if(oscillation_timeout_ > 0.0 &&
            last_oscillation_reset_ + ros::Duration(oscillation_timeout_) < ros::Time::now())
        {
          //如果振荡状态超时了，发布0速度
          publishZeroVelocity();
          //MoveBase状态置为恢复行为
          state_ = CLEARING;
          //恢复行为触发器置为，长时间困在一片小区域
          recovery_trigger_ = OSCILLATION_R;
        }
        
        {
         boost::unique_lock<costmap_2d::Costmap2D::mutex_t> lock(*(controller_costmap_ros_->getCostmap()->getMutex()));
        //局部规划器实例tc_被传入了全局规划后，调用computeVelocityCommands函数计算速度存储在cmd_vel中
        if(tc_->computeVelocityCommands(cmd_vel)){
          ROS_DEBUG_NAMED( "move_base", "Got a valid command from the local planner: %.3lf, %.3lf, %.3lf",
                           cmd_vel.linear.x, cmd_vel.linear.y, cmd_vel.angular.z );

          //测一下临界速度,太小的速度电机接收不了
          if(cmd_vel.angular.z<0.02&&cmd_vel.angular.z>0)//0.009
            cmd_vel.angular.z=0.02;
          else if(cmd_vel.angular.z>-0.02&&cmd_vel.angular.z<0)
            cmd_vel.angular.z=-0.02;
          if(cmd_vel.linear.x<0.02&&cmd_vel.linear.x>0)//0.005
            cmd_vel.linear.x=0.02;
          else if(cmd_vel.linear.x>-0.02&&cmd_vel.linear.x<0)
            cmd_vel.linear.x=-0.02;
          //ROS_ERROR("turn_flag : %d,cmd_vel.linear.x: %f,nearly: %d", turn_flag,cmd_vel.linear.x,nearly);
          if((turn_flag == 1) && (cmd_vel.linear.x < 0)&&(nearly == 0))
          {
            turn_flag = 0;
            std::string global_frame = planner_costmap_ros_->getGlobalFrameID();
            ROS_WARN_STREAM(global_frame);
            std::string local_frame = planner_costmap_ros_->getBaseFrameID();
            ROS_WARN_STREAM(local_frame);
            double angle_of_z = 0 , yaw_goal = 0;
            tf2::Quaternion quat;
            tf2::fromMsg(current_position.pose.orientation, quat);
            //ROS_ERROR("X: %f,Y:%f",current_position.pose.position.x,current_position.pose.position.y);
            double roll, pitch, yaw;
            tf2::Matrix3x3(quat).getRPY(roll, pitch, yaw);
            if(cmd_vel.linear.y > 0)
              {
                yaw_goal = yaw + 3.1415926* 3 / 4;
                angle_of_z = 0.7;
              }
            else
              {
                yaw_goal = yaw + 3.1415926* 5 / 4;
                angle_of_z = -0.7;
              }
              if(yaw_goal >= 3.1415926 )
                yaw_goal -= 3.1415926 * 2;

            //ROS_ERROR("Yaw_goal: %f", yaw_goal);
            yaw_goal *= 10;
            int gg = (int)yaw_goal;
            yaw_goal = (double)gg / 10; 
            ROS_ERROR("Yaw_goal: %f", yaw_goal);
            double yaw_current = 1000;

            while(yaw_current != yaw_goal )
            {
              cmd_vel.linear.x = 0;
              cmd_vel.linear.y = 0;
              cmd_vel.angular.z = angle_of_z;
              vel_pub_.publish(cmd_vel);
            getRobotPose(global_pose, planner_costmap_ros_);
            const geometry_msgs::PoseStamped& current_position = global_pose;
           // tf::poseStampedTFToMsg(global_pose, current_position);
              tf2::fromMsg(current_position.pose.orientation, quat);
              //ROS_ERROR("X: %f,Y:%f",current_position.pose.position.x,current_position.pose.position.y);
              tf2::Matrix3x3(quat).getRPY(roll, pitch, yaw);
              yaw *= 10;
              gg = (int)yaw; 
              yaw_current = (double)gg / 10;
              //ROS_ERROR("Yaw_current: %f", yaw_current);
              cmd_vel.linear.x = 0;
              cmd_vel.linear.y = 0;
              cmd_vel.angular.z = 0;
            }
          }

         //若成功计算速度，上一次有效局部控制的时间设为当前
         last_valid_control_ = ros::Time::now();
          //make sure that we send the velocity command to the base
          //向底盘发送速度控制消息，一个循环只发一次速度命令
          vel_pub_.publish(cmd_vel);
          //如果恢复行为触发器值是局部规划失败，把索引置0
          if(recovery_trigger_ == CONTROLLING_R)
            recovery_index_ = 0;
        }
        //若速度计算失败
        else {
          ROS_DEBUG_NAMED("move_base", "The local planner could not find a valid plan.");
          //计算局部规划用时限制
          ros::Time attempt_end = last_valid_control_ + ros::Duration(controller_patience_);

          //check if we've tried to find a valid control for longer than our time limit
          //若局部规划用时超过限制
          if(ros::Time::now() > attempt_end){
            //we'll move into our obstacle clearing mode
            //发布0速度，进入恢复行为，触发器置为局部规划失败
            publishZeroVelocity();
            state_ = CLEARING;
            recovery_trigger_ = CONTROLLING_R;
          }
          //若局部规划用时没超过限制
          else{
            //otherwise, if we can't find a valid control, we'll go back to planning
            //发布0速度，在机器人当前位置再次回到全局规划
            last_valid_plan_ = ros::Time::now();
            planning_retries_ = 0;
            state_ = PLANNING;
            publishZeroVelocity();

            //enable the planner thread in case it isn't running on a clock
            boost::unique_lock<boost::recursive_mutex> lock(planner_mutex_);
            runPlanner_ = true;
            planner_cond_.notify_one();
            lock.unlock();
          }
        }
        }

        break;

      //we'll try to clear out space with any user-provided recovery behaviors
      //如果全局规划失败，进入了恢复行为状态，我们尝试去用用户提供的恢复行为去清除空间
      case CLEARING:
        ROS_DEBUG_NAMED("move_base","In clearing/recovery state");
        //we'll invoke whatever recovery behavior we're currently on if they're enabled
        //如果允许使用恢复行为，且恢复行为索引值小于恢复行为数组的大小
        if(recovery_behavior_enabled_ && recovery_index_ < recovery_behaviors_.size()){
          ROS_DEBUG_NAMED("move_base_recovery","Executing behavior %u of %zu", recovery_index_, recovery_behaviors_.size());
/*
          move_base_msgs::RecoveryStatus msg;
          msg.pose_stamped = current_position;
          msg.current_recovery_number = recovery_index_;
          msg.total_number_of_recoveries = recovery_behaviors_.size();
          msg.recovery_behavior_name =  recovery_behavior_names_[recovery_index_];

          recovery_status_pub_.publish(msg);
*/
         //开始恢复行为，在executeCycle的循环中一次次迭代恢复行为
          recovery_behaviors_[recovery_index_]->runBehavior();

          //we at least want to give the robot some time to stop oscillating after executing the behavior
          //上一次震荡重置时间设为现在
          last_oscillation_reset_ = ros::Time::now();

          //we'll check if the recovery behavior actually worked
          ROS_DEBUG_NAMED("move_base_recovery","Going back to planning state");
          last_valid_plan_ = ros::Time::now();
          planning_retries_ = 0;
          state_ = PLANNING;

          //update the index of the next recovery behavior that we'll try
          recovery_index_++;
            if(recovery_behavior_enabled_){
            // boost::thread* xm_thread_;
            // xm_thread_ =new boost::thread(&MoveBase::xm_work,this,1);
           // base_server::xmRevive xxm(1);
            ROS_ERROR("Never! Give up!");
          } 
        }
        //若恢复行为无效
        else{
          //打印“所有的恢复行为都失败了，关闭全局规划器”
          ROS_DEBUG_NAMED("move_base_recovery","All recovery behaviors have failed, locking the planner and disabling it.");
          //disable the planner thread
          geometry_msgs::Twist cmd;
           //  ROS_ERROR("give xm's life to god!!!");
          boost::unique_lock<boost::recursive_mutex> lock(planner_mutex_);
          runPlanner_ = false;
          lock.unlock();

          ROS_DEBUG_NAMED("move_base_recovery","Something should abort after this.");

         //反馈失败的具体信息
         if(recovery_trigger_ == CONTROLLING_R){
            ROS_ERROR("Aborting because a valid control could not be found. Even after executing all recovery behaviors");
            ROS_ERROR("xm never die!");

            /*double current_x = current_position.pose.position.x;
            double current_y = current_position.pose.position.y;
  
            ROS_ERROR("goal.x: %f,goal.y: %f",point_.x,point_.y);
            while(!( current_y == point_.y))
              {
                boost::asio::io_service io;
                boost::asio::deadline_timer t(io, boost::posix_time::millisec(20));  
                t.wait(); 
                planner_costmap_ros_->getRobotPose(global_pose);
                tf::poseStampedTFToMsg(global_pose, current_position);
                current_x = current_position.pose.position.x + 0.005;current_y = current_position.pose.position.y + 0.005;
                current_x *= 100;gg = (int)current_x; current_x = (double)gg / 100;
                current_y *= 100;gg = (int)current_y;current_y = (double)gg / 100;
                ROS_ERROR("current_x: %f,current_y: %f",current_x,current_y);
                vel_pub_.publish(cmd);
              }
              cmd.linear.x = 0,cmd.linear.y = 0,cmd.angular.z = 0;
              vel_pub_.publish(cmd);*/
              cmd.linear.x = -0.1,cmd.linear.y = 0,cmd.angular.z = 0;
              for(int i = 0;i < 50;i++)
              {
                boost::asio::io_service io;
                boost::asio::deadline_timer t(io, boost::posix_time::millisec(20));  
                t.wait();
                vel_pub_.publish(cmd);
              }
              state_ = PLANNING;
            return false;
            as_->setAborted(move_base_msgs::MoveBaseResult(), "Failed to find a valid control. Even after executing recovery behaviors.");
         
          }
          else if(recovery_trigger_ == PLANNING_R){
            ROS_ERROR("Aborting because a valid plan could not be found. Even after executing all recovery behaviors");
            ROS_ERROR("xm never die!");
            cmd.linear.x = -0.1,cmd.linear.y = 0,cmd.angular.z = 0;
              for(int i = 0;i < 50;i++)
              {
                boost::asio::io_service io;
                boost::asio::deadline_timer t(io, boost::posix_time::millisec(20));  
                t.wait();
                vel_pub_.publish(cmd);
              }
              state_ = PLANNING;
            as_->setAborted(move_base_msgs::MoveBaseResult(), "Failed to find a valid plan. Even after executing recovery behaviors.");
          }
          else if(recovery_trigger_ == OSCILLATION_R){
            ROS_ERROR("Aborting because the robot appears to be oscillating over and over. Even after executing all recovery behaviors");
            ROS_ERROR("die die die!");
            as_->setAborted(move_base_msgs::MoveBaseResult(), "Robot is oscillating. Even after executing recovery behaviors.");
          }
          resetState();
          return true;
        }
        break;
      default:
        ROS_ERROR("This case should never be reached, something is wrong, aborting");
        resetState();
        //disable the planner thread
        //关闭全局规划器
        boost::unique_lock<boost::recursive_mutex> lock(planner_mutex_);
        runPlanner_ = false;
        lock.unlock();
        as_->setAborted(move_base_msgs::MoveBaseResult(), "Reached a case that should not be hit in move_base. This is a bug, please report it.");
        return true;
    }

    //we aren't done yet
    return false;
  }
  
/*
  geometry_msgs::Point MoveBase::find_true_place(double wx, double wy)
  {
      unsigned int my, mx; 
      int min_x, max_x, min_y, max_y, min_xy = 99999;
      costmap_2d::Costmap2D* costmap_ = planner_costmap_ros_->getCostmap();
      costmap_->worldToMap(wx, wy, mx, my);
      geometry_msgs::Point point_;

      min_x = mx - 30, max_x = mx + 30, min_y = my - 30, max_y = my + 30; 
      if(min_y < 0 )min_y = 0;
      if(max_y > costmap_->getSizeInCellsY())max_y = costmap_->getSizeInCellsY();
      if(min_x < 0 )min_x = 0;
      if(max_x > costmap_->getSizeInCellsX()) max_x = costmap_->getSizeInCellsX();
      //ROS_ERROR("min_x:%d  max_x:%d,min_y:%d,max_y:%d,my: %d,mx: %d",min_x, max_x,min_y,max_y,my,mx);
      for(int x = min_x; x <= max_x; x++)
        for(int y = min_y; y <= max_y; y++)
        {
          

          int x_ = x - mx; int y_ = y - my;
          int distance = sqrt((x_) * (x_) + (y_) * (y_));
          if(costmap_->getCost(x, y) == 0 && distance < min_xy)
          {
            min_xy = distance;
            double wx2, wy2;
            costmap_->mapToWorld(x, y, wx2, wy2); 
            point_.x = wx2, point_.y = wy2;
          }
        }
    return point_;
  }

    bool MoveBase::the_true_place(costmap_2d::Costmap2D* costmap_, unsigned int mx, unsigned int my)
  {
    int min_x, max_x, min_y, max_y;
    min_x = mx - 1, max_x = mx + 1, min_y = my - 1, max_y = my + 1;
    if(min_y < 0) min_y = 0;
    if(max_y > costmap_->getSizeInCellsY()) max_y = costmap_->getSizeInCellsY();
    if(min_x < 0) min_x = 0;
    if(max_x > costmap_->getSizeInCellsX()) max_x = costmap_->getSizeInCellsX();
    for(unsigned int i = min_x; i <= max_x; i++)
      for(unsigned int j = min_y; j <= max_y; j++)
      {
        //ROS_ERROR("cost: %d,x:%d ,y:%d",costmap_->getCost(i,j), i, j);

        if(costmap_->getCost(i,j) < 50)
          return 0;
      }
  return 1;
  }
  */

  // void MoveBase::xm_work(int choice){
//   base_server::xmRevive xm(choice);
// }
  // void MoveBase::xm_never_giveUp(){
  //   ROS_ERROR("I don't want to give up!!!");
  //   ros::NodeHandle nh;
  //   ros::Subscriber detect_obstacle=nh.subscribe("/xm_detect_response",1,&MoveBase::publish,this);
  //   bool detect_door;//如果为true，说明有另一个节点在发布请求
  //   nh.param("detect_door",detect_door,true);
  //   if(detect_door==false){
  //     ros::Publisher detect = nh.advertise<xm_msgs::xm_detect_request>("/xm_detect_request",1);
  //     xm_msgs::xm_detect_request msg;
  //     msg.need = true;
  //     detect.publish(msg);
  //   }
  //   ros::Rate r(1.0);//等待回调队列被填充
  //   r.sleep();
  //   ros::spinOnce();   
  // }
  // void MoveBase::publish(const xm_msgs::xm_detect_response& response){
  //   ros::NodeHandle nh;
  //   ros::Publisher xm_control=nh.advertise<geometry_msgs::Twist>("/mobile_base/mobile_base_controller/smooth_cmd_vel", 1);
  //   geometry_msgs::Twist cmd;
  //   if(response.direction[2]==false){//false就是没有障碍物
  //     cmd.linear.x = 0.25;
  //     xm_control.publish(cmd);
  //     ROS_ERROR("Move front!");
  //   }
  //   else if(response.direction[1]==false&&response.direction[0]==true){
  //     //假如右边没有障碍物左边有，就往右h后退一点
  //     cmd.angular.z = 0.25;
  //     cmd.linear.x = -0.1;
  //     xm_control.publish(cmd);
  //     ROS_ERROR("Move right!");
  //   }
  //   else if(response.direction[0]==false&&response.direction[1]==true){
  //     cmd.angular.z = -0.25;
  //     cmd.linear.x = -0.1;
  //     xm_control.publish(cmd);
  //     ROS_ERROR("Move left!");
  //   }
  //   else{
  //     cmd.linear.x = -0.25;
  //     xm_control.publish(cmd);
  //     ROS_ERROR("Move back!");
  //   }
  //   ROS_ERROR("$$$$XM_CONTROL MADE");
  // }


  bool MoveBase::loadRecoveryBehaviors(ros::NodeHandle node){
    XmlRpc::XmlRpcValue behavior_list;
    if(node.getParam("recovery_behaviors", behavior_list)){
      if(behavior_list.getType() == XmlRpc::XmlRpcValue::TypeArray){
        for(int i = 0; i < behavior_list.size(); ++i){
          if(behavior_list[i].getType() == XmlRpc::XmlRpcValue::TypeStruct){
            if(behavior_list[i].hasMember("name") && behavior_list[i].hasMember("type")){
              //check for recovery behaviors with the same name
              for(int j = i + 1; j < behavior_list.size(); j++){
                if(behavior_list[j].getType() == XmlRpc::XmlRpcValue::TypeStruct){
                  if(behavior_list[j].hasMember("name") && behavior_list[j].hasMember("type")){
                    std::string name_i = behavior_list[i]["name"];
                    std::string name_j = behavior_list[j]["name"];
                    if(name_i == name_j){
                      ROS_ERROR("A recovery behavior with the name %s already exists, this is not allowed. Using the default recovery behaviors instead.", 
                          name_i.c_str());
                      return false;
                    }
                  }
                }
              }
            }
            else{
              ROS_ERROR("Recovery behaviors must have a name and a type and this does not. Using the default recovery behaviors instead.");
              return false;
            }
          }
          else{
            ROS_ERROR("Recovery behaviors must be specified as maps, but they are XmlRpcType %d. We'll use the default recovery behaviors instead.",
                behavior_list[i].getType());
            return false;
          }
        }

        //if we've made it to this point, we know that the list is legal so we'll create all the recovery behaviors
        for(int i = 0; i < behavior_list.size(); ++i){
          try{
            //check if a non fully qualified name has potentially been passed in
            if(!recovery_loader_.isClassAvailable(behavior_list[i]["type"])){
              std::vector<std::string> classes = recovery_loader_.getDeclaredClasses();
              for(unsigned int i = 0; i < classes.size(); ++i){
                if(behavior_list[i]["type"] == recovery_loader_.getName(classes[i])){
                  //if we've found a match... we'll get the fully qualified name and break out of the loop
                  ROS_WARN("Recovery behavior specifications should now include the package name. You are using a deprecated API. Please switch from %s to %s in your yaml file.",
                      std::string(behavior_list[i]["type"]).c_str(), classes[i].c_str());
                  behavior_list[i]["type"] = classes[i];
                  break;
                }
              }
            }

            boost::shared_ptr<nav_core::RecoveryBehavior> behavior(recovery_loader_.createInstance(behavior_list[i]["type"]));

            //shouldn't be possible, but it won't hurt to check
            if(behavior.get() == NULL){
              ROS_ERROR("The ClassLoader returned a null pointer without throwing an exception. This should not happen");
              return false;
            }

            //initialize the recovery behavior with its name
            behavior->initialize(behavior_list[i]["name"], &tf_, planner_costmap_ros_, controller_costmap_ros_);
            recovery_behavior_names_.push_back(behavior_list[i]["name"]);
            recovery_behaviors_.push_back(behavior);
          }
          catch(pluginlib::PluginlibException& ex){
            ROS_ERROR("Failed to load a plugin. Using default recovery behaviors. Error: %s", ex.what());
            return false;
          }
        }
      }
      else{
        ROS_ERROR("The recovery behavior specification must be a list, but is of XmlRpcType %d. We'll use the default recovery behaviors instead.", 
            behavior_list.getType());
        return false;
      }
    }
    else{
      //if no recovery_behaviors are specified, we'll just load the defaults
      return false;
    }

    //if we've made it here... we've constructed a recovery behavior list successfully
    return true;
  }

  //we'll load our default recovery behaviors here
  void MoveBase::loadDefaultRecoveryBehaviors(){
    recovery_behaviors_.clear();
    try{
      //we need to set some parameters based on what's been passed in to us to maintain backwards compatibility
      ros::NodeHandle n("~");
      n.setParam("conservative_reset/reset_distance", conservative_reset_dist_);
      n.setParam("aggressive_reset/reset_distance", circumscribed_radius_ * 4);

      //first, we'll load a recovery behavior to clear the costmap
      boost::shared_ptr<nav_core::RecoveryBehavior> cons_clear(recovery_loader_.createInstance("clear_costmap_recovery/ClearCostmapRecovery"));
      cons_clear->initialize("conservative_reset", &tf_, planner_costmap_ros_, controller_costmap_ros_);
      recovery_behaviors_.push_back(cons_clear);

      //next, we'll load a recovery behavior to rotate in place
      boost::shared_ptr<nav_core::RecoveryBehavior> rotate(recovery_loader_.createInstance("rotate_recovery/RotateRecovery"));
      if(clearing_rotation_allowed_){
        rotate->initialize("rotate_recovery", &tf_, planner_costmap_ros_, controller_costmap_ros_);
        recovery_behaviors_.push_back(rotate);
      }

      //next, we'll load a recovery behavior that will do an aggressive reset of the costmap
      boost::shared_ptr<nav_core::RecoveryBehavior> ags_clear(recovery_loader_.createInstance("clear_costmap_recovery/ClearCostmapRecovery"));
      ags_clear->initialize("aggressive_reset", &tf_, planner_costmap_ros_, controller_costmap_ros_);
      recovery_behaviors_.push_back(ags_clear);

      //we'll rotate in-place one more time
      if(clearing_rotation_allowed_)
        recovery_behaviors_.push_back(rotate);
    }
    catch(pluginlib::PluginlibException& ex){
      ROS_FATAL("Failed to load a plugin. This should not happen on default recovery behaviors. Error: %s", ex.what());
    }

    return;
  }

  void MoveBase::resetState(){
    // Disable the planner thread
    boost::unique_lock<boost::recursive_mutex> lock(planner_mutex_);
    runPlanner_ = false;
    lock.unlock();

    // Reset statemachine
    state_ = PLANNING;
    recovery_index_ = 0;
    recovery_trigger_ = PLANNING_R;
    publishZeroVelocity();

    //if we shutdown our costmaps when we're deactivated... we'll do that now
    if(shutdown_costmaps_){
      ROS_DEBUG_NAMED("move_base","Stopping costmaps");
      planner_costmap_ros_->stop();
      controller_costmap_ros_->stop();
    }
  }

  bool MoveBase::getRobotPose(geometry_msgs::PoseStamped& global_pose, costmap_2d::Costmap2DROS* costmap)
  {
    tf2::toMsg(tf2::Transform::getIdentity(), global_pose.pose);
    geometry_msgs::PoseStamped robot_pose;
    tf2::toMsg(tf2::Transform::getIdentity(), robot_pose.pose);
    robot_pose.header.frame_id = robot_base_frame_;
    robot_pose.header.stamp = ros::Time(); // latest available
    ros::Time current_time = ros::Time::now();  // save time for checking tf delay later

    // get robot pose on the given costmap frame
    try
    {
      tf_.transform(robot_pose, global_pose, costmap->getGlobalFrameID());
    }
    catch (tf2::LookupException& ex)
    {
      ROS_ERROR_THROTTLE(1.0, "No Transform available Error looking up robot pose: %s\n", ex.what());
      return false;
    }
    catch (tf2::ConnectivityException& ex)
    {
      ROS_ERROR_THROTTLE(1.0, "Connectivity Error looking up robot pose: %s\n", ex.what());
      return false;
    }
    catch (tf2::ExtrapolationException& ex)
    {
      ROS_ERROR_THROTTLE(1.0, "Extrapolation Error looking up robot pose: %s\n", ex.what());
      return false;
    }

    // check if global_pose time stamp is within costmap transform tolerance
    if (current_time.toSec() - global_pose.header.stamp.toSec() > costmap->getTransformTolerance())
    {
      ROS_WARN_THROTTLE(1.0, "Transform timeout for %s. " \
                        "Current time: %.4f, pose stamp: %.4f, tolerance: %.4f", costmap->getName().c_str(),
                        current_time.toSec(), global_pose.header.stamp.toSec(), costmap->getTransformTolerance());
      return false;
    }

    return true;
  }
};