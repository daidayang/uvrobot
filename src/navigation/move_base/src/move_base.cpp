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
#include <cmath>

#include <boost/algorithm/string.hpp>
#include <boost/thread.hpp>

#include <geometry_msgs/Twist.h>

#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <fstream>
#include <thread>
#include <tf2/utils.h>
#include <ctime>
#include "std_msgs/String.h"
#include <chrono>

namespace move_base {

  MoveBase::MoveBase(tf2_ros::Buffer& tf) :
    tf_(tf),
    as_(NULL),
    planner_costmap_ros_(NULL), controller_costmap_ros_(NULL),
    bgp_loader_("nav_core", "nav_core::BaseGlobalPlanner"),
    blp_loader_("nav_core", "nav_core::BaseLocalPlanner"), 
    recovery_loader_("nav_core", "nav_core::RecoveryBehavior"),
    planner_plan_(NULL), latest_plan_(NULL), controller_plan_(NULL),
    runPlanner_(false), setup_(false), p_freq_change_(false), c_freq_change_(false), new_global_plan_(false) 
    {
    
    start_task = true;
    is_first_goal = true;
    is_current_goal_reach = false;

    is_new_one_goal_first = true;
    
    is_current_goals_list_over = true;

    is_last_goal = false;

    id_next_goal = 0;
    loop_count = 0;

    loop_wait = 0;

    backup_sentcmd_count = 0;

    simulation_mode = true;

    execute_5minutes.clear();

    robot_pose_list.clear();
    as_ = new MoveBaseActionServer(ros::NodeHandle(), "move_base", boost::bind(&MoveBase::executeCb, this, _1), false);

    ros::NodeHandle private_nh("~");
    ros::NodeHandle nh;

    recovery_trigger_ = PLANNING_R;

    //get some parameters that will be global to the move base node
    std::string global_planner, local_planner;
    private_nh.param("base_global_planner", global_planner, std::string("navfn/NavfnROS"));
    
    //launch move_base_params.yaml 中为 dwa_local_planner/DWAPlannerROS
    private_nh.param("base_local_planner", local_planner, std::string("base_local_planner/TrajectoryPlannerROS"));
    
    private_nh.param("global_costmap/robot_base_frame", robot_base_frame_, std::string("base_link"));
    private_nh.param("global_costmap/global_frame", global_frame_, std::string("map"));
    
    private_nh.param("planner_frequency", planner_frequency_, 0.0);
    private_nh.param("controller_frequency", controller_frequency_, 20.0);
    private_nh.param("planner_patience", planner_patience_, 5.0);
    private_nh.param("controller_patience", controller_patience_, 15.0);
    private_nh.param("max_planning_retries", max_planning_retries_, -1);  // disabled by default

    private_nh.param("oscillation_timeout", oscillation_timeout_, 0.0);
    private_nh.param("oscillation_distance", oscillation_distance_, 0.5);

    //parameters of make_plan service
    private_nh.param("make_plan_clear_costmap", make_plan_clear_costmap_, true);
    private_nh.param("make_plan_add_unreachable_goal", make_plan_add_unreachable_goal_, true);

    private_nh.param("hardware_topic",topic_to_arduino,std::string("xbox"));
    ROS_INFO("hardware_topic: %s",topic_to_arduino.c_str());
    
    private_nh.param("simulation_mode",simulation_mode,true);
    ROS_INFO("simulation_mode: %d",simulation_mode?1:0);
    //
    get_goal_from_map_ = nh.subscribe<geometry_msgs::PoseStamped>("move_base_simple/goal", 10, boost::bind(&MoveBase::saveGoalFromMapCallback, this, _1));
    //获取巡逻任务
    task_sub_ = nh.subscribe<move_base::TaskData>("task", 10, boost::bind(&MoveBase::receiveTaskCallback, this, _1));
    //反馈任务情况
    task_feedback_pub_ = nh.advertise<move_base::TaskData>("task_status_report",10);

    command_pub_ = nh.advertise<std_msgs::String>(topic_to_arduino,10);

    UVRobot_state_pub_ = nh.advertise<std_msgs::Int32>("UVRobot_state", 10);

    UVRobot_state_ = nh.subscribe<std_msgs::Int32>("UVRobot_state", 10, boost::bind(&MoveBase::getCurrentStateCallback, this, _1));

    robot_pose_publisher_ = nh.advertise<geometry_msgs::PoseStamped>("navi_robot_pose", 100);  
    
    trajectory_robot_pose_list_publisher_ = nh.advertise<nav_msgs::Path>("navi_trajectory_robot_pose", 1,true);

    //set up plan triple buffer
    planner_plan_ = new std::vector<geometry_msgs::PoseStamped>();
    latest_plan_ = new std::vector<geometry_msgs::PoseStamped>();
    controller_plan_ = new std::vector<geometry_msgs::PoseStamped>();

    //set up the planner's thread
    planner_thread_ = new boost::thread(boost::bind(&MoveBase::planThread, this));

    ros::NodeHandle action_nh("move_base");
    //for commanding the base
    vel_pub_ = action_nh.advertise<geometry_msgs::Twist>("cmd_vel", 100);
    //vel_pub_wheel = nh.advertise<geometry_msgs::Twist>("cmd_vel_wheel", 1);


    current_goal_pub_ = private_nh.advertise<geometry_msgs::PoseStamped>("current_goal", 10 );

    
    action_goal_pub_ = action_nh.advertise<move_base_msgs::MoveBaseActionGoal>("goal", 10);

    goal_list_pub_ = action_nh.advertise<nav_msgs::Path>("goal_list", 10);

    //we'll provide a mechanism for some people to send goals as PoseStamped messages over a topic
    //they won't get any useful information back about its status, but this is useful for tools
    //like nav_view and rviz
    ros::NodeHandle simple_nh("move_base_simple");
    goal_sub_ = simple_nh.subscribe<geometry_msgs::PoseStamped>("goal", 1, boost::bind(&MoveBase::goalCB, this, _1));

    action_goal_sub_ = simple_nh.subscribe<geometry_msgs::PoseStamped>("goal", 1,boost::bind(&MoveBase::testGetGoalCallBack, this, _1));
    
    ROS_INFO("MoveBase start run...............");
    
    //一个定时器
    exe_goal_timer  = private_nh.createTimer(ros::Duration(1), &MoveBase::timerCallbackSendGoal,this);
    
    select_goal_timer  = private_nh.createTimer(ros::Duration(1), &MoveBase::timerCallbackSelectTask,this);

    //we'll assume the radius of the robot to be consistent with what's specified for the costmaps
    private_nh.param("local_costmap/inscribed_radius", inscribed_radius_, 0.325);
    private_nh.param("local_costmap/circumscribed_radius", circumscribed_radius_, 0.46);
    private_nh.param("clearing_radius", clearing_radius_, circumscribed_radius_);
    private_nh.param("conservative_reset_dist", conservative_reset_dist_, 3.0);

    private_nh.param("shutdown_costmaps", shutdown_costmaps_, false);
    private_nh.param("clearing_rotation_allowed", clearing_rotation_allowed_, true);
    private_nh.param("recovery_behavior_enabled", recovery_behavior_enabled_, true);

    private_nh.param("goal_list_setting", goal_list_file_, std::string(""));
    ROS_INFO("goal_list_setting: %s",goal_list_file_.c_str());

    private_nh.param("charging_path_file", charging_path_file_, std::string(""));
    ROS_INFO("charging_path_file: %s",charging_path_file_.c_str());

    private_nh.param("auto_start", start_task,true);
    ROS_INFO("auto_start: %d",start_task?1:0);
    
    //create the ros wrapper for the planner's costmap... and initializer a pointer we'll use with the underlying map
    planner_costmap_ros_ = new costmap_2d::Costmap2DROS("global_costmap", tf_);
    planner_costmap_ros_->pause();


/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    //initialize the global planner
    try 
    {
      ROS_INFO("Initial the global planner ...............");
      planner_ = bgp_loader_.createInstance(global_planner); //navfn/NavfnROS
      ROS_INFO("Created global_planner %s", global_planner.c_str());
      planner_->initialize(bgp_loader_.getName(global_planner), planner_costmap_ros_);

    } catch (const pluginlib::PluginlibException& ex) 
    {
      ROS_FATAL("Failed to create the %s planner, are you sure it is properly registered and that the containing library is built? Exception: %s", global_planner.c_str(), ex.what());
      exit(1);
    }

    
    
    
    //create the ros wrapper for the controller's costmap... and initializer a pointer we'll use with the underlying map
    controller_costmap_ros_ = new costmap_2d::Costmap2DROS("local_costmap", tf_);
    controller_costmap_ros_->pause();



//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    //create a local planner　launch move_base_params.yaml 中为 dwa_local_planner/DWAPlannerROS
    try 
    {
      ROS_INFO("Initial the local planner ...............");
      tc_ = blp_loader_.createInstance(local_planner);
      ROS_INFO("Created local_planner %s", local_planner.c_str());
      tc_->initialize(blp_loader_.getName(local_planner), &tf_, controller_costmap_ros_);

    } catch (const pluginlib::PluginlibException& ex) 
    {
      ROS_FATAL("Failed to create the %s planner, are you sure it is properly registered and that the containing library is built? Exception: %s", local_planner.c_str(), ex.what());
      exit(1);
    }

    // Start actively updating costmaps based on sensor data
    planner_costmap_ros_->start();//global 
    controller_costmap_ros_->start();//local

    //advertise a service for getting a plan
    make_plan_srv_ = private_nh.advertiseService("make_plan", &MoveBase::planService, this);

    //advertise a service for clearing the costmaps
    clear_costmaps_srv_ = private_nh.advertiseService("clear_costmaps", &MoveBase::clearCostmapsService, this);

    //if we shutdown our costmaps when we're deactivated... we'll do that now
    if(shutdown_costmaps_)
    {
      ROS_DEBUG_NAMED("move_base","Stopping costmaps initially");
      planner_costmap_ros_->stop();
      controller_costmap_ros_->stop();
    }

    //load any user specified recovery behaviors, and if that fails load the defaults
    if(!loadRecoveryBehaviors(private_nh)){
      loadDefaultRecoveryBehaviors();
    }

    //initially, we'll need to make a plan
    state_ = PLANNING;

    //we'll start executing recovery behaviors at the beginning of our list
    recovery_index_ = 0;



/////////////////////////////////////////////////////////////////////////
    //we're all set up now so we can start the action server
    as_->start();
/////////////////////////////////////////////////////////////////////////

    dsrv_ = new dynamic_reconfigure::Server<move_base::MoveBaseConfig>(ros::NodeHandle("~"));
    dynamic_reconfigure::Server<move_base::MoveBaseConfig>::CallbackType cb = boost::bind(&MoveBase::reconfigureCB, this, _1, _2);
    dsrv_->setCallback(cb);

////////////////////////////////////////////////////////////////////////
    //读取路径点
    std::vector<double> wts;
    std::vector<std::string> pre_cmds,back_cmds;
    std::string goalfile = goal_list_file_;
    ROS_INFO("Get the goal list file : %s",goal_list_file_.c_str());

    boost::unique_lock<boost::recursive_mutex> lock_goals_list(current_goals_over_mutex);
    goal_points_list.clear();
    
    loadGoalPointList(goalfile.c_str(),goal_points_list,wts,pre_cmds,back_cmds);
    publishGoalList(goal_points_list);
    ///////////////////////////////////////////////////////////////////////////////////////
    //读取充电路径path
    wts_charging.clear();
    pre_cmds_charging.clear();
    back_cmds_charging.clear();
    std::string goalfile_charging = charging_path_file_;
    ROS_ERROR("get charging path file : %s",charging_path_file_.c_str());

    goal_points_list_for_charging.clear();
    
    loadGoalPointList(goalfile_charging.c_str(),goal_points_list_for_charging,wts_charging,pre_cmds_charging,back_cmds_charging);
    ROS_ERROR("goal_points_list_for_charging count : %d",goal_points_list_for_charging.size());
    //模拟11号任务产生
    move_base::TaskData simTaskData;
    simTaskData.id = 0;
    simTaskData.task_id = 11;
    simTaskData.room_id = "101";
    simTaskData.goal_list.insert(simTaskData.goal_list.end(),goal_points_list.begin(),goal_points_list.end());
    simTaskData.goal_duration_list = wts;//.resize(goal_points_list.size(),20);//单位为s
    simTaskData.type = 1;//0 定时任务，1 即时任务, 

    simTaskData.command_previous = pre_cmds;//.resize(goal_points_list.size(),"");
    simTaskData.command_back     = back_cmds;//.resize(goal_points_list.size(),"");
    simTaskData.exec_time = "2020-12-03 16:00:00";
    simTaskData.states = "";
    simTaskData.add_or_delete = 1;
    simTaskData.resend_task = 0;
    simTaskData.reserve_1 = "";
    simTaskData.reserve_2 = "";
    task_maintenance_list.push_back(simTaskData);

    // //模拟12号任务产生
    // move_base::TaskData simTaskData2;
    // simTaskData2.id = 1;
    // simTaskData2.task_id = 12;
    // simTaskData2.room_id = "101";
    // simTaskData2.goal_list.insert(simTaskData2.goal_list.end(),goal_points_list.begin(),goal_points_list.end());
    // simTaskData2.goal_duration_list.resize(goal_points_list.size(),1);
    // simTaskData2.type = 0;//定时任务

    // simTaskData2.exec_time = "2020-08-03 16:00:00";
    // simTaskData2.states = "";
    // simTaskData2.add_or_delete = 1;
    // simTaskData2.resend_task = 0;
    // simTaskData2.reserve_1 = "";
    // simTaskData2.reserve_2 = "";
    // task_maintenance_list.push_back(simTaskData2);


    goal_points_list.clear();//填充完删掉

    lock_goals_list.unlock();

    enter_charging_ = false;
    UVR_current_state_ = IDLE;
    UVR_previous_state_ = IDLE;
///////////////////////////////////////////////////////////////////////
  }

  void MoveBase::receiveTaskCallback(const move_base::TaskData::ConstPtr &task)
  {
    ROS_INFO("call receiveTaskCallback .............");

    start_task = true;//demo用，收到task就开始运行

    boost::unique_lock<boost::recursive_mutex> lock_goals_list(current_goals_over_mutex);
    if(task->add_or_delete == 1)
    {
      //加入任务维护列表
      if(task->goal_list.size() != task->goal_duration_list.size() ||
      task->goal_list.size() != task->command_previous.size() ||
      task->goal_list.size() != task->command_back.size()
      )
      {
        ROS_INFO("receive task data error !");
        return;
      }

      
      task_maintenance_list.push_back(*task);
    }else
    {
      //删除
      auto it  = task_maintenance_list.begin();

      while(it != task_maintenance_list.end())
      {
        if(it->task_id == task->task_id)
        {
          it = task_maintenance_list.erase(it);
          it--;
        }
        it++;
      }
    }
    lock_goals_list.unlock();

    return;
  }

  void MoveBase::publishGoalList(const std::vector<geometry_msgs::PoseStamped>& path)
  {
    if(path.empty())
    {
      return;
    }

    //create a message for the plan 
    nav_msgs::Path gui_path;
    gui_path.poses.resize(path.size());
    
    //still set a valid frame so visualization won't hit transform issues
    gui_path.header.frame_id = "map";
    gui_path.header.stamp = ros::Time::now();
     
    for(unsigned int i=0; i < path.size(); i++)
    {
      gui_path.poses[i] = path[i];
    }

    goal_list_pub_.publish(gui_path);
  }

  void MoveBase::testGetGoalCallBack(const geometry_msgs::PoseStamped::ConstPtr& test_goal)
  {
    ROS_INFO("tets click,get move_base_goal,get the test goal %f,%f,%f",test_goal->pose.position.x,test_goal->pose.position.y,test_goal->pose.position.z);
    ROS_INFO("tets click,Get the click point ........................");
  }
  void MoveBase::reconfigureCB(move_base::MoveBaseConfig &config, uint32_t level)
  {
    boost::recursive_mutex::scoped_lock l(configuration_mutex_);

    //The first time we're called, we just want to make sure we have the
    //original configuration
    if(!setup_)
    {
      last_config_    = config;
      default_config_ = config;
      setup_ = true;
      return;
    }

    if(config.restore_defaults) 
    {
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

    if(config.base_global_planner != last_config_.base_global_planner) 
    {
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

    if(config.base_local_planner != last_config_.base_local_planner)
    {
      boost::shared_ptr<nav_core::BaseLocalPlanner> old_planner = tc_;
      //create a local planner
      try 
      {
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

    make_plan_clear_costmap_ = config.make_plan_clear_costmap;
    make_plan_add_unreachable_goal_ = config.make_plan_add_unreachable_goal;

    last_config_ = config;
  }
  int MoveBase::selectFromTaskList()
  {
    int task_id = -1;
    //１．先选择即时任务
    for(int i = 0;i<task_maintenance_list.size();++i)
    {
      if(task_maintenance_list[i].type == 1)
      {
        task_id = task_maintenance_list[i].task_id;
        break;
      }
    }
    //找到即时任务
    if(task_id > -1)
    {
      return task_id;
    }
    
    ///////////////////////////////////////////////////////////
    //2.如果没有即时任务，则选择定时任务
    
    std::map<int,double> taskid_time;
    for(int i = 0;i<task_maintenance_list.size();++i)
    {
      if(task_maintenance_list[i].type == 0)//定时任务
      {
        //计算当前时间与定时的时间差
        double deltime = calSpanTime(task_maintenance_list[i].exec_time);
        ROS_INFO("timing time %s, remain seconds %.2f s.",task_maintenance_list[i].exec_time.c_str(),deltime);
        
        if( -300 <= deltime && deltime <= 2)//提前２秒，到退后300秒都在考虑范围之内
        {
          //如果在5分钟内，则查看是否刚刚执行过
          auto it = execute_5minutes.find(task_maintenance_list[i].task_id);

          if(it == execute_5minutes.end())//5分钟内没有执行过
          {
            //加入待选列表中
            taskid_time[task_maintenance_list[i].task_id] = deltime;
          }else
          {
            ROS_INFO("task %d# have executed.",task_maintenance_list[i].task_id);
          }
          
        }else
        {
          //超过5分钟
          execute_5minutes.erase(task_maintenance_list[i].task_id);
        }
        
      }
    }



    //待选列表中，选出最优值
    if(taskid_time.size() == 0)
    {
      return task_id;
    }else
    {
      //选则时间最小的(列表中里面都是负数，数值越小，表示超过执行时刻越多)
      double delta_time = 1000;
      int    task_id_temp = -1;

      for(auto it = taskid_time.begin(); it != taskid_time.end();it++)
      {
        if(it->second < delta_time)
        {
          delta_time = it->second;
          task_id_temp = it->first;
        }
      }

      //找到要执行的task
      execute_5minutes[task_id_temp] = delta_time;
      
      return task_id_temp;

    }
    


    return 0;
  }
  double MoveBase::calSpanTime(std::string &plan_time)
  {
    if(plan_time == "" || plan_time.length()!= 19)
    {
      return 10e5;
    }

    //1.当前时间
    time_t tt = time(NULL);
    tm* datet = localtime(&tt);

    //计划的的时刻
    struct tm plan_tm;  
    plan_tm.tm_year = datet->tm_year;
    plan_tm.tm_mon  = datet->tm_mon;
    plan_tm.tm_mday = datet->tm_mday;
    plan_tm.tm_hour = std::atof(plan_time.substr(11,2).c_str());
    plan_tm.tm_min  = std::atof(plan_time.substr(14,2).c_str());
    plan_tm.tm_sec  = 0;
    time_t plan_tt = mktime(&plan_tm);

    //计算时间差

    double deltaSec = difftime(plan_tt,tt);

    return deltaSec;
  }

    void MoveBase::stateCheck()
  {

    if(UVR_current_state_ != CHARGING)
    {
      enter_charging_ = false;
    }

    //第一次进入充电状态
    if(UVR_current_state_ == CHARGING && !enter_charging_)
    {
      //做充电一系列动作

      //替换充电目标路径
      //....................
      /////////////
      ROS_ERROR("Enter the charging mode .......");
      // boost::unique_lock<boost::recursive_mutex> lock_goal(goal_mutex);
      // is_current_goal_reach = true;
      // lock_goal.unlock();

      // ///////////////////////////////////////////////
      
      // //设置新的目标点后,要设置初始化状态
      // resetState();

      // //disable the planner thread
      // boost::unique_lock<boost::recursive_mutex> lock(planner_mutex_);
      // runPlanner_ = false;
      // lock.unlock();
      ROS_ERROR("goal_points_list count .......%d",goal_points_list.size());
      // as_->setSucceeded(move_base_msgs::MoveBaseResult(), "Enter charging mode.");
      if(goal_points_list.size()>0)
      {
        //１．如果当前正在进行巡逻任务
        //把充电路径添加当当前巡逻列表中当前目标点的后面，原目标列中，未巡逻的列表删除
        boost::unique_lock<boost::recursive_mutex> lock_goals_list(current_goals_over_mutex);

        //goals
        {
          std::vector<geometry_msgs::PoseStamped> goal_points_temp;

          goal_points_temp.insert(goal_points_temp.end(),goal_points_list.begin(),goal_points_list.begin()+id_next_goal);

          ROS_ERROR("charging path goals count .......%d",goal_points_list_for_charging.size());
          goal_points_temp.insert(goal_points_temp.end(),goal_points_list_for_charging.begin(),goal_points_list_for_charging.end());
          
          goal_points_list = goal_points_temp;//;
        }
        //wati time
        {
          std::vector<double> wait_time_temp;

          wait_time_temp.insert(wait_time_temp.end(),wait_time_list.begin(),wait_time_list.begin()+id_next_goal);

          wait_time_temp.insert(wait_time_temp.end(),wts_charging.begin(),wts_charging.end());
          
          wait_time_list = wait_time_temp;//;
        }
        //previous cmd
        {
          std::vector<std::string> cmd_previous_list_temp;

          cmd_previous_list_temp.insert(cmd_previous_list_temp.end(),cmd_previous_list.begin(),cmd_previous_list.begin()+id_next_goal);

          //ROS_ERROR("charging path goals count .......%d",goal_points_list_for_charging.size());
          cmd_previous_list_temp.insert(cmd_previous_list_temp.end(),pre_cmds_charging.begin(),pre_cmds_charging.end());
          
          cmd_previous_list = cmd_previous_list_temp;//;
        }

        //back cmd
        {
          std::vector<std::string> cmd_back_list_temp;

          cmd_back_list_temp.insert(cmd_back_list_temp.end(),cmd_back_list.begin(),cmd_back_list.begin()+id_next_goal);

          //ROS_ERROR("charging path goals count .......%d",goal_points_list_for_charging.size());
          cmd_back_list_temp.insert(cmd_back_list_temp.end(),back_cmds_charging.begin(),back_cmds_charging.end());
          
          cmd_back_list = cmd_back_list_temp;
          
        }


        lock_goals_list.unlock();
      }
      //2.如果当前没有巡逻任务
      else if(goal_points_list.empty())
      {
        //启动
        boost::unique_lock<boost::recursive_mutex> lock_goals_list(current_goals_over_mutex);
        
        //goal_points_list = goal_points_list_for_charging;

        goal_points_list.clear();
        wait_time_list.clear();
        cmd_previous_list.clear();
        cmd_back_list.clear();

        goal_points_list.insert(goal_points_list.end(),goal_points_list_for_charging.begin(),goal_points_list_for_charging.end());

        ///接收task必须检测waittine,cmd 是否个goals的个数一致
        wait_time_list.insert(wait_time_list.end(),wts_charging.begin(),wts_charging.end());
        cmd_previous_list.insert(cmd_previous_list.end(),pre_cmds_charging.begin(),pre_cmds_charging.end());
        cmd_back_list.insert(cmd_back_list.end(),back_cmds_charging.begin(),back_cmds_charging.end());


        is_current_goals_list_over = false;


        lock_goals_list.unlock();


        //初始化，列表点发布
        is_first_goal = true;

        boost::unique_lock<boost::recursive_mutex> lock_goal(goal_mutex);
        is_current_goal_reach = false;
        lock_goal.unlock();
        
        id_next_goal = 0;
        loop_count = 0;

        ROS_ERROR("get charging path goals count: %d,and start running .......",goal_points_list_for_charging.size());

      }
      
    

      enter_charging_ = true;
    }
  }

  void MoveBase::timerCallbackSelectTask(const ros::TimerEvent& e)
  {

    ///////////////////////////////////////////////////////////////////

    ////////////////////////////////////////////////////////////////////
    //如果是充电模式，则不进行任务选择
    if(UVR_current_state_ == CHARGING)
    {
      ROS_ERROR("in charging mode .............");
      backup_sentcmd_count = 0;
      return;
    }
    
    // //如果是刚退出charging,进入offcharing
    if(UVR_current_state_ == OFFCHARGING && UVR_previous_state_ == CHARGING)
    {
      ROS_ERROR("charging complete, blackout.............");

      //首先保证robot在充电桩前，即当前的充电路径已经走，在最后一个点停下(即充电桩前)
      if(!simulation_mode)//只在实际工况判断，模拟工况忽略
      {
        if(!is_current_goals_list_over)
        {
          return;
        }
      }

      //持续发布后退控制命令5s
      if(backup_sentcmd_count < 5)
      {
        //发布命令
        geometry_msgs::Twist cmd_vel;
        cmd_vel.linear.x = -0.25;
        cmd_vel.linear.y = 0.0;
        cmd_vel.angular.z = 0.0;
        vel_pub_.publish(cmd_vel);
        ROS_ERROR("charging complete, back robot command.............%d",backup_sentcmd_count);
        backup_sentcmd_count++;
      }else
      {
        publishState(IDLE);
        geometry_msgs::Twist cmd_vel;
        cmd_vel.linear.x = 0.0;
        cmd_vel.linear.y = 0.0;
        cmd_vel.angular.z = 0.0;
        vel_pub_.publish(cmd_vel);
        ROS_ERROR("charging complete, publish idle state.............");
      }
      
      
      return;
    }
    // // ROS_INFO("timerCallbackSelectTask3 .............");
    // // //ROS_INFO("call timerCallbackSelectTask .............");
    // //ROS_INFO("is_current_goals_list_over ............. %d",is_current_goals_list_over?1:0);

    if(!is_current_goals_list_over)
    {
      //当前巡逻还没完成
      return;
    }
    
    //select newest task
    
    //加载新的巡逻列表
    boost::unique_lock<boost::recursive_mutex> lock_goals_list(current_goals_over_mutex);

    int task_idsel = selectFromTaskList();

    ROS_INFO("timerCallbackSelectTask4 ............. %d",task_idsel);

    if(task_maintenance_list.size() > 0 && task_idsel >= 0)
    {
      ROS_INFO("execute task  %d# .............",task_idsel);
      
      int sel = -1;
      for(int i = 0;i < task_maintenance_list.size();++i)
      {
        if(task_maintenance_list[i].task_id == task_idsel)
        {
          sel = i;
          break;
        }
      }

      goal_points_list.clear();
      wait_time_list.clear();
      cmd_previous_list.clear();
      cmd_back_list.clear();

      if(sel >-1)
      {
        goal_points_list.insert(goal_points_list.end(),task_maintenance_list[sel].goal_list.begin(),task_maintenance_list[sel].goal_list.end());

        ///接收task必须检测waittine,cmd 是否个goals的个数一致
        wait_time_list.insert(wait_time_list.end(),task_maintenance_list[sel].goal_duration_list.begin(),task_maintenance_list[sel].goal_duration_list.end());
        cmd_previous_list.insert(cmd_previous_list.end(),task_maintenance_list[sel].command_previous.begin(),task_maintenance_list[sel].command_previous.end());
        cmd_back_list.insert(cmd_back_list.end(),task_maintenance_list[sel].command_back.begin(),task_maintenance_list[sel].command_back.end());
        /////
        //反馈task状态
        current_execute_task = task_maintenance_list[sel];
        current_execute_task.states = "Starting";
        task_feedback_pub_.publish(current_execute_task);
        
        publishState(WORKING);
        //UVRobot_state_pub_.publish(WORKING);
        //如果是即时任务，执行完比后删除
        if(task_maintenance_list[sel].type == 1)//即时任务
        {
          task_maintenance_list.erase(task_maintenance_list.begin()+sel);
        }
        
      }
      else
      {
        ROS_INFO("No task  select .............");
      }
      
      is_current_goals_list_over = false;
      
    }
    


    lock_goals_list.unlock();



    //初始化，列表点发布
    is_first_goal = true;

    boost::unique_lock<boost::recursive_mutex> lock_goal(goal_mutex);
    is_current_goal_reach = false;
    lock_goal.unlock();
    
    id_next_goal = 0;
    loop_count = 0;

    return;
  }
  void MoveBase::timerCallbackSendGoal(const ros::TimerEvent& e)
  {
  
    if(!start_task)
    {
      ROS_INFO("start_task: false, not running !!!!!");
      return;
    }

    if(goal_points_list.empty())
    {
      is_current_goals_list_over = true;
      ROS_ERROR("goal list is empty..............");
      return;
    }

    if(loop_count < 0)
    {
      loop_count++;
      return;
    }
    
    bool send = false;
    
    if(is_first_goal)
    {
      send = true;
      is_first_goal = false;
    }

    if(is_current_goal_reach)
    {
      
      ROS_ERROR("current_goal_reach,id_next_goal: %d",id_next_goal);

      int wt = wait_time_list[id_next_goal-1];//获取等待时间

      ROS_INFO("current_goal_wait,seconds: %d",wt);

      
      if(loop_wait < wt)
      {
        if(loop_wait == 0)
        {
          //当前点到达,/////////////////////////////////////
          //1.发布cmd_previous

          ROS_INFO("reach the goal,publish previous command,start wait.....");
          //ROS_INFO("id_next_goal: %d",id_next_goal);
          std::string pcmd = cmd_previous_list[id_next_goal-1];
          std::vector<std::string> cmd_list = split(pcmd,'#');
          std_msgs::String msg_cmd;
          for(int i = 0;i<cmd_list.size();++i)
          {
            for(int k = 0;k<3;++k)
            {
              msg_cmd.data = cmd_list[i];
              if(cmd_list[i] == "" || (cmd_list[i] != "" && cmd_list[i].at(0) != 'c') )
              {
                ROS_INFO("skip previous command: %s",cmd_list[i].c_str());
                continue;
              }
              ROS_INFO("send previous command: %s",cmd_list[i].c_str());
              command_pub_.publish(msg_cmd);
            }
            
          }
          

        }
        loop_wait++;

        return;
      }


      loop_wait = 0;

      // std::this_thread::sleep_for(std::chrono::seconds(wt));
      ROS_INFO("wait over,publish back command,send next goal.....");
      //ROS_INFO("id_next_goal: %d",id_next_goal);
      //3.发布cmd_back
      std::string pcmd = cmd_back_list[id_next_goal-1];

      std::vector<std::string> cmd_list = split(pcmd,'#');
      std_msgs::String msg_cmd;
      for(int i = 0;i<cmd_list.size();++i)
      {
        for(int k = 0;k<3;++k)
        {
          // msg_cmd.data = cmd_list[i];
          // command_pub_.publish(msg_cmd);
          // ROS_INFO("send back command: %s",cmd_list[i].c_str());
          msg_cmd.data = cmd_list[i];
          if(cmd_list[i] == "" || (cmd_list[i] != "" && cmd_list[i].at(0) != 'c') )
          {
            ROS_INFO("skip back command: %s",cmd_list[i].c_str());
            continue;
          }
          ROS_INFO("send back command: %s",cmd_list[i].c_str());
          command_pub_.publish(msg_cmd);

        }
        
      }


      ////////////////////////////////////////////////////
      send = true;
      boost::unique_lock<boost::recursive_mutex> lock_goal(goal_mutex);
      is_current_goal_reach = false;
      lock_goal.unlock();
    }

    if(!send)
    {
      return;
    }

    int goal_count = goal_points_list.size();
    ROS_ERROR("goal_count=%d", goal_count);

    //判断是不是最后一个点
    if(id_next_goal >= goal_count - 1)
    {
      is_last_goal = true;
    }else
    {
      is_last_goal = false;
    }
    
    if(id_next_goal >= goal_count)
    {
      if(UVR_current_state_ == CHARGING || UVR_current_state_ == OFFCHARGING)
      {
        return;
      }
      
      boost::unique_lock<boost::recursive_mutex> lock_goals_list(current_goals_over_mutex);
      is_current_goals_list_over = true;
      //当前task完成
      goal_points_list.clear();

      current_execute_task.states = "Done";
      task_feedback_pub_.publish(current_execute_task);

      publishState(IDLE);

      lock_goals_list.unlock();

      return;
    }

    double yaw = tf2::getYaw(goal_points_list[id_next_goal].pose.orientation);
    ROS_INFO("Timer call send new goal: the %d ( %.3f, %.3f, yaw %.3f) .......",id_next_goal, goal_points_list[id_next_goal].pose.position.x,goal_points_list[id_next_goal].pose.position.y,yaw);

    move_base_msgs::MoveBaseActionGoal action_goal;
    action_goal.header.frame_id = "map";
    action_goal.header.stamp = ros::Time::now();
    action_goal.goal.target_pose = goal_points_list[id_next_goal];

    action_goal_pub_.publish(action_goal);

    is_new_one_goal_first = true;

    current_execute_task.states = "Doing";
    task_feedback_pub_.publish(current_execute_task);

    id_next_goal++;
    
    return;
  }
  std::vector<std::string> MoveBase::split(const std::string& str, char delim)
  {
    std::stringstream ss(str);
    std::string s;
    std::vector<std::string> vec;
    while (std::getline(ss, s, delim))
      vec.push_back(s);

    if (!str.empty() && str.back() == delim)
      vec.push_back(std::string());

    return vec;
  }
  void MoveBase::saveGoalFromMapCallback(const geometry_msgs::PoseStamped::ConstPtr& goal)
  {
    ROS_INFO("get the click goal pose from map: (%f, %f, %f), (%f, %f, %f,%f)",
    goal->pose.position.x,goal->pose.position.y,goal->pose.position.z,
    goal->pose.orientation.x,goal->pose.orientation.y,goal->pose.orientation.z,goal->pose.orientation.w);

    {
      std::string workpath = getWorkPath();
      std::string timestr = get_current_datetime();
      std::string filename = workpath + "/goal/current_goal_setting"+timestr+".csv";
      
      std::ofstream goals_file(filename,std::ios::out|std::ios::app);
      goals_file <<std::setiosflags(std::ios::fixed)<<std::setprecision(3);
      goals_file <<goal->pose.position.x<<","<<goal->pose.position.y<<","<<goal->pose.position.z<<",";
      goals_file <<goal->pose.orientation.x<<","<<goal->pose.orientation.y<<","<<goal->pose.orientation.z<<","<< goal->pose.orientation.w<<",";
      goals_file <<5<<","<<"cx#cx"<<","<<"c1#c7"<<std::endl; 
      goals_file.close();
    }


    return;
  }
  void MoveBase::goalCB(const geometry_msgs::PoseStamped::ConstPtr& goal)
  {

    if(!start_task)
    {
      ROS_INFO("start_task: false, not running !!!!!");
      return;
    }

    ROS_INFO("get the goal pose, first ......1111111111111111111111, (%f, %f, %f), (%f, %f, %f,%f)",
    goal->pose.position.x,goal->pose.position.y,goal->pose.position.z,
    goal->pose.orientation.x,goal->pose.orientation.y,goal->pose.orientation.z,goal->pose.orientation.w);
    
    move_base_msgs::MoveBaseActionGoal action_goal;
    
    action_goal.header.stamp = ros::Time::now();
    action_goal.goal.target_pose = *goal;

    action_goal_pub_.publish(action_goal);
  }

  void MoveBase::clearCostmapWindows(double size_x, double size_y)
  {
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

  bool MoveBase::clearCostmapsService(std_srvs::Empty::Request &req, std_srvs::Empty::Response &resp){
    //clear the costmaps
    boost::unique_lock<costmap_2d::Costmap2D::mutex_t> lock_controller(*(controller_costmap_ros_->getCostmap()->getMutex()));
    controller_costmap_ros_->resetLayers();

    boost::unique_lock<costmap_2d::Costmap2D::mutex_t> lock_planner(*(planner_costmap_ros_->getCostmap()->getMutex()));
    planner_costmap_ros_->resetLayers();
    return true;
  }


  bool MoveBase::planService(nav_msgs::GetPlan::Request &req, nav_msgs::GetPlan::Response &resp){
    if(as_->isActive())
    {
      ROS_ERROR("move_base must be in an inactive state to make a plan for an external user");
      return false;
    }
    //make sure we have a costmap for our planner
    if(planner_costmap_ros_ == NULL)
    {
      ROS_ERROR("move_base cannot make a plan for you because it doesn't have a costmap");
      return false;
    }

    geometry_msgs::PoseStamped start;
    //如果没有指定初始pose
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

    if (make_plan_clear_costmap_) 
    {
      //update the copy of the costmap the planner uses
      clearCostmapWindows(2 * clearing_radius_, 2 * clearing_radius_);
    }

    //first try to make a plan to the exact desired goal
    std::vector<geometry_msgs::PoseStamped> global_plan;
    if(!planner_->makePlan(start, req.goal, global_plan) || global_plan.empty())
    {
      ROS_DEBUG_NAMED("move_base","Failed to find a plan to exact goal of (%.2f, %.2f), searching for a feasible goal within tolerance", 
          req.goal.pose.position.x, req.goal.pose.position.y);

      //search outwards for a feasible goal within the specified tolerance
      geometry_msgs::PoseStamped p;
      p = req.goal;
      bool found_legal = false;
      float resolution = planner_costmap_ros_->getCostmap()->getResolution();
      float search_increment = resolution*3.0;
      if(req.tolerance > 0.0 && req.tolerance < search_increment) 
      {
        search_increment = req.tolerance;
      }

      for(float max_offset = search_increment; max_offset <= req.tolerance && !found_legal; max_offset += search_increment) 
      {
        for(float y_offset = 0; y_offset <= max_offset && !found_legal; y_offset += search_increment) 
        {
          for(float x_offset = 0; x_offset <= max_offset && !found_legal; x_offset += search_increment) 
          {

            //don't search again inside the current outer layer
            if(x_offset < max_offset-1e-9 && y_offset < max_offset-1e-9) 
              continue;

            //search to both sides of the desired goal
            for(float y_mult = -1.0; y_mult <= 1.0 + 1e-9 && !found_legal; y_mult += 2.0) 
            {

              //if one of the offsets is 0, -1*0 is still 0 (so get rid of one of the two)
              if(y_offset < 1e-9 && y_mult < -1.0 + 1e-9) 
                continue;

              for(float x_mult = -1.0; x_mult <= 1.0 + 1e-9 && !found_legal; x_mult += 2.0) 
              {
                
                if(x_offset < 1e-9 && x_mult < -1.0 + 1e-9) 
                  continue;

                p.pose.position.y = req.goal.pose.position.y + y_offset * y_mult;
                p.pose.position.x = req.goal.pose.position.x + x_offset * x_mult;

                if(planner_->makePlan(start, p, global_plan))
                {
                  if(!global_plan.empty()){

                    if (make_plan_add_unreachable_goal_) 
                    {
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
    for(unsigned int i = 0; i < global_plan.size(); ++i)
    {
      resp.plan.poses[i] = global_plan[i];
    }

    return true;
  }

  MoveBase::~MoveBase()
  {
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

  bool MoveBase::makePlan(const geometry_msgs::PoseStamped& goal, std::vector<geometry_msgs::PoseStamped>& plan)
  {
    boost::unique_lock<costmap_2d::Costmap2D::mutex_t> lock(*(planner_costmap_ros_->getCostmap()->getMutex()));

    //make sure to set the plan to be empty initially
    plan.clear();

    //since this gets called on handle activate
    if(planner_costmap_ros_ == NULL) 
    {
      ROS_ERROR("Planner costmap ROS is NULL, unable to create global plan");
      return false;
    }

    //get the starting pose of the robot
    geometry_msgs::PoseStamped global_pose;
    if(!getRobotPose(global_pose, planner_costmap_ros_)) 
    {
      ROS_WARN("Unable to get starting pose of robot, unable to create global plan");
      return false;
    }

    const geometry_msgs::PoseStamped& start = global_pose;

    //if the planner fails or returns a zero length plan, planning failed
    //call NavfnROS::makePlan
    if(!planner_->makePlan(start, goal, plan) || plan.empty())
    {
      ROS_DEBUG_NAMED("move_base","Failed to find a  plan to point (%.2f, %.2f)", goal.pose.position.x, goal.pose.position.y);
      return false;
    }

    // std::string filename = "/home/huafy/odom_test_data/global_plan_.csv";
    // std::ofstream plan_test(filename,std::ios::out|std::ios::app);
    // for(int i = 0;i<plan.size();++i)
    // { 
    //   geometry_msgs::PoseStamped ps = plan[i];
    //   plan_test <<std::setiosflags(std::ios::fixed)<<std::setprecision(1);
    //   plan_test <<ps.pose.position.x<<","<<ps.pose.position.y<<","<<ps.pose.position.z<<std::endl; 
    // }
    // plan_test.close();
    double yaw_start = tf2::getYaw(start.pose.orientation);
    double yaw_goal = tf2::getYaw(goal.pose.orientation);
    ROS_INFO("move_base,successed to find a  plan from (%.3f, %.3f, yaw %.3f) to point (%.3f, %.3f, yaw %.3f)", start.pose.position.x, start.pose.position.y,yaw_start,goal.pose.position.x, goal.pose.position.y,yaw_goal);

    return true;
  }

  void MoveBase::publishZeroVelocity()
  {
    geometry_msgs::Twist cmd_vel;
    cmd_vel.linear.x = 0.0;
    cmd_vel.linear.y = 0.0;
    cmd_vel.angular.z = 0.0;
    vel_pub_.publish(cmd_vel);
  }

  void MoveBase::changeAngular2Wheelspeed(geometry_msgs::Twist &cmd_vel,geometry_msgs::Twist &cmd_vel_wheel)
  {
    //change the angular speed to left and right wheel speed

  }

  bool MoveBase::isQuaternionValid(const geometry_msgs::Quaternion& q)
  {
    //first we need to check if the quaternion has nan's or infs
    if(!std::isfinite(q.x) || !std::isfinite(q.y) || !std::isfinite(q.z) || !std::isfinite(q.w)){
      ROS_ERROR("Quaternion has nans or infs... discarding as a navigation goal");
      return false;
    }

    tf2::Quaternion tf_q(q.x, q.y, q.z, q.w);

    //next, we need to check if the length of the quaternion is close to zero
    if(tf_q.length2() < 1e-6){
      ROS_ERROR("Quaternion has length close to zero... discarding as navigation goal");
      return false;
    }

    //next, we'll normalize the quaternion and check that it transforms the vertical vector correctly
    tf_q.normalize();

    tf2::Vector3 up(0, 0, 1);

    double dot = up.dot(up.rotate(tf_q.getAxis(), tf_q.getAngle()));

    if(fabs(dot - 1) > 1e-3){
      ROS_ERROR("Quaternion is invalid... for navigation the z-axis of the quaternion must be close to vertical.");
      return false;
    }

    return true;
  }

  geometry_msgs::PoseStamped MoveBase::goalToGlobalFrame(const geometry_msgs::PoseStamped& goal_pose_msg)
  {
    //获取全局坐标系名字
    std::string global_frame = planner_costmap_ros_->getGlobalFrameID();

    ROS_INFO("global_frame: %s ", global_frame.c_str());

    geometry_msgs::PoseStamped goal_pose, global_pose;

    goal_pose = goal_pose_msg;

    goal_pose.header.frame_id = "map";

    ROS_INFO("goal_frame: %s ", goal_pose.header.frame_id.c_str());
    //just get the latest available transform... for accuracy they should send
    //goals in the frame of the planner
    goal_pose.header.stamp = ros::Time();

    try
    {
      tf_.transform(goal_pose_msg, global_pose, global_frame);//获取全局坐标值
    }
    catch(tf2::TransformException& ex)
    {
      ROS_WARN("Failed to transform the goal pose from %s into the %s frame: %s",
          goal_pose.header.frame_id.c_str(), global_frame.c_str(), ex.what());
      return goal_pose_msg;
    }

    return global_pose;
  }

  void MoveBase::wakePlanner(const ros::TimerEvent& event)
  {
    // we have slept long enough for rate
    planner_cond_.notify_one();
  }

  void MoveBase::planThread()
  {
    ROS_DEBUG_NAMED("move_base_plan_thread","Starting planner thread...");
    ros::NodeHandle n;
    ros::Timer timer;
    bool wait_for_wake = false;
    boost::unique_lock<boost::recursive_mutex> lock(planner_mutex_);
    
    while(n.ok())
    {
      //check if we should run the planner (the mutex is locked)
      while(wait_for_wake || !runPlanner_)
      {
        //if we should not be running the planner then suspend this thread
        ROS_DEBUG_NAMED("move_base_plan_thread","Planner thread is suspending");
        planner_cond_.wait(lock);
        wait_for_wake = false;
      }
      ros::Time start_time = ros::Time::now();

      //time to plan! get a copy of the goal and unlock the mutex
      //获取目标位置,开始进行路线规划
      geometry_msgs::PoseStamped temp_goal = planner_goal_;

      temp_goal.header.frame_id = "map";//当创建plan 失败后,重新发布目标,坐标系为啥丢了

//      ROS_WARN("move_base_plan_thread Planning...%s",temp_goal.header.frame_id.c_str());
      
      lock.unlock();
      ROS_DEBUG_NAMED("move_base_plan_thread","Planning...");

      //run planner
      planner_plan_->clear();
      bool gotPlan = n.ok() && makePlan(temp_goal, *planner_plan_);//调用规划器

      if(gotPlan)
      {
        ROS_INFO("move_base_plan_thread, Got Plan with %zu points!", planner_plan_->size());
        //pointer swap the plans under mutex (the controller will pull from latest_plan_)
        std::vector<geometry_msgs::PoseStamped>* temp_plan = planner_plan_;
        //打印最新的10个点
        ROS_INFO("global plan, first 20 !");
        for(int i = 0;i< 10 && i<planner_plan_->size();++i)
        {
          ROS_INFO("%d ponit: %.3f, %.3f",i+1,(*planner_plan_)[i].pose.position.x,(*planner_plan_)[i].pose.position.y);
        }
        //记录上一次的规划路线,并更新最新的路线
        lock.lock();
        planner_plan_ = latest_plan_;
        latest_plan_ = temp_plan;
        last_valid_plan_ = ros::Time::now();
        planning_retries_ = 0;
        new_global_plan_ = true;

        ROS_INFO("move_base_plan_thread, generated a plan from the base_global_planner");

        //make sure we only start the controller if we still haven't reached the goal
        if(runPlanner_)
          state_ = CONTROLLING;
        if(planner_frequency_ <= 0)
          runPlanner_ = false;
        lock.unlock();
      }
      else if(state_==PLANNING)//if we didn't get a plan and we are in the planning state (the robot isn't moving)
      {
        ROS_DEBUG_NAMED("move_base_plan_thread","No Plan...");
        ros::Time attempt_end = last_valid_plan_ + ros::Duration(planner_patience_);

        //check if we've tried to make a plan for over our time limit or our maximum number of retries
        //issue #496: we stop planning when one of the conditions is true, but if max_planning_retries_
        //is negative (the default), it is just ignored and we have the same behavior as ever
        lock.lock();
        planning_retries_++;

        if(runPlanner_ &&(ros::Time::now() > attempt_end || planning_retries_ > uint32_t(max_planning_retries_)))
        {
          //we'll move into our obstacle clearing mode
          state_ = CLEARING;
          runPlanner_ = false;  // proper solution for issue #523
          publishZeroVelocity();
          recovery_trigger_ = PLANNING_R;
        }

        lock.unlock();
      }

      //take the mutex for the next iteration
      lock.lock();

      //setup sleep interface if needed
      if(planner_frequency_ > 0)
      {
        ros::Duration sleep_time = (start_time + ros::Duration(1.0/planner_frequency_)) - ros::Time::now();
        if (sleep_time > ros::Duration(0.0))
        {
          wait_for_wake = true;
          timer = n.createTimer(sleep_time, &MoveBase::wakePlanner, this);
        }
      }
    }
  }

  void MoveBase::publishTrajectoryRobotPoseList() 
  {
    if (trajectory_robot_pose_list_publisher_.getNumSubscribers() > 0) 
    {

      nav_msgs::Path robot_path;
      robot_path.header.stamp = ros::Time::now();
      robot_path.header.frame_id = "map";

      for(int i = 0;i<robot_pose_list.size();++i)
      {
        geometry_msgs::PoseStamped pose_stamped;
        pose_stamped.pose.position.x = robot_pose_list[i].pose.position.x;
        pose_stamped.pose.position.y = robot_pose_list[i].pose.position.y;
        pose_stamped.pose.position.z = robot_pose_list[i].pose.position.z;

        pose_stamped.pose.orientation = robot_pose_list[i].pose.orientation;

        robot_path.poses.push_back(pose_stamped);
      }

      trajectory_robot_pose_list_publisher_.publish(robot_path);
    }
  }
  void MoveBase::publishRobotPoseAndTrajectory()
  {
      geometry_msgs::PoseStamped robot_pos;
      getRobotPose(robot_pos, planner_costmap_ros_);
      robot_pose_publisher_.publish(robot_pos);

      if(robot_pose_list.size()>5000)
      {
        robot_pose_list.erase(robot_pose_list.begin(),robot_pose_list.begin()+2000);
      }else
      {
        robot_pose_list.push_back(robot_pos);
      }

      publishTrajectoryRobotPoseList();
  }


  void MoveBase::executeCb(const move_base_msgs::MoveBaseGoalConstPtr& move_base_goal)
  {

    if(!isQuaternionValid(move_base_goal->target_pose.pose.orientation))
    {
      as_->setAborted(move_base_msgs::MoveBaseResult(), "Aborting on goal because it was sent with an invalid quaternion");
      return;
    }

    //获取全局目标坐标值
    geometry_msgs::PoseStamped goal = goalToGlobalFrame(move_base_goal->target_pose);
    goal.header.frame_id = "map";
    ROS_INFO("call executeCb: move_base_goal,get the global goal.......222222222222222222222....... %f,%f,%f",goal.pose.position.x,goal.pose.position.y,goal.pose.position.z);
    publishZeroVelocity();
    //we have a goal so start the planner
    boost::unique_lock<boost::recursive_mutex> lock(planner_mutex_);
    planner_goal_ = goal;
    runPlanner_ = true;
    planner_cond_.notify_one();
    lock.unlock();

    current_goal_pub_.publish(goal);
    std::vector<geometry_msgs::PoseStamped> global_plan;

    ros::Rate r(controller_frequency_);
    if(shutdown_costmaps_)
    {
      ROS_INFO("move_base,Starting up costmaps that were shut down previously");
      planner_costmap_ros_->start();
      controller_costmap_ros_->start();
    }

    //we want to make sure that we reset the last time we had a valid plan and control
    last_valid_control_ = ros::Time::now();
    last_valid_plan_ = ros::Time::now();
    last_oscillation_reset_ = ros::Time::now();
    planning_retries_ = 0;

    ros::NodeHandle n;
    while(n.ok())
    {
      
      publishRobotPoseAndTrajectory();


      stateCheck();

      if(c_freq_change_)
      {
        ROS_INFO("Setting controller frequency to %.2f", controller_frequency_);
        r = ros::Rate(controller_frequency_);
        c_freq_change_ = false;
      }

      if(as_->isPreemptRequested())
      {
        if(as_->isNewGoalAvailable()) //修改新的目标点的时候,才会运行
        {

          ROS_INFO("In while loop, get a new goal,new goal available .................66666666666666");

          //if we're active and a new goal is available, we'll accept it, but we won't shut anything down
          move_base_msgs::MoveBaseGoal new_goal = *as_->acceptNewGoal();

          if(!isQuaternionValid(new_goal.target_pose.pose.orientation))
          {
            as_->setAborted(move_base_msgs::MoveBaseResult(), "Aborting on goal because it was sent with an invalid quaternion");
            return;
          }

          goal = goalToGlobalFrame(new_goal.target_pose);

          //we'll make sure that we reset our state for the next execution cycle
          recovery_index_ = 0;
          state_ = PLANNING;

          //we have a new goal so make sure the planner is awake
          //设置新的目标坐标,并唤醒planner规划期
          lock.lock();
          planner_goal_ = goal;
          runPlanner_ = true;
          planner_cond_.notify_one();
          lock.unlock();

          //publish the goal point to the visualizer
          ROS_INFO("move_base has received a goal of x: %.2f, y: %.2f", goal.pose.position.x, goal.pose.position.y);
          current_goal_pub_.publish(goal);

          //make sure to reset our timeouts and counters
          last_valid_control_ = ros::Time::now();
          last_valid_plan_ = ros::Time::now();
          last_oscillation_reset_ = ros::Time::now();
          planning_retries_ = 0;
        }
        else 
        {
          //if we've been preempted explicitly we need to shut things down
          resetState();

          //notify the ActionServer that we've successfully preempted
          ROS_DEBUG_NAMED("move_base","Move base preempting the current goal");
          as_->setPreempted();

          //we'll actually return from execute after preempting
          return;
        }
      }

      //we also want to check if we've changed global frames because we need to transform our goal pose
      //修改全局坐标的时候,才会起作用
      if(goal.header.frame_id != planner_costmap_ros_->getGlobalFrameID())
      {

        ROS_INFO("In while loop, new goal available, but we've changed global frames .................");
        ROS_INFO("goal.header.frame_id %s, planner_costmap_ros_->getGlobalFrameID() %s ",goal.header.frame_id.c_str(),planner_costmap_ros_->getGlobalFrameID().c_str());

        goal = goalToGlobalFrame(goal);
         
        //we want to go back to the planning state for the next execution cycle
        recovery_index_ = 0;
        state_ = PLANNING;

        //we have a new goal so make sure the planner is awake
        lock.lock();
        planner_goal_ = goal;
        runPlanner_ = true;
        planner_cond_.notify_one();
        lock.unlock();

        //publish the goal point to the visualizer
        ROS_INFO("The global frame for move_base has changed, new frame: %s, new goal position x: %.2f, y: %.2f", goal.header.frame_id.c_str(), goal.pose.position.x,goal.pose.position.y);
        current_goal_pub_.publish(goal);

        //make sure to reset our timeouts and counters
        last_valid_control_ = ros::Time::now();
        last_valid_plan_ = ros::Time::now();
        last_oscillation_reset_ = ros::Time::now();
        planning_retries_ = 0;
      }

      //ROS_INFO("move_base: frame: %s, new goal position x: %.2f, y: %.2f", goal.header.frame_id.c_str(), goal.pose.position.x, goal.pose.position.y);
      //for timing that gives real time even in simulation
      ros::WallTime start = ros::WallTime::now();

      //the real work on pursuing a goal is done here

      //获取目标点,然后得到全局plan
      ROS_INFO("In while loop,run executeCycle ..........................77777777777777777777777, global_plan : %d",(int)global_plan.size());
      bool done = executeCycle(goal, global_plan);

      //if we're done, then we'll return from execute
      if(done)
        return;

      //check if execution of the goal has completed in some way

      ros::WallDuration t_diff = ros::WallTime::now() - start;
      ROS_DEBUG_NAMED("move_base","Full control cycle time: %.9f\n", t_diff.toSec());

      r.sleep();
      //make sure to sleep for the remainder of our cycle time
      if(r.cycleTime() > ros::Duration(1 / controller_frequency_) && state_ == CONTROLLING)
        ROS_WARN("Control loop missed its desired rate of %.4fHz... the loop actually took %.4f seconds", controller_frequency_, r.cycleTime().toSec());
    }

    //wake up the planner thread so that it can exit cleanly
    lock.lock();
    runPlanner_ = true;
    planner_cond_.notify_one();
    lock.unlock();

    //if the node is killed then we'll abort and return
    as_->setAborted(move_base_msgs::MoveBaseResult(), "Aborting on the goal because the node has been killed");
    return;
  }

  double MoveBase::distance(const geometry_msgs::PoseStamped& p1, const geometry_msgs::PoseStamped& p2)
  {
    return hypot(p1.pose.position.x - p2.pose.position.x, p1.pose.position.y - p2.pose.position.y);
  }

  bool MoveBase::executeCycle(geometry_msgs::PoseStamped& goal, std::vector<geometry_msgs::PoseStamped>& global_plan)
  {

    //ROS_INFO("In executeCycle, first the global goal.............. %f,%f,%f",goal.pose.position.x,goal.pose.position.y,goal.pose.position.z);
    //ROS_INFO("In executeCycle, global_plan size.............. %d",global_plan.size());
    boost::recursive_mutex::scoped_lock ecl(configuration_mutex_);
    //we need to be able to publish velocity commands
    geometry_msgs::Twist cmd_vel;
    //geometry_msgs::Twist cmd_vel_wheel;//转换角度为左右轮子的线速度

    //update feedback to correspond to our curent position
    geometry_msgs::PoseStamped global_pose;
    getRobotPose(global_pose, planner_costmap_ros_);


    const geometry_msgs::PoseStamped& current_position = global_pose;

    double yaw_start = tf2::getYaw(current_position.pose.orientation);
    double yaw_goal = tf2::getYaw(goal.pose.orientation);

    ROS_INFO("In executeCycle,current robot pose (%.3f,%.3f,%.3f yaw %.3f), goal (%.3f,%.3f,yaw %.3f)",
              current_position.pose.position.x,current_position.pose.position.y,current_position.pose.position.z,yaw_start,
              goal.pose.position.x,goal.pose.position.y,yaw_goal);
    
    //push the feedback out
    move_base_msgs::MoveBaseFeedback feedback;
    feedback.base_position = current_position;
    as_->publishFeedback(feedback);

    //check to see if we've moved far enough to reset our oscillation timeout
    if(distance(current_position, oscillation_pose_) >= oscillation_distance_)
    {
      ROS_ERROR("we've not moved far enough to reset our oscillation timeout ..............oscillation_pose: %f,%f,%f",oscillation_pose_.pose.position.x,oscillation_pose_.pose.position.y,oscillation_pose_.pose.position.z);
      last_oscillation_reset_ = ros::Time::now();
      oscillation_pose_ = current_position;

      //if our last recovery was caused by oscillation, we want to reset the recovery index 
      if(recovery_trigger_ == OSCILLATION_R)
        recovery_index_ = 0;
    }

    //check that the observation buffers for the costmap are current, we don't want to drive blind
    if(!controller_costmap_ros_->isCurrent())
    {
      ROS_WARN("[%s]:Sensor data is out of date, we're not going to allow commanding of the base for safety",ros::this_node::getName().c_str());
      publishZeroVelocity();
      return false;
    }

    //if we have a new plan then grab it and give it to the controller
    if(new_global_plan_)
    {
      ROS_INFO("we have a new plan then grab it and give it to the controller......");
      //make sure to set the new plan flag to false
      new_global_plan_ = false;

      ROS_DEBUG_NAMED("move_base","Got a new plan...swap pointers");

      //do a pointer swap under mutex

      //ROS_INFO("the latest plan: %d ......", latest_plan_->size());

      std::vector<geometry_msgs::PoseStamped>* temp_plan = controller_plan_;

      boost::unique_lock<boost::recursive_mutex> lock(planner_mutex_);
      controller_plan_ = latest_plan_;
      latest_plan_ = temp_plan;
      lock.unlock();
      ROS_DEBUG_NAMED("move_base","pointers swapped!");

      //ROS_INFO("the controller plan: %d ......", controller_plan_->size());

      if(!tc_->setPlan(*controller_plan_))
      {
        //ABORT and SHUTDOWN COSTMAPS
        ROS_ERROR("Failed to pass global plan to the controller, aborting.");
        resetState();

        //disable the planner thread
        lock.lock();
        runPlanner_ = false;
        lock.unlock();

        as_->setAborted(move_base_msgs::MoveBaseResult(), "Failed to pass global plan to the controller.");
        return true;
      }

      //make sure to reset recovery_index_ since we were able to find a valid plan
      if(recovery_trigger_ == PLANNING_R)
        recovery_index_ = 0;
    }

    //the move_base state machine, handles the control logic for navigation
    switch(state_)
    {
      //if we are in a planning state, then we'll attempt to make a plan
      case PLANNING:
        {
          boost::recursive_mutex::scoped_lock lock(planner_mutex_);
          runPlanner_ = true;
          planner_cond_.notify_one();
        }
        ROS_INFO("current state : PLANNING......");
        ROS_INFO("move_base,Waiting for plan, in the planning state.");
        
        break;

      //if we're controlling, we'll attempt to find valid velocity commands
      case CONTROLLING:
        ROS_INFO("current state : CONTROLLING......");

        //check to see if we've reached our goal
        //call DWAPlannerROS::isGoalReached()
        if(tc_->isGoalReached(UVR_current_state_ == CHARGING ))//到达目标点
        {
          ROS_INFO("Goal reached ..........................................");
          ROS_INFO(" ");
          ROS_INFO(" ");
          ROS_INFO(" ");
          ROS_INFO(" ");
          ROS_INFO(" ");

          ROS_ERROR("UVR_state: %d, is_last_goal: %d", UVR_current_state_, is_last_goal?1:0);

          //ROS_DEBUG_NAMED("move_base","Goal reached!");
          resetState();

          //disable the planner thread
          boost::unique_lock<boost::recursive_mutex> lock(planner_mutex_);
          runPlanner_ = false;
          lock.unlock();

          as_->setSucceeded(move_base_msgs::MoveBaseResult(), "Goal reached.");

          boost::unique_lock<boost::recursive_mutex> lock_goal(goal_mutex);
          is_current_goal_reach = true;
          lock_goal.unlock();
          
          is_new_one_goal_first = false;
          ///////////////////////////////////////////////////////////////////////////////
          //如果是充电状态下，则发动到达目标的位置
          if(UVR_current_state_ == CHARGING && is_last_goal)
          {
            std_msgs::String msg_cmd;
            msg_cmd.data = "C1J";
            command_pub_.publish(msg_cmd);
            command_pub_.publish(msg_cmd);
            command_pub_.publish(msg_cmd);
            ROS_INFO("reached the charging station,publish CJ......................");
            ROS_ERROR("reached the charging station,publish CJ......................");
          }
          return true;          
        }

        //check for an oscillation condition
        if(oscillation_timeout_ > 0.0 && last_oscillation_reset_ + ros::Duration(oscillation_timeout_) < ros::Time::now())
        {
          ROS_ERROR("check for an oscillation condition,oscillation_timeout %f",oscillation_timeout_);
          
          publishZeroVelocity();
          state_ = CLEARING;
          recovery_trigger_ = OSCILLATION_R;
        }
        


        {
            boost::unique_lock<costmap_2d::Costmap2D::mutex_t> lock(*(controller_costmap_ros_->getCostmap()->getMutex()));
            
            //调用 DWAPlannerROS的 computeVelocityCommands
            tc_->setNewGoal(is_new_one_goal_first);
            if(tc_->computeVelocityCommands(cmd_vel))
            {
              ROS_INFO( "move_base,  get a valid command from the local planner: (%.3lf, %.3lf, %.3lf)",
                              cmd_vel.linear.x, cmd_vel.linear.y, cmd_vel.angular.z );

              last_valid_control_ = ros::Time::now();
              //make sure that we send the velocity command to the base
              ROS_INFO("move_base,  sent a valid command(cmd_vel)....");
              vel_pub_.publish(cmd_vel);
              
              //对cmd_vel的角速度进行转换,转换角度为左右轮的线速度
              //changeAngular2Wheelspeed(cmd_vel,cmd_vel_wheel);
              //vel_pub_wheel.publish(cmd_vel_wheel);

              if(recovery_trigger_ == CONTROLLING_R)
                recovery_index_ = 0;

              is_new_one_goal_first = false;
            }
            else 
            {
              ROS_DEBUG_NAMED("move_base", "The local planner could not find a valid plan.");
              ros::Time attempt_end = last_valid_control_ + ros::Duration(controller_patience_);
              ros::Time attempt_end_early = last_valid_control_ + ros::Duration(controller_patience_/2);

              //check if we've tried to find a valid control for longer than our time limit
              if(ros::Time::now() > attempt_end)
              {
                //we'll move into our obstacle clearing mode
                publishZeroVelocity();
                state_ = CLEARING;
                recovery_trigger_ = CONTROLLING_R;
              }
              else
              {
                //otherwise, if we can't find a valid control, we'll go back to planning
                last_valid_plan_ = ros::Time::now();
                planning_retries_ = 0;
                state_ = PLANNING;
                publishZeroVelocity();


                //如果当前目标到达不了,即找不到合适的路径,还有新目标,则发布下一下目标.
                if( ros::Time::now() >= attempt_end_early)
                {
                  ROS_ERROR("move_base 111,The local planner could not find a valid plan for current goal,so we send next goal.");
                  ROS_INFO("move_base,The local planner could not find a valid plan for current goal,so we send next goal.");
                  boost::unique_lock<boost::recursive_mutex> lock_goal(goal_mutex);
                  is_current_goal_reach = true;
                  lock_goal.unlock();

                  ///////////////////////////////////////////////
                  is_new_one_goal_first = false;

                  //设置新的目标点后,要设置初始化状态
                  resetState();

                  //disable the planner thread
                  boost::unique_lock<boost::recursive_mutex> lock(planner_mutex_);
                  runPlanner_ = false;
                  lock.unlock();

                  as_->setSucceeded(move_base_msgs::MoveBaseResult(), "Goal reached.");


                  ///////////////////////////////////////////////

                }
                else
                {
                  //enable the planner thread in case it isn't running on a clock
                  boost::unique_lock<boost::recursive_mutex> lock(planner_mutex_);
                  runPlanner_ = true;
                  planner_cond_.notify_one();
                  lock.unlock();
                }
              }
            }
        }

        break;

      //we'll try to clear out space with any user-provided recovery behaviors
      case CLEARING:

        ROS_INFO("current state : CLEARING......");
        //尝试发布下一个目标点
        ///////////////////////////////////////////////////////////////////////////////////////////////////////////////
        // {
        //     ROS_ERROR("move_base 222,The local planner could not find a valid plan for current goal,so we send next goal.");
        //     boost::unique_lock<boost::recursive_mutex> lock_goal(goal_mutex);
        //     is_current_goal_reach = true;
        //     lock_goal.unlock();

        //     ///////////////////////////////////////////////
            
        //     //设置新的目标点后,要设置初始化状态
        //     resetState();

        //     //disable the planner thread
        //     boost::unique_lock<boost::recursive_mutex> lock(planner_mutex_);
        //     runPlanner_ = false;
        //     lock.unlock();

        //     as_->setSucceeded(move_base_msgs::MoveBaseResult(), "Goal reached.");

        //     return true;
        // }
        /////////////////////////////////////////////////////////////////////////////////////////////////////////

        ROS_DEBUG_NAMED("move_base","In clearing/recovery state");
        //we'll invoke whatever recovery behavior we're currently on if they're enabled
        if(recovery_behavior_enabled_ && recovery_index_ < recovery_behaviors_.size())
        {
          ROS_DEBUG_NAMED("move_base_recovery","Executing behavior %u of %zu", recovery_index_, recovery_behaviors_.size());
          recovery_behaviors_[recovery_index_]->runBehavior();

          //we at least want to give the robot some time to stop oscillating after executing the behavior
          last_oscillation_reset_ = ros::Time::now();

          //we'll check if the recovery behavior actually worked
          ROS_DEBUG_NAMED("move_base_recovery","Going back to planning state");
          last_valid_plan_ = ros::Time::now();
          planning_retries_ = 0;
          state_ = PLANNING;

          //update the index of the next recovery behavior that we'll try
          recovery_index_++;
        }
        else
        {
          ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
          //各种补救行为失败后,放弃当前目标
          {
            ROS_ERROR("move_base 222,The local planner could not find a valid plan for current goal,so we send next goal.");
            boost::unique_lock<boost::recursive_mutex> lock_goal(goal_mutex);
            is_current_goal_reach = true;
            lock_goal.unlock();

            ///////////////////////////////////////////////
            
            //设置新的目标点后,要设置初始化状态
            resetState();

            //disable the planner thread
            boost::unique_lock<boost::recursive_mutex> lock(planner_mutex_);
            runPlanner_ = false;
            lock.unlock();

            as_->setSucceeded(move_base_msgs::MoveBaseResult(), "Goal reached.");

            return true;
          }

          ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
          ROS_DEBUG_NAMED("move_base_recovery","All recovery behaviors have failed, locking the planner and disabling it.");
          //disable the planner thread
          boost::unique_lock<boost::recursive_mutex> lock(planner_mutex_);
          runPlanner_ = false;
          lock.unlock();

          ROS_DEBUG_NAMED("move_base_recovery","Something should abort after this.");

          if(recovery_trigger_ == CONTROLLING_R)
          {
            ROS_ERROR("Aborting because a valid control could not be found. Even after executing all recovery behaviors");
            as_->setAborted(move_base_msgs::MoveBaseResult(), "Failed to find a valid control. Even after executing recovery behaviors.");
          }
          else if(recovery_trigger_ == PLANNING_R)
          {
            ROS_ERROR("Aborting because a valid plan could not be found. Even after executing all recovery behaviors");
            as_->setAborted(move_base_msgs::MoveBaseResult(), "Failed to find a valid plan. Even after executing recovery behaviors.");
          }
          else if(recovery_trigger_ == OSCILLATION_R)
          {
            ROS_ERROR("Aborting because the robot appears to be oscillating over and over. Even after executing all recovery behaviors");
            as_->setAborted(move_base_msgs::MoveBaseResult(), "Robot is oscillating. Even after executing recovery behaviors.");
          }
          resetState();
          return true;
        }
        break;


      default:
        ROS_INFO("current state : default......");
        ROS_ERROR("This case should never be reached, something is wrong, aborting");
        resetState();
        //disable the planner thread
        boost::unique_lock<boost::recursive_mutex> lock(planner_mutex_);
        runPlanner_ = false;
        lock.unlock();
        as_->setAborted(move_base_msgs::MoveBaseResult(), "Reached a case that should not be hit in move_base. This is a bug, please report it.");
        return true;
    }

    //we aren't done yet

    ROS_INFO("....................................run once(%s)....................................",get_datetime_str().c_str());
    return false;
  }

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


      // ///不让它原地旋转
      // //next, we'll load a recovery behavior to rotate in place
      // boost::shared_ptr<nav_core::RecoveryBehavior> rotate(recovery_loader_.createInstance("rotate_recovery/RotateRecovery"));
      // if(clearing_rotation_allowed_){
      //   rotate->initialize("rotate_recovery", &tf_, planner_costmap_ros_, controller_costmap_ros_);
      //   recovery_behaviors_.push_back(rotate);
      // }

      //next, we'll load a recovery behavior that will do an aggressive reset of the costmap
      boost::shared_ptr<nav_core::RecoveryBehavior> ags_clear(recovery_loader_.createInstance("clear_costmap_recovery/ClearCostmapRecovery"));
      ags_clear->initialize("aggressive_reset", &tf_, planner_costmap_ros_, controller_costmap_ros_);
      recovery_behaviors_.push_back(ags_clear);

      // ///不让它原地旋转
      // //we'll rotate in-place one more time
      // if(clearing_rotation_allowed_)
      //   recovery_behaviors_.push_back(rotate);
    }
    catch(pluginlib::PluginlibException& ex){
      ROS_FATAL("Failed to load a plugin. This should not happen on default recovery behaviors. Error: %s", ex.what());
    }

    return;
  }

  void MoveBase::resetState()
  {
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
    if(shutdown_costmaps_)
    {
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
    robot_pose.header.stamp = ros::Time();      // latest available
    ros::Time current_time = ros::Time::now();  // save time for checking tf delay later

    // get robot pose on the given costmap frame
    try
    {
      tf_.transform(robot_pose, global_pose, costmap->getGlobalFrameID());
      double robot_yaw = tf2::getYaw(global_pose.pose.orientation);
      ROS_INFO("MoveBase,current robot pose:( x %.3f, y %.3f, z %.3f, yaw %.3f)",global_pose.pose.position.x,global_pose.pose.position.y,global_pose.pose.position.z,robot_yaw);

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
  bool MoveBase::getRobotPoseTest(geometry_msgs::PoseStamped& global_pose)
  {
    // tf2::toMsg(tf2::Transform::getIdentity(), global_pose.pose);
    // geometry_msgs::PoseStamped robot_pose;
    // tf2::toMsg(tf2::Transform::getIdentity(), robot_pose.pose);

    // robot_pose.header.frame_id = "base_link";
    // robot_pose.header.stamp = ros::Time();
    // ros::Time current_time = ros::Time::now();

    // // get robot pose on the given costmap frame
    // try
    // {
    //   tf_.transform(robot_pose, global_pose,"map");
    //   double robot_yaw = tf2::getYaw(global_pose.pose.orientation);
    //   ROS_INFO("In MoveBase Test,current robot pose:( x %.3f, y %.3f, z %.3f, yaw %.3f)",global_pose.pose.position.x,global_pose.pose.position.y,global_pose.pose.position.z,robot_yaw);

    // }
    // catch (tf2::LookupException& ex)
    // {
    //   ROS_ERROR_THROTTLE(1.0, "No Transform available Error looking up robot pose: %s\n", ex.what());
    //   return false;
    // }



    // // geometry_msgs::TransformStamped transformStamped;
    // // try
    // // {
    // //   transformStamped = tf_.lookupTransform("map", "base_link",ros::Time(0), ros::Duration(3.0));

    // //   // geometry_msgs::PointStamped world, velo_link;
    // //   // tfBuffer.transform<geometry_msgs::PointStamped>(world, velo_link, "velo_link", ros::Duration(1.0));
    // //   global_pose.pose.position.x = transformStamped.transform.translation.x;
    // //   global_pose.pose.position.y = transformStamped.transform.translation.y;
    // //   global_pose.pose.position.z = transformStamped.transform.translation.z;
    // //   global_pose.pose.orientation = transformStamped.transform.rotation;

    // //   double robot_yaw = tf2::getYaw(global_pose.pose.orientation);
    // //   ROS_INFO("In MoveBase Test,current robot pose:( x %.3f, y %.3f, z %.3f, yaw %.3f)",global_pose.pose.position.x,global_pose.pose.position.y,global_pose.pose.position.z,robot_yaw);

    // // }
    // // catch (tf2::TransformException &ex) 
    // // {
    // //   ROS_ERROR_THROTTLE(1.0, "No Transform available Error looking up robot pose: %s\n", ex.what());
    // //   return false;
    // // }


    return true;
  }
  void MoveBase::loadGoalPointList(const char* filename, std::vector<geometry_msgs::PoseStamped> &wps,
    std::vector<double> &waittimes,std::vector<std::string> &preCmd,std::vector<std::string> &backCmd)
  {
    std::ifstream ifs(filename);

    if (!ifs)
    {
      return;
    }

    ROS_INFO("File name: %s",filename);

    std::string line;
    std::getline(ifs, line);  // Remove first line

    while (std::getline(ifs, line))
    {
      geometry_msgs::PoseStamped wp;
      double wt = 0;
      std::string pre_c = "";
      std::string back_c = "";
      parseWaypointForVer(line, wp,wt,pre_c,back_c);
      //3位数字,分别为x,y,z坐标
      wps.push_back(wp);
      waittimes.push_back(wt);
      preCmd.push_back(pre_c);
      backCmd.push_back(back_c);
    }

    //给目标点设置方向
    // size_t last = wps.size() - 1;
    // for (size_t i = 0; i < wps.size(); ++i)
    // {
    //   if( i == 0)
    //   {
    //     double yaw = atan2(wps.at(i).pose.position.y,
    //                       wps.at(i).pose.position.x);
    //     //wps.at(i).pose.orientation = tf2::createQuaternionMsgFromYaw(yaw);
    //     tf2::Quaternion q;
    //     q.setRPY(0, 0, yaw);
    //     tf2::convert(q, wps.at(i).pose.orientation);

    //   }
    //   else if (i != last)
    //   {
    //     double yaw = atan2(wps.at(i + 1).pose.position.y - wps.at(i).pose.position.y,
    //                       wps.at(i + 1).pose.position.x - wps.at(i).pose.position.x);
    //     //wps.at(i).pose.orientation = tf2::createQuaternionMsgFromYaw(yaw);
    //     tf2::Quaternion q;
    //     q.setRPY(0, 0, yaw);
    //     tf2::convert(q, wps.at(i).pose.orientation);
    //   }
    //   else
    //   {
    //     wps.at(i).pose.orientation = wps.at(i - 1).pose.orientation;
    //   }
    // }

    // std::cout<<"load points: "<<wps->size()<<std::endl;

  }

  void MoveBase::parseWaypointForVer(const std::string& line,geometry_msgs::PoseStamped &wp,double &wt,std::string &pre_cmd,std::string &back_cmd)
  {
    std::vector<std::string> columns;
    parseColumns(line, &columns);

    // if(columns.size() == 3)
    // {
    //   wp.pose.position.x = std::stod(columns[0]);
    //   wp.pose.position.y = std::stod(columns[1]);
    //   wp.pose.position.z = std::stod(columns[2]);
    // }else if(columns.size() == 7)
    // {
    //   wp.pose.position.x = std::stod(columns[0]);
    //   wp.pose.position.y = std::stod(columns[1]);
    //   wp.pose.position.z = std::stod(columns[2]);

    //   wp.pose.orientation.x = std::stod(columns[3]);
    //   wp.pose.orientation.y = std::stod(columns[4]);                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                           
    //   wp.pose.orientation.z = std::stod(columns[5]);
    //   wp.pose.orientation.w = std::stod(columns[6]);
    // }
    if(columns.size() == 10)
    {
      wp.pose.position.x = std::stod(columns[0]);
      wp.pose.position.y = std::stod(columns[1]);
      wp.pose.position.z = std::stod(columns[2]);

      wp.pose.orientation.x = std::stod(columns[3]);
      wp.pose.orientation.y = std::stod(columns[4]);                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                           
      wp.pose.orientation.z = std::stod(columns[5]);
      wp.pose.orientation.w = std::stod(columns[6]);

      wt = std::stod(columns[7]);

      pre_cmd  = columns[8];
      back_cmd = columns[9];
    }
    
    std::cout<<"("<<wp.pose.position.x<<","<<wp.pose.position.y<<","<<wp.pose.position.z<<"),("<<wp.pose.orientation.x<<","<<wp.pose.orientation.y<<","<<wp.pose.orientation.z<<","<<wp.pose.orientation.w<<"),";
    std::cout<<wt <<","<< pre_cmd <<","<< back_cmd <<std::endl;
  }

  void MoveBase::parseColumns(const std::string& line, std::vector<std::string>* columns)
  {
    std::istringstream ss(line);
    std::string column;
    while (std::getline(ss, column, ','))
    {
      while (1)
      {
        auto res = std::find(column.begin(), column.end(), ' ');
        if (res == column.end())
        {
          break;
        }
        column.erase(res);
      }
      if (!column.empty())
      {
        columns->push_back(column);
      }
    }
  }

  std::string MoveBase::getWorkPath()
  {
    char pPath[1024] = {0};
    getcwd(pPath, 1024);
    std::string workpaths(pPath);
    return workpaths;
  }

  std::string MoveBase::get_current_datetime()
  {
    char strt[128];
    time_t current_t_ = time(NULL);
    strftime(strt, sizeof(strt), "%F_%H", localtime(&current_t_));
    return std::string(strt);
  }

  std::string MoveBase::get_datetime_str()
  {
    auto t = std::chrono::system_clock::to_time_t(std::chrono::system_clock::now());
 
    std::stringstream ss;
    ss << std::put_time(std::localtime(&t), "%Y-%m-%d %H:%M:%S");
  
    std::string str_time = ss.str();
    return str_time;
  }

  void MoveBase::getCurrentStateCallback(const std_msgs::Int32::ConstPtr& msg)
  {
    UVR_previous_state_ = UVR_current_state_;//记录前一刻的robot状态

    int sta = msg->data;
    switch (sta)
    {
        case 1:
            UVR_current_state_ = WORKING;
            break;
        case 2:
            UVR_current_state_ = CHARGING;
            break;
        case 3:
            UVR_current_state_ = IDLE;
            break;
        case 4:
            UVR_current_state_ = FAULT;
            break;
        case 5:
            UVR_current_state_ = OFFCHARGING;
            break;

        default:
            break;
    }

    ROS_ERROR("get uvrobot mode........................ %d", sta);
    //当goal list为空时，用于激活move_base while循环
    stateCheck();
    return;
  }


  void MoveBase::publishState(UVRobotWorkState sta)
  {   
      std_msgs::Int32 stas;
      int state_num = -1;
      switch (sta)
      {
          case WORKING:
              state_num = 1;
              break;
          case CHARGING:
              state_num = 2;
              break;
          case IDLE:
              state_num = 3;
              break;
          case FAULT:
              state_num = 4;
              break;
          case OFFCHARGING:
              state_num = 5;
              break;
          default:
              break;
      }
    
    stas.data = state_num;
    UVRobot_state_pub_.publish(stas);
  }


};
