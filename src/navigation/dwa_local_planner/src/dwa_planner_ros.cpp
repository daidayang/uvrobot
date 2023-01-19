/*********************************************************************
*
* Software License Agreement (BSD License)
*
*  Copyright (c) 2009, Willow Garage, Inc.
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
*   * Neither the name of Willow Garage, Inc. nor the names of its
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
*********************************************************************/

#include <dwa_local_planner/dwa_planner_ros.h>
#include <Eigen/Core>
#include <cmath>

#include <ros/console.h>

#include <pluginlib/class_list_macros.h>

#include <base_local_planner/goal_functions.h>
#include <nav_msgs/Path.h>
#include <tf2/utils.h>

#include <nav_core/parameter_magic.h>
#include <numeric>
//register this planner as a BaseLocalPlanner plugin
PLUGINLIB_EXPORT_CLASS(dwa_local_planner::DWAPlannerROS, nav_core::BaseLocalPlanner)

namespace dwa_local_planner 
{

  void DWAPlannerROS::reconfigureCB(DWAPlannerConfig &config, uint32_t level) 
  {
      if (setup_ && config.restore_defaults) 
      {
        config = default_config_;
        config.restore_defaults = false;
      }

      if ( ! setup_) 
      {
        default_config_ = config;
        setup_ = true;
      }

      // update generic local planner params
      base_local_planner::LocalPlannerLimits limits;
      limits.max_vel_trans = config.max_vel_trans;
      limits.min_vel_trans = config.min_vel_trans;
      limits.max_vel_x = config.max_vel_x;
      limits.min_vel_x = config.min_vel_x;
      limits.max_vel_y = config.max_vel_y;
      limits.min_vel_y = config.min_vel_y;
      limits.max_vel_theta = config.max_vel_theta;
      limits.min_vel_theta = config.min_vel_theta;

      limits.acc_lim_x = config.acc_lim_x;
      limits.acc_lim_y = config.acc_lim_y;
      limits.acc_lim_theta = config.acc_lim_theta;
      limits.acc_lim_trans = config.acc_lim_trans;
      limits.xy_goal_tolerance = config.xy_goal_tolerance;
      limits.yaw_goal_tolerance = config.yaw_goal_tolerance;
      limits.prune_plan = config.prune_plan;

      limits.trans_stopped_vel = config.trans_stopped_vel;
      limits.theta_stopped_vel = config.theta_stopped_vel;
      
      limits.xy_goal_tolerance_for_rotation = config.xy_goal_tolerance_for_rotation;
      limits.xy_goal_tolerance_for_charging = config.xy_goal_tolerance_for_charging;

      ROS_INFO("DWAPlannerROS,limits(config) Setting: max_vel_trans %.4f",limits.max_vel_trans);
      ROS_INFO("DWAPlannerROS,limits(config) Setting: min_vel_trans %.4f",limits.min_vel_trans);
      ROS_INFO("DWAPlannerROS,limits(config) Setting: max_vel_x %.4f",limits.max_vel_x);
      ROS_INFO("DWAPlannerROS,limits(config) Setting: min_vel_x %.4f",limits.min_vel_x);
      ROS_INFO("DWAPlannerROS,limits(config) Setting: max_vel_y %.4f",limits.max_vel_y);
      ROS_INFO("DWAPlannerROS,limits(config) Setting: min_vel_y %.4f",limits.min_vel_y);
      ROS_INFO("DWAPlannerROS,limits(config) Setting: max_vel_theta %.4f",limits.max_vel_theta);
      ROS_INFO("DWAPlannerROS,limits(config) Setting: min_vel_theta %.4f",limits.min_vel_theta);
      ROS_INFO("DWAPlannerROS,limits(config) Setting: acc_lim_x %.4f",limits.acc_lim_x);
      ROS_INFO("DWAPlannerROS,limits(config) Setting: acc_lim_y %.4f",limits.acc_lim_y);
      ROS_INFO("DWAPlannerROS,limits(config) Setting: acc_lim_theta %.4f",limits.acc_lim_theta);
      ROS_INFO("DWAPlannerROS,limits(config) Setting: acc_lim_trans %.4f",limits.acc_lim_trans);
      ROS_INFO("DWAPlannerROS,limits(config) Setting: xy_goal_tolerance %.4f",limits.xy_goal_tolerance);
      ROS_INFO("DWAPlannerROS,limits(config) Setting: yaw_goal_tolerance %.4f",limits.yaw_goal_tolerance);
      ROS_INFO("DWAPlannerROS,limits(config) Setting: prune_plan %.4f",limits.prune_plan);
      ROS_INFO("DWAPlannerROS,limits(config) Setting: trans_stopped_vel %.4f",limits.trans_stopped_vel);
      ROS_INFO("DWAPlannerROS,limits(config) Setting: theta_stopped_vel %.4f",limits.theta_stopped_vel);
      ROS_INFO("DWAPlannerROS,limits(config) Setting: xy_goal_tolerance_for_rotation %.4f",limits.xy_goal_tolerance_for_rotation);
      ROS_INFO("DWAPlannerROS,limits(config) Setting: xy_goal_tolerance_for_charging %.4f",limits.xy_goal_tolerance_for_charging);

      planner_util_.reconfigureCB(limits, config.restore_defaults);

      // update dwa specific configuration
      dp_->reconfigure(config);
  }

  DWAPlannerROS::DWAPlannerROS() : initialized_(false),
      odom_helper_("odom"), setup_(false)
  {
    xy_goal_limited_tolerance_ = 0.05;
    factor_dwa_ = 1;

    angle_list_.clear();
    distance_list50.clear();

    current_goal_stop_action_over_ = false;
    sent_cmd_times_ = 0;
    keepCount_ = 0;
  }

  void DWAPlannerROS::initialize(
      std::string name,
      tf2_ros::Buffer* tf,
      costmap_2d::Costmap2DROS* costmap_ros)
  {

    ROS_INFO("DWAPlannerROS,local planner initiallization ................");
    if (! isInitialized()) 
    {
      ros::NodeHandle private_nh("~/" + name);
      g_plan_pub_ = private_nh.advertise<nav_msgs::Path>("transformed_global_plan", 1);
      l_plan_pub_ = private_nh.advertise<nav_msgs::Path>("local_plan", 1);
      tf_ = tf;
      costmap_ros_ = costmap_ros;
      costmap_ros_->getRobotPose(current_pose_);

      // make sure to update the costmap we'll use for this cycle
      costmap_2d::Costmap2D* costmap = costmap_ros_->getCostmap();

      planner_util_.initialize(tf, costmap, costmap_ros_->getGlobalFrameID());

      //create the actual planner that we'll use.. it'll configure itself from the parameter server
      dp_ = boost::shared_ptr<DWAPlanner>(new DWAPlanner(name, &planner_util_));

      if( private_nh.getParam( "odom_topic", odom_topic_ ))
      {
        odom_helper_.setOdomTopic( odom_topic_ );

        ROS_INFO("odom_topic: %s",odom_topic_.c_str());
      }

      private_nh.param("enable_stop_and_rotation_action", enable_stop_and_rotation_,false);

      ROS_INFO("enable_stop_and_rotation_action: %d",enable_stop_and_rotation_?1:0);

      private_nh.param("stop_and_rotation_distance", stop_and_rotation_distance_,0.4);

      ROS_INFO("stop_and_rotation_distance: %.2f",stop_and_rotation_distance_);

      private_nh.param("stop_and_rotation_angle_tolerance", stop_and_rotation_angle_tolerance_,0.4);

      ROS_INFO("stop_and_rotation_angle_tolerance: %.2f",stop_and_rotation_angle_tolerance_);

      initialized_ = true;

      // Warn about deprecated parameters -- remove this block in N-turtle
      nav_core::warnRenamedParameter(private_nh, "max_vel_trans", "max_trans_vel");
      nav_core::warnRenamedParameter(private_nh, "min_vel_trans", "min_trans_vel");
      nav_core::warnRenamedParameter(private_nh, "max_vel_theta", "max_rot_vel");
      nav_core::warnRenamedParameter(private_nh, "min_vel_theta", "min_rot_vel");
      nav_core::warnRenamedParameter(private_nh, "acc_lim_trans", "acc_limit_trans");
      nav_core::warnRenamedParameter(private_nh, "theta_stopped_vel", "rot_stopped_vel");

      dsrv_ = new dynamic_reconfigure::Server<DWAPlannerConfig>(private_nh);
      dynamic_reconfigure::Server<DWAPlannerConfig>::CallbackType cb = boost::bind(&DWAPlannerROS::reconfigureCB, this, _1, _2);
      dsrv_->setCallback(cb);
    }
    else{
      ROS_WARN("This planner has already been initialized, doing nothing.");
    }
  }
  
  bool DWAPlannerROS::setPlan(const std::vector<geometry_msgs::PoseStamped>& orig_global_plan) 
  {
    if (! isInitialized()) 
    {
      ROS_ERROR("This planner has not been initialized, please call initialize() before using this planner");
      return false;
    }
    //when we get a new plan, we also want to clear any latch we may have on goal tolerances
    latchedStopRotateController_.resetLatching();

    ROS_INFO("set global plan ................................");

    return dp_->setPlan(orig_global_plan);
  }

  bool DWAPlannerROS::isGoalReached(bool ischarging) 
  {
    if (! isInitialized()) 
    {
      ROS_ERROR("This planner has not been initialized, please call initialize() before using this planner");
      return false;
    }
    if ( ! costmap_ros_->getRobotPose(current_pose_)) 
    {
      ROS_ERROR("Could not get robot pose");
      return false;
    }

    geometry_msgs::PoseStamped goal_pose;
    planner_util_.getGoal(goal_pose);
    // //计算水平距离
    // double goal_x = goal_pose.pose.position.x;
    // double goal_y = goal_pose.pose.position.y;

    // double current_distance_xy = base_local_planner::getGoalPositionDistance(current_pose_, goal_x, goal_y);

    // distance_list50.push_back(current_distance_xy);
    // if(distance_list50.size() > 50)
    // {
    //   distance_list50.erase(distance_list50.begin());
    // }

    if(latchedStopRotateController_.isGoalReached(ischarging, &planner_util_, odom_helper_, current_pose_,xy_goal_limited_tolerance_)) 
    {
      double yaw = tf2::getYaw(current_pose_.pose.orientation);
      ROS_INFO("Goal reached (DWAPlannerROS)....................current_pose:(%f, %f, %f), orientation:(%f, %f, %f,%f), yaw: %.3f",
        current_pose_.pose.position.x,current_pose_.pose.position.y,current_pose_.pose.position.z,
        current_pose_.pose.orientation.x,current_pose_.pose.orientation.y,current_pose_.pose.orientation.z,current_pose_.pose.orientation.w,yaw);

      // geometry_msgs::PoseStamped goal_pose;
      // if (planner_util_.getGoal(goal_pose)) 
      {
        double yaw = tf2::getYaw(goal_pose.pose.orientation);
        ROS_INFO("get goal pose (DWAPlannerROS)..............................: goal:(%f, %f, %f), orientation: (%f, %f, %f,%f), yaw: %.3f",
        goal_pose.pose.position.x,goal_pose.pose.position.y,goal_pose.pose.position.z,
        goal_pose.pose.orientation.x,goal_pose.pose.orientation.y,goal_pose.pose.orientation.z,goal_pose.pose.orientation.w,yaw);
      }

      //如果到达目标点了,距离限定恢复正常值,为下一个目标点做准备
      factor_dwa_ = 1;

      //到达目标点后，初始化stop and rotation 相关变量
      current_goal_stop_action_over_ = false;
      sent_cmd_times_ = 0;
      ROS_INFO("-----------reached the goal,and set current_goal_stop_action_over %d and sent_cmd_times %d. ",current_goal_stop_action_over_?1:0,sent_cmd_times_);

      return true;
    } 
    // else if(current_distance_xy < 0.4)
    // {
    //   //如果在目标点附近，距离越来越大，则为远离目标点的情况，则认为xy水平距离到达
    //   if(distance_list50.size() >= 20)
    //   {
    //     std::vector<double> delta_dis;
    //     std::vector<double> temp_data = distance_list50.reverse();

    //     for(int i = 1;i<10 && i< temp_data.size();++i)
    //     {
    //       delta_dis.push_back(temp_data[i-1] - temp_data[i]);
    //     }
        
    //     //判断delta_dis的数据
    //     int bigCount = 0;
    //     for(int i = 0;i< delta_dis.size();++i)
    //     {
    //       if(delta_dis[i] > 0)
    //       {
    //         bigCount ++;
    //       }
    //     }
    //     double rate = 0;
    //     if(bigCount > 0 && delta_dis.size()>5)
    //     {
    //       rate = bigCount/delta_dis.size();
    //     }


    //     if(rate > 0.7)
    //     {
    //       return true;
    //     }

    //   }
    // }
    else
    {
      return false;
    }
  }

  void DWAPlannerROS::publishLocalPlan(std::vector<geometry_msgs::PoseStamped>& path) {
    base_local_planner::publishPlan(path, l_plan_pub_);
  }

  void DWAPlannerROS::publishGlobalPlan(std::vector<geometry_msgs::PoseStamped>& path) {
    base_local_planner::publishPlan(path, g_plan_pub_);
  }

  DWAPlannerROS::~DWAPlannerROS(){
    //make sure to clean things up
    delete dsrv_;
  }

  bool DWAPlannerROS::setNewGoal(bool is_new_goal)
  {
    is_new_goal_first_ = is_new_goal;
  }

  bool DWAPlannerROS::dwaComputeVelocityCommands(geometry_msgs::PoseStamped &global_pose, geometry_msgs::Twist& cmd_vel) 
  {
    // dynamic window sampling approach to get useful velocity commands
    if(! isInitialized())
    {
      ROS_ERROR("This planner has not been initialized, please call initialize() before using this planner");
      return false;
    }

    //获取里程计提供的机器人速度
    geometry_msgs::PoseStamped robot_vel;
    odom_helper_.getRobotVel(robot_vel);

    /* For timing uncomment
    struct timeval start, end;
    double start_t, end_t, t_diff;
    gettimeofday(&start, NULL);
    */

    geometry_msgs::PoseStamped goal_pose;
    if (planner_util_.getGoal(goal_pose)) 
    {
      double yaw = tf2::getYaw(goal_pose.pose.orientation);
      ROS_INFO("(DWAPlannerROS)..............................: goal:(%f, %f, %f), orientation: (%f, %f, %f,%f), yaw: %.3f",
      goal_pose.pose.position.x,goal_pose.pose.position.y,goal_pose.pose.position.z,
      goal_pose.pose.orientation.x,goal_pose.pose.orientation.y,goal_pose.pose.orientation.z,goal_pose.pose.orientation.w,yaw);
    }

  //////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
  
    //计算robot与当前goal的距离
    double goal_x = goal_pose.pose.position.x;
    double goal_y = goal_pose.pose.position.y;

    base_local_planner::LocalPlannerLimits limits = planner_util_.getCurrentLimits();

    double current_distance_xy = base_local_planner::getGoalPositionDistance(global_pose, goal_x, goal_y);

  ///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////


    //compute what trajectory to drive along
    geometry_msgs::PoseStamped drive_cmds;
    drive_cmds.header.frame_id = costmap_ros_->getBaseFrameID();
    
    // call with updated footprint
    //找出最优轨迹
    base_local_planner::Trajectory path = dp_->findBestPath(global_pose, robot_vel, drive_cmds);
    //ROS_ERROR("Best: %.2f, %.2f, %.2f, %.2f", path.xv_, path.yv_, path.thetav_, path.cost_);

    /* For timing uncomment
    gettimeofday(&end, NULL);
    start_t = start.tv_sec + double(start.tv_usec) / 1e6;
    end_t = end.tv_sec + double(end.tv_usec) / 1e6;
    t_diff = end_t - start_t;
    ROS_INFO("Cycle time: %.9f", t_diff);
    */

    //生成控制命令
    //pass along drive commands
    cmd_vel.linear.x = drive_cmds.pose.position.x;
    cmd_vel.linear.y = drive_cmds.pose.position.y;
    
    cmd_vel.angular.z = tf2::getYaw(drive_cmds.pose.orientation);

    angle_list_.push_back(cmd_vel.angular.z);

    if(angle_list_.size() > average_count_)
    {
      angle_list_.erase(angle_list_.begin());
    }
    
    //限定距离内，角度设置为0
    //factor_dwa == 1//说明robot还没有到距离目标点　xy_goal_tolerance　范围之内
    
    if(enable_stop_and_rotation_)
    {
      //如果stop and rotation 动作完成
      if(current_goal_stop_action_over_)
      {        
        ROS_ERROR("-----------Done Stop/Turn. Ang: %.3f, DltXY: %.3f",cmd_vel.angular.z, current_distance_xy);
        ROS_INFO("-----------Done Stop/Turn. Ang: %.3f, DltXY: %.3f, in the range [%.3f, %.3f]",cmd_vel.angular.z, current_distance_xy, xy_goal_limited_tolerance_, stop_and_rotation_distance_);

//        ROS_ERROR("-----------current distance: %.3f, in the range [%.3f, %.3f] , disable the angle velocity.", 
//                      current_distance_xy, xy_goal_limited_tolerance_, stop_and_rotation_distance_);
//        ROS_INFO("-----------current distance: %.3f, in the range [%.3f, %.3f] , disable the angle velocity.", 
//                      current_distance_xy, xy_goal_limited_tolerance_, stop_and_rotation_distance_);
        
        if(current_distance_xy > xy_goal_limited_tolerance_)
        {
          if(cmd_vel.linear.x < 0.02)
          {
            keepCount_++;
          }else
          {
            keepCount_ = 0;
          }
          
          if(keepCount_ > 4)
          {
            ROS_INFO("-----------complete stop and rotation:  the linear velocity is %.3f,nochange,set %.3f",cmd_vel.linear.x,0.09);

            cmd_vel.linear.x = 0.09;
          }
        }
        cmd_vel.angular.z = 0;
      }
    }

    // if(factor_dwa == 2)
    // {
    //   ROS_ERROR("-----------the robot reached the goal position, maybe it starts turning in place.");
    //   ROS_INFO("-----------the robot reached the goal position, maybe it starts turning in place.");
    // }

    //if we cannot move... tell someone
    std::vector<geometry_msgs::PoseStamped> local_plan;
    
    //寻找失败
    if(path.cost_ < 0)
    {
      ROS_DEBUG_NAMED("dwa_local_planner",
          "The dwa local planner failed to find a valid plan, cost functions discarded all candidates. This can mean there is an obstacle too close to the robot.");
      local_plan.clear();
      publishLocalPlan(local_plan);
      return false;
    }

    //找到最优,并给出控制命令
    ROS_INFO("dwa_local_planner, a valid velocity command of (%.3f, %.3f, %.3f) was found for this cycle.", 
                    cmd_vel.linear.x, cmd_vel.linear.y, cmd_vel.angular.z);

    //把最优轨迹点包装成plan格式,即local plan,然后发布
    // Fill out the local plan
    for(unsigned int i = 0; i < path.getPointsSize(); ++i) 
    {
      double p_x, p_y, p_th;
      path.getPoint(i, p_x, p_y, p_th);

      geometry_msgs::PoseStamped p;
      p.header.frame_id = costmap_ros_->getGlobalFrameID();
      p.header.stamp = ros::Time::now();
      p.pose.position.x = p_x;
      p.pose.position.y = p_y;
      p.pose.position.z = 0.0;
      tf2::Quaternion q;
      q.setRPY(0, 0, p_th);
      tf2::convert(q, p.pose.orientation);
      local_plan.push_back(p);
    }

    
    int end  = local_plan.size()-1;
    double length = 0;
    if(end > 0)
    {
      length = sqrt(pow(local_plan[0].pose.position.y - local_plan[end].pose.position.y,2.0) + pow(local_plan[0].pose.position.x - local_plan[end].pose.position.x,2.0));
      //ROS_INFO("get sim path count: %d, length: %f", local_plan.size(),length);
    }
    //publish information to the visualizer
    ROS_INFO("dwaComputeVelocityCommands,publishLocalPlan sim path, count:%d, lenght %.3f: ",local_plan.size(),length);
    for(int i = 0;i< 10 && i<local_plan.size();++i)
    {
      ROS_INFO("%d ponit: %.3f, %.3f",i+1,local_plan[i].pose.position.x,local_plan[i].pose.position.y);
    }
    publishLocalPlan(local_plan);
    return true;
  }


  bool DWAPlannerROS::computeVelocityCommands(geometry_msgs::Twist& cmd_vel) 
  {
    // dispatches to either dwa sampling control or stop and rotate control, depending on whether we have been close enough to goal
    //获取机器人当前的pose
    if ( ! costmap_ros_->getRobotPose(current_pose_)) 
    {
      ROS_ERROR("Could not get robot pose");
      return false;
    }

    //把global plan 映射到局部代价地图上即local plan
    //每个控制周期，会在机器人周围创建珊格（大小为局部代价地图）区域，并且全局路径会被映射到这个区域上
    std::vector<geometry_msgs::PoseStamped> transformed_plan;
    if (!planner_util_.getLocalPlan(current_pose_, transformed_plan)) 
    {
      ROS_ERROR("Could not get local plan");
      return false;
    }

    ROS_INFO("DWAPlannerROS::computeVelocityCommands ,first get local plan (transformed_plan) ..............%d",transformed_plan.size());

    //if the global plan passed in is empty... we won't do anything
    if(transformed_plan.empty()) 
    {
      ROS_WARN_NAMED("dwa_local_planner", "Received an empty transformed plan.");
      return false;
    }
    ROS_INFO("dwa_local_planner, Received a transformed plan with %zu points.", transformed_plan.size());

    // update plan in dwa_planner even if we just stop and rotate, to allow checkTrajectory
    dp_->updatePlanAndLocalCosts(current_pose_, transformed_plan, costmap_ros_->getRobotFootprint());


    ///////////////////////////////////////////new add/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    //在目标点之前一定距离，先停下来，然后旋转方向
    if(enable_stop_and_rotation_)
    {

      //在目标点之前一定距离，先停下来，然后旋转方向
      geometry_msgs::PoseStamped goal_pose;
      if (planner_util_.getGoal(goal_pose)) 
      {
        double yaw = tf2::getYaw(goal_pose.pose.orientation);
      }
      double goal_x = goal_pose.pose.position.x;
      double goal_y = goal_pose.pose.position.y;
      //判断是否进入距离范畴
      double current_distance_xy = base_local_planner::getGoalPositionDistance(current_pose_, goal_x, goal_y);

      double robot_yaw = tf2::getYaw(current_pose_.pose.orientation);
      //计算目标方向
      double goal_th = atan2(goal_y - current_pose_.pose.position.y,goal_x - current_pose_.pose.position.x);

      ROS_INFO("---------stop and rotation, cal the yaw to goal .............%.3f",goal_th);
      ROS_INFO("---------stop and rotation, the robot yaw  .............%.3f",robot_yaw);

      if(!current_goal_stop_action_over_
          && current_distance_xy < stop_and_rotation_distance_ 
          && current_distance_xy > xy_goal_limited_tolerance_)
      {

        ROS_INFO("---------stop and rotation, is_new_goal_first:%d",is_new_goal_first_?1:0);

        if(is_new_goal_first_)
        {
          sent_cmd_times_ = 50;//如果目标点goal距离小于0.4则不进行stop操作（这个时候车子是停止的），直接进行旋转操作

          //获取机器人当前静止状态下的pose
          if (!costmap_ros_->getRobotPose(current_pose_stationary_))
          {
            ROS_ERROR("Could not get robot pose");
            bget_stationary_pose_ = false;
            return false;
          }else
          {
            bget_stationary_pose_ = true;
            
            double yaw = tf2::getYaw(current_pose_stationary_.pose.orientation);
            ROS_INFO("--------enter stop mode: current stationary pose :(%f, %f, %f), yaw: %.3f",
            current_pose_stationary_.pose.position.x,current_pose_stationary_.pose.position.y,current_pose_stationary_.pose.position.z,yaw);
          }
        }
        //持续发布stop控制命令5s
        if(sent_cmd_times_ < 15)
        {
          //发布命令
          //geometry_msgs::Twist cmd_vel;
          cmd_vel.linear.x = 0.0;
          cmd_vel.linear.y = 0.0;
          cmd_vel.angular.z = 0.0;

          //vel_pub_.publish(cmd_vel);

          ROS_ERROR("-------enter stop mode, stop command.............%d",sent_cmd_times_);
          ROS_INFO("--------enter stop mode, stop command.............%d",sent_cmd_times_);

          sent_cmd_times_++;

          publishGlobalPlan(transformed_plan);


          
          //获取机器人当前静止状态下的pose
          if (!costmap_ros_->getRobotPose(current_pose_stationary_))
          {
            ROS_ERROR("Could not get robot pose");
            bget_stationary_pose_ = false;
            return false;
          }else
          {
            bget_stationary_pose_ = true;
            
            double yaw = tf2::getYaw(current_pose_stationary_.pose.orientation);
            ROS_INFO("--------enter stop mode: current stationary pose :(%f, %f, %f), yaw: %.3f",
            current_pose_stationary_.pose.position.x,current_pose_stationary_.pose.position.y,current_pose_stationary_.pose.position.z,yaw);
          }
          


          return true;

        }else if(sent_cmd_times_ >= 15)
        {
          //开始旋转
          //计算目标方向
          //由于旋转过程中，robot位置会发生轻微偏移，所以使用静止是的pose,计算偏航
          goal_th = atan2(goal_y - current_pose_stationary_.pose.position.y, goal_x - current_pose_stationary_.pose.position.x);

          
          //double goal_th = tf2::getYaw(goal_pose.pose.orientation);
          double angle = base_local_planner::getGoalOrientationAngleDifference(current_pose_, goal_th);

          robot_yaw = tf2::getYaw(current_pose_.pose.orientation);

          //ROS_INFO("--------rotation mode, the delta yaw to goal.............%.3f",angle);
          ROS_INFO("---------enter rotation mode, cal the yaw to goal .............%.3f",goal_th);
          ROS_INFO("---------enter rotation mode, the robot yaw  .............%.3f",robot_yaw);
          ROS_INFO("--------enter rotation mode, the delta yaw to goal:%.3f, stop_and_rotation_angle_tolerance: %.3f",angle,stop_and_rotation_angle_tolerance_);

          base_local_planner::LocalPlannerLimits limits = planner_util_.getCurrentLimits();
          //如果满足角度要求
          if (fabs(angle) <= stop_and_rotation_angle_tolerance_)
          {
            //set the velocity command to zero
            cmd_vel.linear.x = 0.0;
            cmd_vel.linear.y = 0.0;
            cmd_vel.angular.z = 0.0;


            //已经实现停止及旋转目标
            current_goal_stop_action_over_ = true;
            sent_cmd_times_ = 0;
            //rotating_to_goal_ = false;
            //vel_pub_.publish(cmd_vel);

            ROS_ERROR("--------complete stop and rotation action, .............");
            ROS_INFO("---------complete stop and rotation action, .............");

          }else
          {
            geometry_msgs::PoseStamped robot_vel;

            if (!latchedStopRotateController_.rotateToGoal(
              current_pose_,
              robot_vel,
              goal_th,
              cmd_vel,
              limits.getAccLimits(),
              dp_->getSimPeriod(),
              limits,
              boost::bind(&DWAPlanner::checkTrajectory, dp_, _1, _2, _3))) 
              {
                ROS_INFO("Error when rotating.");
                return false;
              }

              publishGlobalPlan(transformed_plan);

              ROS_ERROR("------enter rotation mode, rotation command.............");
              ROS_INFO("-------enter rotation mode, rotation command.............");

              return true;
          }
          
        }
      }
      

    }


    ///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////


    //水平距离是否到达终点
    if (latchedStopRotateController_.isPositionReached(&planner_util_, current_pose_)) 
    {
      //publish an empty plan because we've reached our goal position
      //ROS_INFO("in DWAPlannerROS::computeVelocityCommands,position reached !!!!!!!!!!!!!!!!!!!!!!!!!");
      std::vector<geometry_msgs::PoseStamped> local_plan;
      std::vector<geometry_msgs::PoseStamped> transformed_plan;
      publishGlobalPlan(transformed_plan);
      publishLocalPlan(local_plan);

      base_local_planner::LocalPlannerLimits limits = planner_util_.getCurrentLimits();

      bool re = latchedStopRotateController_.computeVelocityCommandsStopRotate(
          cmd_vel,
          limits.getAccLimits(),
          dp_->getSimPeriod(),
          &planner_util_,
          odom_helper_,
          current_pose_,
          boost::bind(&DWAPlanner::checkTrajectory, dp_, _1, _2, _3));

      ROS_INFO("<<<<<<< position reached,turning in place cmd >>>>>>>>.................................................................front");
      
      ROS_INFO("robot current_pose(%.3f, %.3f, %.3f), cmd_vel: (%.3f,%.3f,%.3f)",current_pose_.pose.position.x,current_pose_.pose.position.y,current_pose_.pose.position.z,cmd_vel.linear.x,cmd_vel.linear.y,cmd_vel.angular.z);

      return re;
    } 
    else 
    {
      //还没有到达目标，继续产生控制命令
      ROS_INFO("DWAPlannerROS::computeVelocityCommands,<<<<<< generating twist cmd >>>>>>>,run here ................................................................. back");
      
      bool isOk = dwaComputeVelocityCommands(current_pose_, cmd_vel);

      ROS_INFO("robot current_pose(%.3f, %.3f, %.3f), cmd_vel: (%.3f,%.3f,%.3f)",current_pose_.pose.position.x,current_pose_.pose.position.y,current_pose_.pose.position.z,cmd_vel.linear.x,cmd_vel.linear.y,cmd_vel.angular.z);

      if (isOk) 
      {
        publishGlobalPlan(transformed_plan);
      } else 
      {
        ROS_WARN_NAMED("dwa_local_planner", "DWA planner failed to produce path.");
        std::vector<geometry_msgs::PoseStamped> empty_plan;
        publishGlobalPlan(empty_plan);
      }
      return isOk;
    }
  }


};
