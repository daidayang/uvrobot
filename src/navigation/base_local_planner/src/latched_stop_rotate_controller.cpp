/*
 * latched_stop_rotate_controller.cpp
 *
 *  Created on: Apr 16, 2012
 *      Author: tkruse
 */

#include <base_local_planner/latched_stop_rotate_controller.h>

#include <cmath>

#include <Eigen/Core>

#include <angles/angles.h>
#include <nav_msgs/Odometry.h>

#include <base_local_planner/goal_functions.h>
#include <base_local_planner/local_planner_limits.h>

#include <tf2/utils.h>
#include <algorithm>

namespace base_local_planner {

LatchedStopRotateController::LatchedStopRotateController(const std::string& name) {
  ros::NodeHandle private_nh("~/" + name);
  private_nh.param("latch_xy_goal_tolerance", latch_xy_goal_tolerance_, false);

  rotating_to_goal_ = false;
  factor_ = 1.0;
  xy_goal_tolerance_limit_current_ = -1.0;
  distance_store_list50.clear();
}

LatchedStopRotateController::~LatchedStopRotateController() {}


/**
 * returns true if we have passed the goal position.
 * Meaning we might have overshot on the position beyond tolerance, yet still return true.
 * Also goal orientation might not yet be true
 */
bool LatchedStopRotateController::isPositionReached(LocalPlannerUtil* planner_util,
    const geometry_msgs::PoseStamped& global_pose) 
{
  double xy_goal_tolerance = planner_util->getCurrentLimits().xy_goal_tolerance;

  if(xy_goal_tolerance_limit_current_ < 0)
  {
    xy_goal_tolerance_limit_current_ = xy_goal_tolerance;
  }

  ROS_INFO("LatchedStopRotateController::isPositionReached: xy_goal_tolerance_limit_current:%.3f",xy_goal_tolerance_limit_current_);

  //we assume the global goal is the last point in the global plan
  geometry_msgs::PoseStamped goal_pose;
  if ( ! planner_util->getGoal(goal_pose)) 
  {
    return false;
  }

  double goal_x = goal_pose.pose.position.x;
  double goal_y = goal_pose.pose.position.y;

  //check to see if we've reached the goal position
  if ((latch_xy_goal_tolerance_ && xy_tolerance_latch_) ||
      base_local_planner::getGoalPositionDistance(global_pose, goal_x, goal_y) <= xy_goal_tolerance_limit_current_) {
    xy_tolerance_latch_ = true;
    return true;
  }
  return false;
}

bool LatchedStopRotateController::judgeDistance(double current_distance_xy,std::vector<double> &distance_list)
{
    if(current_distance_xy > 0.2) {
      return false;
    }
    else
    {
      //如果在目标点附近，距离越来越大，则为远离目标点的情况，则认为xy水平距离到达
      if(distance_list.size() > 10)
      {
        std::vector<double> delta_dis;
        std::vector<double> temp_data = distance_list;
        std::reverse(temp_data.begin(),temp_data.end());

        for(int i = 0;i<temp_data.size();++i)
        {
          ROS_INFO("......judgeDistance distance :%.3f",temp_data[i]);
        }

        for(int i = 1;i<=10 && i< temp_data.size();++i)
        {
          delta_dis.push_back(temp_data[i-1] - temp_data[i]);
        }
                
                
        for(int i = 0;i<delta_dis.size();++i)
        {
          ROS_INFO("......judgeDistance delta :%.3f",delta_dis[i]);
        }

        //判断delta_dis的数据
        int bigCount = 0;
        for(int i = 0;i< delta_dis.size();++i)
        {
          if(delta_dis[i] > 0)
          {
            bigCount ++;
          }
        }
        double rate = 0.0;
        if(bigCount > 0 && delta_dis.size()>5)
        {
          rate = bigCount/delta_dis.size();
        }

        ROS_INFO("......judgeDistance bigCount :%d , all count: %d,rate: %.2f",bigCount,delta_dis.size(),rate);

        if(rate > 0.7)
        {
          return true;
        }else
        {
          return false;
        }
      }
    }
}
/**
 * returns true if we have passed the goal position and have reached goal orientation.
 * Meaning we might have overshot on the position beyond tolerance, yet still return true.
 */
bool LatchedStopRotateController::isGoalReached(bool ischarging, LocalPlannerUtil* planner_util,
    OdometryHelperRos& odom_helper,
    const geometry_msgs::PoseStamped& global_pose,
    double &xy_limited_tolerance) 
{

  double xy_goal_tolerance = planner_util->getCurrentLimits().xy_goal_tolerance;
  double yaw_goal_tolerance = planner_util->getCurrentLimits().yaw_goal_tolerance;
  double theta_stopped_vel = planner_util->getCurrentLimits().theta_stopped_vel;
  double trans_stopped_vel = planner_util->getCurrentLimits().trans_stopped_vel;
  double xy_goal_tolerance_for_rotation = planner_util->getCurrentLimits().xy_goal_tolerance_for_rotation;
  double xy_goal_tolerance_for_charging = planner_util->getCurrentLimits().xy_goal_tolerance_for_charging;

  ROS_INFO("LatchedStopRotate::isGoalReached: (xy_goal_tolerance_for_rotation:%f, xy_goal_tolerance:%f, yaw_goal_tolerance:%f,theta_stopped_vel:%f,trans_stopped_vel:%f)",
      xy_goal_tolerance_for_rotation,xy_goal_tolerance,yaw_goal_tolerance,theta_stopped_vel,trans_stopped_vel);
//  ROS_ERROR("GoalReached: xy_goal_tolerance_for_rotation:%f, xy_goal_tolerance:%f, yaw_goal_tolerance:%f",
//      xy_goal_tolerance_for_rotation,xy_goal_tolerance,yaw_goal_tolerance);
  //copy over the odometry information
  //获取odometry信息
  nav_msgs::Odometry base_odom;
  odom_helper.getOdom(base_odom);

  //we assume the global goal is the last point in the global plan
  geometry_msgs::PoseStamped goal_pose;
  if ( ! planner_util->getGoal(goal_pose))
  {
    return false;
  }
  //计算距离
  double goal_x = goal_pose.pose.position.x;
  double goal_y = goal_pose.pose.position.y;

  base_local_planner::LocalPlannerLimits limits = planner_util->getCurrentLimits();

  double current_distance_xy = base_local_planner::getGoalPositionDistance(global_pose, goal_x, goal_y);

  distance_store_list50.push_back(current_distance_xy);
  if(distance_store_list50.size()>50)
  {
    distance_store_list50.erase(distance_store_list50.begin());
  }
  double goal_th = tf2::getYaw(goal_pose.pose.orientation);
  double angle = base_local_planner::getGoalOrientationAngleDifference(global_pose, goal_th);

  bool become_bigger = judgeDistance(current_distance_xy,distance_store_list50);

  if(current_distance_xy < xy_goal_tolerance)
  {
    factor_ = 2;
  }else if(current_distance_xy > 0.50)
  {
    factor_ = 1;
  }else if(become_bigger)
  {
    factor_ = 3;//当检测到robot距离目标点越来越远时，强制扩展xy限定偏移
  }

  double xy_goal_tolerance_limit = xy_goal_tolerance;//默认正常值
  if ( ischarging ) {
    xy_goal_tolerance_limit = xy_goal_tolerance_for_charging;
  }

  if(factor_ == 2)
  {
    xy_goal_tolerance_limit = xy_goal_tolerance_for_rotation;//当接近goal值，开始旋转则选择　xy_goal_tolerance_for_rotation
  }else if(factor_ == 3)
  {
    xy_goal_tolerance_limit = 2*xy_goal_tolerance_for_rotation;//在近距离位置，离goal越来越远时，开始旋转则选择
  }
  
//  ROS_ERROR("LatchedStopRotate::isGoalReached: factor: %f, become_bigger: %d",factor_, become_bigger?1:0);

  xy_limited_tolerance = xy_goal_tolerance_limit;//获取当前的xy限定偏移

  xy_goal_tolerance_limit_current_ = xy_goal_tolerance_limit;

  ROS_INFO("isGoalReached:global_pose(%.3f, %.3f, %.3f),goal:(%.3f,%.3f),delta distance:%.3f, delta yaw:%.3f, xy_goal_tolerance_limit: %.2f",
      global_pose.pose.position.x,global_pose.pose.position.y,global_pose.pose.position.z,goal_x,goal_y,current_distance_xy,angle,xy_goal_tolerance_limit);

  ROS_ERROR("isGoalReached: charging:%d, dlt-xy:%.3f, dlt-yaw:%.3f, xy_goal_tolerance_limit: %.2f",
      ischarging?1:0,current_distance_xy,angle,xy_goal_tolerance_limit);


  //check to see if we've reached the goal position
  if ((latch_xy_goal_tolerance_ && xy_tolerance_latch_) ||
      base_local_planner::getGoalPositionDistance(global_pose, goal_x, goal_y) <= xy_goal_tolerance_limit) 
  {
    //if the user wants to latch goal tolerance, if we ever reach the goal location, we'll
    //just rotate in place
    if (latch_xy_goal_tolerance_ && ! xy_tolerance_latch_) 
    {
      ROS_DEBUG("Goal position reached (check), stopping and turning in place");
      ROS_ERROR("position reached, stopping and turning in place");
      xy_tolerance_latch_ = true;
    }

    ROS_INFO("LatchedStopRotate::isGoalReached: arrive position tolerance.");

    double goal_th = tf2::getYaw(goal_pose.pose.orientation);
    double angle = base_local_planner::getGoalOrientationAngleDifference(global_pose, goal_th);

    ROS_INFO("LatchedStopRotate::isGoalReached: yaw_goal_tolerance: %f, goal_yaw: %f, delta angle: %f",limits.yaw_goal_tolerance,goal_th,angle);
    //check to see if the goal orientation has been reached
    if (fabs(angle) <= limits.yaw_goal_tolerance) 
    {
      ROS_INFO("LatchedStopRotate::isGoalReached: reached yaw tolerance.");
      ROS_ERROR("goal orientation(yaw) has been reached...");
      //make sure that we're actually stopped before returning success
      if (base_local_planner::stopped(base_odom, theta_stopped_vel, trans_stopped_vel)) 
      {
        ROS_INFO("LatchedStopRotate::isGoalReached: stop true.");

        factor_ = 1;//到达目标后，水平偏移限制恢复到最小，为下一个目标使用
        ROS_ERROR("Goal Reached...");
        return true;
      }
    }
  }
  return false;
}

bool LatchedStopRotateController::stopWithAccLimits(const geometry_msgs::PoseStamped& global_pose,
    const geometry_msgs::PoseStamped& robot_vel,
    geometry_msgs::Twist& cmd_vel,
    Eigen::Vector3f acc_lim,
    double sim_period,
    boost::function<bool (Eigen::Vector3f pos,
                          Eigen::Vector3f vel,
                          Eigen::Vector3f vel_samples)> obstacle_check) {

  //slow down with the maximum possible acceleration... we should really use the frequency that we're running at to determine what is feasible
  //but we'll use a tenth of a second to be consistent with the implementation of the local planner.
  double vx = sign(robot_vel.pose.position.x) * std::max(0.0, (fabs(robot_vel.pose.position.x) - acc_lim[0] * sim_period));
  double vy = sign(robot_vel.pose.position.y) * std::max(0.0, (fabs(robot_vel.pose.position.y) - acc_lim[1] * sim_period));

  double vel_yaw = tf2::getYaw(robot_vel.pose.orientation);
  double vth = sign(vel_yaw) * std::max(0.0, (fabs(vel_yaw) - acc_lim[2] * sim_period));

  //we do want to check whether or not the command is valid
  double yaw = tf2::getYaw(global_pose.pose.orientation);
  bool valid_cmd = obstacle_check(Eigen::Vector3f(global_pose.pose.position.x, global_pose.pose.position.y, yaw),
                                  Eigen::Vector3f(robot_vel.pose.position.x, robot_vel.pose.position.y, vel_yaw),
                                  Eigen::Vector3f(vx, vy, vth));

  //if we have a valid command, we'll pass it on, otherwise we'll command all zeros
  if(valid_cmd){
    ROS_DEBUG_NAMED("latched_stop_rotate", "Slowing down... using vx, vy, vth: %.2f, %.2f, %.2f", vx, vy, vth);
    cmd_vel.linear.x = vx;
    cmd_vel.linear.y = vy;
    cmd_vel.angular.z = vth;
    return true;
  }
  ROS_WARN("Stopping cmd in collision");
  cmd_vel.linear.x = 0.0;
  cmd_vel.linear.y = 0.0;
  cmd_vel.angular.z = 0.0;
  return false;
}

bool LatchedStopRotateController::rotateToGoal(
    const geometry_msgs::PoseStamped& global_pose,
    const geometry_msgs::PoseStamped& robot_vel,
    double goal_th,
    geometry_msgs::Twist& cmd_vel,
    Eigen::Vector3f acc_lim,
    double sim_period,
    base_local_planner::LocalPlannerLimits& limits,
    boost::function<bool (Eigen::Vector3f pos,
                          Eigen::Vector3f vel,
                          Eigen::Vector3f vel_samples)> obstacle_check) 
{
  double yaw = tf2::getYaw(global_pose.pose.orientation);
  double vel_yaw = tf2::getYaw(robot_vel.pose.orientation);
  
  cmd_vel.linear.x = 0;
  cmd_vel.linear.y = 0;
  double ang_diff = angles::shortest_angular_distance(yaw, goal_th);

  double v_theta_samp = std::min(limits.max_vel_theta, std::max(limits.min_vel_theta, fabs(ang_diff)));

  //take the acceleration limits of the robot into account
  double max_acc_vel = fabs(vel_yaw) + acc_lim[2] * sim_period;
  double min_acc_vel = fabs(vel_yaw) - acc_lim[2] * sim_period;

  v_theta_samp = std::min(std::max(fabs(v_theta_samp), min_acc_vel), max_acc_vel);

  //we also want to make sure to send a velocity that allows us to stop when we reach the goal given our acceleration limits
  double max_speed_to_stop = sqrt(2 * acc_lim[2] * fabs(ang_diff));
  v_theta_samp = std::min(max_speed_to_stop, fabs(v_theta_samp));

  v_theta_samp = std::min(limits.max_vel_theta, std::max(limits.min_vel_theta, v_theta_samp));

  if (ang_diff < 0) {
    v_theta_samp = - v_theta_samp;
  }

  //we still want to lay down the footprint of the robot and check if the action is legal
  bool valid_cmd = obstacle_check(Eigen::Vector3f(global_pose.pose.position.x, global_pose.pose.position.y, yaw),
      Eigen::Vector3f(robot_vel.pose.position.x, robot_vel.pose.position.y, vel_yaw),
      Eigen::Vector3f( 0.0, 0.0, v_theta_samp));

  if (valid_cmd) {
    ROS_DEBUG_NAMED("dwa_local_planner", "Moving to desired goal orientation, th cmd: %.2f, valid_cmd: %d", v_theta_samp, valid_cmd);
    cmd_vel.angular.z = v_theta_samp;
    return true;
  }
  ROS_WARN("Rotation cmd in collision");
  cmd_vel.angular.z = 0.0;
  return false;

}

bool LatchedStopRotateController::computeVelocityCommandsStopRotate(geometry_msgs::Twist& cmd_vel,
    Eigen::Vector3f acc_lim,
    double sim_period,
    LocalPlannerUtil* planner_util,
    OdometryHelperRos& odom_helper_,
    const geometry_msgs::PoseStamped& global_pose,
    boost::function<bool (Eigen::Vector3f pos,
                          Eigen::Vector3f vel,
                          Eigen::Vector3f vel_samples)> obstacle_check) 
  {
  //we assume the global goal is the last point in the global plan
  geometry_msgs::PoseStamped goal_pose;
  if ( ! planner_util->getGoal(goal_pose)) 
  {
    ROS_ERROR("Could not get goal pose");
    return false;
  }

  base_local_planner::LocalPlannerLimits limits = planner_util->getCurrentLimits();

  //if the user wants to latch goal tolerance, if we ever reach the goal location, we'll
  //just rotate in place
  //检测目标距离是否满足
  if (latch_xy_goal_tolerance_ && ! xy_tolerance_latch_ ) 
  {
    ROS_INFO("Goal position reached, stopping and turning in place");
    xy_tolerance_latch_ = true;
  }
  
  //检测比较方向是否满足
  //check to see if the goal orientation has been reached
  double goal_th = tf2::getYaw(goal_pose.pose.orientation);
  double angle = base_local_planner::getGoalOrientationAngleDifference(global_pose, goal_th);
  
  if (fabs(angle) <= limits.yaw_goal_tolerance) 
  {
    //set the velocity command to zero
    cmd_vel.linear.x = 0.0;
    cmd_vel.linear.y = 0.0;
    cmd_vel.angular.z = 0.0;
    rotating_to_goal_ = false;
  } 
  else 
  {
    ROS_DEBUG("Angle: %f Tolerance: %f", angle, limits.yaw_goal_tolerance);
    geometry_msgs::PoseStamped robot_vel;
    odom_helper_.getRobotVel(robot_vel);
    nav_msgs::Odometry base_odom;
    odom_helper_.getOdom(base_odom);

    //if we're not stopped yet... we want to stop... taking into account the acceleration limits of the robot
    if ( ! rotating_to_goal_ && !base_local_planner::stopped(base_odom, limits.theta_stopped_vel, limits.trans_stopped_vel)) 
    {
      if ( ! stopWithAccLimits(
          global_pose,
          robot_vel,
          cmd_vel,
          acc_lim,
          sim_period,
          obstacle_check)) 
      {
        ROS_INFO("Error when stopping.");
        return false;
      }
      ROS_DEBUG("Stopping...");
    }
    //if we're stopped... then we want to rotate to goal
    else 
    {
      //set this so that we know its OK to be moving
      rotating_to_goal_ = true;
      if ( ! rotateToGoal(
          global_pose,
          robot_vel,
          goal_th,
          cmd_vel,
          acc_lim,
          sim_period,
          limits,
          obstacle_check)) {
        ROS_INFO("Error when rotating.");
        return false;
      }
      ROS_DEBUG("Rotating...");
    }
  }

  return true;

}


} /* namespace base_local_planner */
