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
 * Author: TKruse
 *********************************************************************/

#include <base_local_planner/simple_trajectory_generator.h>

#include <cmath>

#include <base_local_planner/velocity_iterator.h>

namespace base_local_planner {

void SimpleTrajectoryGenerator::initialise(
    const Eigen::Vector3f& pos,
    const Eigen::Vector3f& vel,
    const Eigen::Vector3f& goal,
    base_local_planner::LocalPlannerLimits* limits,
    const Eigen::Vector3f& vsamples,
    std::vector<Eigen::Vector3f> additional_samples,
    bool discretize_by_time) 
{
  initialise(pos, vel, goal, limits, vsamples, discretize_by_time);
  // add static samples if any
  sample_params_.insert(sample_params_.end(), additional_samples.begin(), additional_samples.end());
}


void SimpleTrajectoryGenerator::initialise(
    const Eigen::Vector3f& pos,
    const Eigen::Vector3f& vel,
    const Eigen::Vector3f& goal,
    base_local_planner::LocalPlannerLimits* limits,
    const Eigen::Vector3f& vsamples,
    bool discretize_by_time) 
  {
  /*
   * We actually generate all velocity sample vectors here, from which to generate trajectories later on
   */
  double max_vel_th = limits->max_vel_theta;

  double min_vel_th = -1.0 * max_vel_th;

  discretize_by_time_ = discretize_by_time;

  Eigen::Vector3f acc_lim = limits->getAccLimits();

  pos_ = pos;
  vel_ = vel;
  goal_ = goal;
  limits_ = limits;
  next_sample_index_ = 0;
  sample_params_.clear();

  double min_vel_x = limits->min_vel_x;
  double max_vel_x = limits->max_vel_x;
  double min_vel_y = limits->min_vel_y;
  double max_vel_y = limits->max_vel_y;

  
  // if sampling number is zero in any dimension, we don't generate samples generically
  //3个采样参数及速度控制值
  ROS_INFO("sim samples: (%.3f, %.3f %.3f), limit[min,max] xv:(%.3f,%.3f), yv:(%.3f,%.3f), thv:(%.3f,%.3f), current vel: x %.3f,y %.3f, th %.3f",
              vsamples[0],vsamples[1],vsamples[2],min_vel_x,max_vel_x,min_vel_y,max_vel_y,min_vel_th,max_vel_th,
              vel[0],vel[1],vel[1]);

  if (vsamples[0] * vsamples[1] * vsamples[2] > 0) 
  {
    //compute the feasible velocity space based on the rate at which we run
    Eigen::Vector3f max_vel = Eigen::Vector3f::Zero();
    Eigen::Vector3f min_vel = Eigen::Vector3f::Zero();

    if ( ! use_dwa_) 
    {
      // there is no point in overshooting the goal, and it also may break the
      // robot behavior, so we limit the velocities to those that do not overshoot in sim_time
      double dist = hypot(goal[0] - pos[0], goal[1] - pos[1]);
      max_vel_x = std::max(std::min(max_vel_x, dist / sim_time_), min_vel_x);
      max_vel_y = std::max(std::min(max_vel_y, dist / sim_time_), min_vel_y);

      // if we use continous acceleration, we can sample the max velocity we can reach in sim_time_
      max_vel[0] = std::min(max_vel_x, vel[0] + acc_lim[0] * sim_time_);
      max_vel[1] = std::min(max_vel_y, vel[1] + acc_lim[1] * sim_time_);
      max_vel[2] = std::min(max_vel_th, vel[2] + acc_lim[2] * sim_time_);

      min_vel[0] = std::max(min_vel_x, vel[0] - acc_lim[0] * sim_time_);
      min_vel[1] = std::max(min_vel_y, vel[1] - acc_lim[1] * sim_time_);
      min_vel[2] = std::max(min_vel_th, vel[2] - acc_lim[2] * sim_time_);
    } 
    else 
    {
      // with dwa do not accelerate beyond the first step, we only sample within velocities we reach in sim_period
      max_vel[0] = std::min(max_vel_x, vel[0] + acc_lim[0] * sim_period_);
      max_vel[1] = std::min(max_vel_y, vel[1] + acc_lim[1] * sim_period_);
      max_vel[2] = std::min(max_vel_th, vel[2] + acc_lim[2] * sim_period_);

      min_vel[0] = std::max(min_vel_x, vel[0] - acc_lim[0] * sim_period_);
      min_vel[1] = std::max(min_vel_y, vel[1] - acc_lim[1] * sim_period_);
      min_vel[2] = std::max(min_vel_th, vel[2] - acc_lim[2] * sim_period_);
    }

    Eigen::Vector3f vel_samp = Eigen::Vector3f::Zero();

    // ROS_ERROR("x_it: %.3f, %.3f, %.3f",min_vel[0], max_vel[0], vsamples[0]);
    // ROS_ERROR("y_it: %.3f, %.3f, %.3f",min_vel[1], max_vel[1], vsamples[1]);
    // ROS_ERROR("th_it: %.3f, %.3f, %.3f",min_vel[2], max_vel[2], vsamples[2]);

    VelocityIterator x_it(min_vel[0], max_vel[0], vsamples[0]);
    VelocityIterator y_it(min_vel[1], max_vel[1], vsamples[1]);
    VelocityIterator th_it(min_vel[2], max_vel[2], vsamples[2]);

    for(; !x_it.isFinished(); x_it++) 
    {
      vel_samp[0] = x_it.getVelocity();
      for(; !y_it.isFinished(); y_it++) 
      {
        vel_samp[1] = y_it.getVelocity();
        for(; !th_it.isFinished(); th_it++) 
        {
          vel_samp[2] = th_it.getVelocity();
          //ROS_DEBUG("Sample %f, %f, %f", vel_samp[0], vel_samp[1], vel_samp[2]);
          sample_params_.push_back(vel_samp);
        }
        th_it.reset();
      }
      y_it.reset();
    }


  }
}

void SimpleTrajectoryGenerator::setParameters(
    double sim_time,
    double sim_granularity,
    double angular_sim_granularity,
    bool use_dwa,
    double sim_period) 
{
  sim_time_ = sim_time;
  sim_granularity_ = sim_granularity;
  angular_sim_granularity_ = angular_sim_granularity;
  use_dwa_ = use_dwa;
  continued_acceleration_ = ! use_dwa_;
  sim_period_ = sim_period;

  ROS_WARN("config.sim_time: %f",sim_time_);
  ROS_WARN("config.sim_granularity: %f",sim_granularity_);
  ROS_WARN("config.angular_sim_granularity: %f",angular_sim_granularity_);
  ROS_WARN("config.use_dwa: %d",use_dwa_?1:0);
  ROS_WARN("continued_acceleration_: %d",!use_dwa_?1:0);
  ROS_WARN("sim_period_: %f",sim_period_);

}

/**
 * Whether this generator can create more trajectories
 */
bool SimpleTrajectoryGenerator::hasMoreTrajectories() 
{
  return next_sample_index_ < sample_params_.size();
}

/**
 * Create and return the next sample trajectory
 */
bool SimpleTrajectoryGenerator::nextTrajectory(Trajectory &comp_traj) 
{
  //ROS_INFO("hasMoreTrajectories...........");
  bool result = false;
  if (hasMoreTrajectories()) 
  {
    if (generateTrajectory(
        pos_,
        vel_,
        sample_params_[next_sample_index_],
        comp_traj)) 
    {
      result = true;
    }
  }
  next_sample_index_++;
  return result;
}

/**
 * @param pos current position of robot
 * @param vel desired velocity for sampling
 */
bool SimpleTrajectoryGenerator::generateTrajectory(
      Eigen::Vector3f pos,
      Eigen::Vector3f vel,
      Eigen::Vector3f sample_target_vel,
      base_local_planner::Trajectory& traj) 
{
  

  double vmag = hypot(sample_target_vel[0], sample_target_vel[1]);

  //ROS_ERROR("Delta velocity: %.3f",vmag);

  double eps = 1e-4;
  traj.cost_   = -1.0; // placed here in case we return early
  //trajectory might be reused so we'll make sure to reset it
  traj.resetPoints();

  // make sure that the robot would at least be moving with one of
  // the required minimum velocities for translation and rotation (if set)
  if ((limits_->min_vel_trans >= 0 && vmag + eps < limits_->min_vel_trans) &&
      (limits_->min_vel_theta >= 0 && fabs(sample_target_vel[2]) + eps < limits_->min_vel_theta)) 
  {
    return false;
  }
  // make sure we do not exceed max diagonal (x+y) translational velocity (if set)
  if (limits_->max_vel_trans >=0 && vmag - eps > limits_->max_vel_trans) 
  {
    return false;
  }

  int num_steps;
  if (discretize_by_time_) 
  {
    num_steps = ceil(sim_time_ / sim_granularity_);
  } 
  else 
  {
    //compute the number of steps we must take along this trajectory to be "safe"
    double sim_time_distance = vmag * sim_time_; //the distance the robot would travel in sim_time if it did not change velocity
    double sim_time_angle = fabs(sample_target_vel[2]) * sim_time_; // the angle the robot would rotate in sim_time
    num_steps =
        ceil(std::max(sim_time_distance / sim_granularity_,
            sim_time_angle    / angular_sim_granularity_));
    
  }

  if (num_steps == 0) 
  {
    return false;
  }

  //compute a timestep
  double dt = sim_time_ / num_steps;
  traj.time_delta_ = dt;

  Eigen::Vector3f loop_vel;
  if (continued_acceleration_) 
  {
    // assuming the velocity of the first cycle is the one we want to store in the trajectory object
    loop_vel = computeNewVelocities(sample_target_vel, vel, limits_->getAccLimits(), dt);
    traj.xv_     = loop_vel[0];
    traj.yv_     = loop_vel[1];
    traj.thetav_ = loop_vel[2];
  } 
  else 
  {
    // assuming sample_vel is our target velocity within acc limits for one timestep
    loop_vel =   sample_target_vel;
    traj.xv_     = sample_target_vel[0];
    traj.yv_     = sample_target_vel[1];
    traj.thetav_ = sample_target_vel[2];

  }

  //ROS_ERROR("sample_target_vel: %.3f, %.3f %.3f, dt: %.3f, num_steps %d",sample_target_vel[0],sample_target_vel[1],sample_target_vel[2],dt,num_steps);

  //simulate the trajectory and check for collisions, updating costs along the way
  int addpoints_count = 0;
  for (int i = 0; i < num_steps; ++i) 
  {
    addpoints_count++;

    //add the point to the trajectory so we can draw it later if we want
    if(traj.getPointsSize() == 0)
    {
      traj.addPoint(pos[0], pos[1], pos[2]);
    }else
    {
      double x,y,th;
      traj.getEndpoint(x, y,th);
      double distance = sqrt(pow(x - pos[0],2.0) + pow(y - pos[1],2.0));
      
      if(distance > 0.01)
      {
        traj.addPoint(pos[0], pos[1], pos[2]);
      }
    }
    
      

    if (continued_acceleration_) 
    {
      //calculate velocities
      loop_vel = computeNewVelocities(sample_target_vel, loop_vel, limits_->getAccLimits(), dt);
      //ROS_WARN_NAMED("Generator", "Flag: %d, Loop_Vel %f, %f, %f", continued_acceleration_, loop_vel[0], loop_vel[1], loop_vel[2]);
    }

    //update the position of the robot using the velocities passed in
    pos = computeNewPositions(pos, loop_vel, dt);

  } // end for simulation steps
  
  //判断轨迹长度
  if(traj.getPointsSize() > 1)
  {
    double first_x = 0,first_y = 0,fisrt_th = 0;
    double end_x = 0,end_y = 0,end_th = 0;

    traj.getPoint(0,first_x,first_y,fisrt_th);
    traj.getEndpoint(end_x,end_y,end_th);

    double length = sqrt(pow(first_x - end_x,2.0) + pow(first_y - end_y,2.0));

    //计算第一个点到目标的距离
    double distance_to_goal = sqrt(pow(first_x - goal_[0],2.0) + pow(first_y - goal_[1],2.0));
    //根据当前位置到目标的距离,确定local path最短限制，如果小于此值，local path就保留一个点
    //这样保证，robot方向与目标方向尽量一致时，robot才开始移动，这样就尽可能的减小了robot掉头或转向时的方位(就是尽可能的让robot原点旋转，然后移动)
    double dis_limit = 0.1;
    if(distance_to_goal >0.5)
    {
      dis_limit = 0.3;
    }
    //ROS_INFO("generateTrajectory,sim path, count:%d, lenght %.3f: ",traj.getPointsSize(),length);
    //path　太短保留第一个点，这样保证只旋转(线速度为0)
    if(length < dis_limit)
    {
      traj.resetPoints();
      traj.addPoint(first_x,first_y,fisrt_th);
    }

  }



  //ROS_WARN("addpoints_count  %d",addpoints_count);

  return true; // trajectory has at least one point
}

Eigen::Vector3f SimpleTrajectoryGenerator::computeNewPositions(const Eigen::Vector3f& pos,
    const Eigen::Vector3f& vel, double dt) 
{
  Eigen::Vector3f new_pos = Eigen::Vector3f::Zero();
  new_pos[0] = pos[0] + (vel[0] * cos(pos[2]) + vel[1] * cos(M_PI_2 + pos[2])) * dt;
  new_pos[1] = pos[1] + (vel[0] * sin(pos[2]) + vel[1] * sin(M_PI_2 + pos[2])) * dt;
  new_pos[2] = pos[2] + vel[2] * dt;
  return new_pos;
}

/**
 * change vel using acceleration limits to converge towards sample_target-vel
 */
Eigen::Vector3f SimpleTrajectoryGenerator::computeNewVelocities(const Eigen::Vector3f& sample_target_vel,
    const Eigen::Vector3f& vel, Eigen::Vector3f acclimits, double dt) 
{
  Eigen::Vector3f new_vel = Eigen::Vector3f::Zero();

  for (int i = 0; i < 3; ++i) 
  {
    if (vel[i] < sample_target_vel[i]) 
    {
      new_vel[i] = std::min(double(sample_target_vel[i]), vel[i] + acclimits[i] * dt);
      ROS_ERROR("new_vel add: %d, sample_target_vel[i] %.3f, vel[i] %.3f, %.3f acclimits[i],dt %f",i,double(sample_target_vel[i]),vel[i],acclimits[i],dt);
    } else 
    {
      new_vel[i] = std::max(double(sample_target_vel[i]), vel[i] - acclimits[i] * dt);

      ROS_ERROR("new_vel add: %d, sample_target_vel[i] %.3f, vel[i] %.3f, %.3f acclimits[i],dt %f",i,double(sample_target_vel[i]),vel[i],acclimits[i],dt);
    }
  }
  return new_vel;
}

} /* namespace base_local_planner */
