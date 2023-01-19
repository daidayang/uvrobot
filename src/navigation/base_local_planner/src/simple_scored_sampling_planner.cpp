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

#include <base_local_planner/simple_scored_sampling_planner.h>

#include <ros/console.h>

namespace base_local_planner {
  
  SimpleScoredSamplingPlanner::SimpleScoredSamplingPlanner(std::vector<TrajectorySampleGenerator*> gen_list, std::vector<TrajectoryCostFunction*>& critics, int max_samples) 
  {
    max_samples_ = max_samples;
    gen_list_ = gen_list;
    critics_ = critics;
  }

  //计算轨迹得分
  double SimpleScoredSamplingPlanner::scoreTrajectory(Trajectory& traj, double best_traj_cost) 
  {
    double traj_cost = 0;
    int gen_id = 0;
    for(std::vector<TrajectoryCostFunction*>::iterator score_function = critics_.begin(); score_function != critics_.end(); ++score_function) 
    {
      TrajectoryCostFunction* score_function_p = *score_function;
      if (score_function_p->getScale() == 0) 
      {
        continue;
      }
      //call ObstacleCostFunction::scoreTrajectory function
      double cost = score_function_p->scoreTrajectory(traj);
      if (cost < 0) 
      {
        ROS_DEBUG("Velocity %.3lf, %.3lf, %.3lf discarded by cost function  %d with cost: %f", traj.xv_, traj.yv_, traj.thetav_, gen_id, cost);
        traj_cost = cost;
        break;
      }

      if (cost != 0) 
      {
        cost *= score_function_p->getScale();
      }

      traj_cost += cost;

      if (best_traj_cost > 0) 
      {
        // since we keep adding positives, once we are worse than the best, we will stay worse
        if (traj_cost > best_traj_cost) 
        {
          break;
        }
      }
      gen_id ++;
    }


    return traj_cost;
  }

  //通过模拟采样获取最优轨迹
  bool SimpleScoredSamplingPlanner::findBestTrajectory(Trajectory& traj, std::vector<Trajectory>* all_explored) 
  {

    ROS_INFO("In local planner,run DWAPlanner::findBestTrajectory ..................");

    Trajectory loop_traj;
    Trajectory best_traj;
    double loop_traj_cost, best_traj_cost = -1;

    bool gen_success;
    int count, count_valid;

    for (std::vector<TrajectoryCostFunction*>::iterator loop_critic = critics_.begin(); loop_critic != critics_.end(); ++loop_critic) 
    {
      TrajectoryCostFunction* loop_critic_p = *loop_critic;
      if (loop_critic_p->prepare() == false) 
      {
        ROS_WARN("A scoring function failed to prepare");
        return false;
      }
    }

    //给采样轨迹逐一评分,这里只用一个轨迹生成器gen_list_,一旦生成功,则结束for循环
    int loop_count = 0;
    for (std::vector<TrajectorySampleGenerator*>::iterator loop_gen = gen_list_.begin(); loop_gen != gen_list_.end(); ++loop_gen) 
    {
      loop_count++;
      ROS_INFO("Trajectory Sample Generator List: %d",loop_count);
      count = 0;
      count_valid = 0;
      TrajectorySampleGenerator* gen_ = *loop_gen;
      while (gen_->hasMoreTrajectories()) 
      {
        
        //循环,遍历所有的采样工况
        //获取下一个采样轨迹
        gen_success = gen_->nextTrajectory(loop_traj);
        if (gen_success == false) 
        {
          // TODO use this for debugging
          continue;
        }
        /////////////////////////////////////////////////////////////////////
        
        // //剔除线速度为0,而角速度不为0的工况，即robot不能只旋转
        // if(std::fabs(loop_traj.thetav_) > 0.01 && loop_traj.xv_ < 0.1)
        // {
        //   continue;
        // }

        /////////////////////////////////////////////////////////////////////
        loop_traj.ids_ = count;
        //给该采样轨迹进行评分
        loop_traj_cost = scoreTrajectory(loop_traj, best_traj_cost);

        if (all_explored != NULL) 
        {
          loop_traj.cost_ = loop_traj_cost;
          all_explored->push_back(loop_traj);
        }
        //上面是轨迹生成，下面是对轨迹进行评分
        
        if (loop_traj_cost >= 0) 
        {
          count_valid++;
          if (best_traj_cost < 0 || loop_traj_cost < best_traj_cost) 
          {
            best_traj_cost = loop_traj_cost;
            best_traj = loop_traj;
          }
        }

        //ROS_INFO("sample trajectory: ids %d, cost %.3f, sample velo:[xv %.3f, yv %.3f, thav %.3f]",loop_traj.ids_,loop_traj_cost,loop_traj.xv_,loop_traj.yv_,loop_traj.thetav_);
        count++;
        if (max_samples_ > 0 && count >= max_samples_) 
        {
          break;
        }        
      }

      ////////////////////////////////////////////////////////////
      //当robot与goal成180度夹角时，local path的点只有1个或者距离相近的几个，
      //这种情况下，计算local path的cost时，线速度一样，则cost一样，与角速度无关，
      //因此这种情况下，我们选择角度最大组合，作为最后的值
      std::vector<Trajectory> all_explored_samecost;
      for(int i = 0;i<all_explored->size();++i)
      {
        if(best_traj_cost >= 0 && fabs((*all_explored)[i].cost_ - best_traj_cost)<0.01)
        {
          all_explored_samecost.push_back((*all_explored)[i]);
        }
      }
      //筛选出cost相等的组合后，再选角度最大的
      ROS_INFO("same cost trajectory count: %d, all trajectory count: %d ", all_explored_samecost.size(),all_explored->size());
      ROS_INFO("first the best sample trajectory: ids %d, cost %.3f, sample velo:[xv %.3f, yv %.3f, thav %.3f]",best_traj.ids_,best_traj_cost,best_traj.xv_,best_traj.yv_,best_traj.thetav_);
      if(all_explored_samecost.size()>1)//1个的话是它自己
      {
        //先选择最大的
        for(int i = 0;i<all_explored_samecost.size();++i)
        {
          if(all_explored_samecost[i].thetav_ > best_traj.thetav_)
          {
            best_traj = all_explored_samecost[i];
          }
        }


        //如果第一遍选择后，最大的加速度也是负值,则选择最小的值
        if(best_traj.thetav_ < 0.0)
        {
          for(int i = 0;i<all_explored_samecost.size();++i)
          {
            if(all_explored_samecost[i].thetav_ < best_traj.thetav_)
            {
              best_traj = all_explored_samecost[i];
            }
          }
        }
      }
      ///////////////////////////////////////////////////////////

      if (best_traj_cost >= 0) 
      {
        traj.xv_ = best_traj.xv_;
        traj.yv_ = best_traj.yv_;
        traj.thetav_ = best_traj.thetav_;

        
        ROS_INFO(" last the best sample trajectory: ids %d, cost %.3f, sample velo:[xv %.3f, yv %.3f, thav %.3f]",best_traj.ids_,best_traj_cost,best_traj.xv_,best_traj.yv_,best_traj.thetav_);
        traj.cost_ = best_traj_cost;
        traj.resetPoints();
        double px, py, pth;
        for (unsigned int i = 0; i < best_traj.getPointsSize(); i++) 
        {
          best_traj.getPoint(i, px, py, pth);
          traj.addPoint(px, py, pth);
        }
      }
      ROS_DEBUG("Evaluated %d trajectories, found %d valid", count, count_valid);
      if (best_traj_cost >= 0) 
      {
        // do not try fallback generators
        break;
      }
    }
    return best_traj_cost >= 0;
  }

  
}// namespace
