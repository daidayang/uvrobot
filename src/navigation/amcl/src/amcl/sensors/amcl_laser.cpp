/*
 *  Player - One Hell of a Robot Server
 *  Copyright (C) 2000  Brian Gerkey   &  Kasper Stoy
 *                      gerkey@usc.edu    kaspers@robotics.usc.edu
 *
 *  This library is free software; you can redistribute it and/or
 *  modify it under the terms of the GNU Lesser General Public
 *  License as published by the Free Software Foundation; either
 *  version 2.1 of the License, or (at your option) any later version.
 *
 *  This library is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 *  Lesser General Public License for more details.
 *
 *  You should have received a copy of the GNU Lesser General Public
 *  License along with this library; if not, write to the Free Software
 *  Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
 *
 */
///////////////////////////////////////////////////////////////////////////
//
// Desc: AMCL laser routines
// Author: Andrew Howard
// Date: 6 Feb 2003
// CVS: $Id: amcl_laser.cc 7057 2008-10-02 00:44:06Z gbiggs $
//
///////////////////////////////////////////////////////////////////////////

#include <sys/types.h> // required by Darwin
#include <math.h>
#include <stdlib.h>
#include <assert.h>
#ifdef HAVE_UNISTD_H
#include <unistd.h>
#endif

#include "amcl/sensors/amcl_laser.h"
#include "ros/ros.h"

using namespace amcl;

////////////////////////////////////////////////////////////////////////////////
// Default constructor
AMCLLaser::AMCLLaser(size_t max_beams, map_t* map) : AMCLSensor(), 
						     max_samples(0), max_obs(0), 
						     temp_obs(NULL)
{
  this->time = 0.0;

  this->max_beams = max_beams;
  this->map = map;

  return;
}

AMCLLaser::~AMCLLaser()
{
  if(temp_obs){
	for(int k=0; k < max_samples; k++){
	  delete [] temp_obs[k];
	}
	delete []temp_obs; 
  }
}

void 
AMCLLaser::SetModelBeam(double z_hit,
                        double z_short,
                        double z_max,
                        double z_rand,
                        double sigma_hit,
                        double lambda_short,
                        double chi_outlier)
{
  this->model_type = LASER_MODEL_BEAM;
  this->z_hit = z_hit;
  this->z_short = z_short;
  this->z_max = z_max;
  this->z_rand = z_rand;
  this->sigma_hit = sigma_hit;
  this->lambda_short = lambda_short;
  this->chi_outlier = chi_outlier;
}

void 
AMCLLaser::SetModelLikelihoodField(double z_hit,
                                   double z_rand,
                                   double sigma_hit,
                                   double max_occ_dist)
{
  this->model_type = LASER_MODEL_LIKELIHOOD_FIELD;
  this->z_hit = z_hit;
  this->z_rand = z_rand;
  this->sigma_hit = sigma_hit;

  map_update_cspace(this->map, max_occ_dist);
}

void 
AMCLLaser::SetModelLikelihoodFieldProb(double z_hit,
				       double z_rand,
				       double sigma_hit,
				       double max_occ_dist,
				       bool do_beamskip,
				       double beam_skip_distance,
				       double beam_skip_threshold, 
				       double beam_skip_error_threshold)
{
  this->model_type = LASER_MODEL_LIKELIHOOD_FIELD_PROB;
  this->z_hit = z_hit;
  this->z_rand = z_rand;
  this->sigma_hit = sigma_hit;
  this->do_beamskip = do_beamskip;
  this->beam_skip_distance = beam_skip_distance;
  this->beam_skip_threshold = beam_skip_threshold;
  this->beam_skip_error_threshold = beam_skip_error_threshold;
  map_update_cspace(this->map, max_occ_dist);
}


////////////////////////////////////////////////////////////////////////////////
// Apply the laser sensor model
bool AMCLLaser::UpdateSensor(pf_t *pf, AMCLSensorData *data)
{
  if (this->max_beams < 2)
    return false;

  // Apply the laser sensor model
  if(this->model_type == LASER_MODEL_BEAM)
  {
    pf_update_sensor(pf, (pf_sensor_model_fn_t) BeamModel, data);
  }
  else if(this->model_type == LASER_MODEL_LIKELIHOOD_FIELD)
  {
    ROS_INFO("LASER_MODEL_LIKELIHOOD_FIELD......");
    pf_update_sensor(pf, (pf_sensor_model_fn_t) LikelihoodFieldModel, data);
  }
  else if(this->model_type == LASER_MODEL_LIKELIHOOD_FIELD_PROB)
  {
    ROS_INFO("LASER_MODEL_LIKELIHOOD_FIELD_PROB......");
    pf_update_sensor(pf, (pf_sensor_model_fn_t) LikelihoodFieldModelProb, data); 
  }
  else
  {
    pf_update_sensor(pf, (pf_sensor_model_fn_t) BeamModel, data);
  }

  return true;
}


////////////////////////////////////////////////////////////////////////////////
// Determine the probability for the given pose
double AMCLLaser::BeamModel(AMCLLaserData *data, pf_sample_set_t* set)
{
  AMCLLaser *self;
  int i, j, step;
  double z, pz;
  double p;
  double map_range;
  double obs_range, obs_bearing;
  double total_weight;
  pf_sample_t *sample;
  pf_vector_t pose;

  self = (AMCLLaser*) data->sensor;

  total_weight = 0.0;

  // Compute the sample weights
  for (j = 0; j < set->sample_count; j++)
  {
    sample = set->samples + j;
    pose = sample->pose;

    // Take account of the laser pose relative to the robot
    pose = pf_vector_coord_add(self->laser_pose, pose);

    p = 1.0;

    step = (data->range_count - 1) / (self->max_beams - 1);
    for (i = 0; i < data->range_count; i += step)
    {
      obs_range = data->ranges[i][0];
      obs_bearing = data->ranges[i][1];

      // Compute the range according to the map
      map_range = map_calc_range(self->map, pose.v[0], pose.v[1],
                                 pose.v[2] + obs_bearing, data->range_max);
      pz = 0.0;

      // Part 1: good, but noisy, hit
      z = obs_range - map_range;
      pz += self->z_hit * exp(-(z * z) / (2 * self->sigma_hit * self->sigma_hit));

      // Part 2: short reading from unexpected obstacle (e.g., a person)
      if(z < 0)
        pz += self->z_short * self->lambda_short * exp(-self->lambda_short*obs_range);

      // Part 3: Failure to detect obstacle, reported as max-range
      if(obs_range == data->range_max)
        pz += self->z_max * 1.0;

      // Part 4: Random measurements
      if(obs_range < data->range_max)
        pz += self->z_rand * 1.0/data->range_max;

      // TODO: outlier rejection for short readings

      assert(pz <= 1.0);
      assert(pz >= 0.0);
      //      p *= pz;
      // here we have an ad-hoc weighting scheme for combining beam probs
      // works well, though...
      p += pz*pz*pz;
    }

    sample->weight *= p;
    total_weight += sample->weight;
  }

  return(total_weight);
}

/*
概率机器人中的似然域模型缺乏一个合理的物理解释，它是一种“特设（ad hoc）”算法。不必计算相对于任何有意义的传感器物理生成模型的条件概率。
这种方法在实践中运行效果良好。即使在混乱的空间，得到的后验也更光滑，同时计算更高效。 主要思想就是首先将传感器扫描的终点, 映射到地图的全局坐标空间。
然后，假定三种操作和不确定性的来源。最后再利用混合权值对三种分布进行综合，得到最后的置信度。这三个权值参数可以通过手动调节或者利用最大似然估计得到。
*/
//该模型的作用是：遍历粒子更新概率，计算所有粒子概率的总和
//函数中用的噪声类型 1.小的测量噪声(高斯) 2 由于未检测到对象引起的误差（检测失败）3 随机意外噪声
//意外对象引起的误差这种情况下测量距离的概率用指数分布来描述，这里似然域未用到。
double AMCLLaser::LikelihoodFieldModel(AMCLLaserData *data, pf_sample_set_t* set)
{
  AMCLLaser *self;
  int i, j, step;
  double z, pz;
  double p;
  double obs_range, obs_bearing;
  double total_weight;
  pf_sample_t *sample;
  pf_vector_t pose;
  pf_vector_t hit;

  self = (AMCLLaser*) data->sensor;

  total_weight = 0.0;

  // Compute the sample weights
  for (j = 0; j < set->sample_count; j++)
  {
    sample = set->samples + j;
    pose = sample->pose;//遍历每个粒子，这是粒子对应的位姿，是经运动模型更新后,先验位姿
    //ROS_INFO("pose LikelihoodFieldModel: %f, %f, %f",pose.v[0],pose.v[1],pose.v[3]);
    
    // Take account of the laser pose relative to the robot
    //这个应该是通过机器人与全局坐标系的位姿（每个粒子的位姿），计算激光雷达相对于全局坐标系的位姿。
    //方便后续将激光雷达扫描的终点转换到全局坐标系
    //点进去pf_vector_coord_add这个函数，
    //b对应的就是《概率机器人》P128,6.4第一个公式中的机器人在t时刻的位姿，a代表“与机器人固连的传感器局部坐标系位置”
    pose = pf_vector_coord_add(self->laser_pose, pose);//激光雷达的位姿转换到世界坐标系

    p = 1.0;

    // Pre-compute a couple of things
    //高斯误差  、随机偶然误差   还有最远点的概率
    double z_hit_denom = 2 * self->sigma_hit * self->sigma_hit;//测量噪声的方差
    double z_rand_mult = 1.0/data->range_max;//无法解释的随机测量的分母

    //因为时间问题，并不是全部点都进行似然计算的，这里只是间隔地选点，我这里设置的是一共选择60个点～～～

    step = (data->range_count-1)/(self->max_beams-1);//计算步长 只取一组数据中的max_beams个点

    // Step size must be at least 1
    if(step < 1)
      step = 1;

    for (i = 0; i < data->range_count; i += step) //遍历激光点数，间隔选点，因为计算时间原因
    {
      obs_range = data->ranges[i][0];  //观测到的距离
      obs_bearing = data->ranges[i][1]; //角度

      // This model ignores max range readings
      //如果obs_range > data->range_max 则结束跳过以下更新（即：如果测距传感器输出了最大值z(k,t)= Z_max，则这些坐标在物理世界没有任何意义
	    //似然域测量模型简单地将大于最大距离的读数丢弃）
      if(obs_range >= data->range_max)
        continue;

      // Check for NaN
      if(obs_range != obs_range)
        continue;

      pz = 0.0;
      

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
      //(针对采集的同组激光点,粒子越远离激光原点,与采集的该组激光点的距离越大,最终计算的粒子权重越小
      //因为采集的激光点相对激光原点的位置已定,当粒子位置偏离激光原点越远,越多激光点跑到有效地图外(有效地图区域是粒子位置确定的),跑到有效地图外,则该激光取最大距离,距离越大,概率越小)
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////      
      // Compute the endpoint of the beam计算激光雷达点最远端的世界坐标
      //将激光点映射到地图中
      hit.v[0] = pose.v[0] + obs_range * cos(pose.v[2] + obs_bearing);
      hit.v[1] = pose.v[1] + obs_range * sin(pose.v[2] + obs_bearing);

      // Convert to map grid coords.
      ////将点编入格栅地图中
      int mi, mj;
      mi = MAP_GXWX(self->map, hit.v[0]);//将点转换到格栅地图中
      mj = MAP_GYWY(self->map, hit.v[1]);
      

      //当该点位于边界时，看做最大障碍物距离，否则取最小障碍物距离，当最小距离小于设定的波束跳跃距离时，可用点+1，同时确定高斯噪声。
      
      // Part 1: Get distance from the hit to closest obstacle.
      // Off-map penalized as max distance

      if(!MAP_VALID(self->map, mi, mj))//激光点越界,地图外看作最大障碍物距离
        z = self->map->max_occ_dist;
      else
        z = self->map->cells[MAP_INDEX(self->map,mi,mj)].occ_dist;//当前激光点与最近障碍物栅格之间的距离
      
      //书本算法流程公式
      // Gaussian model
      // NOTE: this should have a normalization of 1/(sqrt(2pi)*sigma)
      pz += self->z_hit * exp(-(z * z) / z_hit_denom);//高斯模型
      
      
      // Part 2: random measurements
      pz += self->z_rand * z_rand_mult;//在加上随机模型，检测概率

      // TODO: outlier rejection for short readings

      assert(pz <= 1.0);
      assert(pz >= 0.0);
      // p *= pz; (按教材上,应该是此式子才对)
      // here we have an ad-hoc weighting scheme for combining beam probs
      // works well, though...
      p += pz*pz*pz;

      //z 越大，pz 越小，粒子权重越小

    }//每个粒子的所有激光雷达点循环

    sample->weight *= p;
    total_weight += sample->weight;
  }//粒子循环

  return(total_weight);
}

double AMCLLaser::LikelihoodFieldModelProb(AMCLLaserData *data, pf_sample_set_t* set)
{
  AMCLLaser *self;
  int i, j, step;
  double z, pz;
  double log_p;
  double obs_range, obs_bearing;
  double total_weight;
  pf_sample_t *sample;
  pf_vector_t pose;
  pf_vector_t hit;

  self = (AMCLLaser*) data->sensor;

  total_weight = 0.0;

  //激光个数
  step = ceil((data->range_count) / static_cast<double>(self->max_beams)); 
  
  // Step size must be at least 1
  if(step < 1)
    step = 1;

  // Pre-compute a couple of things
  double z_hit_denom = 2 * self->sigma_hit * self->sigma_hit;
  double z_rand_mult = 1.0/data->range_max;

  double max_dist_prob = exp(-(self->map->max_occ_dist * self->map->max_occ_dist) / z_hit_denom);

  //Beam skipping - ignores beams for which a majoirty of particles do not agree with the map
  //prevents correct particles from getting down weighted because of unexpected obstacles 
  //such as humans 

  bool do_beamskip = self->do_beamskip;
  double beam_skip_distance = self->beam_skip_distance;
  double beam_skip_threshold = self->beam_skip_threshold;
  
  //we only do beam skipping if the filter has converged 
  if(do_beamskip && !set->converged)
  {
    do_beamskip = false;
  }

  //we need a count the no of particles for which the beam agreed with the map 
  int *obs_count = new int[self->max_beams]();

  //we also need a mask of which observations to integrate (to decide which beams to integrate to all particles) 
  bool *obs_mask = new bool[self->max_beams]();
  
  int beam_ind = 0;
  
  //realloc indicates if we need to reallocate the temp data structure needed to do beamskipping 
  bool realloc = false; 

  if(do_beamskip){
    if(self->max_obs < self->max_beams){
      realloc = true;
    }

    if(self->max_samples < set->sample_count){
      realloc = true;
    }

    if(realloc){
      self->reallocTempData(set->sample_count, self->max_beams);     
      fprintf(stderr, "Reallocing temp weights %d - %d\n", self->max_samples, self->max_obs);
    }
  }

  // Compute the sample weights
  for (j = 0; j < set->sample_count; j++)
  {
    sample = set->samples + j;
    pose = sample->pose;

    // Take account of the laser pose relative to the robot
    pose = pf_vector_coord_add(self->laser_pose, pose);

    log_p = 0;
    
    beam_ind = 0;
    
    for (i = 0; i < data->range_count; i += step, beam_ind++)
    {
      obs_range = data->ranges[i][0];
      obs_bearing = data->ranges[i][1];

      // This model ignores max range readings
      if(obs_range >= data->range_max){
        continue;
      }

      // Check for NaN
      if(obs_range != obs_range){
        continue;
      }

      pz = 0.0;

      // Compute the endpoint of the beam
      hit.v[0] = pose.v[0] + obs_range * cos(pose.v[2] + obs_bearing);
      hit.v[1] = pose.v[1] + obs_range * sin(pose.v[2] + obs_bearing);

      // Convert to map grid coords.
      int mi, mj;
      mi = MAP_GXWX(self->map, hit.v[0]);
      mj = MAP_GYWY(self->map, hit.v[1]);
      
      // Part 1: Get distance from the hit to closest obstacle.
      // Off-map penalized as max distance
      
      if(!MAP_VALID(self->map, mi, mj))
      {
	      pz += self->z_hit * max_dist_prob;
      }
      else
      {
        z = self->map->cells[MAP_INDEX(self->map,mi,mj)].occ_dist;
        if(z < beam_skip_distance)
        {
          obs_count[beam_ind] += 1;
        }
        pz += self->z_hit * exp(-(z * z) / z_hit_denom);
      }
       
      // Gaussian model
      // NOTE: this should have a normalization of 1/(sqrt(2pi)*sigma)
      
      // Part 2: random measurements
      pz += self->z_rand * z_rand_mult;

      assert(pz <= 1.0); 
      assert(pz >= 0.0);

      // TODO: outlier rejection for short readings
            
      if(!do_beamskip)
      {
	      log_p += log(pz);
      }
      else
      {
	      self->temp_obs[j][beam_ind] = pz; 
      }
    }

    if(!do_beamskip)
    {
      sample->weight *= exp(log_p);
      total_weight += sample->weight;
    }
  }
  
  if(do_beamskip)
  {
    int skipped_beam_count = 0; 
    for (beam_ind = 0; beam_ind < self->max_beams; beam_ind++)
    {
      if((obs_count[beam_ind] / static_cast<double>(set->sample_count)) > beam_skip_threshold)
      {
	      obs_mask[beam_ind] = true;
      }
      else
      {
	      obs_mask[beam_ind] = false;
	      skipped_beam_count++; 
      }
    }

    //we check if there is at least a critical number of beams that agreed with the map 
    //otherwise it probably indicates that the filter converged to a wrong solution
    //if that's the case we integrate all the beams and hope the filter might converge to 
    //the right solution
    bool error = false; 

    if(skipped_beam_count >= (beam_ind * self->beam_skip_error_threshold))
    {
      fprintf(stderr, "Over %f%% of the observations were not in the map - pf may have converged to wrong pose - integrating all observations\n", (100 * self->beam_skip_error_threshold));
      error = true; 
    }

    for (j = 0; j < set->sample_count; j++)
    {
      sample = set->samples + j;
      pose = sample->pose;

      log_p = 0;

      for (beam_ind = 0; beam_ind < self->max_beams; beam_ind++)
      {
        if(error || obs_mask[beam_ind])
        {
          log_p += log(self->temp_obs[j][beam_ind]);
        }
      }
      
      sample->weight *= exp(log_p);
      
      total_weight += sample->weight;
    }      
  }

  delete [] obs_count; 
  delete [] obs_mask;
  return(total_weight);
}

void AMCLLaser::reallocTempData(int new_max_samples, int new_max_obs){
  if(temp_obs){
    for(int k=0; k < max_samples; k++){
      delete [] temp_obs[k];
    }
    delete []temp_obs; 
  }
  max_obs = new_max_obs; 
  max_samples = fmax(max_samples, new_max_samples); 

  temp_obs = new double*[max_samples]();
  for(int k=0; k < max_samples; k++){
    temp_obs[k] = new double[max_obs]();
  }
}
