/*
 * Copyright 2015-2019 Autoware Foundation. All rights reserved.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#include <ros/ros.h>
#include <ros/console.h>
#include <visualization_msgs/MarkerArray.h>
#include <std_msgs/Int32.h>
#include <tf/transform_datatypes.h>

#include <iostream>
#include <vector>
#include <string>
#include <nav_msgs/Path.h>
#include <sensor_msgs/LaserScan.h>
#include <ros/ros.h>

namespace
{
ros::Publisher g_local_mark_pub;
ros::Publisher g_global_mark_pub;
ros::Publisher g_goal_mark_pub;
ros::Publisher g_transformed_global_mark_pub;

ros::Publisher scan_pub;

std_msgs::ColorRGBA _initial_color;
std_msgs::ColorRGBA _global_color;
std_msgs::ColorRGBA g_local_color;
const double g_global_alpha = 0.2;
const double g_local_alpha = 1.0;
int _closest_waypoint = -1;

int callback_count = 0;

visualization_msgs::MarkerArray g_global_marker_array;
visualization_msgs::MarkerArray g_local_waypoints_marker_array;
visualization_msgs::MarkerArray g_goal_list_marker_array;
visualization_msgs::MarkerArray g_transformed_global_marker_array;

bool g_config_manual_detection = true;

enum class ChangeFlag : int32_t
{
  right     = 1,
  straight  = 2,
  left      = 3,

  unknown = -1,
};

typedef std::underlying_type<ChangeFlag>::type ChangeFlagInteger;

void setLifetime(double sec, visualization_msgs::MarkerArray* marker_array)
{
  ros::Duration lifetime(sec);
  for (auto& marker : marker_array->markers)
  {
    marker.lifetime = lifetime;
  }
}
void publishMarkerArray(const visualization_msgs::MarkerArray& marker_array, const ros::Publisher& publisher, bool delete_markers=false)
{
  visualization_msgs::MarkerArray msg;

  // insert local marker
  msg.markers.insert(msg.markers.end(), marker_array.markers.begin(), marker_array.markers.end());

  if (delete_markers)
  {
    for (auto& marker : msg.markers)
    {
      marker.action = visualization_msgs::Marker::DELETE;
    }
  }

  publisher.publish(msg);
}



void createLocalPathOrientationMarker(const nav_msgs::Path& lane_waypoint)
{
  visualization_msgs::MarkerArray tmp_marker_array;

  visualization_msgs::Marker lane_waypoint_marker;
  lane_waypoint_marker.header.frame_id = "map";
  lane_waypoint_marker.header.stamp = ros::Time::now();
  lane_waypoint_marker.ns = "local_point_orientation_marker";
  
  lane_waypoint_marker.type = visualization_msgs::Marker::ARROW;
  lane_waypoint_marker.action = visualization_msgs::Marker::ADD;
  lane_waypoint_marker.scale.x = 0.02;
  lane_waypoint_marker.scale.y = 0.005;
  lane_waypoint_marker.scale.z = 0.005;
  lane_waypoint_marker.color.g = 1.0;
  lane_waypoint_marker.color.a = 1.0;
  lane_waypoint_marker.frame_locked = true;

  for (unsigned int i = 0; i < lane_waypoint.poses.size(); i++)
  {
    lane_waypoint_marker.id = i+1;
    lane_waypoint_marker.pose = lane_waypoint.poses[i].pose;

    g_local_waypoints_marker_array.markers.push_back(lane_waypoint_marker);
  }

}

void createTransformedGlobalPathOrientationMarker(const nav_msgs::Path& lane_waypoint)
{
  visualization_msgs::MarkerArray tmp_marker_array;

  visualization_msgs::Marker lane_waypoint_marker;
  lane_waypoint_marker.header.frame_id = "map";
  lane_waypoint_marker.header.stamp = ros::Time::now();
  lane_waypoint_marker.ns = "transformed_global_point_orientation_marker";
  
  lane_waypoint_marker.type = visualization_msgs::Marker::ARROW;
  lane_waypoint_marker.action = visualization_msgs::Marker::ADD;
  lane_waypoint_marker.scale.x = 0.02;
  lane_waypoint_marker.scale.y = 0.005;
  lane_waypoint_marker.scale.z = 0.005;
  lane_waypoint_marker.color.r = 1.0;
  lane_waypoint_marker.color.a = 1.0;
  lane_waypoint_marker.frame_locked = true;

  for (unsigned int i = 0; i < lane_waypoint.poses.size(); i++)
  {
    lane_waypoint_marker.id = i+1;
    lane_waypoint_marker.pose = lane_waypoint.poses[i].pose;

    g_transformed_global_marker_array.markers.push_back(lane_waypoint_marker);
  }

}


void createGlobalPathOrientationMarker(const nav_msgs::Path& lane_waypoints_array)
{
  visualization_msgs::MarkerArray tmp_marker_array;
  visualization_msgs::Marker lane_waypoint_marker;
  lane_waypoint_marker.header.frame_id = "map";
  lane_waypoint_marker.header.stamp = ros::Time::now();
  lane_waypoint_marker.type = visualization_msgs::Marker::ARROW;
  lane_waypoint_marker.action = visualization_msgs::Marker::ADD;
  lane_waypoint_marker.scale.x = 0.02;
  lane_waypoint_marker.scale.y = 0.005;
  lane_waypoint_marker.scale.z = 0.005;
  lane_waypoint_marker.color.b = 1.0;
  lane_waypoint_marker.color.a = 1.0;
  lane_waypoint_marker.frame_locked = true;

  int count = 1;
  //for (auto lane : lane_waypoints_array.lanes)
  {
    lane_waypoint_marker.ns = "global_path_waypoint_orientation_marker_" + std::to_string(count);

    for (int i = 0; i < lane_waypoints_array.poses.size(); i++)
    {
      lane_waypoint_marker.id = i;
      lane_waypoint_marker.pose = lane_waypoints_array.poses[i].pose;
      tmp_marker_array.markers.push_back(lane_waypoint_marker);
    }
    count++;
  }

  g_global_marker_array.markers.insert(g_global_marker_array.markers.end(), tmp_marker_array.markers.begin(),
                                       tmp_marker_array.markers.end());
}


void createGoalMarker(const nav_msgs::Path& lane_waypoints_array)
{
  visualization_msgs::MarkerArray tmp_marker_array;
  visualization_msgs::Marker lane_waypoint_marker;
  lane_waypoint_marker.header.frame_id = "map";
  lane_waypoint_marker.header.stamp = ros::Time::now();
  lane_waypoint_marker.type = visualization_msgs::Marker::ARROW;
  lane_waypoint_marker.action = visualization_msgs::Marker::ADD;
  lane_waypoint_marker.scale.x = 0.1;
  lane_waypoint_marker.scale.y = 0.01;
  lane_waypoint_marker.scale.z = 0.01;
  lane_waypoint_marker.color.r = 1.0;
  lane_waypoint_marker.color.a = 1.0;
  lane_waypoint_marker.frame_locked = true;

  int count = 1;
  //for (auto lane : lane_waypoints_array.lanes)
  {
    lane_waypoint_marker.ns = "goal_list_orientation_marker";

    for (int i = 0; i < lane_waypoints_array.poses.size(); i++)
    {
      lane_waypoint_marker.id = i;
      lane_waypoint_marker.pose = lane_waypoints_array.poses[i].pose;
      tmp_marker_array.markers.push_back(lane_waypoint_marker);
    }
    count++;
  }

  g_goal_list_marker_array.markers.insert(g_goal_list_marker_array.markers.end(), tmp_marker_array.markers.begin(),
                                       tmp_marker_array.markers.end());
}

void getGlobalPathCallback(const nav_msgs::Path& msg)
{
  publishMarkerArray(g_global_marker_array, g_global_mark_pub, true);
  g_global_marker_array.markers.clear();
  //createGlobalLaneArrayVelocityMarker(*msg);
  createGlobalPathOrientationMarker(msg);
  //createGlobalLaneArrayChangeFlagMarker(*msg);
  publishMarkerArray(g_global_marker_array, g_global_mark_pub);
}

void getLocalPathCallback(const nav_msgs::Path& msg)
{
  publishMarkerArray(g_local_waypoints_marker_array, g_local_mark_pub, true);
  g_local_waypoints_marker_array.markers.clear();

  createLocalPathOrientationMarker(msg);
  
  publishMarkerArray(g_local_waypoints_marker_array, g_local_mark_pub);
}

void getTransformedGlobalPathCallback(const nav_msgs::Path& msg)
{
  publishMarkerArray(g_transformed_global_marker_array, g_transformed_global_mark_pub, true);
  g_transformed_global_marker_array.markers.clear();

  createTransformedGlobalPathOrientationMarker(msg);
  
  publishMarkerArray(g_transformed_global_marker_array, g_transformed_global_mark_pub);
}

void getGoalListCallback(const nav_msgs::Path& msg)
{
  publishMarkerArray(g_goal_list_marker_array, g_goal_mark_pub, true);
  g_goal_list_marker_array.markers.clear();

  createGoalMarker(msg);

  publishMarkerArray(g_goal_list_marker_array, g_goal_mark_pub);
}


//接收到laserScan，把laserscan转换成点云数据，并存储到buffer中
void getLaserScanCallback(const sensor_msgs::LaserScanConstPtr& message)
{
  // project the laser into a point cloud
  // sensor_msgs::PointCloud2 cloud;
  // cloud.header = message->header;

  // //ROS_INFO("1 message.header.frame_id: %s",message->header.frame_id.c_str());
  
  // // std::string framid = message->header.frame_id;
  // // int str_index = framid.find('/');
  // // if(str_index != std::string::npos)
  // // {
  // //   ROS_INFO("find origin_frame '/' index:  %d ", str_index);
  // //   framid.erase(str_index,1);
  // // }

  // // ROS_INFO("1 message.header.frame_id: %s",framid.c_str());
  
  // //转换成点云数据
  // // project the laserscan into a point cloud
  // try
  // {
  //   projector_.transformLaserScanToPointCloud(message->header.frame_id, *message, cloud, *tf_);
  // }
  // catch (tf2::TransformException &ex)
  // {
  //   ROS_WARN("High fidelity enabled, but TF returned a transform exception to frame %s: %s", global_frame_.c_str(),
  //            ex.what());
  //   projector_.projectLaser(*message, cloud);
  // }

  // // buffer the point cloud
  // buffer->lock();
  // buffer->bufferCloud(cloud);
  // buffer->unlock();
  
  //ROS_INFO("1 cloud.header.frame_id: %s",cloud.header.frame_id.c_str());
        //populate the LaserScan message
    callback_count++;

    int num_readings = message->ranges.size();
    int half_ = num_readings/2;
    int quarter_ = num_readings/4;
    int part8_ = num_readings/8;

    sensor_msgs::LaserScan scan;
    scan.header.stamp = message->header.stamp;
    scan.header.frame_id = message->header.frame_id;
    scan.angle_min = -1.57;
    scan.angle_max = 1.57;
    scan.angle_increment = message->angle_increment;// 3.14 / num_readings;
    scan.time_increment = message->time_increment;//(1 / laser_frequency) / (num_readings);
    scan.range_min = message->range_min;
    scan.range_max = message->range_max;


    // scan.ranges = message->ranges;

    // scan.intensities = message->intensities;




    int k = 0;
    // for(unsigned int i = quarter_; i < 3*quarter_ && k < half_; ++i)
    // {
    //   scan.ranges[k] = message->ranges[i];

    //   scan.intensities[k] = message->intensities[i];

    //   k++;
    // }
  /////////////////////////////////////////////////////////////////////////////////////////
    //////180
    // scan.ranges.resize(180);
    // scan.intensities.resize(180);
    // for(unsigned int i = 3*quarter_; i < num_readings; ++i)
    // {
    //   scan.ranges[k] = message->ranges[i];

    //   scan.intensities[k] = message->intensities[i];

    //   k++;
    // }

    // for(unsigned int i = 0; i < quarter_ ; ++i)
    // {
    //   scan.ranges[k] = message->ranges[i];

    //   scan.intensities[k] = message->intensities[i];

    //   k++;
    // }

    // scan.ranges.resize(90);
    // scan.intensities.resize(90);
    // for(unsigned int i = quarter_; i < 2*quarter_ ; ++i)
    // {
    //   scan.ranges[k] = message->ranges[i];

    //   scan.intensities[k] = message->intensities[i];

    //   k++;
    // }

  //////////////////////////////////////////////////////////////////////////////////////////
    /////270
    scan.ranges.resize(270);
    scan.intensities.resize(270);
    for(unsigned int i = 6*part8_; i < num_readings; ++i)
    {
      scan.ranges[k] = message->ranges[i];

      scan.intensities[k] = message->intensities[i];

      k++;
    }

    for(unsigned int i = 0; i < 4*part8_ ; ++i)
    {
      scan.ranges[k] = message->ranges[i];

      scan.intensities[k] = message->intensities[i];

      k++;
    }



    scan_pub.publish(scan);


    ROS_INFO("num_readings: %d, half: %d, quarter :%d, count: %d ",num_readings,half_,quarter_,callback_count);

}

}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "filter_scan_data");
  ros::NodeHandle nh;
  ros::NodeHandle private_nh("~");


  ros::Subscriber scan_data_sub = nh.subscribe("scan", 10, getLaserScanCallback);


  scan_pub = nh.advertise<sensor_msgs::LaserScan>("scan_change", 50);


  // g_local_mark_pub = nh.advertise<visualization_msgs::MarkerArray>("local_path_mark", 10, true);
  // g_global_mark_pub = nh.advertise<visualization_msgs::MarkerArray>("global_path_mark", 10, true);

  // g_transformed_global_mark_pub = nh.advertise<visualization_msgs::MarkerArray>("transformed_global_path_mark", 10, true);


  // g_goal_mark_pub = nh.advertise<visualization_msgs::MarkerArray>("goal_list_mark", 10, true);

  // // initialize path color
  // _initial_color.g = 0.7;
  // _initial_color.b = 1.0;
  // _global_color = _initial_color;
  // _global_color.a = g_global_alpha;
  // g_local_color = _initial_color;
  // g_local_color.a = g_local_alpha;

  ros::spin();
}
