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

namespace
{
ros::Publisher g_local_mark_pub;
ros::Publisher g_global_mark_pub;
ros::Publisher g_goal_mark_pub;
ros::Publisher g_transformed_global_mark_pub;

std_msgs::ColorRGBA _initial_color;
std_msgs::ColorRGBA _global_color;
std_msgs::ColorRGBA g_local_color;
const double g_global_alpha = 0.2;
const double g_local_alpha = 1.0;
int _closest_waypoint = -1;

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



// void createGlobalLaneArrayVelocityMarker(const autoware_msgs::LaneArray& lane_waypoints_array)
// {
//   visualization_msgs::MarkerArray tmp_marker_array;
//   // display by markers the velocity of each waypoint.
//   visualization_msgs::Marker velocity_marker;
//   velocity_marker.header.frame_id = "map";
//   velocity_marker.header.stamp = ros::Time::now();
//   velocity_marker.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
//   velocity_marker.action = visualization_msgs::Marker::ADD;
//   velocity_marker.scale.z = 0.4;
//   velocity_marker.color.a = 1.0;
//   velocity_marker.color.r = 1;
//   velocity_marker.color.g = 1;
//   velocity_marker.color.b = 1;
//   velocity_marker.frame_locked = true;

//   int count = 1;
//   for (auto lane : lane_waypoints_array.lanes)
//   {
//     velocity_marker.ns = "global_velocity_lane_" + std::to_string(count);
//     for (int i = 0; i < static_cast<int>(lane.waypoints.size()); i++)
//     {
//       velocity_marker.id = i;
//       geometry_msgs::Point relative_p;
//       relative_p.y = 0.5;
//       velocity_marker.pose.position = calcAbsoluteCoordinate(relative_p, lane.waypoints[i].pose.pose);
//       velocity_marker.pose.position.z += 0.2;

//       // double to string
//       std::string vel = std::to_string(mps2kmph(lane.waypoints[i].twist.twist.linear.x));
//       velocity_marker.text = std::to_string(lane.waypoints[i].wpstate.lanechange_state)+"_"+std::to_string(i+1)+"_" + vel.erase(vel.find_first_of(".") + 2);
//       tmp_marker_array.markers.push_back(velocity_marker);
//     }
//     count++;
//   }

//   g_global_marker_array.markers.insert(g_global_marker_array.markers.end(), tmp_marker_array.markers.begin(),
//                                        tmp_marker_array.markers.end());
// }

// void createGlobalLaneArrayChangeFlagMarker(const autoware_msgs::LaneArray& lane_waypoints_array)
// {
//   visualization_msgs::MarkerArray tmp_marker_array;
//   // display by markers the velocity of each waypoint.
//   visualization_msgs::Marker marker;
//   marker.header.frame_id = "map";
//   marker.header.stamp = ros::Time::now();
//   marker.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
//   marker.action = visualization_msgs::Marker::ADD;
//   marker.scale.z = 0.4;
//   marker.color.r = 1;
//   marker.color.g = 1;
//   marker.color.b = 1;
//   marker.color.a = 1.0;
//   marker.frame_locked = true;

//   int count = 1;
//   for (auto lane : lane_waypoints_array.lanes)
//   {
//     marker.ns = "global_change_flag_lane_" + std::to_string(count);
//     for (int i = 0; i < static_cast<int>(lane.waypoints.size()); i++)
//     {
//       marker.id = i;
//       geometry_msgs::Point relative_p;
//       relative_p.x = -0.1;
//       marker.pose.position = calcAbsoluteCoordinate(relative_p, lane.waypoints[i].pose.pose);
//       marker.pose.position.z += 0.2;

//       // double to string
//       std::string str = "";
//       if (lane.waypoints[i].change_flag == static_cast<ChangeFlagInteger>(ChangeFlag::straight))
//       {
//         str = "S";
//       }
//       else if (lane.waypoints[i].change_flag == static_cast<ChangeFlagInteger>(ChangeFlag::right))
//       {
//         str = "R";
//       }
//       else if (lane.waypoints[i].change_flag == static_cast<ChangeFlagInteger>(ChangeFlag::left))
//       {
//         str = "L";
//       }
//       else if (lane.waypoints[i].change_flag == static_cast<ChangeFlagInteger>(ChangeFlag::unknown))
//       {
//         str = "U";
//       }else
//       {
//         str = "S";
//       }
      

//       marker.text = str;

//       tmp_marker_array.markers.push_back(marker);
//     }
//     count++;
//   }

//   g_global_marker_array.markers.insert(g_global_marker_array.markers.end(), tmp_marker_array.markers.begin(),
//                                        tmp_marker_array.markers.end());
// }

// void createLocalWaypointVelocityMarker(std_msgs::ColorRGBA color, int closest_waypoint,
//                                        const autoware_msgs::Lane& lane_waypoint)
// {
//   // display by markers the velocity of each waypoint.
//   visualization_msgs::Marker velocity;
//   velocity.header.frame_id = "map";
//   velocity.header.stamp = ros::Time::now();
//   velocity.ns = "local_waypoint_velocity";
//   velocity.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
//   velocity.action = visualization_msgs::Marker::ADD;
//   velocity.scale.z = 0.4;
//   velocity.color = color;
//   velocity.frame_locked = true;

//   for (int i = 0; i < static_cast<int>(lane_waypoint.waypoints.size()); i++)
//   {
//     velocity.id = i;
//     geometry_msgs::Point relative_p;
//     relative_p.y = -0.65;
//     velocity.pose.position = calcAbsoluteCoordinate(relative_p, lane_waypoint.waypoints[i].pose.pose);
//     velocity.pose.position.z += 0.2;

//     // double to string
//     std::ostringstream oss;
//     oss << std::fixed << std::setprecision(1) << mps2kmph(lane_waypoint.waypoints[i].twist.twist.linear.x);
//     velocity.text = oss.str();

//     g_local_waypoints_marker_array.markers.push_back(velocity);
//   }
// }

// void createGlobalLaneArrayMarker(std_msgs::ColorRGBA color, const nav_msgs::Path& lane_waypoints_array)
// {
//   visualization_msgs::Marker lane_waypoint_marker;
//   lane_waypoint_marker.header.frame_id = "map";
//   lane_waypoint_marker.header.stamp = ros::Time::now();
//   lane_waypoint_marker.ns = "global_lane_array_marker";
//   lane_waypoint_marker.type = visualization_msgs::Marker::LINE_STRIP;
//   lane_waypoint_marker.action = visualization_msgs::Marker::ADD;
//   lane_waypoint_marker.scale.x = 1.0;
//   lane_waypoint_marker.color = color;
//   lane_waypoint_marker.frame_locked = true;

//   int count = 0;
//   for (auto lane : lane_waypoints_array.lanes)
//   {
//     lane_waypoint_marker.points.clear();
//     lane_waypoint_marker.id = count;

//     for (auto el : lane.waypoints)
//     {
//       geometry_msgs::Point point;
//       point = el.pose.pose.position;
//       lane_waypoint_marker.points.push_back(point);
//     }
//     g_global_marker_array.markers.push_back(lane_waypoint_marker);
//     count++;
//   }
// }

// void createGlobalLaneArrayOrientationMarker(const autoware_msgs::LaneArray& lane_waypoints_array)
// {
//   visualization_msgs::MarkerArray tmp_marker_array;
//   visualization_msgs::Marker lane_waypoint_marker;
//   lane_waypoint_marker.header.frame_id = "map";
//   lane_waypoint_marker.header.stamp = ros::Time::now();
//   lane_waypoint_marker.type = visualization_msgs::Marker::ARROW;
//   lane_waypoint_marker.action = visualization_msgs::Marker::ADD;
//   lane_waypoint_marker.scale.x = 0.25;
//   lane_waypoint_marker.scale.y = 0.05;
//   lane_waypoint_marker.scale.z = 0.05;
//   lane_waypoint_marker.color.r = 1.0;
//   lane_waypoint_marker.color.a = 1.0;
//   lane_waypoint_marker.frame_locked = true;

//   int count = 1;
//   for (auto lane : lane_waypoints_array.lanes)
//   {
//     lane_waypoint_marker.ns = "global_lane_waypoint_orientation_marker_" + std::to_string(count);

//     for (int i = 0; i < static_cast<int>(lane.waypoints.size()); i++)
//     {
//       lane_waypoint_marker.id = i;
//       lane_waypoint_marker.pose = lane.waypoints[i].pose.pose;
//       tmp_marker_array.markers.push_back(lane_waypoint_marker);
//     }
//     count++;
//   }

//   g_global_marker_array.markers.insert(g_global_marker_array.markers.end(), tmp_marker_array.markers.begin(),
//                                        tmp_marker_array.markers.end());
// }

// void createLocalPathMarker(std_msgs::ColorRGBA color, const nav_msgs::Path& lane_waypoint)
// {
//   visualization_msgs::Marker lane_waypoint_marker;
//   lane_waypoint_marker.header.frame_id = "map";
//   lane_waypoint_marker.header.stamp = ros::Time::now();
//   lane_waypoint_marker.ns = "local_path_marker";
//   lane_waypoint_marker.id = 0;
//   lane_waypoint_marker.type = visualization_msgs::Marker::LINE_STRIP;
//   lane_waypoint_marker.action = visualization_msgs::Marker::ADD;
//   lane_waypoint_marker.scale.x = 0.2;
//   lane_waypoint_marker.color = color;
//   lane_waypoint_marker.frame_locked = true;

//   for (unsigned int i = 0; i < lane_waypoint.waypoints.size(); i++)
//   {
//     geometry_msgs::Point point;
//     point = lane_waypoint.waypoints[i].pose.pose.position;
//     lane_waypoint_marker.points.push_back(point);
//   }
//   g_local_waypoints_marker_array.markers.push_back(lane_waypoint_marker);
// }

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
    //lane_waypoint_marker.points.push_back(point);

    // lane_waypoint_marker.id = i;
    // lane_waypoint_marker.pose = lane_waypoints_array.poses[i].pose;
    // tmp_marker_array.markers.push_back(lane_waypoint_marker);
    g_local_waypoints_marker_array.markers.push_back(lane_waypoint_marker);
  }

  // int end  = lane_waypoint.poses.size()-1;
  
  // if(end>0)
  // {
  //   double length = sqrt(pow(lane_waypoint.poses[0].pose.position.y - lane_waypoint.poses[end].pose.position.y,2) + pow(lane_waypoint.poses[0].pose.position.x - lane_waypoint.poses[end].pose.position.x,2));
  //   ROS_INFO("get local path count: %d, length: %f", lane_waypoint.poses.size(),length);
  // }

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
    //lane_waypoint_marker.points.push_back(point);

    // lane_waypoint_marker.id = i;
    // lane_waypoint_marker.pose = lane_waypoints_array.poses[i].pose;
    // tmp_marker_array.markers.push_back(lane_waypoint_marker);
    g_transformed_global_marker_array.markers.push_back(lane_waypoint_marker);
  }

  // int end  = lane_waypoint.poses.size()-1;
  
  // if(end>0)
  // {
  //   double length = sqrt(pow(lane_waypoint.poses[0].pose.position.y - lane_waypoint.poses[end].pose.position.y,2) + pow(lane_waypoint.poses[0].pose.position.x - lane_waypoint.poses[end].pose.position.x,2));
  //   ROS_INFO("get local path count: %d, length: %f", lane_waypoint.poses.size(),length);
  // }

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

// void lightCallback(const autoware_msgs::TrafficLightConstPtr& msg)
// {
//   std_msgs::ColorRGBA global_color;
//   global_color.a = g_global_alpha;

//   std_msgs::ColorRGBA local_color;
//   local_color.a = g_local_alpha;

//   switch (msg->traffic_light)
//   {
//     case TRAFFIC_LIGHT_RED:
//       global_color.r = 1.0;
//       _global_color = global_color;
//       local_color.r = 1.0;
//       g_local_color = local_color;
//       break;
//     case TRAFFIC_LIGHT_GREEN:
//       global_color.g = 1.0;
//       _global_color = global_color;
//       local_color.g = 1.0;
//       g_local_color = local_color;
//       break;
//     case TRAFFIC_LIGHT_UNKNOWN:
//       global_color.b = 1.0;
//       global_color.g = 0.7;
//       _global_color = global_color;
//       local_color.b = 1.0;
//       local_color.g = 0.7;
//       g_local_color = local_color;
//       break;
//     default:
//       ROS_ERROR("unknown traffic_light");
//       return;
//   }
// }

// void receiveAutoDetection(const autoware_msgs::TrafficLightConstPtr& msg)
// {
//   if (!g_config_manual_detection)
//     lightCallback(msg);
// }

// void receiveManualDetection(const autoware_msgs::TrafficLightConstPtr& msg)
// {
//   if (g_config_manual_detection)
//     lightCallback(msg);
// }

// void configParameter(const autoware_config_msgs::ConfigLaneStopConstPtr& msg)
// {
//   g_config_manual_detection = msg->manual_detection;
// }

void getGlobalPathCallback(const nav_msgs::Path& msg)
{
  if(msg.poses.size() <2)
  {
    return;
  }
  publishMarkerArray(g_global_marker_array, g_global_mark_pub, true);
  g_global_marker_array.markers.clear();
  //createGlobalLaneArrayVelocityMarker(*msg);
  createGlobalPathOrientationMarker(msg);
  //createGlobalLaneArrayChangeFlagMarker(*msg);
  publishMarkerArray(g_global_marker_array, g_global_mark_pub);
}

void getLocalPathCallback(const nav_msgs::Path& msg)
{
  if(msg.poses.size() < 2)
  {
    return;
  }

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



}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "waypoints_marker_publisher");
  ros::NodeHandle nh;
  ros::NodeHandle private_nh("~");

  // subscribe traffic light
  // ros::Subscriber light_sub = nh.subscribe("light_color", 10, receiveAutoDetection);
  // ros::Subscriber light_managed_sub = nh.subscribe("light_color_managed", 10, receiveManualDetection);

  // subscribe global waypoints
  ros::Subscriber global_path_sub = nh.subscribe("move_base/NavfnROS/plan", 10, getGlobalPathCallback);
  ros::Subscriber local_path_sub = nh.subscribe("move_base/DWAPlannerROS/local_plan", 10, getLocalPathCallback);
  
  //????????????gobal path
  ros::Subscriber transformed_path_sub = nh.subscribe("move_base/DWAPlannerROS/transformed_global_plan", 10, getTransformedGlobalPathCallback);

  ros::Subscriber goal_list_sub = nh.subscribe("move_base/goal_list", 10, getGoalListCallback);

  // subscribe local waypoints
  // ros::Subscriber final_sub = nh.subscribe("final_waypoints", 10, finalCallback);
  // ros::Subscriber closest_sub = nh.subscribe("closest_waypoint", 10, closestCallback);

  // subscribe config
  //ros::Subscriber config_sub = nh.subscribe("config/lane_stop", 10, configParameter);

  g_local_mark_pub = nh.advertise<visualization_msgs::MarkerArray>("local_path_mark", 10, true);
  g_global_mark_pub = nh.advertise<visualization_msgs::MarkerArray>("global_path_mark", 10, true);

  g_transformed_global_mark_pub = nh.advertise<visualization_msgs::MarkerArray>("transformed_global_path_mark", 10, true);


  g_goal_mark_pub = nh.advertise<visualization_msgs::MarkerArray>("goal_list_mark", 10, true);

  // initialize path color
  _initial_color.g = 0.7;
  _initial_color.b = 1.0;
  _global_color = _initial_color;
  _global_color.a = g_global_alpha;
  g_local_color = _initial_color;
  g_local_color.a = g_local_alpha;

  ros::spin();
}
