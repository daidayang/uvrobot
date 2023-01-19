/*
 *  Copyright (c) 2015, Nagoya University
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions are met:
 *
 *  * Redistributions of source code must retain the above copyright notice, this
 *    list of conditions and the following disclaimer.
 *
 *  * Redistributions in binary form must reproduce the above copyright notice,
 *    this list of conditions and the following disclaimer in the documentation
 *    and/or other materials provided with the distribution.
 *
 *  * Neither the name of Autoware nor the names of its
 *    contributors may be used to endorse or promote products derived from
 *    this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 *  AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 *  IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 *  DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
 *  FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 *  DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 *  SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 *  OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 *  OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*/

#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <std_msgs/Float32.h>
#include <tf/transform_datatypes.h>
#include <fstream>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/Path.h>
namespace sys_logic
{
class Display_coordinate_system //设置避障可用区域的边界系列点
{
public:
  Display_coordinate_system();
  ~Display_coordinate_system();
private:
  // functions
  void createLocalPointMarker(const nav_msgs::Path &lane_waypoint);
  void createTextMarker();
  void publishMarker();
public:
  void drawCoordinateSystemWaypoints();
  // handle
  ros::NodeHandle nh_;
  ros::NodeHandle private_nh_;
  // publisher
  ros::Publisher g_point_mark_pub;
  // subscriber
  ros::Subscriber sub_pose;
  std::vector<geometry_msgs::PoseStamped> all_poselist;
  visualization_msgs::MarkerArray g_waypoints_marker_array;
};

Display_coordinate_system::Display_coordinate_system() : private_nh_("~")
{
  // parameter settings
  all_poselist.clear();

  //sub_pose = nh_.subscribe("/initialpose", 10, &Display_coordinate_system::getClickPoseCallback,this);
  g_point_mark_pub = nh_.advertise<visualization_msgs::MarkerArray>("coordinate_system_waypoints", 10, true);
}

Display_coordinate_system::~Display_coordinate_system()
{
  ;
}

// void Display_coordinate_system::getClickPoseCallback(const geometry_msgs::PoseWithCovarianceStampedConstPtr& msg)
// {
//   std::cout<<"get click Pose Data: "<<msg->pose.pose.position.x<<","<<msg->pose.pose.position.y<<","<<msg->pose.pose.position.z<<std::endl;
//   autoware_msgs::Waypoint p;
//   p.pose.pose.position.x  = msg->pose.pose.position.x;
//   p.pose.pose.position.y  = msg->pose.pose.position.y;
//   p.pose.pose.position.z  = msg->pose.pose.position.z;
//   p.pose.pose.orientation = msg->pose.pose.orientation;
//   //drawNewWaypoints(p);
// }
void Display_coordinate_system::drawCoordinateSystemWaypoints()
{
  
  all_poselist.clear();
  double i = -1000;
  while( i<1000)
  {
    geometry_msgs::PoseStamped px;
    px.pose.position.x  = i;
    px.pose.position.y  = 0;
    px.pose.position.z  = 0;
    all_poselist.push_back(px);


    geometry_msgs::PoseStamped py;
    py.pose.position.x  = 0;
    py.pose.position.y  = i;
    py.pose.position.z  = 0;
    all_poselist.push_back(py);

    i+= 0.25;
  }
  //每两个坐标之间加入三个数

  //发布显示
  nav_msgs::Path oordinate_system_lane;
  for(int i = 0;i<all_poselist.size();++i)
  {
    oordinate_system_lane.poses.push_back(all_poselist[i]);
  }

  createLocalPointMarker(oordinate_system_lane);
  createTextMarker();
  publishMarker();

  g_waypoints_marker_array.markers.clear();
  return;
}

void Display_coordinate_system::createLocalPointMarker(const nav_msgs::Path &lane_waypoint)
{
  visualization_msgs::Marker lane_waypoint_marker;
  lane_waypoint_marker.header.frame_id = "map";
  lane_waypoint_marker.header.stamp = ros::Time();
  lane_waypoint_marker.ns = "coordinate_system_point_marker";
  lane_waypoint_marker.id = 0;
  lane_waypoint_marker.type = visualization_msgs::Marker::POINTS;
  lane_waypoint_marker.action = visualization_msgs::Marker::ADD;
  lane_waypoint_marker.scale.x = 0.02;
  lane_waypoint_marker.scale.y = 0.02;
  lane_waypoint_marker.scale.z = 0.02;
  lane_waypoint_marker.color.r = 1.0;
   lane_waypoint_marker.color.a = 1.0;
  lane_waypoint_marker.frame_locked = true;

  for (unsigned int i = 0; i < lane_waypoint.poses.size(); i++)
  {
    geometry_msgs::Point point;
    point = lane_waypoint.poses[i].pose.position;
    lane_waypoint_marker.points.push_back(point);
  }
  g_waypoints_marker_array.markers.push_back(lane_waypoint_marker);

}
void Display_coordinate_system::publishMarker()
{
  visualization_msgs::MarkerArray marker_array;
  //insert marker
  marker_array.markers.insert(marker_array.markers.end(), g_waypoints_marker_array.markers.begin(),
                              g_waypoints_marker_array.markers.end());

  g_point_mark_pub.publish(marker_array);
}
void Display_coordinate_system::createTextMarker()
{
  //visualization_msgs::MarkerArray pub_textlabel;
	std_msgs::ColorRGBA color_white;
	color_white.r = 1.0f;
	color_white.g = 1.0f;
	color_white.b = 1.0f;
	color_white.a = 1.0f;
  std::map<std::string,geometry_msgs::Point> text_list;
  geometry_msgs::Point ps;
  ps.x = 3;
  ps.y = 0;
  ps.z = 0;
  text_list["+x"] = ps;

  ps.x = 0;
  ps.y = 3;
  ps.z = 0;
  text_list["+y"] = ps;
  int k = 1000;
	for (auto oneit : text_list)
	{
		visualization_msgs::Marker marker_textlabel;
		marker_textlabel.header.frame_id = "map";
		marker_textlabel.ns = "coordinate_system_text_marker";
		marker_textlabel.id = k;
    marker_textlabel.color = color_white;
		/* Set the text label */
		marker_textlabel.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
		marker_textlabel.scale.z = 0.5;

    marker_textlabel.text = oneit.first;
		marker_textlabel.pose.position = oneit.second;
		marker_textlabel.pose.orientation.x = 0.0;
		marker_textlabel.pose.orientation.y = 0.0;
		marker_textlabel.pose.orientation.z = 0.0;
		marker_textlabel.pose.orientation.w = 0.0;
		k++;
		g_waypoints_marker_array.markers.push_back(marker_textlabel);
	}
}

} // namespace
int main(int argc, char **argv)
{
  ros::init(argc, argv, "display_coordinate_system");
  sys_logic::Display_coordinate_system dcs;
  dcs.drawCoordinateSystemWaypoints();
  ros::spin();
  return 0;
}
