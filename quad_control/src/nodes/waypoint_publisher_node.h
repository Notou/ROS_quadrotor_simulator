/*
 * Copyright 2015 Fadri Furrer, ASL, ETH Zurich, Switzerland
 * Copyright 2015 Michael Burri, ASL, ETH Zurich, Switzerland
 * Copyright 2015 Mina Kamel, ASL, ETH Zurich, Switzerland
 * Copyright 2015 Janosch Nikolic, ASL, ETH Zurich, Switzerland
 * Copyright 2015 Markus Achtelik, ASL, ETH Zurich, Switzerland
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0

 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */


#include <std_msgs/Empty.h>
#include <slamdunk_msgs/QualityStamped.h>

#include "quad_control/quad_controller.h"

namespace quad_control {

class WaypointPublisherNode {
 public:
  WaypointPublisherNode();
  ~WaypointPublisherNode();

  void InitializeParams();
//  void Publish();

 private:

  //Subscribers
  ros::Subscriber cmd_pos_sub_;
  ros::Subscriber odometry_sub_;
  ros::Subscriber cmd_threednav_sub_;
  ros::Subscriber quality_sub_;

  //Publisher
  ros::Publisher trajectory_pub;
  ros::Publisher takeoff_pub;
  ros::Publisher land_pub;

  //Waypoint variables
  geometry_msgs::Twist command_trajectory;
  geometry_msgs::Pose threedNav_trajectory;
  geometry_msgs::Pose desired_wp;
  nav_msgs::Odometry current_gps_;

  //General
  tf::Quaternion q;
  double gps_roll, gps_pitch, gps_yaw;

  double start_time;
  double current_time;
  int qualityState;

  bool is3DNav = false;
  bool isLost = false;
  void CommandTrajectoryCallback(const geometry_msgs::TwistConstPtr& command_trajectory_msg);
  void OdometryCallback(const nav_msgs::OdometryConstPtr& odometry_msg);
  void threedNavCallback(const geometry_msgs::PoseConstPtr& threed_nav_msg);
  void qualityCallback(slamdunk_msgs::QualityStamped::ConstPtr const& quality);
};
}
