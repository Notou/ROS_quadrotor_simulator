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


#include <tf2_ros/transform_listener.h>
#include <geometry_msgs/TransformStamped.h>
#include <geometry_msgs/Twist.h>
#include <bebop_msgs/Ardrone3PilotingStateFlyingStateChanged.h>

#include "quad_control/quad_controller.h"

namespace quad_control {

class PositionControllerNode {
 public:
  PositionControllerNode();
  ~PositionControllerNode();

  void InitializeParams();
  void Publish();
  void Run();

 private:

  PositionController position_controller_;

  std::string namespace_;

  //Subscribers
  ros::Subscriber cmd_trajectory_sub_;
  ros::Subscriber landed_sub;

  ros::Publisher odom_pub_;
  ros::Publisher bebop_topic;

  //Control variables
  nav_msgs::Odometry current_gps_;
  geometry_msgs::Twist control_msg_;
  geometry_msgs::Pose wp;


  int flyingState;

  void WaypointCallback(const geometry_msgs::PoseConstPtr& trajectory_reference_msg);
  void OdometryCallback();
  void LandedCallback(const bebop_msgs::Ardrone3PilotingStateFlyingStateChangedPtr& state_ptr);
};

}
