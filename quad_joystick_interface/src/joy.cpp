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


#include "quad_joystick_interface/joy.h"


Joy::Joy() {
  ros::NodeHandle nh;
  ros::NodeHandle pnh("~");

  trajectory_pub = nh.advertise<geometry_msgs::Twist>("command/trajectory", 10);


  trajectory_msg.linear.x = 0.0;	// m
  trajectory_msg.linear.y = 0.0;	// m
  trajectory_msg.linear.z = 0.0;	// m
  trajectory_msg.angular.z = 0.0;	// rad

  // Hack for sending button statuses
  trajectory_msg.angular.x = 0.0;	// takeoff and land
  trajectory_msg.angular.y = 0.0;	// auto

  //Initialize Parameters

  // Map similar to RC set-up
  pnh.param("axis_roll_", axes_.roll, 4);	// RS <->
  pnh.param("axis_pitch_", axes_.pitch, 3);	// RS up/down
  pnh.param("axis_thrust_", axes_.thrust, 1);	// LS up/down
  pnh.param("axis_yaw_", axes_.yaw, 0);		// LS <->

  pnh.param("axis_direction_roll", axes_.roll_direction, -1);
  pnh.param("axis_direction_pitch", axes_.pitch_direction, 1);
  pnh.param("axis_direction_thrust", axes_.thrust_direction, 1);
  pnh.param("axis_direction_yaw", axes_.yaw_direction, 1);

  pnh.param("max_v_xy", max_.v_xy, 1.0);  // [m/s]
  pnh.param("max_roll", max_.roll, 5.0);  // [rad]
  pnh.param("max_pitch", max_.pitch, 5.0);  // [rad]
  pnh.param("max_yaw_rate", max_.rate_yaw, 2.0);  // [rad/s]
  pnh.param("max_thrust", max_.thrust, 5.0);  // [N]
  pnh.param("thrust_offset", max_.thrust_offset, 2.75);  // [N]


  pnh.param("button_ctrl_enable_", buttons_.ctrl_enable_gps, 1); // B
  pnh.param("button_ctrl_mode_", buttons_.ctrl_enable_mission, 3); // Y
  pnh.param("button_takeoff_", buttons_.takeoff, 0);	// A
  pnh.param("button_land_", buttons_.land, 2);		// X
  pnh.param("button_ctrl_auto", buttons_.ctrl_enable_autonomous, 5);	// RB
  pnh.param("button_ctrl_3dnav", buttons_.ctrl_enable_3dnav, 4);	// LB

  namespace_ = nh_.getNamespace();

  //Subscribe to message "joy" and callback JoyCallback
  joy_sub_ = nh_.subscribe("joy", 10, &Joy::JoyCallback, this);
}

void Joy::JoyCallback(const sensor_msgs::JoyConstPtr& msg) {

  trajectory_msg.linear.x = msg->axes[axes_.pitch] * max_.pitch * axes_.pitch_direction;
  trajectory_msg.linear.y = msg->axes[axes_.roll] * max_.roll * axes_.roll_direction;	// ROS y axis pos to the left
  trajectory_msg.linear.z = (msg->axes[axes_.thrust] * max_.thrust * axes_.thrust_direction);
  trajectory_msg.angular.z = msg->axes[axes_.yaw] * max_.rate_yaw * axes_.yaw_direction;

  // Hack for sending button states
  trajectory_msg.angular.x = msg->buttons[buttons_.takeoff];	// takeoff
  trajectory_msg.angular.x = -1 * msg->buttons[buttons_.land];	// land
  trajectory_msg.angular.y = msg->buttons[buttons_.ctrl_enable_3dnav];	// auto_mode

  trajectory_pub.publish(trajectory_msg);

}


int main(int argc, char** argv) {

  ros::init(argc, argv, "quad_joy_interface");	//Specify node name
  Joy joy;

  ros::spin();

  return 0;
}
