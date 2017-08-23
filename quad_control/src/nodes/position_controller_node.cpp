/*
* Copyright 2015 Wil Selby
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

//TODO debug messages

#include "position_controller_node.h"

//#include "quad_control/parameters_ros.h"


namespace quad_control {

  PositionControllerNode::PositionControllerNode(){

    InitializeParams();


  }


  PositionControllerNode::~PositionControllerNode() {}

  void PositionControllerNode::InitializeParams(){

    ros::NodeHandle pnh("~");

    GetVehicleParameters(pnh, &vehicle_parameters_);

    position_controller_.InitializeParameters(pnh);

    wp.position.x = 0.0;
    wp.position.y = 0.0;
    wp.position.z = 0.5;
    wp.yaw = 0.0;

    ROS_INFO("Position_controller_node Parameters Initialized.");

  }

  void PositionControllerNode::Publish(){

    //Control message header information
    ros::Time update_time = ros::Time::now();
    control_msg_.header.stamp = update_time;
    control_msg_.header.frame_id = "quad_position_ctrl_frame";

    geometry_msgs::Twist twist;
    twist.linear.x = control_msg_.pitch;
    twist.linear.y = -control_msg_.roll;
    twist.linear.z = control_msg_.thrust;
    twist.angular.z = control_msg_.yaw_rate;

    { // Adapt the output to be used by the simulation
      control_msg_.thrust *= 4;
      control_msg_.thrust += 8.33;
      control_msg_.pitch *= (20.0 * M_PI / 180.0);
      control_msg_.roll *= (20.0 * M_PI / 180.0);
      control_msg_.yaw_rate *= (100.0 * M_PI / 180.0);
    }
    printf("x: %f, y: %f, z: %f, yaw: %f\n", twist.linear.x, twist.linear.y, twist.linear.z, twist.angular.z);
    ctrl_pub_.publish(control_msg_);
    bebop_topic.publish(twist);

  }

  void PositionControllerNode::Run(){
    tf2_ros::Buffer tfBuffer;
    tf2_ros::TransformListener tfListener(tfBuffer);
    ros::NodeHandle nh;

    // Subscribers
    cmd_trajectory_sub_ = nh.subscribe("command/waypoint", 10, &PositionControllerNode::WaypointCallback, this);

    //Publishers
    ctrl_pub_ = nh.advertise<mav_msgs::CommandRollPitchYawrateThrust>("command/roll_pitch_yawrate_thrust", 10);  //Used in simulation
    odom_pub_ = nh.advertise<nav_msgs::Odometry>("odometry", 10);
    bebop_topic = nh.advertise<geometry_msgs::Twist>("/bebop/cmd_vel", 1);

    ros::Rate rate(20.0);
    while (nh.ok()){
      geometry_msgs::TransformStamped transformStamped;
      try{
        transformStamped = tfBuffer.lookupTransform("world", "cam0",
        ros::Time(0),ros::Duration(1.0));
      }
      catch (tf2::TransformException &ex) {
        ROS_WARN("%s",ex.what());
        ros::Duration(1.0).sleep();
        continue;
      }

      // calc velocity
      double timeSinceLastTf = (transformStamped.header.stamp.sec - current_gps_.header.stamp.sec) + ((int)transformStamped.header.stamp.nsec - (int)current_gps_.header.stamp.nsec)*0.000000001;
      //Diff of seconds plus diff of nanoseconds (nanosec have to be divided by 10e9) Warn nsec are unsigned so substraction will always output unsigned so need to cast to int
      if (timeSinceLastTf > 0) {
        current_gps_.twist.twist.linear.x = (transformStamped.transform.translation.x - current_gps_.pose.pose.position.x)/timeSinceLastTf;
        current_gps_.twist.twist.linear.y = (transformStamped.transform.translation.y - current_gps_.pose.pose.position.y)/timeSinceLastTf;
        current_gps_.twist.twist.linear.z = (transformStamped.transform.translation.z - current_gps_.pose.pose.position.z)/timeSinceLastTf;

        // copy pose to odom msg
        current_gps_.header = transformStamped.header;
        current_gps_.header.frame_id = "world";
        current_gps_.child_frame_id = "cam0";
        current_gps_.pose.pose.position.x = transformStamped.transform.translation.x;
        current_gps_.pose.pose.position.y = transformStamped.transform.translation.y;
        current_gps_.pose.pose.position.z = transformStamped.transform.translation.z;
        current_gps_.pose.pose.orientation = transformStamped.transform.rotation;
        OdometryCallback();
        odom_pub_.publish(current_gps_);
      }

      ros::spinOnce();
      rate.sleep();
    }
  }


  // Callbacks
  void PositionControllerNode::WaypointCallback(const mav_msgs::CommandTrajectoryConstPtr& trajectory_reference_msg){

    //ROS_WARN("Position_controller_node got first Waypoint message.");

    // Update desired waypoint
    wp = *trajectory_reference_msg;

  }

  void PositionControllerNode::OdometryCallback(){

    //ROS_INFO_ONCE("Position_controller_node got first GPS message.");
    //Publish if in GPS mode
    if(wp.jerk.x){
      //Position Control Loop
      position_controller_.CalculatePositionControl(wp, current_gps_, &control_msg_);
      Publish();
    }
  }

}

//Main
int main(int argc, char** argv) {

  ros::init(argc, argv, "position_controller_node");

  quad_control::PositionControllerNode position_controller_node;
  position_controller_node.Run();
  // ros::spin();

  return 0;
}
