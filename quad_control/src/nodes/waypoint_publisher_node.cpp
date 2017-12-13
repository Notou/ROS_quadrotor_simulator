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

#include "waypoint_publisher_node.h"

namespace quad_control {

  WaypointPublisherNode::WaypointPublisherNode(){
    InitializeParams();
    ros::NodeHandle nh;

    // Subscribers
    cmd_pos_sub_ = nh.subscribe("command/trajectory", 11, &WaypointPublisherNode::CommandTrajectoryCallback, this);
    odometry_sub_ = nh.subscribe("odometry", 1, &WaypointPublisherNode::OdometryCallback, this);
    cmd_threednav_sub_ = nh.subscribe("/cmd_3dnav", 1, &WaypointPublisherNode::threedNavCallback, this);
    quality_sub_ = nh.subscribe("/quality", 1, &WaypointPublisherNode::qualityCallback, this);

    //Publisher
    trajectory_pub = nh.advertise<mav_msgs::CommandTrajectory>("command/waypoint", 10);
    takeoff_pub = nh.advertise<std_msgs::Empty>("/bebop/takeoff", 10);
    land_pub = nh.advertise<std_msgs::Empty>("/bebop/land", 10);

    ROS_INFO_ONCE("Started Waypoint Publisher.");

  }


  WaypointPublisherNode::~WaypointPublisherNode() {}

  void WaypointPublisherNode::InitializeParams(){
    qualityState = 1;
    current_time = start_time = ros::Time::now().toSec();

    ROS_INFO("Waypoint_publisher_node Paramters Initialized.");

  }

  void WaypointPublisherNode::CommandTrajectoryCallback(const mav_msgs::CommandTrajectoryConstPtr& command_trajectory_msg){
    command_trajectory = *command_trajectory_msg;
  }

  void WaypointPublisherNode::threedNavCallback(const mav_msgs::CommandTrajectoryConstPtr& threed_nav_msg){
    threedNav_trajectory = *threed_nav_msg;
  }

  void WaypointPublisherNode::qualityCallback(slamdunk_msgs::QualityStamped::ConstPtr const& quality){
    qualityState = (*quality).quality.value;
  }

  void WaypointPublisherNode::OdometryCallback(const nav_msgs::OdometryConstPtr& odometry_msg){
    current_gps_ = *odometry_msg;

    //Convert quaternion to Euler angles
    tf:quaternionMsgToTF(current_gps_.pose.pose.orientation, q);
    tf::Matrix3x3(q).getRPY(gps_roll, gps_pitch, gps_yaw);

    //Maintain pitch while maneuvering
    if(fabs(command_trajectory.position.x) >= .01){
      desired_wp.position.x = current_gps_.pose.pose.position.x + command_trajectory.position.x/4;
    }
    //Maintain roll while maneuvering
    if(fabs(command_trajectory.position.y) >= .01){
      desired_wp.position.y = current_gps_.pose.pose.position.y + command_trajectory.position.y/4;
    }
    //Maintain yaw while maneuvering
    if(fabs(command_trajectory.yaw) >= .01){
      desired_wp.yaw = gps_yaw + command_trajectory.yaw;
    }
    //Maintain altitude while maneuvering
    if(fabs(command_trajectory.position.z) >= .01){
      desired_wp.position.z = current_gps_.pose.pose.position.z + command_trajectory.position.z;
    }

    //Launch mode
    if(command_trajectory.snap.x){
      printf("Taking off!\n");
      std_msgs::Empty empty;
      desired_wp.position.x = current_gps_.pose.pose.position.x;
      desired_wp.position.y = current_gps_.pose.pose.position.y;
      desired_wp.position.z = 1.0;
      desired_wp.yaw = gps_yaw;
      desired_wp.jerk.x = 1;	//Set flag for position controller
      takeoff_pub.publish(empty);
      //ros::Duration(0.5).sleep();

      command_trajectory.snap.x=0;//reset command
    }

    if(qualityState == 0 || qualityState >= 3){
      printf("Bad slam quality: %d\n", qualityState);
      if (!command_trajectory.jerk.x) {
        start_time = ros::Time::now().toSec();
        command_trajectory.jerk.x=1;
        desired_wp.jerk.x = 0;
      }
        current_time = ros::Time::now().toSec();
        if((current_time-start_time) > 10){
          command_trajectory.snap.y = 1;
        }
    }else{
    	if(command_trajectory.jerk.x){
   	     command_trajectory.jerk.x=0;
        desired_wp.jerk.x = 1;
      }
    }
    //Land mode
    if(command_trajectory.snap.y){
      printf("Landing\n" );
      std_msgs::Empty empty;
      land_pub.publish(empty);
      desired_wp.position.x = current_gps_.pose.pose.position.x;
      desired_wp.position.y = current_gps_.pose.pose.position.y;
      desired_wp.position.z = 0.0;
      desired_wp.yaw = gps_yaw;
      //desired_wp.jerk.x = 0;	//Set flag for position controller
      trajectory_pub.publish(desired_wp);
      land_pub.publish(empty);
      ros::Duration(0.5).sleep();  //Ensure time for drone to actually land
      land_pub.publish(empty);  //To be really sure

      command_trajectory.snap.y=0; //reset command
    }

    is3DNav = is3DNav != command_trajectory.snap.z;

    if(is3DNav){

      if( (threedNav_trajectory.position.x==0.0) && (threedNav_trajectory.position.y==0.0) && (threedNav_trajectory.position.z==0.0)){
        printf("3D trajectory activated but is full of 0s\n");
      }else{
        desired_wp.position.x = threedNav_trajectory.position.x;
        desired_wp.position.y = threedNav_trajectory.position.y;
        desired_wp.position.z = threedNav_trajectory.position.z;
        desired_wp.yaw = threedNav_trajectory.yaw;
      }
    }
    desired_wp.header.stamp = ros::Time::now();
    desired_wp.header.frame_id = "desired_waypoint_frame";
    trajectory_pub.publish(desired_wp);
  }

}

//Main
int main(int argc, char** argv) {
  ros::init(argc, argv, "waypoint_publisher_node");
  quad_control::WaypointPublisherNode waypoint_publisher_node;
  ros::spin();

  return 0;
}
