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
    
    ros::NodeHandle nh;
    InitializeParams();

    // Subscribers
    cmd_pos_sub_ = nh.subscribe("command/trajectory", 11, &WaypointPublisherNode::CommandTrajectoryCallback, this);
    odometry_sub_ = nh.subscribe("odometry", 1, &WaypointPublisherNode::OdometryCallback, this);
    cmd_threednav_sub_ = nh.subscribe("/cmd_3dnav", 1, &WaypointPublisherNode::threedNavCallback, this);
    quality_sub_ = nh.subscribe("/quality", 1, &WaypointPublisherNode::qualityCallback, this);

    //Publisher
    trajectory_pub = nh.advertise<geometry_msgs::Pose>("command/waypoint", 10);
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

  void WaypointPublisherNode::CommandTrajectoryCallback(const geometry_msgs::TwistConstPtr& command_trajectory_msg){
    command_trajectory = *command_trajectory_msg;
  }

  void WaypointPublisherNode::threedNavCallback(const geometry_msgs::PoseConstPtr& threed_nav_msg){
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
    if(fabs(command_trajectory.linear.x) >= .01){
      desired_wp.position.x = current_gps_.pose.pose.position.x + command_trajectory.linear.x;
    }
    //Maintain roll while maneuvering
    if(fabs(command_trajectory.linear.y) >= .01){
      desired_wp.position.y = current_gps_.pose.pose.position.y + command_trajectory.linear.y;
    }
    //Maintain yaw while maneuvering
    if(fabs(command_trajectory.angular.z) >= .01){
      desired_wp.orientation.z = gps_yaw + command_trajectory.angular.z;
    }
    //Maintain altitude while maneuvering
    if(fabs(command_trajectory.linear.z) >= .01){
      desired_wp.position.z = current_gps_.pose.pose.position.z + command_trajectory.linear.z;
    }

    //Takeoff mode
    if(command_trajectory.angular.x == 1){
      printf("Taking off!\n");
      std_msgs::Empty empty;
      desired_wp.position.x = current_gps_.pose.pose.position.x;
      desired_wp.position.y = current_gps_.pose.pose.position.y;
      desired_wp.position.z = 1.0;
      desired_wp.orientation.z = gps_yaw;
      desired_wp.orientation.x = 1;	//Set flag for position controller to start running
      takeoff_pub.publish(empty);
      //ros::Duration(0.5).sleep();

      command_trajectory.angular.x=0;//reset command
    }

    if(qualityState == 0 || qualityState >= 3){
      printf("Bad slam quality: %d\n", qualityState);
      if (!isLost) {
        start_time = ros::Time::now().toSec();
        isLost = true;
        desired_wp.orientation.x = 0; //Set flag for position controller to stop running
      }
        current_time = ros::Time::now().toSec();
        if((current_time-start_time) > 10){
          command_trajectory.angular.x = -1; //Land
        }
    }else{
    	if(isLost){
   	     isLost = false;
        desired_wp.orientation.x = 1;
      }
    }
    //Land mode
    if(command_trajectory.angular.x == -1){
      printf("Landing\n" );
      std_msgs::Empty empty;
      land_pub.publish(empty);
      desired_wp.position.x = current_gps_.pose.pose.position.x;
      desired_wp.position.y = current_gps_.pose.pose.position.y;
      desired_wp.position.z = 0.0;
      desired_wp.orientation.z = gps_yaw;
      //desired_wp.angular.x = 0;	//Set flag for position controller to stop running
      trajectory_pub.publish(desired_wp);
      land_pub.publish(empty);
      ros::Duration(0.5).sleep();  //Ensure time for drone to actually land
      land_pub.publish(empty);  //To be really sure

      command_trajectory.angular.x=0; //reset command
    }

    is3DNav = is3DNav != command_trajectory.angular.y;

    if(is3DNav){

      if( (threedNav_trajectory.position.x==0.0) && (threedNav_trajectory.position.y==0.0) && (threedNav_trajectory.position.z==0.0)){
        printf("3D trajectory activated but is full of 0s\n");
      }else{
        desired_wp.position.x = threedNav_trajectory.position.x;
        desired_wp.position.y = threedNav_trajectory.position.y;
        desired_wp.position.z = threedNav_trajectory.position.z;
        desired_wp.orientation.z = threedNav_trajectory.orientation.z;
      }
    }
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
