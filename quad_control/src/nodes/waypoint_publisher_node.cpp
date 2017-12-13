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

  // std::vector<quad_control::WaypointWithTime> WaypointWithTime::Read_waypoints(std::vector<quad_control::WaypointWithTime> waypoints){
  //
  //   std::ifstream wp_file("/home/wil/ros/catkin_ws/src/arducopter_slam/quad_control/resource/kitchen_short_waypoints.txt");
  //   //wg_waypoints.txt  kitchen_waypoints.txt
  //
  //   if (wp_file.is_open()) {
  //     double t, x, y, z, yaw;
  //     // Only read complete waypoints.
  //     while (wp_file >> t >> x >> y >> z >> yaw) {
  //       waypoints.push_back(WaypointWithTime(t, x, y, z, yaw * DEG_2_RAD));
  //     }
  //     wp_file.close();
  //     printf("Read %d waypoints. \n", (int )waypoints.size());
  //     return waypoints;
  //   }
  //
  //   else {
  //     ROS_ERROR_STREAM("Unable to open poses file: ");
  //     //return 0;
  //   }
  //
  // }

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

    ros::NodeHandle pnh("~");

    waypoints_read = 0;
    published = 0;
    qualityState = 1;
    current_time = ros::Time::now().toSec();
    start_time = ros::Time::now().toSec();

    ROS_INFO("Waypoint_publisher_node Paramters Initialized.");

  }

  void WaypointPublisherNode::CommandTrajectoryCallback(const mav_msgs::CommandTrajectoryConstPtr& command_trajectory_msg){
    //ROS_INFO_ONCE("Position_controller_node got first Trajectory message.");
    //Convert to Eigen
    mav_msgs::eigenCommandTrajectoryFromMsg(*command_trajectory_msg, &command_trajectory);
  }

  void WaypointPublisherNode::threedNavCallback(const mav_msgs::CommandTrajectoryConstPtr& threed_nav_msg){
    //ROS_INFO_ONCE("Position_controller_node got first 3d Nav message.");
    //Convert to Eigen
    mav_msgs::eigenCommandTrajectoryFromMsg(*threed_nav_msg, &threedNav_trajectory);
  }

  void WaypointPublisherNode::qualityCallback(slamdunk_msgs::QualityStamped::ConstPtr const& quality){
    qualityState = (*quality).quality.value;
  }

  void WaypointPublisherNode::OdometryCallback(const nav_msgs::OdometryConstPtr& odometry_msg){

    //ROS_INFO_ONCE("Position_controller_node got first GPS message.");

    current_gps_ = *odometry_msg;

    //Convert quaternion to Euler angles
    tf:quaternionMsgToTF(current_gps_.pose.pose.orientation, q);
    tf::Matrix3x3(q).getRPY(gps_roll, gps_pitch, gps_yaw);
    ROS_DEBUG("RPY = (%lf, %lf, %lf)", gps_roll, gps_pitch, gps_yaw);

    //Maintain pitch while maneuvering
    if(fabs(command_trajectory.position(0)) >= .01){
      desired_wp.position.x = current_gps_.pose.pose.position.x + command_trajectory.position(0)/4;
    }
    //Maintain roll while maneuvering
    if(fabs(command_trajectory.position(1)) >= .01){
      desired_wp.position.y = current_gps_.pose.pose.position.y + command_trajectory.position(1)/4;
    }

    //Maintain yaw while maneuvering
    if(fabs(command_trajectory.yaw) >= .01){
      desired_wp.yaw = gps_yaw + command_trajectory.yaw;
    }

    //Maintain altitude while maneuvering
    if(fabs(command_trajectory.position(2)) >= .01){
      desired_wp.position.z = current_gps_.pose.pose.position.z + command_trajectory.position(2);
    }

    //Launch mode
    if(command_trajectory.snap(0)){
      printf("Taking off!\n");
      std_msgs::Empty empty;
      desired_wp.position.x = current_gps_.pose.pose.position.x;
      desired_wp.position.y = current_gps_.pose.pose.position.y;
      desired_wp.position.z = 1.0;
      desired_wp.yaw = gps_yaw;
      desired_wp.jerk.x = 1;	//Set flag for position controller
      takeoff_pub.publish(empty);
      //ros::Duration(0.5).sleep();

      command_trajectory.snap(0)=0;//reset command
    }

    if(qualityState == 0 || qualityState >= 3){
      printf("Bad slam quality: %d\n", qualityState);
      if (!command_trajectory.jerk(0)) {
        start_time = ros::Time::now().toSec();
        command_trajectory.jerk(0)=1;
        desired_wp.jerk.x = 0;
      }
        current_time = ros::Time::now().toSec();
        if((current_time-start_time) > 10){
          command_trajectory.snap(1) = 1;
        }
    }else{
    	if(command_trajectory.jerk(0)){
   	     command_trajectory.jerk(0)=0;
        desired_wp.jerk.x = 1;
      }
    }
    //Land mode
    if(command_trajectory.snap(1)){
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

      command_trajectory.snap(1)=0; //reset command
    }

    control_mode.UpdateSwitchValue(command_trajectory.jerk(1));
    auto_mode.UpdateSwitchValue(command_trajectory.jerk(2));
    threednav_mode.UpdateSwitchValue(command_trajectory.snap(2));

    //Mission Mode triggered
    if(control_mode.GetSwitchValue()){

      ROS_INFO("Waypoint Mission Mode triggered");

      // if(!waypoints_read){
      //   waypoints = waypoint_utility.Read_waypoints(waypoints);
      //
      //   int size = waypoints.size();
      //   printf("Start publishing #%d waypoints \n", size);
      //
      //   waypoints_read = 1;
      // }
      //
      // if(i < waypoints.size()){
      //
      //   const WaypointWithTime& wp = waypoints[i];
      //
      //   if(!published){
      //
      //     printf("Publishing #%d x=%f y=%f z=%f yaw=%f, and wait for %fs. \n", (int)i, wp.wp.position.x, wp.wp.position.y, wp.wp.position.z, wp.wp.yaw, wp.waiting_time);
      //
      //     published = 1;
      //     start_time = ros::Time::now().toSec();
      //   }
      //
      //   if((current_time-start_time) < wp.waiting_time){
      //
      //     desired_wp.position.x = wp.wp.position.x;
      //     desired_wp.position.y = wp.wp.position.y;
      //     desired_wp.position.z = wp.wp.position.z;
      //     desired_wp.yaw = wp.wp.yaw;
      //
      //
      //     desired_wp.header.stamp = ros::Time::now();
      //     desired_wp.header.frame_id = "desired_mission_frame";
      //
      //     current_time = ros::Time::now().toSec();
      //
      //   }
      //   else{
      //     i++;
      //     published = 0;
      //
      //   }
      //
      // }
    }
    //Autonomous Mode triggered
    else if(auto_mode.GetSwitchValue()){

      ROS_INFO("Autonomous Mode triggered");



      desired_wp.header.stamp = ros::Time::now();
      desired_wp.header.frame_id = "desired_auto_frame";
    }
    else if(threednav_mode.GetSwitchValue()){

      ROS_INFO("3d Navigation Mode triggered");

      if(command_trajectory.jerk(0) || desired_wp.jerk.x ==0){


      }else if( (threedNav_trajectory.position(0)==0.0) && (threedNav_trajectory.position(1)==0.0) && (threedNav_trajectory.position(2)==0.0)){
        printf("3D trajectory activated but is full of 0s\n");
      }else{

        desired_wp.position.x = threedNav_trajectory.position(0);
        desired_wp.position.y = threedNav_trajectory.position(1);
        desired_wp.position.z = threedNav_trajectory.position(2);
        desired_wp.yaw = threedNav_trajectory.yaw;
      }


      desired_wp.header.stamp = ros::Time::now();
      desired_wp.header.frame_id = "3dnav_mission_frame";
    }
    //Simple GPS Mode or Mission Mode disabled
    else{
      //clear waypoints variable
      ROS_INFO("RESET");
      waypoints.clear();
      i = 0;
      waypoints_read = 0;
      published = 0;

      //modes
      desired_wp.snap.x = command_trajectory.snap(0);	// takeoff
      desired_wp.snap.y = command_trajectory.snap(1);	// land
      //desired_wp.jerk.x = command_trajectory.jerk(0);	//enable GPS
      desired_wp.jerk.y = 0.0;	// enable mission

      desired_wp.header.stamp = ros::Time::now();
      desired_wp.header.frame_id = "desired_waypoint_frame";

    }
    //printf("Position Controller enabled: %f\n", desired_wp.jerk.x);
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
