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

namespace quad_control {

  PositionControllerNode::PositionControllerNode(){
    InitializeParams();
  }


  PositionControllerNode::~PositionControllerNode() {}

  void PositionControllerNode::InitializeParams(){

    ros::NodeHandle pnh("~");
    position_controller_.InitializeParameters(pnh);

    wp.position.x = 0.0;
    wp.position.y = 0.0;
    wp.position.z = 1;
    wp.orientation.z = 0.0;
  }

  void PositionControllerNode::Publish(){

    geometry_msgs::Twist twist;
    twist.linear.x = control_msg_.angular.y;
    twist.linear.y = -control_msg_.angular.x;
    twist.linear.z = control_msg_.linear.z;
    twist.angular.z = control_msg_.angular.z;

    printf("x: %f, y: %f, z: %f, yaw: %f\n", twist.linear.x, twist.linear.y, twist.linear.z, twist.angular.z);
    bebop_topic.publish(twist);

  }

  void PositionControllerNode::Run(){
    tf2_ros::Buffer tfBuffer;
    tf2_ros::TransformListener tfListener(tfBuffer);
    ros::NodeHandle nh;

    // Subscribers
    cmd_trajectory_sub_ = nh.subscribe("command/waypoint", 1, &PositionControllerNode::WaypointCallback, this);
    landed_sub = nh.subscribe("/bebop/states/ardrone3/PilotingState/FlyingStateChanged", 1, &PositionControllerNode::LandedCallback, this);

    //Publishers
    odom_pub_ = nh.advertise<nav_msgs::Odometry>("odometry", 1);
    bebop_topic = nh.advertise<geometry_msgs::Twist>("/bebop/cmd_vel", 1);
    flyingState = bebop_msgs::Ardrone3PilotingStateFlyingStateChanged::state_hovering;

    double lastdX, lastdY, lastdZ;
    double rollAngle, pitchAngle, yawAngle;
    tf::Quaternion q;
    geometry_msgs::TransformStamped transformStamped;
    double timeSinceLastTf;
    double zShift = 0.1; //Left camera used as origin has position [0.05,0.1,0.1]m compared to the center of the drone/SLAMdunk assembly but we want to point the center of the slamdunk [0,0,0.1]m

    ros::Rate rate(20.0);
    while (nh.ok()){
      try{ //Get position from TF
        transformStamped = tfBuffer.lookupTransform("world", "cam0",
        ros::Time(0),ros::Duration(1.0));
      }
      catch (tf2::TransformException &ex) {
        ROS_WARN("%s",ex.what());
        ros::Duration(1.0).sleep();
        continue;
      }

      // calc velocity
      timeSinceLastTf = (transformStamped.header.stamp.sec - current_gps_.header.stamp.sec) + ((int)transformStamped.header.stamp.nsec - (int)current_gps_.header.stamp.nsec)*0.000000001;
      //Diff of seconds plus diff of nanoseconds (nanosec have to be divided by 10e9) Warn nsec are unsigned so substraction will always output unsigned so need to cast to int
      if (timeSinceLastTf > 0) {
        timeSinceLastTf = 0.05;

        { //Shift oringin point towards center of slamdunk
          // tf:quaternionMsgToTF(transformStamped.transform.rotation, q);
          // tf::Matrix3x3(q).getRPY(rollAngle, pitchAngle, yawAngle);
          // double yShift = 0/*0.1 * cos(yawAngle) + 0.05 * sin(yawAngle)*/;
          // double xShift = 0/*0.05 * cos(yawAngle) + 0.1 * sin(yawAngle)*/;
          // transformStamped.transform.translation.x += xShift;
          // transformStamped.transform.translation.y += yShift;
          transformStamped.transform.translation.z += zShift;
        }

        { //Average speeds to avoid hiccups
          //current_gps_.twist.twist.linear.x = lastdX;
          //current_gps_.twist.twist.linear.y = lastdY;
          //current_gps_.twist.twist.linear.z = lastdZ;
          lastdX = (transformStamped.transform.translation.x - current_gps_.pose.pose.position.x)/timeSinceLastTf;
          lastdY = (transformStamped.transform.translation.y - current_gps_.pose.pose.position.y)/timeSinceLastTf;
          lastdZ = (transformStamped.transform.translation.z - current_gps_.pose.pose.position.z)/timeSinceLastTf;

          current_gps_.twist.twist.linear.x += lastdX;
          current_gps_.twist.twist.linear.y += lastdY;
          current_gps_.twist.twist.linear.z += lastdZ;
          current_gps_.twist.twist.linear.x /= 2;
          current_gps_.twist.twist.linear.y /= 2;
          current_gps_.twist.twist.linear.z /= 2;
        }
        printf("Speed: x %f, y %f, z %f, dt %f\n", current_gps_.twist.twist.linear.x, current_gps_.twist.twist.linear.y, current_gps_.twist.twist.linear.z, timeSinceLastTf);

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
  void PositionControllerNode::WaypointCallback(const geometry_msgs::PoseConstPtr& trajectory_reference_msg){
    // Update desired waypoint
    wp = *trajectory_reference_msg;
  }

  void PositionControllerNode::OdometryCallback(){
    //Publish if told to
    if(wp.orientation.x){
      //Position Control Loop
      position_controller_.CalculatePositionControl(wp, current_gps_, &control_msg_);
      //Do not publish anything if drone not in a stable flying phase
      if (flyingState != bebop_msgs::Ardrone3PilotingStateFlyingStateChanged::state_flying && flyingState != bebop_msgs::Ardrone3PilotingStateFlyingStateChanged::state_hovering  && flyingState != bebop_msgs::Ardrone3PilotingStateFlyingStateChanged::state_takingoff) {
        return;
      }
      Publish();
    }
  }

  void PositionControllerNode::LandedCallback(const bebop_msgs::Ardrone3PilotingStateFlyingStateChangedPtr& state_ptr){
    flyingState = state_ptr->state;
  }
}

//Main
int main(int argc, char** argv) {

  ros::init(argc, argv, "position_controller_node");

  quad_control::PositionControllerNode position_controller_node;
  position_controller_node.Run();

  return 0;
}
