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

#include <mav_msgs/CommandTrajectory.h>
#include <mav_msgs/CommandMotorSpeed.h>
#include <mav_msgs/CommandRollPitchYawrateThrust.h>
#include <mav_msgs/MotorSpeed.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/PoseStamped.h>
#include <sensor_msgs/Imu.h>
#include <planning_msgs/WayPoint.h>
#include <planning_msgs/eigen_planning_msgs.h>
#include <planning_msgs/conversions.h>
#include <ros/ros.h>
#include <ros/callback_queue.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_datatypes.h>
#include <cmath>


namespace quad_control {


class PositionController{
  public:
   PositionController();
   ~PositionController();

  void InitializeParameters(const ros::NodeHandle& pnh);

  void CalculatePositionControl(mav_msgs::CommandTrajectory wp, nav_msgs::Odometry current_gps, mav_msgs::CommandRollPitchYawrateThrust *des_attitude_output);

  mav_msgs::CommandRollPitchYawrateThrust des_attitude_cmds;

  private:

  //General
  tf::Quaternion q;
  double gps_roll, gps_pitch, gps_yaw;
  double gps_x, gps_y, gps_z;

  ros::Time last_time;
  ros::Time sim_time;
  double dt;

  //Position Controller
  double x_er, y_er, z_er, yaw_er;
  double x_er_sum, y_er_sum, z_er_sum, yaw_er_sum;
  double cp, ci, cd;

  //X PID
  double x_KI_max, x_KP, x_KI, x_KD;

  //Y PID
  double y_KI_max, y_KP, y_KI, y_KD;

  //Z PID
  double z_KI_max, z_KP, z_KI, z_KD, z_target;

  //Yaw PID
  double yaw_KI_max, yaw_KP, yaw_KI, yaw_KD;
  double yaw_target;

  double roll_des, pitch_des, yaw_des, thrust_des;

  double acceleration_theshold;

};
}
