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

#define wrap_180(x) (x < -M_PI ? x+(2*M_PI) : (x > M_PI ? x - (2*M_PI): x))
#define stick_0(x) (x < -0.002 ? x : (x > 0.002 ? x: 0))
#define limit(x, min, max) (x < min ? min : (x > max ? max: x))
#include "quad_control/quad_controller.h"


namespace quad_control {

  //Position Controller
  PositionController::PositionController() {}
  PositionController::~PositionController() {}


  void PositionController::InitializeParameters(const ros::NodeHandle& pnh){

    //General parameters
    last_time = ros::Time::now();

    //Altitude PID
    pnh.getParam("x_PID/P", x_KP);
    pnh.getParam("x_PID/I", x_KI);
    pnh.getParam("x_PID/I_max", x_KI_max);
    pnh.getParam("x_PID/D", x_KD);

    pnh.getParam("y_PID/P", y_KP);
    pnh.getParam("y_PID/I", y_KI);
    pnh.getParam("y_PID/I_max", y_KI_max);
    pnh.getParam("y_PID/D", y_KD);

    pnh.getParam("z_PID/P", z_KP);
    pnh.getParam("z_PID/I", z_KI);
    pnh.getParam("z_PID/I_max", z_KI_max);
    pnh.getParam("z_PID/D", z_KD);

    pnh.getParam("yaw_PID/P", yaw_KP);
    pnh.getParam("yaw_PID/I", yaw_KI);
    pnh.getParam("yaw_PID/I_max", yaw_KI_max);
    pnh.getParam("yaw_PID/D", yaw_KD);

    pnh.getParam("acceleration_theshold", acceleration_theshold);


    x_er = 0;
    y_er = 0;
    z_er = 0;
    yaw_er = 0;
    x_er_sum = 0;
    y_er_sum = 0;
    z_er_sum = 0;
    yaw_er_sum = 0;
    roll_des = 0;
    pitch_des = 0;
    thrust_des = 0;
    yaw_des = 0;

  }

  void PositionController::CalculatePositionControl(geometry_msgs::Pose wp, nav_msgs::Odometry current_gps, geometry_msgs::Twist *des_attitude_output){


    //Convert quaternion to Euler angles
    tf:quaternionMsgToTF(current_gps.pose.pose.orientation, q);
    tf::Matrix3x3(q).getRPY(gps_roll, gps_pitch, gps_yaw);
    ROS_DEBUG("RPY = (%lf, %lf, %lf)", gps_roll, gps_pitch, gps_yaw);

    // Get simulator time
    sim_time = ros::Time::now();
    dt = (sim_time - last_time).toSec();
    last_time = sim_time;
    if (dt == 0.0) return;

    gps_x = current_gps.pose.pose.position.x;
    gps_y = current_gps.pose.pose.position.y;
    gps_z = current_gps.pose.pose.position.z;

    //X PID
    x_er = wp.position.x - gps_x;
    (x_er_sum > 0 && x_er < 0) ? x_er_sum = 0 :
    (x_er_sum < 0 && x_er > 0) ? x_er_sum = 0 :
    x_er_sum = x_er_sum + x_er * dt;
    x_er_sum = limit(x_er_sum, -1 * x_KI_max, x_KI_max);
    ROS_INFO("x %f max %f, y %f max %f, z %f max %f, w %f max %f\n", x_er_sum, x_KI_max, y_er_sum, y_KI_max, z_er_sum, z_KI_max, yaw_er_sum, yaw_KI_max );
    cp = x_er * x_KP;
    ci = x_KI * x_er_sum;
    cd = x_KD * current_gps.twist.twist.linear.x;
    pitch_des = cp + ci + cd;
    pitch_des = limit(pitch_des, -0.5, 0.5);
    printf("X   cp: %f, ci: %f, cd: %f    error: %f, target: %f, current: %f\n", cp, ci, cd, x_er, wp.position.x, gps_x);


    //Y PID
    y_er = wp.position.y - gps_y;
    (y_er_sum > 0 && y_er < 0) ? y_er_sum = 0 :
    (y_er_sum < 0 && y_er > 0) ? y_er_sum = 0 :
    y_er_sum = y_er_sum + y_er * dt;
    y_er_sum = limit(y_er_sum, -1 * y_KI_max, y_KI_max);

    cp = y_er * y_KP;
    ci = y_KI * y_er_sum;
    cd = y_KD * current_gps.twist.twist.linear.y;
    roll_des = cp + ci +  cd;	//Positive Y axis and roll angles inversely related
    roll_des = limit(roll_des, -0.5, 0.5);
    printf("Y   cp: %f, ci: %f, cd: %f    error: %f, target: %f, current: %f\n", cp, ci, cd, y_er, wp.position.y, gps_y);

    //Z PID
    z_er = wp.position.z - gps_z;
    (z_er_sum > 0 && z_er < 0) ? z_er_sum = 0 :
    (z_er_sum < 0 && z_er > 0) ? z_er_sum = 0 :
    z_er_sum = z_er_sum + z_er * dt;
    z_er_sum = limit(z_er_sum, -1 * z_KI_max, z_KI_max);

    cp = z_er * z_KP;
    ci = z_KI * z_er_sum;
    cd = z_KD * current_gps.twist.twist.linear.z;
    thrust_des = cp + ci + cd;
    thrust_des = limit(thrust_des, -0.5, 0.5);
    printf("Z   cp: %f, ci: %f, cd: %f    error: %f, target: %f, current: %f\n", cp, ci, cd, z_er, wp.position.z, gps_z);

    //Yaw PID
    yaw_er = wrap_180(wp.orientation.z - gps_yaw);
    yaw_er_sum = yaw_er_sum + yaw_er * dt;
    yaw_er_sum = limit(yaw_er_sum, -1 * yaw_KI_max, yaw_KI_max);

    cp = yaw_er * yaw_KP;
    ci = yaw_KI * yaw_er_sum;
    cd = yaw_KD * current_gps.twist.twist.angular.z;
    yaw_des = cp + ci + cd;
    yaw_des = limit(yaw_des, -1, 1);
    printf("YAW cp: %f, ci: %f, cd: %f    error: %f, target: %f, current: %f\n", cp, ci, cd, yaw_er, wp.orientation.z, gps_yaw);

    { // Convert to local coordinates (positions and speeds are given in a global reference frame so the PID is too)
      double x = pitch_des;
      double y = roll_des;
      pitch_des =  x*cos(gps_yaw) - y*sin(gps_yaw);
      roll_des =  x*sin(gps_yaw) + y*cos(gps_yaw);
    }

    roll_des = stick_0(roll_des);
    pitch_des = stick_0(pitch_des);
    thrust_des = stick_0(thrust_des);
    yaw_des = stick_0(yaw_des);

    //des_attitude_cmds.angular.x = roll_des;
    //des_attitude_cmds.angular.y = pitch_des;
    des_attitude_cmds.angular.z = yaw_des;
    des_attitude_cmds.linear.z = thrust_des;


    //Smooth commands to avoid vibrations
    if(fabs(des_attitude_cmds.angular.x-roll_des) > acceleration_theshold){
       des_attitude_cmds.angular.x += acceleration_theshold * (des_attitude_cmds.angular.x-roll_des > 0 ? -1 : 1);
    }else{
      	des_attitude_cmds.angular.x = roll_des;
    }

    if(fabs(des_attitude_cmds.angular.y-pitch_des) > acceleration_theshold){
       des_attitude_cmds.angular.y += acceleration_theshold * (des_attitude_cmds.angular.y-pitch_des > 0 ? -1 : 1);
    }else{
      	des_attitude_cmds.angular.y = pitch_des;
    }
    *des_attitude_output = des_attitude_cmds;

  }
}
