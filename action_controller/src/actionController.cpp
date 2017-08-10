#include <ros/ros.h>
#include <actionlib/server/action_server.h>
#include <pthread.h>

#include <trajectory_msgs/MultiDOFJointTrajectory.h>
#include <action_controller/MultiDofFollowJointTrajectoryAction.h>
#include <geometry_msgs/Twist.h>

#include <tf/transform_broadcaster.h>
#include <tf/transform_datatypes.h>
#include <mav_msgs/CommandTrajectory.h>
#include <nav_msgs/Odometry.h>

class Controller{
private:
	typedef actionlib::ActionServer<action_controller::MultiDofFollowJointTrajectoryAction> ActionServer;
	typedef ActionServer::GoalHandle GoalHandle;
public:
	Controller(ros::NodeHandle &n) :
	node_(n),
	action_server_(node_, "multi_dof_joint_trajectory_action",
	boost::bind(&Controller::goalCB, this, _1),
	boost::bind(&Controller::cancelCB, this, _1),
	false),
	has_active_goal_(false)
	{
		created=0;
		trajectory_pub = node_.advertise<mav_msgs::CommandTrajectory>("/cmd_3dnav", 10);
		odometry_sub_ = node_.subscribe("/quad/odometry", 10, &Controller::OdometryCallback, this);
		action_server_.start();

		printf("\n\n Node ready! \n\n");
	}
private:
	ros::NodeHandle node_;
	ActionServer action_server_;
	ros::Subscriber odometry_sub_;

	ros::Publisher trajectory_pub;
	mav_msgs::CommandTrajectory desired_wp;

	tf::Quaternion q;
	double des_roll, des_pitch, des_yaw;

	geometry_msgs::Twist empty;
	geometry_msgs::Transform_<std::allocator<void> > currentPosition;
	double xError, yError, zError;
	pthread_t trajectoryExecutor;
	int created;

	bool has_active_goal_;
	GoalHandle active_goal_;
	trajectory_msgs::MultiDOFJointTrajectory_<std::allocator<void> > toExecute;


	void cancelCB(GoalHandle gh){
		if (active_goal_ == gh)
		{
			// Stops the controller.
			if(created){
				printf("Stop thread \n");
				pthread_cancel(trajectoryExecutor);
				created=0;
			}
			mav_msgs::CommandTrajectory emptyTrajectory;
			trajectory_pub.publish(emptyTrajectory);

			// Marks the current goal as canceled.
			active_goal_.setCanceled();
			has_active_goal_ = false;
		}
	}

	void goalCB(GoalHandle gh){
		if (has_active_goal_)
		{
			// Stops the controller.
			if(created){
				pthread_cancel(trajectoryExecutor);
				created=0;
			}
			mav_msgs::CommandTrajectory emptyTrajectory;
			trajectory_pub.publish(emptyTrajectory);

			// Marks the current goal as canceled.
			active_goal_.setCanceled();
			has_active_goal_ = false;
		}

		gh.setAccepted();
		active_goal_ = gh;
		has_active_goal_ = true;
		toExecute = gh.getGoal()->trajectory;


		if(pthread_create(&trajectoryExecutor, NULL, threadWrapper, this)==0){
			created=1;
			printf("Thread for trajectory execution created \n");
		} else {
			printf("Thread creation failed! \n");
		}

	}

	static void* threadWrapper(void* arg) {
		Controller * mySelf=(Controller*)arg;
		mySelf->executeTrajectory();
		return NULL;
	}

	void OdometryCallback(const nav_msgs::OdometryConstPtr& odometry_msg){
		currentPosition.translation.x = (*odometry_msg).pose.pose.position.x;
		currentPosition.translation.y = (*odometry_msg).pose.pose.position.y;
		currentPosition.translation.z = (*odometry_msg).pose.pose.position.z;
	}

	void executeTrajectory(){
		if(toExecute.joint_names[0]=="virtual_joint" && toExecute.points.size()>0){
			for(int k=0; k<toExecute.points.size(); k++){

				geometry_msgs::Transform_<std::allocator<void> > waypoint=toExecute.points[k].transforms[0];

				desired_wp.position = waypoint.translation;

				//Convert quaternion to Euler angles
				tf:quaternionMsgToTF(waypoint.rotation, q);
				tf::Matrix3x3(q).getRPY(des_roll, des_pitch, des_yaw);

				//Cause the drone to point itself (and the camera) towards where it's going
				if(k+1 <toExecute.points.size()){
					// atan2( y_err, x_err)
					desired_wp.yaw = atan2((toExecute.points[k+1].transforms[0].translation.y - waypoint.translation.y), (toExecute.points[k+1].transforms[0].translation.x-waypoint.translation.x));
				}else{
					desired_wp.yaw = des_yaw;
				}

				desired_wp.jerk.x = 1;

				desired_wp.header.stamp = ros::Time::now();
				desired_wp.header.frame_id = "3dnav_action_frame";
				trajectory_pub.publish(desired_wp);

				while( calcDistanceToWaypoint(currentPosition.translation, waypoint.translation) > 0.5 ){ //Wait for waypoint to be reached
					ros::Duration(0.1).sleep();
				}
			}
		}
		active_goal_.setSucceeded();
		has_active_goal_=false;
		created=0;
	}

	double calcDistanceToWaypoint(geometry_msgs::Vector3 position, geometry_msgs::Vector3 waypoint){
		xError = (position.x - waypoint.x) * (position.x - waypoint.x);
		yError = (position.y - waypoint.y) * (position.y - waypoint.y);
		zError = (position.z - waypoint.z) * (position.z - waypoint.z);
		return (xError + yError + zError);
	}

};

int main(int argc, char** argv)
{
	ros::init(argc, argv, "action_controller_node");
	ros::NodeHandle node;//("~");
	Controller control(node);

	ros::spin();

	return 0;
}
