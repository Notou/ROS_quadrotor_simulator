#include <ros/ros.h>
#include <actionlib/server/action_server.h>
#include <pthread.h>
#include <cmath>

#include <trajectory_msgs/MultiDOFJointTrajectory.h>
#include <action_controller/MultiDofFollowJointTrajectoryAction.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Pose.h>

#include <tf/transform_broadcaster.h>
#include <tf/transform_datatypes.h>
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
		lastLook.x = 1000;
		trajectory_pub = node_.advertise<geometry_msgs::Pose>("/cmd_3dnav", 10);
		odometry_sub_ = node_.subscribe("/quad/odometry", 10, &Controller::OdometryCallback, this);
		action_server_.start();
	}
private:
	ros::NodeHandle node_;
	ActionServer action_server_;
	ros::Subscriber odometry_sub_;

	ros::Publisher trajectory_pub;
	geometry_msgs::Pose desired_wp;
	geometry_msgs::Pose waypoint;

	tf::Quaternion q;
	double des_roll, des_pitch, des_yaw;

	geometry_msgs::Twist empty;
	geometry_msgs::Pose currentPosition;
	double xError, yError, zError, yawError;
	geometry_msgs::Point lastLook;
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
			geometry_msgs::Pose emptyTrajectory;
			emptyTrajectory.position = currentPosition.position;
			emptyTrajectory.orientation.z = desired_wp.orientation.z;
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
			geometry_msgs::Pose emptyTrajectory;
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
		currentPosition.position = (*odometry_msg).pose.pose.position;
		tf:quaternionMsgToTF((*odometry_msg).pose.pose.orientation, q);
    tf::Matrix3x3(q).getRPY(currentPosition.orientation.x, currentPosition.orientation.y, currentPosition.orientation.z); //Warning! The quaternion is not used to hold quaternion values (no w is stored) but angular values instead
	}

	void executeTrajectory(){
		if(toExecute.joint_names[0]=="virtual_joint" && toExecute.points.size()>0){
			for(int k=0; k<toExecute.points.size(); k++){

				waypoint.position.x=toExecute.points[k].transforms[0].translation.x;
				waypoint.position.y=toExecute.points[k].transforms[0].translation.y;
				waypoint.position.z=toExecute.points[k].transforms[0].translation.z;
				waypoint.orientation=toExecute.points[k].transforms[0].rotation;

				//Avoid entering possible loops;
				for (size_t i = k+1; i < toExecute.points.size(); i++) {
					if(calcDistanceToWaypoint(currentPosition.position, toExecute.points[i].transforms[0].translation, desired_wp.orientation.z, desired_wp.orientation.z) < 1){
						k = i-1;
						continue;
					}
				}

				//Convert quaternion to Euler angles
				tf:quaternionMsgToTF(waypoint.orientation, q);
				tf::Matrix3x3(q).getRPY(des_roll, des_pitch, des_yaw);

				//Cause the drone to point itself (and the camera) towards where it's going
				desired_wp.orientation.z = des_yaw;
				if(k+1 <toExecute.points.size()){
					if(calcHorizontalDistanceToWaypoint(lastLook, waypoint.position) > 3){
						// atan2( y_err, x_err)
						printf("Last look: [%f, %f, %f]\n", lastLook.x, lastLook.y, lastLook.z);
						lastLook = currentPosition.position;
						desired_wp.orientation.z = atan2((toExecute.points[k+1].transforms[0].translation.y - waypoint.position.y), (toExecute.points[k+1].transforms[0].translation.x-waypoint.position.x));
					}
				}
				if(calcHorizontalDistanceToWaypoint(currentPosition.position, waypoint.position) > 0.001 && desired_wp.orientation.z != des_yaw){
					desired_wp.position = currentPosition.position;
					trajectory_pub.publish(desired_wp);
					printf("Rotating before translating: distance: %f\n",calcDistanceToWaypoint(currentPosition.position, currentPosition.position, currentPosition.orientation.z, desired_wp.orientation.z) );
					while( calcDistanceToWaypoint(currentPosition.position, currentPosition.position, currentPosition.orientation.z, desired_wp.orientation.z) > 0.3 ){ //Wait for waypoint to be reached
						printf("Rotating before translating\n");
						ros::Duration(0.3).sleep();
					}
					printf("Lookup done!");
					k--;
					continue;
				}
				desired_wp.position = waypoint.position;
				trajectory_pub.publish(desired_wp);

				while( calcDistanceToWaypoint(currentPosition.position, waypoint.position, currentPosition.orientation.z, desired_wp.orientation.z) > 1 ){ //Wait for waypoint to be reached
					ros::Duration(0.1).sleep();
				}
			}
			while( calcDistanceToWaypoint(currentPosition.position, waypoint.position, currentPosition.orientation.z, desired_wp.orientation.z) > 0.1 ){ //Wait for waypoint to be reached
					ros::Duration(0.05).sleep();
				}
		}
		active_goal_.setSucceeded();
		has_active_goal_=false;
		created=0;
	}

	double calcDistanceToWaypoint(geometry_msgs::Point position, geometry_msgs::Point waypoint, double orientation, double desired_orientation){
		xError = (position.x - waypoint.x) * (position.x - waypoint.x);
		yError = (position.y - waypoint.y) * (position.y - waypoint.y);
		zError = (position.z - waypoint.z) * (position.z - waypoint.z);
		yawError = wrap_180(desired_orientation - orientation) * wrap_180(desired_orientation - orientation);
		return sqrt(xError + yError + zError + yawError);
	}

	double calcDistanceToWaypoint(geometry_msgs::Point position, geometry_msgs::Vector3 waypoint, double orientation, double desired_orientation){
		xError = (position.x - waypoint.x) * (position.x - waypoint.x);
		yError = (position.y - waypoint.y) * (position.y - waypoint.y);
		zError = (position.z - waypoint.z) * (position.z - waypoint.z);
		yawError = wrap_180(desired_orientation - orientation) * wrap_180(desired_orientation - orientation);
		return sqrt(xError + yError + zError + yawError);
	}

	double calcHorizontalDistanceToWaypoint(geometry_msgs::Point position, geometry_msgs::Point waypoint){
		xError = (position.x - waypoint.x) * (position.x - waypoint.x);
		yError = (position.y - waypoint.y) * (position.y - waypoint.y);
		return sqrt(xError + yError);
	}

	double wrap_180(double x){ return(x < -M_PI ? x+(2*M_PI) : (x > M_PI ? x - (2*M_PI): x));}

};

int main(int argc, char** argv)
{
	ros::init(argc, argv, "action_controller_node");
	ros::NodeHandle node;//("~");
	Controller control(node);

	ros::spin();

	return 0;
}
