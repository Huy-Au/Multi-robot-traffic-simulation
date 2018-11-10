#include "ros/ros.h"
#include "geometry_msgs/Twist.h"
#include "sensor_msgs/LaserScan.h"
#include <tf/transform_listener.h>
#include "nav_msgs/Odometry.h"
#include <boost/lexical_cast.hpp>
#include <math.h>
#include <string>
//#include <tuple>
using namespace std;

#define MAX_ROBOTS_NUM 20
#define FORWARD_SPEED 0.4
#define MIN_SCAN_ANGLE -1.0/180*M_PI
#define MAX_SCAN_ANGLE +1.0/180*M_PI
#define MIN_PROXIMITY_RANGE 0.5

struct WorldCoord{
	double x;
	double y;
	double angle;
};


int robot_id;
ros::Publisher cmd_vel_pub; // publisher for movement commands
ros::Subscriber laser_scan_sub; // subscriber to the robot's laser scan topic
bool keepMoving = true;
string tf_prefix;

void move_forward();
void goToGoal(char* id, double x, double y);
void scanCallback(const sensor_msgs::LaserScan::ConstPtr& scan);
WorldCoord getRobotPosition(char* robot_id);
void goToIntersection();


int main(int argc, char **argv)
{
	if (argc < 2) {
		ROS_ERROR("You must specify robot id and goal.");
		return -1;
	}

	char *id = argv[1];
	robot_id = atoi(id);

	// Check that robot id is between 0 and MAX_ROBOTS_NUM
	if (robot_id > MAX_ROBOTS_NUM || robot_id < 0 ) {
	    ROS_ERROR("The robot's ID must be an integer number between 0 an 40");
	    return -1;
	}

	double goal_x, goal_y;
	goal_x = atof(argv[2]);
    goal_y = atof(argv[3]);

	ROS_INFO("moving robot no. %d\n", robot_id);

	// Create a unique node name
	string node_name = "move_robot_";
	node_name += id;
	cout << node_name;

	ros::init(argc, argv, node_name);
	ros::NodeHandle nh;

	// Get tf_prefix from the parameter server
	nh.getParam("tf_prefix", tf_prefix);

	// cmd_vel_topic = "robot_X/cmd_vel"
	string cmd_vel_topic_name = "tb";
	cmd_vel_topic_name += id;
	cmd_vel_topic_name += "/cmd_vel";
	cmd_vel_pub = nh.advertise<geometry_msgs::Twist>(cmd_vel_topic_name, 10);
	
	double distance;
	double angle, adjangle;
	ros::Rate rate(10);


	long adjCounter = 0;
	do{

		WorldCoord mypos = getRobotPosition(id);
		ROS_INFO("My x: %f, my y: %f, my angle: %f", mypos.x, mypos.y, mypos.angle);
		distance = sqrt(pow(goal_x - mypos.x, 2) + pow(goal_y - mypos.y, 2));	
		ROS_INFO("Distance to goal: %f", distance);
		angle = atan2(goal_y - mypos.y, goal_x - mypos.x);


		if (angle < 0) {
			angle = 2*M_PI + angle;
			// Bad solution to edge case of 0 degrees where minor fluctuations can jump angle from
			// 0 to 6.28
			if (angle > 6.1) {
				angle = 0;
			}
		}
		// Requires old tf frame to compare with, first call will be an error second call will save frame to be compared with
		if(adjCounter < 2){
			adjCounter++;
			continue;
		}
		ROS_INFO("Angle to goal is %f", angle);
		geometry_msgs::Twist cmd_vel;
		cmd_vel.linear.x = FORWARD_SPEED;
		if (adjCounter % 2 == 0){
			adjangle = angle - mypos.angle;
			cmd_vel.angular.z = 2*adjangle;
		} else {
			cmd_vel.angular.z = 0;
		}
		cmd_vel_pub.publish(cmd_vel);
		ros::spinOnce(); // Need to call this function often to allow ROS to process incoming messages
		rate.sleep();		
		adjCounter++;
	} while (distance > 0.3);

	// Stop the robot
	geometry_msgs::Twist stop_cmd_vel;
	stop_cmd_vel.linear.x = 0.0;
	stop_cmd_vel.angular.z = 0.0;
	cmd_vel_pub.publish(stop_cmd_vel);
	ROS_INFO("robot no. %d stopped", robot_id);

	return 0;
}

WorldCoord getRobotPosition(char* robot_id)
{
    tf::TransformListener listener;
    tf::StampedTransform transform;
	WorldCoord results;
	double angle;

    try {
    	string robot_str = "/tb";
    	robot_str += boost::lexical_cast<string>(robot_id);
    	string base_footprint_frame = tf::resolve(robot_str, "base_footprint");
		ROS_INFO("==============\n%s\n", base_footprint_frame.c_str());

        listener.waitForTransform("/map", base_footprint_frame, ros::Time(0), ros::Duration(10.0));
        listener.lookupTransform("/map", base_footprint_frame, ros::Time(0), transform);

        results.x = transform.getOrigin().x();
        results.y = transform.getOrigin().y();
		angle = tf::getYaw(transform.getRotation());
		if (angle < 0){
			angle = 2*M_PI + angle;
		}
		results.angle = angle;

    }
    catch (tf::TransformException &ex) {
        ROS_ERROR("%s",ex.what());
    }
    return results;
}

void goToIntersection()
{





}


/**
void goToGoal(char* id, double x, double y)
{
	double yaw, distance;
	WorldCoord goal;
	//Facing forward
	if (x == 1.0 && y == -0.25){
		yaw = M_PI;
	} else if (x == 0.25 && y == 1){ //facing left
		yaw = 3*M_PI/2;
	} else if(x == -1 && y == 0.25){ //facing down
		yaw = 0;
	} else{ //facing right
		yaw = M_PI/2;
	}
	goal = getRobotPosition(id);
	distance = sqrt(pow(x - goal.x, 2) + pow(y - goal.y, 2));
}


void move_forward()
{
	// Drive forward at a given speed.
	geometry_msgs::Twist cmd_vel;
	cmd_vel.linear.x = FORWARD_SPEED;
	cmd_vel.angular.z = 0.0;
	cmd_vel_pub.publish(cmd_vel);
	ros::spinOnce(); // Need to call this function often to allow ROS to process incoming messages
	rate.sleep();

	// Loop at 10Hz, publishing movement commands until we shut down
	ros::Rate rate(10);

	while (ros::ok() && keepMoving) // Keep spinning loop until user presses Ctrl+C
	{
		cmd_vel_pub.publish(cmd_vel);
		ros::spinOnce(); // Need to call this function often to allow ROS to process incoming messages
		rate.sleep();
	}

	// Stop the robot
	geometry_msgs::Twist stop_cmd_vel;
	stop_cmd_vel.linear.x = 0.0;
	stop_cmd_vel.angular.z = 0.0;
	cmd_vel_pub.publish(stop_cmd_vel);

	ROS_INFO("robot no. %d stopped", robot_id);

}





// Process the incoming laser scan message
void scanCallback(const sensor_msgs::LaserScan::ConstPtr& scan)
{
	// Find the closest range between the defined minimum and maximum angles
	int minIndex = ceil((MIN_SCAN_ANGLE - scan->angle_min) / scan->angle_increment);
	int maxIndex = floor((MAX_SCAN_ANGLE - scan->angle_min) / scan->angle_increment);

	float closestRange = scan->ranges[minIndex];
	for (int currIndex = minIndex + 1; currIndex <= maxIndex; currIndex++) {
		if (scan->ranges[currIndex] < closestRange) {
			closestRange = scan->ranges[currIndex];
		}
	}

	//ROS_INFO_STREAM("Closest range: " << closestRange);

	if (closestRange < MIN_PROXIMITY_RANGE) {
		ROS_INFO("Collision imminent for robot");
		keepMoving = false;
	}
}
*/
