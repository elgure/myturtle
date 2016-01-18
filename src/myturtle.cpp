/*
The MIT License (MIT)

Copyright (c) 2016 Lecomte Tristan

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all
copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
SOFTWARE.
*/

#include "ros/ros.h"
#include "geometry_msgs/Twist.h"
#include "turtlesim/Pose.h"
#include <sstream>

#define V1 6.1
#define V2 6.2
#define V3 6.3

#define CW  0
#define CCW 1

using namespace std;

ros::Publisher velocity_publisher;
ros::Subscriber pose_subscriber;	// to determine the position for turning the robot in an absolute orientation --> in the setDesiredOrientation fn
turtlesim::Pose turtlesim_pose;

int i = 0;
int direction = 0;
int velocity = 0;

const double PI = 3.14159265359;

void move(double speed, double distance, bool isForward);
void rotate(double angular_speed, double angle, bool cloclwise);	//this will rotate the turtle at specified angle from its current angle
double degrees2radians(double angle_in_degrees);		
double setDesiredOrientation(double desired_angle_radians);	//this will rotate the turtle at an absolute angle, whatever its current angle is
void poseCallback(const turtlesim::Pose::ConstPtr & pose_message);	//Callback fn everytime the turtle pose msg is published over the /turtle1/pose topic.
void turtleSpin(int direction,int velocity);

int main(int argc, char **argv)
{
	// Initiate new ROS node named "talker"
	ros::init(argc, argv, "turtlesim_cleaner");
	ros::NodeHandle n;
	double speed, angular_speed;
	double distance, angle;
	bool isForward, clockwise;

	velocity_publisher = n.advertise<geometry_msgs::Twist>("/turtle1/cmd_vel", 1000);
	pose_subscriber = n.subscribe("/turtle1/pose", 10, poseCallback);	//call poseCallback everytime the turtle pose msg is published over the /turtle1/pose topic.
	ros::Rate loop_rate(0.1);

	ROS_INFO("\n\n\n ********START TESTING*********\n");

 ros::Rate loop(0.5);
 for(i=0; i < 9; i++)  // 48 when 6.3 otherwise 9
 {
 	cout << i << endl;
 	turtleSpin(CW, V1);  // change both turtleSpin between V1 V2 V3 for differente output
 	loop.sleep();
 	turtleSpin(CCW, V1); // change both turtleSpin between V1 V2 V3 for differente output
 	loop.sleep();
 }
	ros::spin();
	return 0;
}

/*
	Degree to radians conversion.
 */
double degrees2radians(double angle_in_degrees)
{
	return angle_in_degrees *PI /180.0;
}

/*
 	Callback function to update the pose of the turtle.
 */
void poseCallback(const turtlesim::Pose::ConstPtr & pose_message)
{
	turtlesim_pose.x=pose_message->x;
	turtlesim_pose.y=pose_message->y;
	turtlesim_pose.theta=pose_message->theta;
}

/*
	Makes spin the robot in a definite direction with
	a definite velocity.
*/

void turtleSpin(int direction,int velocity)
{
	geometry_msgs::Twist vel_msg;

	double constant_speed = velocity; 
	double radius = 10;
	ros::Rate loop(1);

	if (direction == CW)
		vel_msg.linear.x =  radius;
	else 
		vel_msg.linear.x = -radius;
		vel_msg.linear.y = 0;
		vel_msg.linear.z = 0;

		vel_msg.angular.x = 0;
		vel_msg.angular.y = 0;
		vel_msg.angular.z = constant_speed;

	velocity_publisher.publish(vel_msg);
	ros::spinOnce();

	loop.sleep();

	velocity_publisher.publish(vel_msg);
}

/*  TURTLESIM USEFULL FUNCTION  */
/*        NOT USED ATM          */

/*
   move the turtle with a given linear velocity (speed)
   for a define distance and a straight direction (isForward).
   isForward = true -> forward, !isForward -> backward 
   Not use ATM
 */
void move(double speed, double distance, bool isForward)
{
	geometry_msgs::Twist vel_msg;
	//set a define (speed) linear velocity in the x-axis
	if (isForward)
		vel_msg.linear.x =abs(speed);
	else
		vel_msg.linear.x =-abs(speed);
	vel_msg.linear.y =0;
	vel_msg.linear.z =0;
	//set a the angular velocity in the y-axis
	//straight motion = no angular velocity
	vel_msg.angular.x = 0;
	vel_msg.angular.y = 0;
	vel_msg.angular.z = 0;

	double t0 = ros::Time::now().toSec();
	double current_distance = 0.0;
	ros::Rate loop_rate(100);
	do{
		velocity_publisher.publish(vel_msg);
		double t1 = ros::Time::now().toSec();
		current_distance = speed * (t1-t0);
		ros::spinOnce();
		loop_rate.sleep();
		//cout<<(t1-t0)<<", "<<current_distance <<", "<<distance<<endl;
	}while(current_distance<distance);
	vel_msg.linear.x =0;
	velocity_publisher.publish(vel_msg);
}

/*
   makes the turtle turn with a certain angular velocity (angular_speed), 
   to a definite angle (relative_angle), CW or CCW (bool clockwise).
   Not use ATM
 */
void rotate (double angular_speed, double relative_angle, bool clockwise)
{
	geometry_msgs::Twist vel_msg;
	//set the linear velocity in the x-axis
	vel_msg.linear.x =0;
	vel_msg.linear.y =0;
	vel_msg.linear.z =0;
	//set the angular velocity in the y-axis
	vel_msg.angular.x = 0;
	vel_msg.angular.y = 0;
	if (clockwise)
		vel_msg.angular.z =-abs(angular_speed);
	else
	 	vel_msg.angular.z =abs(angular_speed);

	double t0 = ros::Time::now().toSec();
	double current_angle = 0.0;
	ros::Rate loop_rate(1000);
	do{
		velocity_publisher.publish(vel_msg);
		double t1 = ros::Time::now().toSec();
		current_angle = angular_speed * (t1-t0);
		ros::spinOnce();
		loop_rate.sleep();
		//cout<<(t1-t0)<<", "<<current_angle <<", "<<relative_angle<<endl;
	}while(current_angle<relative_angle);
	vel_msg.angular.z =0;
	velocity_publisher.publish(vel_msg);
}

/*
	Turns the robot to a desried absolute angle.
 	Not use ATM
 */
double setDesiredOrientation(double desired_angle_radians)
{	
	double relative_angle_radians = desired_angle_radians - turtlesim_pose.theta;
	//if we want to turn at a perticular orientation, we subtract the current orientation from it
	bool clockwise = ((relative_angle_radians<0)?true:false);
	//cout<<desired_angle_radians <<","<<turtlesim_pose.theta<<","<<relative_angle_radians<<","<<clockwise<<endl;
	rotate (abs(relative_angle_radians), abs(relative_angle_radians), clockwise);
}