#include<ros/ros.h>
#include<geometry_msgs/Pose.h>
#include<iostream>
#include <stdio.h>
#include <unistd.h>
#include <termios.h>

using namespace std;
using namespace ros;

int main(int argc, char **argv)
{

	
	init(argc,argv,"p3dx_mover_initialized");
	NodeHandle nh;
	Publisher pub =nh.advertise<geometry_msgs::Pose>("RosAria/cmd_vel",1000);

	// Rate rate(10);

	geometry_msgs::Pose msg;
	msg.position.x=4.0;
	msg.position.y=0.213;
	msg.position.z=0.0;
	msg.orientation.z=-0.858577;
	msg.orientation.w=-0.51268;
	pub.publish(msg);
		
	return 0;
}