#include<ros/ros.h>
#include<geometry_msgs/Twist.h>
#include<iostream>
#include <stdio.h>
#include <unistd.h>
#include <termios.h>

using namespace std;
using namespace ros;

int main(int argc, char **argv)
{

	struct termios old_tio, new_tio;
    unsigned char c;

    /* get the terminal settings for stdin */
    tcgetattr(STDIN_FILENO,&old_tio);

    /* we want to keep the old setting to restore them a the end */
    new_tio=old_tio;

    /* disable canonical mode (buffered i/o) and local echo */
    new_tio.c_lflag &=(~ICANON & ~ECHO);

    /* set the new settings immediately */
    tcsetattr(STDIN_FILENO,TCSANOW,&new_tio);


	init(argc,argv,"p3dx_mover_initialized");
	NodeHandle nh;
	Publisher pub =nh.advertise<geometry_msgs::Twist>("/RosAria/cmd_vel",1000);
	cout<<"press w,a,s,d, to move the robot and nothing to keep still"<<endl;


	while(ok())
	{
		char key='z';
		//cin>>key;
		while((key=getchar())!=EOF)
		{
			geometry_msgs::Twist msg;
			switch(key)
			{
				case 'w':	msg.linear.x=-1.0;
							msg.angular.z=0.0;
							break;
				case 'a':	msg.angular.z=1.0;
							msg.linear.x=0.0;
							break;
				case 's':	msg.linear.x=1.0;
							msg.angular.z=0.0;
							break;
				case 'd':	msg.angular.z=-1.0;
							msg.linear.x=0.0;
							break;
				case 'z':	msg.angular.z=0.0;
							msg.linear.x=0.0;
							break;
				default:	msg.angular.z=0.0;
							msg.linear.x=0.0;
							break;
			}
			pub.publish(msg);
		}
		geometry_msgs::Twist msg1;
		pub.publish(msg1);
	}


	/* restore the former settings */
    tcsetattr(STDIN_FILENO,TCSANOW,&old_tio);

	return 0;
}