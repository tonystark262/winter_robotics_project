## Control of Pioneer-3DX robot using ROS
#### Duration:
from --- to ---

This document is aimed to provide a quick guide to the people who might wish to work on Pioneer robots through Robot Operating System(ROS).

#### ROS implementation to control the Poineer 3dx robot

- **Getting introduced to Robot Operating System(ROS)** \
ROS (Robot Operating System) provides libraries and tools to help software developers create robot applications. It provides hardware abstraction, device drivers, libraries, visualizers, message-passing, package management, and more. The following document was one of the good reference for us to getting introduced: [A general introduction to ROS](https://www.cse.sc.edu/~jokane/agitr/) 
- **Installing Ubuntu 16.04**\
Latest LTS version of ROS is ROS Kinetic Kame,  which is available only for Wily (Ubuntu 15.10), Xenial (Ubuntu 16.04) and Jessie (Debian 8) for debian packages. The following link helped us to install Ubuntu (16.04 LTS) in our laptops: [Tutorial for installing ubuntu alongside Windows 10](https://itsfoss.com/install-ubuntu-1404-dual-boot-mode-windows-8-81-uefi/) 
- **Installing and getting started with ROS Kinetic**\
We installed ROS Kinetic as per the steps given on the official website of ROS. <http://wiki.ros.org/kinetic/Installation/Ubuntu>  

	Creating ROS workspace:\
    Setting upshell environment, so that some ROS-specific commands are available: 
    ```shell
    . /opt/ros/kinetic/setup.bash	 
    ```
    Creating a catkin workspace. (Workspace directory may have any name, catkin_ws is used here as an example)
    ```shell
    mkdir -p ~/catkin_ws/src
	cd ~/catkin_ws/src
	catkin_init_workspace
	cd ~/catkin_ws
	catkin_make
    ```

- **Installing ROSARIA package**\
The RosAria node provides a ROS interface for most Adept MobileRobots, MobileRobots Inc., and ActivMedia mobile robot bases. Information from the robot base, and velocity and acceleration control, is implemented via a RosAria node, which publishes topics providing data recieved from the robot's embedded controller by ARIA, and sets desired velocity, acceleration and other commands in ARIA when new commands are received from command topics.\
We downloaded and installed appropriate ROSARIA package from <http://robots.mobilerobots.com/wiki/Aria> as per the guideline given on the official ROS website [tutorial for installing ROSARIA](http://wiki.ros.org/ROSARIA/Tutorials/How%20to%20use%20ROSARIA)

- **Installing MobileSim simulation**\
Simulator software downloaded and installed from [its website](http://robots.mobilerobots.com/wiki/MobileSim). As we are running <span style="font-family:shell;">amd64</span>, installing the <span style="font-family:shell;">'ia32-libs-gtk'</span> and <span style="font-family:shell;">'ia32-libs'</span> packages using <span style="font-family:shell;">apt-get</span> are necessary, and then installing the MobileSim package with the <span style="font-family:shell;">--force-architecture</span> option to <span style="font-family:shell;">dpkg</span>.\
NOTE: Gazebo simulator can also be used which is already being installed.

- **Connecting ROSARIA and MobileSim**\
To make connection between ROSARIA and MobileSim, we run commands given below in different terminals.
	```shell
	roscore
	```
  ```shell
   MobileSim                   (choose 'No Map')
  ```
  ```shell
  cd catkin_ws/                (catkin_ws is name of workspace)  
  . devel/setup.bash
  catkin_make
  rosrun rosaria RosAria
  ```
	Now, to get the velocity and position of the robot with respect to initial position,
    ```shell
    rostopic echo /RosAria/pose	
    ```

- **Moving of robot in simulator**\
To move the robot with keyboard, we run commands given below in a new terminal.
  ```shell
  cd catkin_ws/src/
  git clone https://github.com/tonystark262/winter_robotics_project.git
  cd ..
  . devel/setup.bash
  catkin_make
  rosrun p3dx_keyboard_mover mover
  ```
- **Moving original Pioneer-3DX**\
To run the code on original robot, we need ubuntu(same version) and ROS installed in it. Debian 5 operating system was pre-installed in it. There is a serial port and four USB ports on the robot to connect monitor, mouse and keyboard. We installed ubuntu in it booting through a bootable pen-drive. While installing, we gave the robot username 'p3dx' and set the password 'mobilerobots'. After that we need to access its terminal from our PCs. For that we connected the robot to IITD_WIFI(same as PCs) and get its IP address through <span style="font-family:bash;">ifconfig</span> command. Now terminal of the robot can be accessed through this command:
  ```shell
  ssh p3dx@ip_address               (ip_address = 10.194.19.29 in our case)
  ```
  Enter the password of the onboard PC ('mobilerobots' in out case). <span style="font-family:shell;">'exit'</span> command leads back to the local terminal.\
  To run the code and move the robot, we follow the same procedure given above except the simulation part. This time the robot will move in real instead of simulator.

# Technical Explanation
### P3DX Keyboard Controller
The package p3dx_keyboard_mover is made to control the robot using the keyboard in ROS. This package can be used with other packages to move the robot. It is independent package and does not require any other package dependency and can serve as master controller in case of robot failure. The initilisation commands are required by ROS to communicate with terminal.
The main code is as:
```cpp
while(ok())
	{
		char key='z';
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
```
The while loop check is ros is working in each of the cycle. If the ros is working it continues to work else exits. In each loops the pressed key is recognised from the keyboard and the command is executed. We use w,a,s,d for moving a unit step in the forward, left, back, right direction. If no key is recognised the message is published to stop the robot and prevernt it from moving indefinitely. In each loop message is published to the ROS for current situation. This does not take into account any current orientation of the robot. All the directions metioned are w.r.t the Robot.

## Odometry motion control
In this module we implemented ROS code for moving the P3DX robot from one point to another using the PID algorithm and the odometry feedback from the robot. We provide the motivation and general idea of the algorithm and how we implemented it.

#### PID motivation and genearal introduction
PID stands fo Proportional, Intergral and Derivative controller. This is one of the most basic control tehnique used for motion control. 
The P(proportional) controller gives the output signal in proportion to the error signal. This is the main control signal which drives the robot to the destination fast. More is the error form the destination faster it will move the robot to make it reach the destination. 
The I(integral) controller is used to make the steady state error approach to zero. This controller mainly functions when the robot is very nead the goal. At this point the error is very small but not zero. Hence due to low error the propotional controller is not able to provide the sufficient power to move the robot. But the I controller accumulates the errror over time and adds it to the previous error so that accumulated error becomes sufficiently large to move the robot.
The D(derivate) controller is used to reduce the oscillations in the robot motion. When the robot starts at very far distance from the goal, the error is high. Hence after start it gains very high speed and till the time it reaches the goal, it crosses it without stopping. Hence the D controller reduce the speed in porportion to the rate of change of error.If the error is oscillating very fast it reduces it and hence makes the transient motion smooth.

#### Implementation
For moving the robot from one point to another we need to know its co-ordinates in its frame. For this we use the readings from the odometer in the robot. We subscibe to the odometry messages in the ROS.
```cpp
void positionReceived(const nav_msgs::Odometry &msg)
{
  current_position=msg;
  flag=true;
}
....
....
Subscriber sub=nh2.subscribe("/RosAria/pose",10,&positionReceived);
```
We are working in 2D motion i.e. is allowed to move only in the X-Y plane. Hence we need to get the anlge in X-Y plane for easy controll. Hence we convert the quadration angle into readable 2D degree angle. The function returns the angle it is away from the goal.
```cpp
double getYaw(nav_msgs::Odometry msg)
{
  double x,y,z,w;
  x=current_position.pose.pose.orientation.x;
  y=current_position.pose.pose.orientation.y;
  z=current_position.pose.pose.orientation.z;
  w=current_position.pose.pose.orientation.w;
  double siny=2.0*(w*z+x*y);
  double cosy=1.0-2.0*(y*y+z*z);
  return atan2(siny,cosy);
}
```

Finally we implement the PID. The error is calulated as the difference between the gaol and current position. 

```cpp
theta=getYaw(temp_pose);
dis=distance(x,y,target_x,target_y);        //Linear Proportional controller
angle=getAngle(target_x,target_y,x,y)-theta;  //Angular Proportional Controller
....
....
diff_dis=dis-last_distance;                 //Linear differentiator controller
intregal_distance+=dis;                     //Linear integral controller    
diff_angle=angle-last_angle;                  //Angular differentiatior controller
integral_angle+=angle;                        //Angular integral controller
....
....
vel.linear.x=Kp_linear*dis+Kd_linear*diff_dis+Ki_linear*intregal_distance;
vel.angular.z=Kp_angular*angle+Ki_angular*integral_angle+Kd_angular*diff_angle;
```
 The linear velocity is set from the distance and the angular velocity from the angular error. All the PID paprameters have been tuned manually.  
