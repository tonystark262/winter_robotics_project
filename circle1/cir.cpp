#include<ros/ros.h>
#include<geometry_msgs/Twist.h>
#include<geometry_msgs/Pose.h>
#include<nav_msgs/Odometry.h>
#include<tf/tf.h>
#include<cmath>
#include<iostream>
#include<tf/transform_broadcaster.h>
#include<angles/angles.h>

using namespace std;
using namespace ros;

nav_msgs::Odometry current_position;
Publisher pub;
bool flag=false;
double x=0,y=0,theta=0;
double dis=1,angle=0;
double target_x=0,target_y=0,tolerance=0;


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


double distance(double x1,double y1,double x2,double y2)
{
  double a=x1-x2;
  double b=y1-y2;
  a*=a;
  b*=b;
  return sqrt(a+b);
}

double getAngleFromQuaternion(nav_msgs::Odometry msg)
{
  double roll, pitch, yaw;
  tf::Quaternion quater;
  tf::quaternionMsgToTF(msg.pose.pose.orientation,quater);
  tf::Matrix3x3(quater).getRPY(roll,pitch,yaw);
  yaw=angles::normalize_angle_positive(yaw);
  cout<<"                     "<<yaw<<"\n";
  return yaw;
}

double getAngle(double x1,double y1,double x2,double y2)
{
  double a=x1-x2;
  double b=y1-y2;
  return atan2(b,a);
}

bool checkTolerance(double d1,double d2)
{
  if(d1<=d2)
  {
    return true;
  }
  return false;
}

void positionReceived(const nav_msgs::Odometry &msg)
{
  current_position=msg;
  flag=true;
}

void goToGoal()
{
  bool cur_flag=false,prev_flag=false;
  geometry_msgs::Twist vel;
  nav_msgs::Odometry temp_pose=current_position;
  double last_distance=0,intregal_distance=0,last_angle=0,integral_angle=0;
  double diff_dis,diff_angle;
  double Kp_linear=1,Ki_linear=0,Kd_linear=10;
  double Kp_angular=5,Ki_angular=0,Kd_angular=10;

  Rate rate(10);
  do
  {
    cur_flag=flag;

    temp_pose=current_position;

    x=temp_pose.pose.pose.position.x;
    y=temp_pose.pose.pose.position.y;
    theta=getYaw(temp_pose);
    dis=distance(x,y,target_x,target_y);        //Linear Proportional controller
    angle=getAngle(target_x,target_y,x,y)-theta;  //Angular Proportional Controller

    if(!prev_flag)
    {
      prev_flag=true;
      last_distance=dis;
      last_angle=angle;
    }
    
    diff_dis=dis-last_distance;                 //Linear differentiator controller
    intregal_distance+=dis;                     //Linear integral controller    
    diff_angle=angle-last_angle;                  //Angular differentiatior controller
    integral_angle+=angle;                        //Angular integral controller

    vel.linear.x=Kp_linear*dis+Kd_linear*diff_dis+Ki_linear*intregal_distance;
    vel.linear.y=0;
    vel.linear.z=0;
    vel.angular.x=0;
    vel.angular.y=0;
    vel.angular.z=Kp_angular*angle+Ki_angular*integral_angle+Kd_angular*diff_angle;

    cout<<x<<" "<<y<<" "<<theta<<" "<<dis<<" "<<vel.linear.x<<" "<<vel.angular.z<<"\n";

    last_distance=dis;
    last_angle=angle;
    if(!cur_flag)
    {
      vel.linear.x=0;
      vel.angular.z=0;
    }

    pub.publish(vel);
    spinOnce();
    rate.sleep();

  }while(!checkTolerance(dis,tolerance) || !cur_flag);

  vel.linear.x=0;
  vel.angular.z=0;
  pub.publish(vel);
  exit(0);
  
}

int main(int argc, char **argv)
{
  init(argc,argv,"ready_to_move_along_a_path");
  NodeHandle nh1,nh2;
  Subscriber sub=nh2.subscribe("/RosAria/pose",10,&positionReceived);
  pub =nh1.advertise<geometry_msgs::Twist>("/RosAria/cmd_vel",1000);    
  cout<<"enter new co-ordinates and tolerance ( anything else to exit )\n";
  cin>>target_x>>target_y>>tolerance;
  Rate rate(0.5);
  goToGoal();
  rate.sleep();
  spin();
  return 0;
}