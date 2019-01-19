#include "ros/ros.h"
#include <geometry_msgs/Twist.h>
#include <sensor_msgs/Joy.h>

#include <sstream>


ros::Publisher cmd_pub;
ros::Subscriber sub;

void teleopCallback(const sensor_msgs::Joy::ConstPtr& joy);



int main(int argc, char **argv)
{

  ros::init(argc, argv, "teleop_joy");

  ros::NodeHandle n;
  cmd_pub = n.advertise<geometry_msgs::Twist>("/cmd_vel", 10);
  sub = n.subscribe<sensor_msgs::Joy>("joy", 10, teleopCallback);

  ros::spin();


  return 0;
}




void teleopCallback(const sensor_msgs::Joy::ConstPtr& joy)
{
	geometry_msgs::Twist twist;

	//twist.linear.x = joy->axes[1]*0.5;
	//twist.angular.z = joy->axes[0]*0.5;
	twist.linear.x = joy->axes[1]*2;
	twist.angular.z = joy->axes[0]*2;

	cmd_pub.publish(twist);
}
