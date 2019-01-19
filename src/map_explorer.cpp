
#include "ros/ros.h"
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/LaserScan.h>
#include <iostream>
#include <vector>

#define ROBOT_SPEED_MAX 0.4
#define INF_INIT_SENSOR 100000

//#define DEBUG_PRINT

ros::Subscriber scanSub;

void scanCallback(const sensor_msgs::LaserScan::ConstPtr& scan);

float leftSensor(INF_INIT_SENSOR), frontSensor(INF_INIT_SENSOR), rightSensor(INF_INIT_SENSOR);

enum FollowingBorder {NONE, LEFT, RIGHT};

int main(int argc, char **argv)
{
	ros::init(argc, argv, "robmob_map_explorer");
	ros::NodeHandle n;

	// To border walls
	FollowingBorder fBorder = NONE;

	// index of robot sensors
	int const iRight = 0, iFront = 360, iLeft = 719;
	
	ros::Publisher cmd_pub = n.advertise<geometry_msgs::Twist>("/cmd_vel", 10); // pour déplacer le robot
	scanSub = n.subscribe<sensor_msgs::LaserScan>("/scan", 10, scanCallback);

	std::cout << "Map exploration..." << std::endl;
	ros::Rate rate(10.0); // frequence de rafraichissement
	
	while(n.ok())
	{			
		geometry_msgs::Twist twist;
		
		if (fBorder == NONE && (rightSensor <1 || frontSensor<1 || leftSensor<1))
		{
			if (rightSensor < leftSensor)
			{
				std::cout << "BORDER RIGHT" << std::endl;
				fBorder = RIGHT;
			}
			else
			{
				std::cout << "BORDER LEFT" << std::endl;
				fBorder = LEFT;
			}
		}

		std::cout << "r : " << rightSensor << " f : " << frontSensor << " l : " << leftSensor << std::endl;


		switch (fBorder)
		{
			case RIGHT : 
				if( rightSensor > 1 && frontSensor > 1 && leftSensor > 1)
				{
					std::cout << "2 : tourner à gauche" << std::endl;
					twist.linear.x = 0;
					twist.angular.z = -ROBOT_SPEED_MAX;
				}
				else
				{
					if( (rightSensor < 1 && frontSensor > 1) || (rightSensor < 1 && leftSensor < 1 && frontSensor > 1) )
					{
						std::cout << "3 : avancer tout droit" << std::endl;
						twist.linear.x = ROBOT_SPEED_MAX;
						twist.angular.z = 0;
					}
					else
					{
						std::cout << "1 : tourner à droite" << std::endl;
						twist.linear.x = 0;
						twist.angular.z = ROBOT_SPEED_MAX;
					}
				}
				break;

			case LEFT :
				if( rightSensor > 1 && frontSensor > 1 && leftSensor > 1)
				{
					std::cout << "2 : tourner à droite" << std::endl;
					twist.linear.x = 0;
					twist.angular.z = ROBOT_SPEED_MAX;
				}
				else
				{
					if( (leftSensor < 1 && frontSensor > 1) || (rightSensor < 1 && leftSensor < 1 && frontSensor > 1) )
					{
						std::cout << "3 : avancer tout droit" << std::endl;
						twist.linear.x = ROBOT_SPEED_MAX;
						twist.angular.z = 0;
					}
					else
					{
						std::cout << "1 : tourner à gauche" << std::endl;
						twist.linear.x = 0;
						twist.angular.z = -ROBOT_SPEED_MAX;
					}
				}
				break;
			
			default : // no wall previously found
				twist.linear.x = ROBOT_SPEED_MAX;
				twist.angular.z = 0;
				break;
			
		}
		
		
		cmd_pub.publish(twist);
		
		ros::spinOnce();
		rate.sleep();
	} // end while(n.ok())
	
	
  return 0;
}


void scanCallback(const sensor_msgs::LaserScan::ConstPtr& scan)
{
	int iRight = 0, iFront = 360, iLeft = 719;
/*	
	leftSensor = scan->ranges[iLeft];
	frontSensor = scan->ranges[iFront];
	rightSensor = scan->ranges[iRight];
*/	

	rightSensor = INF_INIT_SENSOR;
	for(int i(0); i < 240; i++)
		if( scan->ranges[i] < rightSensor )
			rightSensor = scan->ranges[i];
	
	frontSensor = INF_INIT_SENSOR;
	for(int i(240); i < 480; i++)
		if( scan->ranges[i] < frontSensor )
			frontSensor = scan->ranges[i];

	leftSensor = INF_INIT_SENSOR;
	for(int i(480); i < 720; i++)
		if( scan->ranges[i] < leftSensor )
			leftSensor = scan->ranges[i];
/*
	rightSensor /= 240;
	frontSensor /= 240;
	leftSensor /= 240;
*/
}






