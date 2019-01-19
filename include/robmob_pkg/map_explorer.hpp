
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

class MapExplorer
{

public:
	MapExplorer();

	
	void scanCallback(const sensor_msgs::LaserScan::ConstPtr& scan);
	

private:
	


}




