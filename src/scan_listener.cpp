#include "ros/ros.h"
#include <sensor_msgs/LaserScan.h>

//#include <sstream>


#define DIST_LASER_THRESH 1
#define INF_INIT_SENSOR 100000

ros::Subscriber scanSub;

void scanCallback(const sensor_msgs::LaserScan::ConstPtr& scan);

float leftSensor(INF_INIT_SENSOR), frontSensor(INF_INIT_SENSOR), rightSensor(INF_INIT_SENSOR);

int main(int argc, char **argv)
{
  ros::init(argc, argv, "scan_listener");

  ros::NodeHandle n;
  scanSub = n.subscribe<sensor_msgs::LaserScan>("/scan", 10, scanCallback);

	ros::Rate rate(10.0);
	while(n.ok())
	{
		std::cout << "left : " << leftSensor<< std::endl;
		std::cout << "front : " << frontSensor << std::endl;
		std::cout << "right : " << rightSensor << std::endl;	
  		ros::spinOnce();
		rate.sleep();
	}
	
  return 0;
}


// if ranges < 1 -> proximité

void scanCallback(const sensor_msgs::LaserScan::ConstPtr& scan)
{
	//std::cout << "callback" << std::endl;
	
	int iRight = 0, iFront = 360, iLeft = 719;

	leftSensor = scan->ranges[iLeft];
	frontSensor = scan->ranges[iFront];
	rightSensor = scan->ranges[iRight];
	
	/*
	for(size_t i(0) ; i < scan->ranges.size() ; ++i)
	{
		//if (scan->ranges[i] > DIST_LASER_THRESH)
		std::cout << "left : " << scan->ranges[iLeft] << std::endl;
		std::cout << "front : " << scan->ranges[iFront] << std::endl;
		std::cout << "right : " << scan->ranges[iRight] << std::endl;
	}
	*/
	
}



