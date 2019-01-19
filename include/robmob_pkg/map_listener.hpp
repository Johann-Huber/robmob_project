#ifndef MAP_LISTENER_HPP
#define MAP_LISTENER_HPP

#include "ros/ros.h"
#include <cstdlib>
#include <nav_msgs/GetMap.h>
#include <nav_msgs/OccupancyGrid.h>
#include <nav_msgs/MapMetaData.h>
#include <std_msgs/Header.h>
#include <tf/transform_listener.h>

#include <opencv2/videoio.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/calib3d.hpp>
#include <opencv2/highgui.hpp>     
#include <vector>
#include <iostream>
#include <fstream>
#include <boost/shared_ptr.hpp>

#include "robmob_pkg/utils.hpp"



class MapListener{
	
public:
	MapListener(ros::NodeHandle& n);
	
	bool listen(ros::NodeHandle& n);
	
	static void targetSelectionCallback(const geometry_msgs::PointStamped::ConstPtr& ps);
	
	void showMap();
	void dispData();
	
	void targetReached(){ _isThereTarget = false;}
	
	cv::Mat getImgOut(){ return _imgOut; }
	MapInfo getMapInfo(){ return _mapInfo; }
	MapPos getMapPosRobot(){ return _mpRobot; }
	MapPos getMapPosTarget(){ return _mpTarget; }
	bool isInit(){ return _isInit; }
	bool isThereTarget(){ return _isThereTarget; }
	
	void targetPointCallback(const geometry_msgs::PointStamped::ConstPtr& target);
	
private:
	cv::Mat _imgOut;
	MapInfo _mapInfo;
	MapPos _mpRobot;
	MapPos _mpTarget;
	
	bool _isInit;
	bool _isThereTarget;
	
	// For map :
	ros::ServiceClient _client;
	nav_msgs::GetMap _srv;
	
	// For robot position :
	tf::TransformListener _listener;
	ros::Subscriber _subTargetPoint;
	

};

#endif


