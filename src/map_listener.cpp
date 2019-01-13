#include "robmob_pkg/map_listener.hpp"



MapListener::MapListener(ros::NodeHandle& n)
{
	_client = n.serviceClient<nav_msgs::GetMap>("/dynamic_map");
	_isInit = false;
}



bool MapListener::listen(ros::NodeHandle& n)
{
	if (_client.call(_srv))
	{
		nav_msgs::OccupancyGrid map = _srv.response.map;
		nav_msgs::MapMetaData mapData = map.info;

		// Get Map image 
		cv::Mat imgMap(mapData.height, mapData.width, CV_8UC1, map.data.data());
		cv::Mat imgMapInv = cv::Scalar::all(255) - imgMap;

		//cv::Mat imgMapFlip(imgMapInv.size(),imgMapInv.type()); // TODO: ça marche la copie directe ?
		imgMapInv.copyTo(_imgOut);
		cv::flip(imgMapInv, _imgOut, 0); 

   	// Get infos
  		_mapInfo = MapInfo( static_cast<double>(mapData.resolution), mapData.origin.position.x, mapData.origin.position.y );
  		
  		
  		// Get Robot Pos
	  	tf::StampedTransform transform;
	  	
	  	bool posFound = false;
	  	while(!posFound)
		{
			try{
				_listener.lookupTransform("/map", "/base_link", ros::Time(0), transform);
				posFound = true;
			}
			catch (tf::TransformException ex){
				ROS_ERROR("%s",ex.what());
				ros::Duration(1.0).sleep();
			}
		}
		_mpRobot = MapPos(transform.getOrigin().x(), transform.getOrigin().y());
		
		
		//Get Target Pos
		boost::shared_ptr<geometry_msgs::PointStamped const> sharedPtr;
		geometry_msgs::PointStamped ps;

		sharedPtr = ros::topic::waitForMessage<geometry_msgs::PointStamped>("/clicked_point", n);
  	  	if (sharedPtr != NULL)
      	ps = *sharedPtr;
    	else
         ROS_INFO("No message received");
		
		_mpTarget = MapPos(ps.point.x,ps.point.y);
		
		
		// Initialisation done
		_isInit = true;
  }
  else
  {
    ROS_ERROR("(MapListener::listen) Failed to call service.");
    return false;
  }

}



void MapListener::showMap()
{
	if(_isInit)
	{
		cv::imshow( "Image Map", _imgOut );
		cv::waitKey(0);
	}
	else std::cout << "(MapListener::showMap) unable to show map : _isInit = false" << std::endl;
}



void MapListener::dispData()
{
	if(_isInit)
	{
		std::cout << "PosRobot : " << _mpRobot.x << "," << _mpRobot.y << std::endl
					 << "PosTarget : " << _mpTarget.x << "," << _mpTarget.y << std::endl
			  		 << "Resolution : " << _mapInfo.resolution << std::endl
			 		 << "PosOrigin = " << _mapInfo.xOrigin << "," << _mapInfo.yOrigin << std::endl;	
	}
	else std::cout << "(MapListener::dispData) unable to disp data : _isInit = false" << std::endl;
}




