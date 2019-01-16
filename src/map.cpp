#include "robmob_pkg/map.hpp"

Map::Map(MapListener& mapListner, std::string const windowName )
	:		_winName(windowName)
{ 	
	// Binary
	mapListner.getImgOut().copyTo(_imgBin);
	
	// BGR
	cv::cvtColor(_imgBin, _imgBGR, CV_GRAY2BGR);
	
	// Dimensions
	_height = _imgBGR.rows;
	_width = _imgBGR.cols;
	
	// Thresholding
	cv::threshold( _imgBin, _imgBin, THRESHOLD_VAL, MAX_BIN_VAL, THRESHOLD_TYPE_BIN );		
	
	// Map info init
	_mapInfo = MapInfo(mapListner.getMapInfo());
  	_posRobot = convertPosM2P(mapListner.getMapPosRobot());
  	_posTarget = convertPosM2P(mapListner.getMapPosTarget());
	//std::cout << "pos robot : " << _posRobot.i << "," << _posRobot.j << std::endl
		//		<< "pos target : " << _posTarget.i << "," << _posTarget.j << std::endl;
	
	
	// Erosion
	int erosion_size = ERODE_SIZE;
	int erosion_type = 0; // 0 = MORPH_RECT, 1 = MORPH_CROSS
	
	cv::Mat element = cv::getStructuringElement( erosion_type,
                       cv::Size( 2*erosion_size + 1, 2*erosion_size+1 ),
                       cv::Point( erosion_size, erosion_size ) );
  
   cv::erode( _imgBin, _imgBinEroded, element );
   
   /*
   cv::imshow( "map eroded" , _imgBinEroded );
   cv::waitKey(0);
	*/
}

// useless
bool Map::initMapInfo(std::string const infoPath)
{
	std::ifstream f(infoPath, std::ifstream::in);
	
	/* Pos robot */
	
	MapPos mapPosRob; 
	
	f.ignore(11,'\n');
	f >> mapPosRob.x;
	f >> mapPosRob.y;
   f.ignore(1,'\n');
   /* Map info */
   
   float res, xO, yO;
	
	f.ignore(13,'\n');
	f >> res;
	f.ignore(1,'\n');
	
	f.ignore(5,'\n');
	f >> xO;
	f.ignore(5,'\n');
	f >> yO;
	
	f.close();
	
	//std::cout << "Pos robot : x = " << mapPosRob.x << " y = " << mapPosRob.y << std::endl;
	//std::cout << "res = " << res << " x0 = " << xO << " y0 = " << yO << std::endl; // debug	
	
	_mapInfo = MapInfo(res, xO, yO);
  	_posRobot = convertPosM2P(mapPosRob);
  	//std::cout << "Pos robot : i = " << _posRobot.i << " j = " << _posRobot.j << std::endl;
  	
	return true;
}


Pos Map::convertPosM2P(MapPos mp)
{

	return Pos( (_mapInfo.yOrigin - mp.y)/_mapInfo.resolution + _height , 
					( mp.x - _mapInfo.xOrigin )/_mapInfo.resolution 
				 );

}


std::vector<MapPos> Map::convertOutputPath(std::vector<Pos> vPxlPos)
{

	// conversion to /map frame in meter 
	for(auto&& v : vPxlPos)
	{		
		
		_vOutputPath.push_back( MapPos(_mapInfo.xOrigin + v.j*1.0*_mapInfo.resolution , 
								_mapInfo.yOrigin + _mapInfo.resolution*(this->_height - v.i )*1.0)
							);
	}
	
	for(size_t ind(0) ; ind < _vOutputPath.size() ; ++ind)
		_vOutputPath[ind].disp();
	
	return _vOutputPath;
}


bool Map::writeOutputPath(std::string const outputPath)
{
	// Save infos
	std::ofstream f(outputPath, std::ios::out | std::ios::trunc);

	if(f)  // si l'ouverture a réussi
	{
		f << _vOutputPath.size() << std::endl;
		for (auto&& v: _vOutputPath)
			f << v.x << " " <<  v.y << std::endl;
	
    	f.close();
   }
   else{
   	std::cerr << "Fail to open outputPath.txt !" << std::endl; 
   	return false;
   }
  	
  	return true;	
}


void Map::drawMap()
{
	cv::imshow( _winName, _imgBGR );
   cv::waitKey(0);
}

void Map::drawMap(std::vector<Pos> vertices)
{
	cv::Mat imgMapVertices(_imgBGR.clone());
	
	for (auto&& v : vertices)
		cv::circle(imgMapVertices, cv::Point(v.j,v.i), 2, cv::Scalar(255,0,0), -1, 8); // map point : blue
	
	cv::imshow( _winName, imgMapVertices );
	
   cv::waitKey(0);
}


void Map::drawMap(const Pos pt)
{
	cv::Mat imgMapPt(_imgBGR.clone());

	cv::circle(imgMapPt, cv::Point(pt.j,pt.i), 2, cv::Scalar(255,0,0), -1, 8); // Point(x,y) <=> (j,i)
	cv::imshow( _winName, imgMapPt );
	
   cv::waitKey(0);
}


void Map::drawMapSrcDest(const Pos src, const Pos dest, const std::vector<Pos> vertices)
{
	cv::Mat imgMap(_imgBGR.clone());

	for (auto&& v : vertices)
		cv::circle(imgMap, cv::Point(v.j,v.i), 2, cv::Scalar(255,0,0), -1, 8); // map point : blue
		
	cv::circle(imgMap, cv::Point(src.j,src.i), 2, cv::Scalar(0,255,0), -1, 8); // source : green
	cv::circle(imgMap, cv::Point(dest.j,dest.i), 2, cv::Scalar(0,0,255), -1, 8); // target : red
	cv::imshow( _winName, imgMap );
	
   cv::waitKey(0);
}


void Map::drawMapSrcDest(const Pos src, const Pos dest)
{
	cv::Mat imgMap(_imgBGR.clone());

	cv::circle(imgMap, cv::Point(src.j,src.i), 2, cv::Scalar(0,255,0), -1, 8); // source : green
	cv::circle(imgMap, cv::Point(dest.j,dest.i), 2, cv::Scalar(0,0,255), -1, 8); // target : red
	
	cv::imshow( _winName, imgMap );
   cv::waitKey(0);
}


void Map::drawMapGraph(const std::vector<Pos> vertices, const std::vector<std::pair<int,int>>* graph)
{
	cv::Mat imgMap(_imgBGR.clone());
	
	// Vertices
	for(size_t i(0); i < vertices.size() ; ++i)
	{
		if( i == vertices.size() - 2 )
			cv::circle(imgMap, cv::Point(vertices[i].j,vertices[i].i), 2, cv::Scalar(0,255,0), -1, 8); // source : green
		else if ( i == vertices.size() - 1 )
			cv::circle(imgMap, cv::Point(vertices[i].j,vertices[i].i), 2, cv::Scalar(0,0,255), -1, 8); // target : red
		else 
			cv::circle(imgMap, cv::Point(vertices[i].j,vertices[i].i), 2, cv::Scalar(255,0,0), -1, 8); // map point : blue
	}
		
	// Edges
	for(size_t i(0); i < vertices.size(); ++i)
	{
		
		for(size_t j(0); j < graph[i].size(); ++j)
		{
			cv::Point pt1 (vertices[i].j , vertices[i].i);
			cv::Point pt2 (vertices[graph[i][j].first].j , vertices[graph[i][j].first].i);
			
			cv::line(imgMap, pt1, pt2, cv::Scalar(255,0,0), 1, 8);
		}	
	}
	
	cv::imshow( _winName, imgMap );
   cv::waitKey(0);
}


void Map::drawMapGraphPath(const std::vector<Pos> vertices, const std::vector<std::pair<int,int>>* graph, const std::vector<Pos> shortpath)
{
	cv::Mat imgMap(_imgBGR.clone());
	
	// Vertices
	for(size_t i(0); i < vertices.size() ; ++i)
	{
		if( i == vertices.size() - 2 )
			cv::circle(imgMap, cv::Point(vertices[i].j,vertices[i].i), 2, cv::Scalar(0,255,0), -1, 8); // source : green
		else if ( i == vertices.size() - 1 )
			cv::circle(imgMap, cv::Point(vertices[i].j,vertices[i].i), 2, cv::Scalar(0,0,255), -1, 8); // target : red
		else 
			cv::circle(imgMap, cv::Point(vertices[i].j,vertices[i].i), 2, cv::Scalar(255,0,0), -1, 8); // map point : blue
	}
		
	// Edges
	for(size_t i(0); i < vertices.size(); ++i)
	{
		
		for(size_t j(0); j < graph[i].size(); ++j)
		{
			cv::Point pt1 (vertices[i].j , vertices[i].i);
			cv::Point pt2 (vertices[graph[i][j].first].j , vertices[graph[i][j].first].i);
			
			cv::line(imgMap, pt1, pt2, cv::Scalar(255,0,0), 1, 8);
		}	
	}
	
	// Shortest path
	for(size_t i(1); i < shortpath.size(); ++i)
	{
		cv::Point pt1 (shortpath[i].j , shortpath[i].i);
		cv::Point pt2 (shortpath[i-1].j , shortpath[i-1].i);
		
		cv::line(imgMap, pt1, pt2, cv::Scalar(0,255,0), 1, 8);

	}
	
	cv::imshow( _winName, imgMap );
   cv::waitKey(0);


}

void Map::drawMapShortestPath(const std::vector<Pos> shortpath)
{
	cv::Mat imgMap(_imgBGR.clone());

	// source : green
	cv::circle(imgMap, cv::Point(shortpath[shortpath.size()-1].j,shortpath[shortpath.size()-1].i), 2, cv::Scalar(0,255,0), -1, 8); 
	// target : red
	cv::circle(imgMap, cv::Point(shortpath[0].j,shortpath[0].i), 2, cv::Scalar(0,0,255), -1, 8); 
	
	// Shortest path
	for(size_t i(1); i < shortpath.size(); ++i)
	{
		cv::Point pt1 (shortpath[i].j , shortpath[i].i);
		cv::Point pt2 (shortpath[i-1].j , shortpath[i-1].i);
		
		cv::line(imgMap, pt1, pt2, cv::Scalar(0,255,0), 1, 8);

	}
	
	cv::imshow( _winName, imgMap );
   cv::waitKey(0);


}


int Map::getPxlValBin(const int i,const int j)
{
	int val = (int)_imgBin.at<uchar>(i,j);
	//std::cout << "val : " << val << std::endl;
	return val;
}

int Map::getPxlValBinEroded(const int i,const int j)
{
	int val = (int)_imgBinEroded.at<uchar>(i,j);
	//std::cout << "val : " << val << std::endl;
	return val;
}





