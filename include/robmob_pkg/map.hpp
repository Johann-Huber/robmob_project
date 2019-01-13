#ifndef MAP_HPP
#define MAP_HPP

#include <opencv2/videoio.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/calib3d.hpp>
#include <opencv2/highgui.hpp>      //for imshow
#include <vector>
#include <iostream>
#include <string>
#include <fstream>

#include "robmob_pkg/map_listener.hpp"
#include "robmob_pkg/utils.hpp"

#define THRESHOLD_VAL 127
#define MAX_BIN_VAL 255
#define THRESHOLD_TYPE_BIN 0

#define ERODE_SIZE 10




class Map{

private:
	cv::Mat _imgBGR;
	cv::Mat _imgBin;
	cv::Mat _imgBinEroded;
	std::string _winName;
	int _height;
	int _width;
	
	MapInfo _mapInfo;
	Pos _posRobot;
	Pos _posTarget;

	std::vector<MapPos> _vOutputPath;	
	

public:
	Map(MapListener& mapListner, std::string const windowName );
	bool initMapInfo(std::string const infoPath);
	
	void drawMap();
	void drawMap(const std::vector<Pos> vertices);
	void drawMap(const Pos pt);
	void drawMapSrcDest(const Pos src, const Pos dest, const std::vector<Pos> vertices);
	void drawMapSrcDest(const Pos src, const Pos dest);
	void drawMapGraph(const std::vector<Pos> vertices, const std::vector<std::pair<int,int>>* graph);
	void drawMapGraphPath(const std::vector<Pos> vertices, const std::vector<std::pair<int,int>>* graph, const std::vector<Pos> shortpath);
	void drawMapShortestPath(const std::vector<Pos> shortpath);
	
	Pos convertPosM2P(MapPos mp);
	std::vector<MapPos> convertOutputPath(std::vector<Pos> vPxlPos);
	
	bool writeOutputPath(std::string const outputPath);
	
	int getHeight(){ return _height; } 
	int getWidth(){ return _width; }
	int getPxlValBin(const int i,const int j);
	int getPxlValBinEroded(const int i,const int j);
	Pos getPosRobot(){ return _posRobot; }
	Pos getPosTarget(){ return _posTarget; }
};

#endif
