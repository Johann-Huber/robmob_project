#ifndef MAP_TREE_HPP
#define MAP_TREE_HPP

#include <iostream>
#include <vector>
#include <ctime>
#include <set> 
    
#include "robmob_pkg/map.hpp"
#include "robmob_pkg/utils.hpp"

#define NB_PTS_MAX 500
#define CONNEXITY_RATE 3 // distance between two points less than (1/CONNEXITY_RATE) of the map diagonal : connexity 

class MapTree{

private:
	Map _map;
	float _distRef;
	
	std::vector<Pos> _vertices;
	
	std::vector<std::pair<int,int>> *G; // graph
	std::vector<int> D; // min distance from the source

public:
	MapTree(Map const& map);
	
	std::vector<Pos> computeShorestPath(const Pos& src, const Pos& dest); // main function
	
	void generateRandVertices();
	void selectVertices();
	bool isNear(const Pos& p1, const Pos& p2, const float& distRef );
	void initGraph(const Pos& source, const Pos& target);
	bool isWayClear(const Pos& p1, const Pos& p2);
	void dijkstra();
	std::vector<Pos> getShortestPath();

	
	void dispVerticesInfos(); 
	
	std::vector<Pos> getVertices(){ return _vertices; }
	
	std::vector<std::pair<int,int>>* getGraph(){ return G; }

};

#endif

