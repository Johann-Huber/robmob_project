#ifndef UTILS_HPP
#define UTILS_HPP

#include <iostream>
#include <limits>

#define OBSTACLE_TEST_SAMPLING 20 // nmber of test to perform on every graph edge to determine if there is a collision

#define INF std::numeric_limits<int>::max()


/* Pos
 ******
 * Easier to manipulate coordinates with image coord (i,j), instead of (x,y)
 */
struct Pos{
	int i;
	int j;
	
	Pos(): i(-1), j(-1)
	{}
	
	Pos(const int& I, const int& J): i(I), j(J)
	{}
	
	void disp(){
		std::cout << "*** Pos : (" << i << "," << j << ")" << std::endl;
	}
	
};


/* MapInfo
 ******
 * Struct to save and manipulate map informations.
 */
struct MapInfo{
	float resolution;
	
	float xOrigin;
	float yOrigin; 
	
	MapInfo(): resolution(0.0), xOrigin(0.0),yOrigin(0.0)
	{}
	
	MapInfo(const float& res, const float& xO, const float& yO): resolution(res), xOrigin(xO),yOrigin(yO)
	{}
	
	MapInfo(MapInfo const& mp): resolution(mp.resolution), xOrigin(mp.xOrigin),yOrigin(mp.yOrigin)
	{}
	
	void disp(){
		std::cout << "*** MapInfo : res = " << resolution << " , origin : x : " << xOrigin << " y : " << yOrigin << std::endl;
	}
	
};


/* MapPos
 ******
 * Struct to manipulate map informations inside /map basis.
 */
struct MapPos{
	float x;
	float y;
		
	MapPos(): x(0.0), y(0.0)
	{}
	
	MapPos(const float& X, const float& Y): x(X), y(Y)
	{}
	
	double norm()
	{
	
		return sqrt(x*x + y*y);
	}
	
	void disp(){
		std::cout << "*** Map Pos : (" << x << "," << y << ") m" << std::endl;
	}
};

#endif
