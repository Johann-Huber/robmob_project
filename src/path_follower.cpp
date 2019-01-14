#include "ros/ros.h"
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>
#include <tf/transform_listener.h>
#include <geometry_msgs/PointStamped.h>
#include <iostream>
#include <sstream>
#include <fstream>
#include <vector>

#include "robmob_pkg/map_listener.hpp"
#include "robmob_pkg/map.hpp"
#include "robmob_pkg/map_tree.hpp"
#include "robmob_pkg/utils.hpp"

#define ROBOT_SPEED 1
#define THRESHOLD 0.7 // seuil
#define INTERPOL_SAMPLE 0.8 // Pas pour l'interpolation
#define PI 3.141597
#define NAME_MAP_WIN "Map window"

/* MapPos
 ******
 * Struct to manipulate map informations inside /map basis.
 */
 /*
struct MapPos{
	double x, y, z;

	MapPos(): x(0.0), y(0.0), z(0.0)
	{}
	
	MapPos(const double& X, const double& Y, const double& Z): x(X), y(Y), z(Z)
	{}
	
	double norm()
	{
	
		return sqrt(x*x + y*y + z*z );
	}
	
	void disp(){
		std::cout << "*** Map Pos : (" << x << "," << y << "," << z << ") m" << std::endl;
	}
};
*/

struct VectPosture{
	double x, y, z;
	double roll, pitch, yaw;

	
	VectPosture(tf::StampedTransform transform)
	{
		tf::Quaternion q(transform.getRotation().x(), 
								transform.getRotation().y(), 
								transform.getRotation().z(), 
						   	transform.getRotation().w());
						 
	   x = transform.getOrigin().x();
	   y = transform.getOrigin().y();
	   z = transform.getOrigin().z();
	   
	   tf::Matrix3x3 m(q);
	   m.getRPY(roll, pitch, yaw);
	   
	}
};
 


/* 
 * fonction d'interpolation
 * Param : les points de la traj
 * retourn : les points de la trajectoire interpolée
 */
 
std::vector<MapPos> interpolationTrajectory(std::vector<MapPos> vPosPts)
{
	std::vector<MapPos> vPosInterpol;
	
	// ajout du premier point
	if (vPosPts.size() > 0)
			vPosInterpol.push_back(vPosPts[0]);
	else 
	{	
		std::cout << "(WARNING) Empty list of map pos (in path_follower::interpolationTrajectory)" << std::endl;
		return vPosInterpol;
	}
	
	// pour chaque point de la carte
	for(size_t iPtMap(1) ; iPtMap < vPosPts.size() ; ++iPtMap) // entre n et n-1 : début = 1
	{
		
		double xlast = vPosPts[iPtMap-1].x;
		double ylast = vPosPts[iPtMap-1].y;
		double x = vPosPts[iPtMap].x;
		double y = vPosPts[iPtMap].y;
	

		// sampling
		float norm(std::sqrt((xlast - x)*(xlast - x) + (ylast - y)*(ylast - y)));
		
		int nbPtInterpol = norm/INTERPOL_SAMPLE; // nb de point plaçable selon l'interpole
		
		float dx(0);
		float dy(0);

		for(int it(0) ; it < nbPtInterpol ; ++it)
		{
			dx += (x - xlast)/(nbPtInterpol+1);
			dy += (y - ylast)/(nbPtInterpol+1);
			
			vPosInterpol.push_back(MapPos(xlast+dx , ylast+dy));
			
		}
	
		vPosInterpol.push_back(vPosPts[iPtMap]);
	}
	
	
	return vPosInterpol;
}


// erreur entre pos du robot et le point à atteindre
double posErrorToPt(VectPosture posRob, MapPos pt2Reach)
{
	return ( sqrt((posRob.x-pt2Reach.x)*(posRob.x-pt2Reach.x) + (posRob.y-pt2Reach.y)*(posRob.y-pt2Reach.y)) );
}

/*
// USELESS
// Lit dans le fichier "pathToTxt" les coordonées des points du plus court chemin dans la carte
// Retourne un nav_msgs:Path initialisé avec les valeurs du chemin
nav_msgs::Path readPath(std::string const pathToTxt)
{
	nav_msgs::Path pathMsg;
	
	std::ifstream f(pathToTxt, std::ifstream::in);

	if(f)
	{
		int nbPts;
		f >> nbPts;
		
		pathMsg.header.stamp = ros::Time::now();
		pathMsg.header.frame_id = "map";

		for(int i(0); i < nbPts ; ++i)
		{
			geometry_msgs::PoseStamped ps;
			
			// Header
			ps.header.stamp = ros::Time::now();
			ps.header.frame_id = "map";
			
			// Postion
			float x, y;
			f >> x;
			f >> y;
			
			
			ps.pose.position.x = x;
			ps.pose.position.y = y;
			ps.pose.position.z = 0;
			
			pathMsg.poses.push_back( ps );
		}
		
	}
	else{
		std::cout << "erreur à l'ouverture" << std::endl;
	}
	
	return pathMsg;
}



*/
/*
// USELESS
std::vector<MapPos> defineTrajectory(std::string const pathToTxt)
{
	// TODO: même chose mais tableau de mappoint en param
	// Récupération du chemin
	nav_msgs::Path pathMsg = readPath(pathToTxt);
		
	// Récupération des points de la carte
		
	std::vector<MapPos> vPosPts; // <- points de la carte à atteindre

	for(size_t ind(0); ind < pathMsg.poses.size() ; ++ind)
	{
		vPosPts.push_back(MapPos(pathMsg.poses[ind].pose.position.x, 
									 	 pathMsg.poses[ind].pose.position.y, 
										 pathMsg.poses[ind].pose.position.z));
	}

	// Interpolation de la trajectoire : points intermédiaires 
	std::vector<MapPos> vPosInt = interpolationTrajectory(vPosPts);

	return vPosInt;
}
*/


geometry_msgs::PointStamped stampedNextPoint(MapPos mp)
{
	geometry_msgs::PointStamped rvizPt2Reach;
		
	rvizPt2Reach.header.stamp = ros::Time::now();
	rvizPt2Reach.header.frame_id = "map";
		
	rvizPt2Reach.point.x = mp.x;
	rvizPt2Reach.point.y = mp.y;
	rvizPt2Reach.point.z = 0.0;

	return rvizPt2Reach;
}




/****************************************************************
 * main()
 ****************************************************************
 *
 */

int main(int argc, char **argv)
{
	ros::init(argc, argv, "robmob_path_follower");
	ros::NodeHandle n;


	// **************************** Map initialization ****************************
	
	// Set the map
	MapListener mapListner(n);
	mapListner.listen(n);

	// Create map
	Map mapSim(mapListner, NAME_MAP_WIN);


	// **************************** Find shortest path ****************************
	
	// Create map tree
	MapTree mapTree(mapSim);

	// Display map
	//mapSim.drawMapSrcDest(src, dest);

	// Compute :
	Pos src(mapSim.getPosRobot()), dest(mapSim.getPosTarget());
	std::vector<Pos> sp = mapTree.computeShorestPath(src, dest); // pxl

	// Display res :
	//mapSim.drawMapGraphPath(mapTree.getVertices(), mapTree.getGraph(), sp); // with the graph
	//mapSim.drawMapShortestPath(sp); // without the graph

	// Convert in meter
	std::vector<MapPos> outputPath =  mapSim.convertOutputPath(sp);
	


	// **************************** Robot Control ****************************
	

	ros::Publisher ptPub = n.advertise<geometry_msgs::PointStamped>("/point_to_reach", 10); // debug : affichage point à atteindre
	ros::Publisher cmd_pub = n.advertise<geometry_msgs::Twist>("/cmd_vel", 10); // pour déplacer le robot

	tf::TransformListener listener;

   // Compute the trajectory interpolation
	std::vector<MapPos> vPosInt = interpolationTrajectory(outputPath);
	
	int iPt2Reach(0); // indice point à atteindre
	double lastErrAngle(0);


	ros::Rate rate(30.0); // frequence de rafraichissement
	while (n.ok())
	{
		// Lecture posture robot
		tf::StampedTransform transform; 
	
		try{
			listener.lookupTransform("/map", "/base_link",  
							          	   ros::Time(0), transform);
		}
		catch (tf::TransformException &ex){
			ROS_ERROR("%s",ex.what()); 
			ros::Duration(1.0).sleep(); 
			continue;
		}

		VectPosture vPosRob(transform);


		// Test passage au point suivant
		if( posErrorToPt(vPosRob, vPosInt[iPt2Reach]) < THRESHOLD && iPt2Reach != (vPosInt.size()-1)) 
														// si on est proche du point à atteindre, et qu'on est pas au dernier point
		{
			std::cout << "------------------------------------Passage du point " << iPt2Reach << std::endl;
			iPt2Reach++;
		}
		
		
		// Affichage du point dans Rviz
		ptPub.publish(stampedNextPoint(vPosInt[iPt2Reach]));


		// Calcul du vecteur cible : entre le robot et le prochain point à atteindre :
		MapPos vectToReach( vPosInt[iPt2Reach].x - vPosRob.x, vPosInt[iPt2Reach].y - vPosRob.y );

		// Calcul angle vecteur cible		
		double angle2Reach = std::acos( (vectToReach.x)*1.0/vectToReach.norm() ); //angle orienté v2reach,x0
		if( vectToReach.y < 0  ) // produit vectoriel < 0 -> sin negatif
			angle2Reach= -angle2Reach;
		

	 	// Calcul erreurs
	 	double errAngle = angle2Reach - vPosRob.yaw ;
	 	if (errAngle > PI )
	 	{ 
	 		std::cout << "TOO HIGH : err (avant) = " << errAngle << " err angle après : " << errAngle -2.0*PI << std::endl;
	 		errAngle -= 2.0*PI;
	 	}
	 	if (errAngle < -PI )
	 	{
	 		std::cout << "TOO LOW : err (avant) = " << errAngle << " err angle après : " << errAngle + 2.0*PI << std::endl;
	 		errAngle += 2.0*PI;
	   }
		std::cout << "Angles : angle2Reach = " << angle2Reach << ", yaw = " << vPosRob.yaw << ", errAngle = " << errAngle << std::endl;

		double errPos = vectToReach.norm();
		//std::cout << "Pos : errPos = " << errPos << std::endl;
		
		
		// Commande robot
		int kpv=4;
		int kpw=5, kdw=1; //d indispensable pour le robot réel
		
		geometry_msgs::Twist twist;
		twist.linear.x = kpv*errPos;
		twist.angular.z = kpw*errAngle + kdw*(errAngle - lastErrAngle);
		
		std::cout << "Cmd : v = " << twist.linear.x << ", w = " << twist.angular.z  << std::endl;

		cmd_pub.publish(twist);
		
		lastErrAngle = errAngle;
      rate.sleep(); // gère le rafraîchissement
  }

  return 0;
}



