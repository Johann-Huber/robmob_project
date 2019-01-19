#include "ros/ros.h"
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>
#include <tf/transform_listener.h>
#include <sensor_msgs/LaserScan.h>
#include <geometry_msgs/PointStamped.h>
#include <iostream>
#include <sstream>
#include <fstream>
#include <vector>

#include "robmob_pkg/map_listener.hpp"
#include "robmob_pkg/map.hpp"
#include "robmob_pkg/map_tree.hpp"
#include "robmob_pkg/utils.hpp"

#define ROBOT_SPEED_MAX 0.4
#define THRESHOLD 0.7 // seuil
#define THRESHOLD_GOAL 0.2
#define INTERPOL_SAMPLE 0.8 // Pas pour l'interpolation
#define PI 3.141597

#define NAME_MAP_WIN "Map window"

//#define DEBUG_PRINT

// for scan callback :
#define INF_INIT_SENSOR 100000
float leftSensor(INF_INIT_SENSOR), frontSensor(INF_INIT_SENSOR), rightSensor(INF_INIT_SENSOR);

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
			
			if (it == 0) // more point near the beginning
				vPosInterpol.push_back(MapPos(xlast+((x - xlast)/(nbPtInterpol+1)/2) , 
														ylast+((y - ylast)/(nbPtInterpol+1)/2)));
			
			if(nbPtInterpol >= 3) // long distances -> less points
			{
				if(it%2 !=0)
					vPosInterpol.push_back(MapPos(xlast+dx , ylast+dy));
			}
			else{
				vPosInterpol.push_back(MapPos(xlast+dx , ylast+dy));
			}
			
			if (it == nbPtInterpol-1) // more point near the end
				vPosInterpol.push_back(MapPos(xlast+dx+ ((x - xlast)/(nbPtInterpol+1)/2) , 
														ylast+dy+ ((y - ylast)/(nbPtInterpol+1)/2)));
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



nav_msgs::Path setPath(std::vector<MapPos> vPosInt)
{
	nav_msgs::Path pathMsg;
	
	pathMsg.header.stamp = ros::Time::now();
	pathMsg.header.frame_id = "map";

	for(size_t ind(0); ind<vPosInt.size() ; ++ind)
	{
		geometry_msgs::PoseStamped ps;
			
		// Header
		ps.header.stamp = ros::Time::now();
		ps.header.frame_id = "map";
		
		// Postion
		ps.pose.position.x = vPosInt[ind].x;
		ps.pose.position.y = vPosInt[ind].y;
		ps.pose.position.z = 0;
		
		pathMsg.poses.push_back( ps );
	}
	
	return pathMsg;
}



void scanCallback(const sensor_msgs::LaserScan::ConstPtr& scan)
{
	int iRight = 0, iFront = 360, iLeft = 719;

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

	// **************************** Initialization ****************************
	
	ros::Publisher ptPub = n.advertise<geometry_msgs::PointStamped>("/point_to_reach", 10); // debug : affichage point à atteindre
	ros::Publisher cmd_pub = n.advertise<geometry_msgs::Twist>("/cmd_vel", 10); // pour déplacer le robot
	ros::Publisher pathPub = n.advertise<nav_msgs::Path>("/shortest_path", 10);
	ros::Subscriber scanSub = n.subscribe<sensor_msgs::LaserScan>("/scan", 10, scanCallback); // laser sensor to avoid obstacles
	tf::TransformListener listener;

	// Set the map
	MapListener mapListner(n);

	// **************************** Listening map ****************************

	while(n.ok())
	{

		mapListner.listen(n);

		// Create map
		Map mapSim(mapListner, NAME_MAP_WIN);


		// **************************** Find & display path ****************************
		
		// Create map tree
		MapTree mapTree(mapSim);

		// Display map
		//mapSim.drawMapSrcDest(src, dest);

		// Compute shortest path:
		Pos src(mapSim.getPosRobot()), dest(mapSim.getPosTarget());
		std::vector<Pos> sp = mapTree.computeShorestPath(src, dest); // pxl

		// Display res :
		//mapSim.drawMapShortestPath(sp); // without the graph

		// Convert in meter
		std::vector<MapPos> outputPath =  mapSim.convertOutputPath(sp);
		
		// Compute the trajectory interpolation
		std::vector<MapPos> vPosInt = interpolationTrajectory(outputPath);
		
		// Set the gazebo path
		nav_msgs::Path gazeboPath = setPath(vPosInt);
		

		// **************************** Robot Control ****************************
		

		int iPt2Reach(0); // indice point à atteindre
		double lastErrAngle(0);
		
		// to avoid point-inside-obstacle block
		int timeToReachPt(0); 
		int const delayToReachPt = 100;

		ros::Rate rate(30.0); // frequence de rafraichissement
		std::cout << "Path following..." << std::endl;
		while ( mapListner.isThereTarget() ) //(n.ok())
		{
			// Affichage du path
			
			pathPub.publish(gazeboPath); //displayPath(vPosInt); 
		
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
			if( posErrorToPt(vPosRob, vPosInt[iPt2Reach]) < THRESHOLD && iPt2Reach < (vPosInt.size()-1)) 
															// si on est proche du point à atteindre, et qu'on est pas au dernier point
			{
				#ifdef DEBUG_PRINT
				 std::cout << "------------------------------------Passage du point " << iPt2Reach << std::endl;
				#endif
				iPt2Reach++;
				timeToReachPt = 0;
			}
			if(timeToReachPt >= delayToReachPt && iPt2Reach < (vPosInt.size()-1))
			{
				//std::cout << "-------Time is up : Going to the next point " << iPt2Reach << std::endl;
				iPt2Reach++;
				timeToReachPt = 0;
			}

			if( posErrorToPt(vPosRob, vPosInt[iPt2Reach]) < THRESHOLD_GOAL && iPt2Reach == (vPosInt.size()-1) &&
					mapListner.isThereTarget())
			{
				mapListner.targetReached();
				
				std::cout << "Target reached." << std::endl;
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
		 		//std::cout << "TOO HIGH : err (avant) = " << errAngle << " err angle après : " << errAngle -2.0*PI << std::endl;
		 		errAngle -= 2.0*PI;
		 	}
		 	if (errAngle < -PI )
		 	{
		 		//std::cout << "TOO LOW : err (avant) = " << errAngle << " err angle après : " << errAngle + 2.0*PI << std::endl;
		 		errAngle += 2.0*PI;
			}
			double errPos = vectToReach.norm();
				
				
				
				
			// Commande robot
			/*
			float kpv=0.5;
			float kpw=1, kdw=0; //d indispensable pour le robot réel
			*/		
			
			geometry_msgs::Twist twist;
			//std::cout << "r : " << rightSensor << " f : " << frontSensor << " l : " << leftSensor << std::endl;
			// **************************** Subsumption ****************************
			if (frontSensor<0.5 ) // front obstacle
			{
				if ( rightSensor < leftSensor )
				{
					//std::cout << "obstacle right side : turn left" << std::endl;
					twist.linear.x = 0;
					twist.angular.z = ROBOT_SPEED_MAX;
				}
				else
				{
					//std::cout << "obstacle left side : turn right" << std::endl;
					twist.linear.x = 0;
					twist.angular.z = -ROBOT_SPEED_MAX;
				}
			}
			else
			{
				if( rightSensor <0.5 || leftSensor <0.5) // bordering obstacle
				{
					//std::cout << "bordering obstacle : move forward" << std::endl;
					twist.linear.x = ROBOT_SPEED_MAX;
					twist.angular.z = 0;
				}
				else // No obstacle : standard pathfollowing
				{
					// **************************** PID ****************************
					float kpv=0.5;
					float kpw=0.7, kdw=0.2;
					

					float cmdv = kpv*errPos;
					if ( cmdv > ROBOT_SPEED_MAX) cmdv = ROBOT_SPEED_MAX; // set max speed
							
					twist.linear.x = cmdv;
					twist.angular.z = kpw*errAngle + kdw*(errAngle - lastErrAngle);	
					
					#ifdef DEBUG_PRINT
					 std::cout << "Angles : angle2Reach = " << angle2Reach << ", yaw = " << vPosRob.yaw << ", errAngle = " << errAngle << std::endl;
					std::cout << "Pos : errPos = " << errPos << std::endl;
					std::cout << "Cmd : v = " << twist.linear.x << ", w = " << twist.angular.z  << std::endl;
					#endif
				}
			}
			
			cmd_pub.publish(twist);
			
			lastErrAngle = errAngle;
			timeToReachPt++;
			
			ros::spinOnce();
		   rate.sleep(); // gère le rafraîchissement

	  	}
	  
	  	std::cout << "Ending ..." << std::endl;
	  	for (int iEnd(0); iEnd < 120 ; iEnd++)
	  	{
	  		geometry_msgs::Twist twist;
	  		twist.linear.x = 0;
			twist.angular.z = 0;
			
			cmd_pub.publish(twist);
			rate.sleep();
	  	}
	  
	  	std::cout << "Ending ok. Another target could be selected." << std::endl;
	  	
	  	bool isAnswered(false);
	  	while(!isAnswered)
	  	{
		  	std::string answer;
		  	std::cout << "Select another target ? (yes/no)" << std::endl;
		  	std::cin >> answer;
		  	if (answer == "no")
		  	{
		  		std::cout << "Shutting down node." << std::endl;
		  		return 0;
		  	}
		  	else
		  	{
			  	if (answer == "yes")
			  		isAnswered = true;
			  	else
			  		std::cout << "answer should be [yes] or [no]" << std::endl;
			}
	  	}
	  	
	} // end while(n.ok())
	
	
  return 0;
}



