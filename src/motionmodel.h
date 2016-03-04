#ifndef SENSORMODEL_H
#define SENSORMODEL_H

#include <map.h>
#include <geometry_msgs/Pose.h>
#include <nav_msgs/Odometry.h>
using namespace nav_msgs;
using namespace geometry_msgs;

#define ALPHA_1 5
#define ALPHA_2 5
#define ALPHA_3 0.5
#define ALPHA_4 0.5
#define ALPHA_5 0.5
#define ALPHA_6 0.5

/* 
   motionmodel calculates p(s_t|s_{t-1},a_{t-1},m)

   Mapstruct and MapCell are defined in map.h 
*/

float motionmodel(MapCell &st,    // state at time t
		  MapCell &stp,  // state at time t-1
		  Odometry &odom, // odometry from t-1 to t
		  MapStruct *map,
		  float dt);

float veloctity_motion_model(MapCell &st,    // state at time t (row, col, Theta)
		  MapCell &stp,  // state at time t-1 (row, col, Theta)
		  Odometry &odom, // odometry from t-1 to t 
		  MapStruct *map,
		  float dt );

double prob_triangular(double a, double b);
double prob_gauss(double a, double b) ;

#endif

