#ifndef SENSORMODEL_H
#define SENSORMODEL_H

#include <map.h>
#include <geometry_msgs/Pose.h>
#include <nav_msgs/Odometry.h>
using namespace nav_msgs;
using namespace geometry_msgs;

/* 
   motionmodel calculates p(s_t|s_{t-1},a_{t-1},m)

   Mapstruct and MapCell are defined in map.h 
*/

float motionmodel(MapCell &st,    // state at time t
		  MapCell &stp,  // state at time t-1
		  Odometry &odom, // odometry from t-1 to t
		  MapStruct *map);

float prob_triangular(float linear_vel, float angular_vel);

#endif

