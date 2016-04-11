#ifndef MOTIONMODEL_H
#define MOTIONMODEL_H

#include <map.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Pose2D.h>
#include <nav_msgs/Odometry.h>
using namespace nav_msgs;
using namespace geometry_msgs;

/* 
   motionmodel calculates p(s_t|s_{t-1},a_{t-1},m)

   Mapstruct and MapCell are defined in map.h 
*/

float motionmodel(MapCoord &stp,    // state at time t
		  MapCoord &st,  // state at time t-1
		  Odometry &odom, // odometry from t-1 to t
		  MapStruct *map,
		  float dt);

void motionmodel_init();

Pose2D samplemotionmodel(Pose2D &st,    // state at time t
			 Odometry &odom, // odometry from t-1 to t
			 MapStruct *map,
			 float dt);


#endif

