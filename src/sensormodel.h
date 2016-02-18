
#ifndef SENSORMODEL_H
#define SENSORMODEL_H

#include <map.h>

#include <geometry_msgs/Pose.h>
#include <nav_msgs/Odometry.h>
using namespace nav_msgs;
using namespace geometry_msgs;

/* 
   sensormodel calculates p(s | z, m) 
   Mapstruct and MapCell are defined in map.h 
   r is the distance reading in meters (z)
*/
   


double sensormodel(MapCell &cell, // pose of the robot
		   double r,      // range reading
		   MapStruct *map);

#endif

