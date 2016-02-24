
#ifndef SENSORMODEL_H
#define SENSORMODEL_H

#include <map.h>

#include <geometry_msgs/Pose.h>
#include <nav_msgs/Odometry.h>
using namespace nav_msgs;
using namespace geometry_msgs;

// Stuff
#define ETA 1.0
#define B  0.1
#define LAMBDA .2
#define MAX 20.0
#define MIN 2.0

// Alpha values
#define ALPHA_1 0.3
#define ALPHA_2 0.5
#define ALPHA_3 0.1
#define ALPHA_4 0.05


#define PI 3.14159 


/* 
   sensormodel calculates p(s | z, m) 
   Mapstruct and MapCell are defined in map.h 
   r is the distance reading in meters (z)
*/
   
typedef struct{
  float z_hit;
  float z_short;
  float z_max;
  float z_rand;
  float phi_hit;
  float lambda_short;
}Theta ; 


double calc_p_hit(r, r_hit);
double calc_p_short(r, r_hit);
double calc_p_max();
double calc_p_rand();


double sensormodel(MapCell &cell, // pose of the robot
		   double r,      // range reading
		   MapStruct *map);

#endif

