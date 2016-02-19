
#include <stdlib.h>
#include <stdio.h>

#include <map.h>
#include <sensormodel.h>

// #include<geometry_msgs/Pose.h>
// using namespace ros;
// using namespace geometry_msgs;



/* 
   sensormodel gives the probability of the pose being 'cell'
   given the range reading and the map.
   Mapstruct is defined in map.h 
   Cell is defined in map.h 
   r is in mm
*/

double sensormodel(MapCell &cell, // pose of the robot 
		   double r,     // range reading
		   MapStruct *map
		   ) 
{
  // this is here just to give it something to do.  
  // return (cell.row*cell.col)/(float)(map->width*map->height);
  float z_hit, z_short, z_max, z_rand;
  int dx, dy ; 

  // RAY CASTING
  dx = sin(cell.theta) * r ;
  dy = cos(cell.theta) * r ;

  step = 2*dy - dx ;

  






}


double p_hit()
{


}


double p_short()
{




}

double p_max()
{


}

double p_rand()
{




}

