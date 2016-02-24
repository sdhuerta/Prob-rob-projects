
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


}


double calc_p_hit(r, r_hit)
{
  double p_hit;
  double e_term;

  e = -.5 * pow((r - r_hit),2.0) / B ;

  p_hit = ETA * 1 / sqrt(2 * PI * B) * exp(e) ;

  return p_hit ;
}


double calc_p_short(r, r_hit)
{
  double p_short;
  double e_term ;

  if(r < r_hit)
    p_short = ETA * LAMBDA * exp(-LAMBDA * r) ;

  else
    p_short = 0;

  return p_short; 
}

double calc_p_max()
{
  double p_max;

  p_max = ETA * 1.0 / MIN ;

  return p_max ;
}

double calc_p_rand()
{
  double p_rand;

  p_rand = ETA * 1.0 / MAX ;

  return p_rand;
}

double ray_cast(MapCell pos, double dist, MapStruct *map)
{
  rows = map.height;
  cols = map.width;

  float x2, y2, dx, dy;
  float delta1, delta2;

  // RAY CASTING
  x2 = sin(cell.theta) * dist ;
  y2 = cos(cell.theta) * dist ;

  dx = x2 - cell.col;
  dy = y2 - cell.row;

  delta1 = 2.0 * dy ;
  delta2 = delta1 - dx ;

  old_row = pos.row;
  old_col = pos.col;

  pix = delta2 ;

  while(new_row != (int)y2 && new_col != (int)x2 )
  {
    if(map.rows[new_row][new_col] == 255)
      return distance ;

    if(pix < 0)
    {
      
    }
  }


  
}