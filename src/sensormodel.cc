
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
  double r_hit, p_hit, p_short, p_max, p_rand ;

  // if(map->rows[cell.row][cell.col] == 0)
  //   r_hit = 0 ;
  // else
  r_hit = ray_cast(cell, r, map) ;

  //printf("x: %5d y: %5d Theta: %6.4f r_hit: %6.4lf\n", cell.col, cell.row, cell.theta, r_hit);

  p_hit = ALPHA_1 * calc_p_hit(r, r_hit) ;
  p_short = ALPHA_2 * calc_p_short(r, r_hit) ;
  p_max = ALPHA_3 * calc_p_max() ;
  p_rand = ALPHA_4 * calc_p_rand() ;

  return p_hit + p_short + p_max + p_rand ;
}


double calc_p_hit(double r, double r_hit)
{
  double p_hit;
  double e_term;

  e_term = -.5 * pow((r - r_hit),2.0) / B ;

  p_hit = ETA * 1 / sqrt(2 * PI * B) * exp(e_term) ;

  return p_hit ;
}


double calc_p_short(double r, double r_hit)
{
  if( r < r_hit )
    return ETA * LAMBDA * exp(-LAMBDA * r); 
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


double ray_cast(MapCell &cell, double range, MapStruct *map) 
{
  if(map->rows[cell.row][cell.col] == 0)
    return ;
  int x1, x2, y1, y2;

  x1 = cell.col ;
  x2 = cos(cell.thata) * range ;

  y1 = cell.row ;
  y2 = sin(cell.theta) * range ;

  dx = x1 - x2 ;
  dy = y1 - y2 ;

  D = 2.0 * dy - dx ;
}