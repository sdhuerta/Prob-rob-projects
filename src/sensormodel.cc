
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
  int x1, x2, y1, y2;
  int dx1 = 0, dx2 = 0, dy1 = 0, dy2 = 0;
  int short_side, long_side ;
  int rise, run ;
  float theta, r = 0;

  x1 = cell.col;
  y1 = map->height - cell.row;

  theta = -cell.theta ;

  x2 = cell.col + range * cos(theta) ;
  y2 = cell.row + range * sin(theta) ;

  rise = y2 - y1;
  run = x2 - x1;

  short_side = abs(rise) ;
  long_side = abs(run) ;

  if(short_side > long_side)
  {
    int temp = long_side ;
    long_side = short_side ;
    short_side = temp ;
    if(rise < 0)
      dy2 = -1 ;
    else if( rise > 0)
      dy2 = 1 ;

    dx2 = 0 ;
  }

  if(run < 0)
  {
    dx2 = -1 ;
    dx1 = -1 ;
  }
  else
  {
    dx2 = 1 ;
    dx1 = 1 ;
  }
  if(rise < 0)
    dy1 = -1;
  else
    dy1 = 1;

  int numer = long_side >> 1 ;

  for(int i = 0; i < long_side; i++ )
  { 
    // actual find
    if(y1 > map->height - 1 || x1 > map->width - 1 ||
       y1 < 0 || x1 < 0 || map->rows[y1][x1] == 0 )
    {
      r = sqrt(pow(cell.row - y1,2) + pow(cell.col - x1,2));

      return r ;
      
    }

    numer += short_side ;
    if( numer >= long_side)
    {
      numer -= long_side ;
      x1 += dx1 ;
      y1 += dy1 ;
    }
    else
    {
      x1 += dx2 ;
      y1 += dx2 ;
    }
  }

  r = sqrt(pow(cell.row - y1,2) + pow(cell.col - x1,2));

  return r ;

}