
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

  r_hit = ray_cast(cell, r, map) ;

  p_hit = ALPHA_1 * calc_p_hit(r, r_hit) ;
  p_short = ALPHA_2 * calc_p_short(r, r_hit) ;
  p_max = ALPHA_3 * calc_p_max() ;
  p_rand = ALPHA_4 * calc_p_rand() ;

  return p_hit + p_short + p_max + p_rand ;
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
  int dx1, dx2, dy1, dy2 = 0;
  int short_side, long_side ;

  x1 = cell.col;
  y1 = cell.row;

  x2 = cell.col * cos(cell.theta) ;
  y2 = cell.row * sin(cell.theta) ;

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
    dx2 = -1
    dx = -1 ;
  }
  else
  {
    dx2 = 1 ;
    dx = 1 ;
  }
  if(rise < 0)
    dy = -1;
  else
    dy = 1;

  int numer = long_side >> 1 ;

  for(int i = 0; i < long_side; i++ )
  {
    char 
    // actual find
    if(MapCell.rows[y1][x1] == 255 
       || y1 > MapCell.height 
       || x1 > MapCell.height)
    {
      return range ;

    }
    numer += short_side ;
    if( numer >= long_side)
    {
      numer -= long_side ;
      x1 += dx1 ;
      y1 += dy1 ;
      range++ ;
    }
    else
    {
      x1 += dx2 ;
      y1 += dx2 ;
      range++ ;
    }
  }

  return range ;

}