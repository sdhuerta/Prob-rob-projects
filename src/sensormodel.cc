
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

  r_hit = ray_cast2(cell, map) ;

  if( r_hit > MAX_RANGE)
    r_hit = MAX_RANGE ;

  // printf("row: %d   col:%d   theta: %lf\n", cell.row, cell.col, cell.theta);

  //printf("x: %5d y: %5d Theta: %6.4f r_hit: %6.4lf\n", cell.col, cell.row, cell.theta, r_hit);

  p_hit = ALPHA_1 * calc_p_hit( r, r_hit ) ;
  p_short = ALPHA_2 * calc_p_short( r, r_hit ) ;
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
  else
    return 0;
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


double ray_cast2(MapCell &cell, MapStruct *map)
{
  if( map->rows[cell.row][cell.col] == 0)
    return 0;
  int x,y;
  double theta = -cell.theta ;
  double range = MAX_RANGE / map->res ;
  int xstep, ystep ;

  int x0 = cell.col ;
  int x1 = round(cos(theta) * range) + x0 ;
  int y0 = cell.row ;
  int y1 = round(sin(theta) * range) + y0 ;

  double d_error = 0 ;
  double dx = x1 - x0;
  double dy = y1 - y0;
  double error = 0;

  if(dx < 0)
    xstep = -1;
  else if(dx > 0)
    xstep = 1;
  else
    xstep = 0 ;

  if(dy < 0)
    ystep = -1;
  else if(dy > 0)
    ystep = 1;
  else
    ystep = 0 ;

  if(dx != 0)
      d_error = abs(dy/dx) ;
  else
      d_error= 0 ;

  y = y0;
  x = x0;

  for( ; x != x1; x += xstep)
  {
    if(x < 0 || x > map->width-1 || y < 0 || y > map->height-1)
      return MAX_RANGE ;
    else if( map->rows[y][x] == 0)
      return map->res * sqrt(pow(x-x0,2) + pow(y-y0,2)) ;

    error += d_error ;

    while( error >= 0.5 )
    {
      if(x < 0 || x > map->width-1 || y < 0 || y > map->height-1)
        return MAX_RANGE ;
      else if( map->rows[y][x] == 0)
        return map->res * sqrt(pow(x-x0,2) + pow(y-y0,2)) ;

      y += ystep ;
      error -=  1.0 ; 
    }
  }

//return MAX_RANGE ;
}

double ray_cast(MapCell &cell, MapStruct *map) 
{
  if( map->rows[cell.row][cell.col] == 0)
    return 0;
  int x1, x2, y1, y2, w, h;
  int x, y;
  int numer, longest, shortest ; 
  double range ;
  double theta, dis ;

  int dx1 = 0 ;
  int dx2 = 0 ;
  int dy1 = 0 ;
  int dy2 = 0 ; 

  range = MAX_RANGE / map->res ;

  //printf("%lf\n", range);

  theta = -cell.theta ;

  // printf("%lf\n",theta);

  // Change range to MAX_RANGE

  x1 = cell.col;
  x2 = cos(theta) * range + x1;

  y1 = cell.row;
  y2 = sin(theta) * range + y1;

  w = x2 - x1 ;
  h = y2 - y1 ;

  if( w < 0 )
  {
    dx1 = -1 ;
    dx2 = -1 ;
  }
  else if( w > 0 )
  {
    dx1 = 1 ;
    dx2 = 1 ;
  }

  if( h < 0 )
    dy1 = -1 ;
  else if( h > 0 )
    dy1 = 1 ;

  longest = abs(w) ;
  shortest = abs(h) ;

  if(!(longest > shortest))
  {
    longest = abs(h) ;
    shortest = abs(w) ;
    if( h < 0 )
      dy2 = -1 ;
    else if( h > 0 )
      dy2 = 1 ;
      dx2 = 0 ;
  }

  x = x1 ;
  y = y1 ;

  numer = longest >> 1 ; // divide by 2 ;

  for( int i = 0; i < longest; i++ )
  {
    if( x >= map->width || x < 0 || y >= map->height || y < 0 )
    {
      return MAX_RANGE ;
    }
    else if(map->rows[y][x] == 0)
    {
      dis = sqrt(pow((x-x1),2) + pow((y-y1),2))-1;

      return dis * map->res;
    }

    numer += shortest ;

    if(!(numer < longest))
    {
      numer -= longest ;
      x += dx1 ;
      y += dy1 ;
    }
    else
    {
      x += dx2 ;
      y += dy2 ;
    }
  }

  dis = sqrt(pow((x-x1),2) + pow((y-y1),2));

  return dis *map->res ;

}