/*
  Original Author: Dr. Pyeatt
  Corrections made by: Daniel Nix

  This file began as Dr. Pyeatt's solution to the sensor model
  but I found errors and (I think) I fixed them.


  I haven't messed with alpha values but I think they may need
  to be changed to account for the new MAX_RANGE
*/
#include <stdlib.h>
#include <stdio.h>
#include <string>
using namespace std;
#include <map.h>
#include <sensormodel.h>

#include<geometry_msgs/Pose.h>
using namespace ros;
using namespace geometry_msgs;


double sqr(double x)
{
  return x*x;
}

/* 
   sensormodel gives the probability of the pose being 'cell'
   given the range reading and the map.
   Mapstruct is defined in map.h 
   Cell is defined in map.h 
   r is in mm
*/

#define MAX_RANGE 5.0


const double alpha[] = {0.3,0.5,0.1,0.05};

double Phit(double r,double r_e)
{
  static const double b=0.1;
  static const double NORM=1.0;
  double e;
  e = exp(-0.5*sqr(r-r_e)/b);
  return NORM * e * (1.0/sqrt(2.0*M_PI*b));
}

double Punexp(double r,double r_e)
{
  static const double NORM=1.0;
  static const double lambda=0.2;
  if(r<r_e)
    return NORM*lambda*exp(-lambda*r);
  return 0.0;
}


double Prand(double r,double r_e)
{
  static const double NORM=1.0;
  return NORM/MAX_RANGE;
}

double Pmax(double r,double r_e)
{
  static const double NORM=1.0;
  static const double rsmall=2.0;
  if(r>=MAX_RANGE)
    return NORM/rsmall;
  else
    return 0.0;
}



double Pobs(double r,double r_e)
{
  // printf("Phit=%lf Punexp=%lf Pmax=%lf Prand=%lf\n",
  // 	 Phit(r,r_e),
  // 	 Punexp(r,r_e),
  // 	 Pmax(r,r_e),
  // 	 Prand(r,r_e)
  // 	 );
  
  return
    alpha[0]*Phit(r,r_e) +
    alpha[1]*Punexp(r,r_e) +
    alpha[2]*Pmax(r,r_e) +
    alpha[3]*Prand(r,r_e);
}


#define THRESHOLD 127

static int checkmap(MapStruct *map,int x,int y)
{
  int tmp=map->rows[y][x] ;
#ifdef SINGLERAY
  map->rows[y][x]=128; 
#endif
  if (tmp < THRESHOLD)
    return 1;
  return 0;
}


static double cast_ray(MapCoord &start,MapCoord &end,MapStruct *map)
{
  int
    x0=start.col,
    x0i=start.col,
    x1=end.col,
    y0=start.row,
    y0i=start.row,
    y1=end.row,
    dx = abs(x1-x0),
    sx = (x0<x1 ? 1 : -1),
    dy = abs(y1-y0),
    sy = (y0<y1 ? 1 : -1),
    err = (dx>dy ? dx : -dy)/2,
    e2,hit;

  while((x0!=x1 || y0!=y1) &&
	x0>0 &&
	x0<map->width &&
	y0>0 &&
	y0<map->height &&
	!(hit=checkmap(map,x0,y0)))
    {
      e2 = err;
      if (e2 >-dx)
	{
	  err -= dy;
	  x0 += sx;
	}
      if (e2 < dy)
	{
	  err += dx;
	  y0 += sy;
	}
    }

  if(hit)
    return sqrt(sqr(x0-x0i-1.0)+sqr(y0-y0i-1.0))*map->res;
  return MAX_RANGE;
}

// this function can be used to get some noise-free range data
// for testing;
void gimme_some_perfect_readings(MapCoord &cell, // pose of the robot
				 MapStruct *map)
{
  Pose2D pose;
  double theta,theta2;
  MapCoord endpoint;
  int i;
  double r_e;
  for(i=0;i<16;i++)
    {
      theta= (2*M_PI * i)/16.0;
      pose = CellToPose(map,cell);
      theta2 = theta + pose.theta;
      pose.x = MAX_RANGE *  cos(theta2) + pose.x;
      pose.y = MAX_RANGE * -sin(theta2) + pose.y;
      endpoint = PoseToCell(map,pose);
      r_e = cast_ray(cell,endpoint,map);
      // Prints angle of sensor in robot frame and a simulated range reading
      // Sensor is located at center of robot.
      printf("%lf %lf\n",theta,r_e);
    }
}


double sensormodel(MapCoord &cell, // pose of the sensor
		   double theta,
		   double r,     // range reading
		   MapStruct *map
		   ) 
{
  Pose2D pose;
  MapCoord endpoint;
  double r_e;
  pose = CellToPose(map,cell);
  pose.theta += theta;
  pose.x = MAX_RANGE * cos(pose.theta) + pose.x;
  pose.y = MAX_RANGE * sin(pose.theta) + pose.y;
  endpoint = PoseToCell(map,pose);
  r_e = cast_ray(cell,endpoint,map);
  if(r_e>MAX_RANGE)
    {
      // printf("r_e exceeds max: %lf\n",r_e);
      return Pobs(r,MAX_RANGE);
    }

  // printf("Ray from pose (%f, %f, %f) (%d, %d) to (%d, %d)
  //       range reading: %lf\n",
  //        r_e);
  return Pobs(r,r_e);
 }




  // used for testing the components
  // for(double j=0.0;j<=MAX_RANGE;j+=0.5)
  //   {
  //     for(double i=0.0;i<=MAX_RANGE;i+=0.5)
  //       printf("%lf\n",Pobs(i,j));
  // 	   printf("%lf\n",Phit(i,j));
  // 	   printf("%lf\n",Punexp(i,j));
  //       printf("%lf\n",Pmax(i,j));
  //       printf("%lf\n",Prand(i,j));
  //     printf("\n");
  //   }
  // exit(1);

