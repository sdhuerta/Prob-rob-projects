/* Author: Larry Pyeatt
   Date: 1-27-2016

   This program tests the motionmodel function, by calculating
   p(s'|s,u) for each cell in an occupancy grid map.  It creates
   36 maps, each with a unique value for theta, and combines
   them for the final output.
*/

/* include system headers */
#include <png.h>
#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include <list>

using namespace std;

/* my headers */
#include <map.h>
#include <motionmodel.h>


typedef struct{
  int x,y,theta;
}PointOfInterest;


/* function to print usage message */
void usage(char *name)
{
  fprintf(stderr,"Usage: %s map_file res x y theta dx dtheta\n",name);
  fprintf(stderr," map_file is a PNG file used as a map in"
    " the stage simulator.\n");
  fprintf(stderr," res is the width and height of each cell of the map in meters,\n");
  fprintf(stderr," x is the initial map row of the robot,\n");
  fprintf(stderr," y is the initial map column of the robot,\n");
  fprintf(stderr," theta is the initial heading of the robot,\n");
  fprintf(stderr," dx is the forward velocity, and\n");
  fprintf(stderr," dtheta is the angular velocity.\n");
  fprintf(stderr,"Distances are given in integer mm\n");
  fprintf(stderr,"Angles are given in integer degrees.\n");
  exit(100);
}

#define NUM_ANGLES 36

int main(int argc,char **argv)
{
  MapStruct *map;
  double ***mapdata;
  double **finalmapdata;
  double tmp;
  char *ofname,*basename,*tmpcp;
  int width,height;
  int row,col,ang;
  MapCell s_curr;
  MapCell s_initial;
  Odometry odom;
  Twist twist;

  list<MapCell> POI,nextPOI;
  list<MapCell>::iterator cp;
  
  if(argc != 8)
    usage(argv[0]);

  /* read the occupancy grid map */
  map = readmap(argv,1);
  
  width = map->width;
  height = map->height;
  map->res = atof(argv[2]);

  printf("map is %d by %d\n",width,height);

  s_initial.row = atoi(argv[3]);
  s_initial.col = atoi(argv[4]);
  s_initial.theta = atof(argv[5]) * M_PI/180.0;
  s_initial.value = 1.0;

  POI.push_back(s_initial);

  odom.twist.twist.linear.x = atoi(argv[6]);
  odom.twist.twist.linear.y = 0.0;
  odom.twist.twist.linear.z = 0.0;
  odom.twist.twist.angular.x = 0.0;
  odom.twist.twist.angular.y = 0.0;
  odom.twist.twist.angular.z = atof(argv[7]) * M_PI/180.0;

  /* extract the base map file name without extension or path */
  if((basename=(char*)malloc(strlen(argv[1])+16))==NULL)
    {
      perror(argv[0]);
      exit(20);
    }
  if((ofname=(char*)malloc(strlen(argv[1])+16))==NULL)
    {
      perror(argv[0]);
      exit(20);
    }
  strcpy(ofname,argv[1]);
  if((tmpcp=strrchr(ofname,'.'))==NULL)
    {
      fprintf(stderr,"%s: '%s' has no extension!\n",argv[0],argv[1]);
      exit(21);
    }
  *tmpcp=0; // remove extension from png file name
  if((tmpcp=strrchr(ofname,'/'))!=NULL)
    tmpcp++;
  else
    tmpcp = ofname;
  strcpy(basename,tmpcp);

  /* create one probability map for each theta between 0 and 350 in 10
     degree increments */
  
  if((mapdata = (double***)malloc(NUM_ANGLES*sizeof(double**)))==NULL)
    {
      perror("Unable to allocate memory");
      exit(1);
    }
  
  for(ang=0;ang<NUM_ANGLES;ang++)
    if((mapdata[ang] = allocate_double_map(width,height))==NULL)
      {
  perror("Unable to allocate memory");
  exit(1);
      }

  if((finalmapdata = allocate_double_map(width,height))==NULL)
    {
      perror("Unable to allocate memory");
      exit(1);
    }
    
  // initialize mapdata
  for(ang=0;ang<NUM_ANGLES;ang++)
    for(row=0;row<height;row++)
      for(col=0;col<width;col++)
  mapdata[ang][row][col] = 0.0;

  for(ang=0;ang<NUM_ANGLES;ang++)
    for(row=0;row<height;row++)
      for(col=0;col<width;col++)
  {
    s_curr.row = row;
    s_curr.col = col;
    s_curr.theta = ang * M_PI / (NUM_ANGLES);
    tmp = motionmodel(s_curr, // next state
          s_initial, // initial state
          odom,   // action
          map,0.5);// map and dt
    mapdata[ang][row][col] += tmp;
  }

  for(row=0;row<height;row++)
    for(col=0;col<width;col++)
      finalmapdata[row][col]=0.0;

  for(ang=0;ang<NUM_ANGLES;ang++)
    for(row=0;row<height;row++)
      for(col=0;col<width;col++)
  finalmapdata[row][col]+=mapdata[ang][row][col];
  
  double max = finalmapdata[0][0];
  for(row=0;row<height;row++)
    for(col=0;col<width;col++)
      if(finalmapdata[row][col]>max)
  max = finalmapdata[row][col];

  // add point where robot is
  finalmapdata[s_initial.row][s_initial.col] = max;
  
  // add walls
  for(row=0;row<height;row++)
    for(col=0;col<width;col++)
      if(map->rows[row][col]<128)
  finalmapdata[row][col] = max;
  
  //      sprintf(ofname,"%s%03d.pgm",basename,theta);
  sprintf(ofname,"%s.pgm",basename);
  printf("writing %s\n",ofname);
  write_double_map(ofname,finalmapdata,width,height,1);

  return 0;
}
  
