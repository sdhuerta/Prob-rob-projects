/* Author: Larry Pyeatt
   Date: 1-27-2016

   This program tests the motionmodel function, by calculating
   p(s'|s,u) for each cell in an occupancy grid map.  It creates
   36 maps, each with a unique value for theta.
*/

/* include system headers */
#include <png.h>
#include <stdlib.h>
#include <stdio.h>
#include <string.h>

/* my headers */
#include <map.h>
#include <motionmodel.h>

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
  fprintf(stderr," dx is the forward movement, and\n");
  fprintf(stderr," dtheta is the rotation.\n");
  fprintf(stderr,"Distances are given in integer mm\n");
  fprintf(stderr,"Angles are given in integer degrees.\n");
  exit(100);
}


int main(int argc,char **argv)
{
  MapStruct *map;
  float **mapdata;
  char *ofname,*basename,*tmpcp;
  int width,height;
  int res;
  //  int x,y,t,
  //  a1,d,a2;
  int row,col,theta;
  MapCell s_curr;
  MapCell s_prev;
  Odometry odom;
  Twist twist;

  printf("%d\n",argc);
  if(argc != 8)
    usage(argv[0]);

  /* read the occupancy grid map */
  map = readmap(argv,1);
  
  width = map->width;
  height = map->height;
  map->res = atof(argv[2]);

  s_prev.row = atoi(argv[3]);
  s_prev.col = atoi(argv[4]);
  s_prev.theta = atoi(argv[5]) * M_PI/180.0;

  odom.twist.twist.linear.x = atoi(argv[6]);
  odom.twist.twist.linear.y = 0.0;
  odom.twist.twist.linear.z = 0.0;
  odom.twist.twist.angular.x = 0.0;
  odom.twist.twist.angular.y = 0.0;
  odom.twist.twist.angular.z = atoi(argv[7]);

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
  for(theta=0;theta<360;theta+=10)
    {
      mapdata = allocate_float_map(width,height);
      for(row=0;row<height;row++)
	for(col=0;col<width;col++)
	  if(map->rows[row][col]>128)
	    {
	      s_curr.row = row;
	      s_curr.col = col;
	      s_curr.theta = theta * M_PI/180;
	      mapdata[row][col] = motionmodel(s_curr, // next state
					      s_prev, // initial state
					      odom,
					      map); // action
	    }
	  else
	    mapdata[row][col] = 0.0;
      sprintf(ofname,"%s%03d.pgm",basename,theta);
      printf("writing %s\n",ofname);
      write_float_map(ofname,mapdata,width,height,0);
      free_float_map(mapdata,height);
    }

  return 0;
}
  
