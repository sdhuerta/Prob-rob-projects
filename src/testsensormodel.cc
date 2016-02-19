/* Author: Larry Pyeatt
   Date: 1-27-2016

   This program tests the sensormodel function, by calculating
   p(x,y,theta|z) for each cell in an occupancy grid map.  It creates
   36 maps, each with a unique value for theta.
*/

/* include system headers */
#include <png.h>
#include <stdlib.h>
#include <stdio.h>
#include <string.h>

/* my headers */
#include <map.h>
#include <sensormodel.h>



/* function to print usage message */
void usage(char *name)
{
  fprintf(stderr,"Usage: %s map_file res dx dy a r\n",name);
  fprintf(stderr," map_file is a PNG file used as a map in"
	  " the stage simulator.\n");
  fprintf(stderr," res is the width and height of each cell of the map,\n");
  fprintf(stderr," dx is the x coordinate of the sensor measured from the center of the robot,\n");
  fprintf(stderr," dy is the y coordinate of the sensor measured from the center of the robot,\n");
  fprintf(stderr," a is angle in degrees of the sensor beam (robot coordinates), and\n");
  fprintf(stderr," r is the range reading received.\n");
  fprintf(stderr,"Distances are given in integer meters\n");
  fprintf(stderr,"Angles are given in integer tenths of a degree.\n");
  exit(100);
}


int main(int argc,char **argv)
{
  MapStruct *map;
  float **mapdata;
  char *ofname,*basename,*tmpcp;
  int width,height;
  int res,dx,dy,a,r;
  int row,col,theta;
  MapCell cell;
  Twist sensor_position;
  Pose pose;

  if(argc != 7)
    usage(argv[0]);

  /* read the occupancy grid map */
  map = readmap(argv,1);
  
  width = map->width;
  height = map->height;
  map->res = atof(argv[2]);
  
  sensor_position.linear.x = atof(argv[3]);
  sensor_position.linear.y = atof(argv[4]);
  sensor_position.linear.z = 0.0;
  sensor_position.angular.x = 0.0;
  sensor_position.angular.y = 0.0;
  sensor_position.angular.z = atoi(argv[5]) * M_PI/180.0;

  r = atof(argv[6]);

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
      	      cell.row = row;
      	      cell.col = col;
      	      cell.theta = theta * M_PI/180;

      	      // // convert from current cell to a pose
      	      // pose = CellToPose(map,cell);

      	      // // apply the twist to adjust sensor location
      	      

      	      // // convert back into cell
      	      // cell = PoseToCell(map,pose);
      	      
      	      mapdata[row][col] = sensormodel(cell,r,map);
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
  
