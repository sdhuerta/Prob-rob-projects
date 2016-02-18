#ifndef MAP_H
#define MAP_H

#include <png.h>
#include<ros/ros.h>
#include<geometry_msgs/Pose.h>
using namespace ros;
using namespace geometry_msgs;

/* A simple structure for accessing a PNG image */
typedef struct{
  int width,height;
  double res;
  unsigned char **rows;
  png_structp png_ptr;
  png_infop info_ptr;
}MapStruct;

typedef struct{
  int row;
  int col;
  double theta;
}MapCell;


/* readmap will read a PNG file (used as a map) using argv[arg] as
   file name.  It does a lot of error checking and will exit if there
   are problems. */
MapStruct* readmap(char **argv,int arg);

/* These functions allocate and free a 2D array of floats */
float** allocate_float_map(int width, int height);
void free_float_map(float** mapdata,int height);

/* Convert Pose to map cell */
MapCell PoseToCell(MapStruct *map, Pose &pose);

/* Convert map cell to Pose by finding the center point of the cell */
Pose CellToPose(MapStruct *map, MapCell &cell);

/* write_float_map scales the values in a float map to between 0 and
   255, then writes the data as a Portable Gray Map pgm file.
   Filename should end in ".pgm" when this function is called. */
void write_float_map(char *filename,float **data,int width,int height,int autoscale);

#endif

