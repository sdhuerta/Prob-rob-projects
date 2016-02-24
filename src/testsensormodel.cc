/* Author: Larry Pyeatt
   Date: 1-27-2016

   This program tests the sensormodel function by calculating
   p(s|z,map) for each cell in an occupancy grid map.  It creates
   36 maps, each with a unique value for theta, then combines
   them for the final output.
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
  fprintf(stderr,"Usage: %s map_file res range_file\n",name);
  fprintf(stderr," map_file is a PNG file used as a map in"
    " the stage simulator.\n");
  fprintf(stderr," res is the width and height of each cell of the map, and \n");
  fprintf(stderr," range_file is a file of angle and range readings.\n");
  fprintf(stderr,"Distances are given in meters\n");
  fprintf(stderr,"Angles are given in integer degrees.\n");
  exit(100);
}



#define NUM_ANGLES 36

int main(int argc,char **argv)
{
  MapStruct *map;
  float ***mapdata;
  float **finalmapdata;
  char *ofname,*basename,*tmpcp;
  int width,height;
  int res,dx,dy,a;
  int row,col,ang;
  double r,theta;
  MapCell cell;
  Pose pose;

  if(argc != 4)
    usage(argv[0]);

  /* read the occupancy grid map */
  map = readmap(argv,1);
  
  width = map->width;
  height = map->height;
  map->res = atof(argv[2]);
  

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

  float tmp;

  // open file for reading angle and range data
  FILE *sensorfile;
  if((sensorfile=fopen(argv[3],"r"))==NULL)
    {
      perror(argv[0]);
      exit(25);
    }

  mapdata = (float ***)malloc(NUM_ANGLES*sizeof(float**));
  for(ang=0;ang<NUM_ANGLES;ang++)
    mapdata[ang] = allocate_float_map(width,height);
  
  for(ang=0;ang<NUM_ANGLES;ang++)
    for(row=0;row<height;row++)
      for(col=0;col<width;col++)
  mapdata[ang][row][col] = 1.0;
  
  while(fscanf(sensorfile,"%lf%lf",&theta,&r)==2)
    {
      for(ang=0;ang<NUM_ANGLES;ang++)
      // for(ang=0;ang<1;ang++) //used for testing an individual angle
      {
        for(row=0;row<height;row++)
          for(col=0;col<width;col++)
            {
              if(map->rows[row][col]>128)
                {
                  cell.row = row;
                  cell.col = col;
                  cell.theta = theta + ((ang*M_PI*2.0)/NUM_ANGLES);
                  tmp = sensormodel(cell,r,map);
                  if(tmp>=0.0)
                    mapdata[ang][row][col] *= tmp;
                  else
                    mapdata[ang][row][col] = 0.0;
                }
              else
                mapdata[ang][row][col] = 0.0;
            }
      }
    }

  finalmapdata = allocate_float_map(width,height);
  for(row=0;row<height;row++)
    for(col=0;col<width;col++)
      finalmapdata[row][col] = 0.0;
  
  
  for(ang=0;ang<NUM_ANGLES;ang++)
    for(row=0;row<height;row++)
      for(col=0;col<width;col++)
  finalmapdata[row][col]+= mapdata[ang][row][col];

  double max = finalmapdata[0][0];
  for(row=0;row<height;row++)
    for(col=0;col<width;col++)
      if(finalmapdata[row][col]>max)
  max = finalmapdata[row][col];

  printf("max is %lf\n",max);
  for(row=0;row<height;row++)
    for(col=0;col<width;col++)
      finalmapdata[row][col]/=max;

    
  // add walls
  for(row=0;row<height;row++)
    for(col=0;col<width;col++)
      if(map->rows[row][col]<128)
  finalmapdata[row][col] = 1.0;


  //  sprintf(ofname,"%s%03d.pgm",basename,theta);
  sprintf(ofname,"%s.pgm",basename);
  printf("writing %s\n",ofname);
  write_float_map(ofname,finalmapdata,width,height,1);

  free_float_map(finalmapdata,height);

  return 0;
}
  
