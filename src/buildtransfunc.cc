/* Author: Larry Pyeatt
   Date: 3-3-2016

   This program uses the sample motion model to create the transition
   probability function which specifies P(s,a,s') for all a,s' where
   s' are an x,y offset from s and resulting state.
*/

/* include system headers */
#include <png.h>
#include <stdlib.h>
#include <stdio.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <unistd.h>
#include <math.h>
#include <gsl/gsl_rng.h>
#include <gsl/gsl_randist.h>
#include <string.h>
#include <mapcellset.h>
#include <transmodel.h>
using namespace std;

/* my headers */
#include <map.h>
#include <motionmodel.h>

static gsl_rng *srng1, *srng2, *srng3;

void random_sample_init()
{
  unsigned long int seeds[3];
  int fd;
  if((fd=open("/dev/random",O_RDONLY))<0)
    {
      perror("motionmodel_init: unable to open /dev/random");
      exit(3);
    }
  read(fd,seeds,sizeof(unsigned long int) * 3);
  close(fd);
  srng1 = gsl_rng_alloc(gsl_rng_mt19937);
  srng2 = gsl_rng_alloc(gsl_rng_mt19937);
  srng3 = gsl_rng_alloc(gsl_rng_mt19937);
  gsl_rng_set(srng1,seeds[0]);
  gsl_rng_set(srng2,seeds[1]);
  gsl_rng_set(srng3,seeds[2]);
}


#define NUM_ACT 6
#define NUM_ANGLES 4

ActionDef action[NUM_ACT] = {
  {0,0},      // stop
  {1,0},      // forward 1 m/s
  {1,M_PI/2},   // forward 1 m/s and turn left 90 degrees/second
  {1,-M_PI/2},  // forward 1 m/s and turn right 90 degrees/second
  {0,M_PI/2},   // turn left 90 degrees/second
  {0,-M_PI/2}  // turn right 900 degrees/second
  //  {-1,0},     // reverse 1 m/s
  //  {-1,M_PI/2},  // reverse 1 m/s and turn left 90 degrees/second
  //{-1,-M_PI/2} // reverse 1 m/s and turn right 90 degrees/second
};

  // {0.5,0},      // forward 0.5 m/s
  // {-0.5,0},     // reverse 0.5 m/s
  // {0,M_PI*0.5},   // turn left 90 degrees/second
  // {0,-M_PI*0.5},  // turn right 90 degrees/second
  // {0.5,M_PI*0.5},   // forward 0.5 m/s and turn left 180 degrees/second
  // {0.5,-M_PI*0.5},  // forward 0.5 m/s and turn right 180 degrees/second
  // {-0.5,M_PI*0.5},  // reverse 1 m/s and turn left 180 degrees/second
  // {-0.5,-M_PI*0.5}, // reverse 1 m/s and turn right 180 degrees/second
  
#define NUMSAMP 100000


int main(int argc,char **argv)
{
  MapStruct *map;
  double ***mapdata;
  int row,col,ang;
  MapCoord s_initial;
  Odometry odom;
  Twist twist;
  Pose2D pose;
  int i,act;
  char *filename = (char*)"map15.png";
  int count=0;
  Pose2D samples[NUMSAMP];
  TransitionModel Tr(NUM_ACT,NUM_ANGLES);
  double sum;
  MapCoord s,sp;

  // Add the action definitions to the TransitionModel
  for(i=0;i<NUM_ACT;i++)
    Tr.setAction(i,action[i]);
  
  // For every possible following pose, calculate the
  // probability.

  // load a blank 15x15 map.
  map = readmap(&filename,0,NUM_ANGLES,0.1);

  motionmodel_init();

  random_sample_init();

  mapdata = (double ***)malloc(map->nAngles*sizeof(double**));
  for(ang=0;ang<map->nAngles;ang++)
    {
      //printf("allocating %d\n",ang);
      mapdata[ang] = allocate_double_map(map->width,map->height);
      if(mapdata[ang]==NULL)
	{
	  perror("Error allocating:");
	  exit(4);
	}
    }
  
  printf("width: %d  height: %d\n",map->width,map->height);

  for(act=0;act<NUM_ACT;act++)
    {
      odom.twist.twist.linear.x = action[act].linear_vel;
      odom.twist.twist.linear.y = 0.0;
      odom.twist.twist.linear.z = 0.0;
      odom.twist.twist.angular.x = 0.0;
      odom.twist.twist.angular.y = 0.0;
      odom.twist.twist.angular.z = action[act].angular_vel;
      
      for(ang=0;ang<NUM_ANGLES;ang++)
	{
	  for(i=0;i<map->nAngles;i++)
	    for(row=0;row<15;row++)
	      for(col=0;col<15;col++)
		mapdata[i][row][col] = 0.0;
	  
	  s_initial.row = map->height/2;
	  s_initial.col = map->width/2;
	  s_initial.angle = ang;

	  pose = CellToPose(map,s_initial);
	  //	  printf("%lf %lf %lf\n",pose.x,pose.y,pose.theta);
	  
	  for(i=0;i<NUMSAMP;i++)
	    { // get random samples from center cell
	      samples[i].x = pose.x+((gsl_rng_uniform_pos(srng1)-0.5)*map->res);
	      samples[i].y = pose.y+((gsl_rng_uniform_pos(srng2)-0.5)*map->res);
	      samples[i].theta = pose.theta+
		(M_PI * 2.0* (gsl_rng_uniform_pos(srng3)-0.5) / map->nAngles);
	    }
	  
	  // apply motion model to all samples
	  for(i=0;i<NUMSAMP;i++)
	    samples[i] = samplemotionmodel(samples[i],odom,map,0.1);

	  // Bin the resulting samples
	  for(i=0;i<NUMSAMP;i++)
	    {
	      s_initial=PoseToCell(map,samples[i]);
	      if(s_initial.angle < map->nAngles &&
		 s_initial.angle >= 0 &&
		 s_initial.row < map->height &&
		 s_initial.row >= 0 &&
		 s_initial.col < map->width &&
		 s_initial.col >= 0)
		mapdata[s_initial.angle][s_initial.row][s_initial.col]+=1.0;
	    }

	  // Normalize
	  sum = 0.0;
	  for(i=0;i<map->nAngles;i++)
	    for(row=0;row<map->height;row++)
	      for(col=0;col<map->width;col++)
		sum += mapdata[i][row][col];
	  for(i=0;i<map->nAngles;i++)
	    for(row=0;row<map->height;row++)
	      for(col=0;col<map->width;col++)
		mapdata[i][row][col] /= sum;
	  // Trim
	  for(i=0;i<map->nAngles;i++)
	    for(row=0;row<map->height;row++)
	      for(col=0;col<map->width;col++)
		if(mapdata[i][row][col] <0.01)
		  mapdata[i][row][col] = 0.0;
	  // Normalize again
	  sum = 0.0;
	  for(i=0;i<map->nAngles;i++)
	    for(row=0;row<map->height;row++)
	      for(col=0;col<map->width;col++)
		sum += mapdata[i][row][col];
	  for(i=0;i<map->nAngles;i++)
	    for(row=0;row<map->height;row++)
	      for(col=0;col<map->width;col++)
		mapdata[i][row][col] /= sum;

	  printf("action: %d  angle: %d\n",act,ang);
	  for(i=0;i<map->nAngles;i++)
	    for(row=0;row<map->height;row++)
	      for(col=0;col<map->width;col++)
		if(mapdata[i][row][col] > 0.0)
		  {
		    s.row=0;
		    s.col=0;
		    s.angle=ang;
		    sp.row=row-7;
		    sp.col=col-7;
		    sp.angle=i;
		    Tr.addTransition(s, act, sp, mapdata[i][row][col]);
		    //printf("%d %d %d %lf\n",row-7,col-7,i,mapdata[i][row][col]);
		    count++;
		  }
	  
	  //printf("\n");
	}
    }

  Tr.write((char*)"transitions.dat");
  printf("%d transition probabilities generated\n",count);
  
  return 0;
}
  
