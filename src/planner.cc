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
#include <values.h>
#include <gsl/gsl_rng.h>
#include <gsl/gsl_randist.h>
#include <string.h>

using namespace std;

/* my headers */
#include <map.h>
#include <transmodel.h>



/* function to print usage message */
void usage(char *name)
{
  fprintf(stderr,"Usage: %s map_file trans_file row col angle\n",name);
  fprintf(stderr," map_file is a PNG file used as a map in"
	  " the stage simulator.\n");
  fprintf(stderr," trans_file is the transition probability file and \n");
  fprintf(stderr," row col angle is the goal map cell and angle.\n");
  exit(1);
}


#define gamma 0.99

int main(int argc,char **argv)
{
  unsigned i;
  int row,col,ang,act;
  TransitionModel Tr;
  vector<MapCell> list;
  MapStruct *map;
  MapCell goal[36];
  MapCoord s,sp;
  vector<MapCell> tlist;
  double Psasp,R,Vsp,bestval,curval,maxval;
  unsigned char ***pi;
  double ***val,***nextval,***tmp;
  int best;
  
  if(argc != 6)
    usage(argv[0]);

  // read transition and reward function
  Tr.read(argv[2]);

  // read the map file
  map = readmap(argv,1,Tr.numAngles(),0.1);

  // set the goal(s)
  goal[0].coord.row = atoi(argv[3]);
  goal[0].coord.col = atoi(argv[4]);
  goal[0].coord.angle = atoi(argv[5]);
  goal[0].value = 0.0;
  if(goal[0].coord.col<0 || goal[0].coord.col >= map->width)
    {
      fprintf(stderr,"column is out of range for this map.\n");
      exit(2);
    }
  if(goal[0].coord.row<0 || goal[0].coord.row >= map->height)
    {
      fprintf(stderr,"row is out of range for this map.\n");
      exit(2);
    }
  if(goal[0].coord.angle<0 || goal[0].coord.angle >= map->nAngles)
    {
      fprintf(stderr,"angle is out of range for this map.\n");
      exit(2);
    }

  for(i=1;i<36;i++)
    {
      goal[i]=goal[0];
      goal[i].coord.angle = i;
    }

  // allocate space for the policy and initialize to action 0
  // allocate space for the value function and initialize to 0.0
  pi = new unsigned char**[map->nAngles];
  val = new double**[map->nAngles];
  nextval = new double**[map->nAngles];
  for(ang=0;ang<map->nAngles;ang++)
    {
      pi[ang] = new unsigned char*[map->height];
      val[ang] = new double*[map->height];
      nextval[ang] = new double*[map->height];
      for(row=0;row<map->height;row++)
	{
	  pi[ang][row] = new unsigned char[map->width];
	  val[ang][row] = new double[map->width];
	  nextval[ang][row] = new double[map->width];
	  for(col=0;col<map->width;col++)
	    {
	      pi[ang][row][col] = 0;
	      val[ang][row][col] = 0.0;
	    }
	}
    }


  // for(ang=0;ang<map->nAngles;ang++)
  //   for(row=0;row<map->height;row++)
  //     for(col=0;col<map->width;col++)
  // 	if(map->rows[row][col]<64)
  // 	  val[ang][row][col] = -100000;

  
  double valdiff = 0.0;
   
  for(int loops=0;loops<200;loops++)
    {
      valdiff = 0.0;
      // run value iteration step
      for(ang=0;ang<map->nAngles;ang++)
	{
	  s.row=0;
	  s.col=0;
	  s.angle=ang;
	  for(row=0;row<map->height;row++)
	    {
	      s.row = row;
	      for(col=0;col<map->width;col++)
		{
		  s.col = col;
		  if(map->rows[s.row][s.col] > 64)
		    {
		      maxval = -MAXDOUBLE;
		      best = 0;
		      for(act=0;act < Tr.numActions(); act++)
			{
			  curval=0.0;
			  // Get transition probs for current map cell
			  // for current action
			  // act = pi[ang][row][col];
			  tlist = Tr.getTransitions(s,act);
			  for(i=0;i < tlist.size(); i++)
			    {
			      //sp.row = s.row + tlist[i].coord.row;
			      sp.row = s.row + tlist[i].coord.row;
			      sp.col = s.col + tlist[i].coord.col;
			      sp.angle = s.angle + tlist[i].coord.angle;
			      while(sp.angle<0)
				sp.angle += map->nAngles;
			      while(sp.angle >= map->nAngles)
				sp.angle -= map->nAngles;
			      Psasp = tlist[i].value;
			      // getReward will change sp if necessary
			      R = Tr.getReward(s,act,sp,map,goal,36);
			      Vsp = val[sp.angle][sp.row][sp.col];
			      //val[ang][row][col] += Psasp*(R + gamma*Vsp);
			      curval += Psasp*(R + gamma * Vsp);
			    }
			  if(s.row == goal[0].coord.row &&
			     s.col == goal[0].coord.col)
			    {
			      printf("goal state. Action: %d R: %lf Vsp: %lf Psasp: %lf\n",act,R,Vsp,Psasp);
			    }
			  if(curval > maxval)
			    {
			      maxval = curval;
			      best = act;
			    }
			}
		      valdiff += fabs(val[ang][row][col]-maxval);
		      nextval[ang][row][col]=maxval;
		      pi[ang][row][col]=best;
		    }
		}
	    }
	}
      tmp = val;
      val = nextval;
      nextval = tmp;
      printf("Change: %lf\n",valdiff);
    }
  // // do policy improvement step
  // for(ang=0;ang<map->nAngles;ang++)
  //   for(row=0;row<map->height;row++)
  //     for(col=0;col<map->width;col++)
  // 	if(map->rows[row][col] > 10)
  // 	  {
  // 	    best = 0;
  // 	    bestval = 0.0;
  // 	    act = 0;
  // 	    s.angle = ang;
  // 	    s.row = row;
  // 	    s.col = col;
  // 	    tlist = Tr.getTransitions(s,act);
  // 	    for(i=0;i < tlist.size(); i++)
  // 	      { // calculate value for action 0
  // 		sp.row = s.row + tlist[i].coord.row;
  // 		sp.col = s.col + tlist[i].coord.col;
  // 		sp.angle = s.angle + tlist[i].coord.angle;
  // 		if(sp.row >= 0 && sp.row < map->height &&
  // 		   sp.col >= 0 && sp.col < map->width)
  // 		  {
  // 		    while(sp.angle<0)
  // 		      sp.angle += map->nAngles;
  // 		    while(sp.angle >= map->nAngles)
  // 		      sp.angle -= map->nAngles;
  // 		    tlist = Tr.getTransitions(s,act);
  // 		    Psasp = tlist[i].value;
  // 		    R = Tr.getReward(s,act,sp,map,&goal,1);
  // 		    Vsp = val[sp.angle][sp.row][sp.col];
  // 		    bestval += Psasp*(R + gamma * Vsp);
  // 		  }
  // 	      } 
  // 	    for(act=1;act<Tr.numActions();act++)
  // 	      { // calculate values for remaining actions and choose best
  // 		curval = 0.0;
  // 		tlist = Tr.getTransitions(s,act);
  // 		for(i=0;i < tlist.size(); i++)
  // 		  {
  // 		    sp.row = s.row + tlist[i].coord.row;
  // 		    sp.col = s.col + tlist[i].coord.col;
  // 		    sp.angle = s.angle + tlist[i].coord.angle;
  // 		    if(sp.row >= 0 && sp.row < map->height &&
  // 		       sp.col >= 0 && sp.col < map->width)
  // 		      {
  // 			while(sp.angle<0)
  // 			  sp.angle += map->nAngles;
  // 			while(sp.angle >= map->nAngles)
  // 			  sp.angle -= map->nAngles;
  // 			tlist = Tr.getTransitions(s,act);
  // 			Psasp = tlist[i].value;
  // 			R = Tr.getReward(s,act,sp,map,&goal,1);
  // 			Vsp = val[sp.angle][sp.row][sp.col];
  // 			curval += Psasp*(R + gamma * Vsp);
  // 		      }
  // 		  }
  // 		if(curval>bestval)
  // 		  {
  // 		    bestval = curval;
  // 		    best = act;
  // 		  }
  // 	      }
  // 	    pi[ang][row][col] = best;
  // 	  }


 
  double ***valuefunc;
  valuefunc=new double**[Tr.numAngles()];
  for(ang=0;ang<map->nAngles;ang++)
    valuefunc[ang] = allocate_double_map(map->width,map->height);

  double minval =  val[0][0][0];
  for(ang=0;ang<map->nAngles;ang++)
    for(row=0;row<map->height;row++)
      for(col=0;col<map->width;col++)
	{
	  if(map->rows[row][col]<64 && val[ang][row][col]<minval)
	    minval = val[ang][row][col];
	  valuefunc[ang][row][col] = val[ang][row][col];
	  printf("Vsp: %lf\n",val[ang][row][col]);
	}

  for(i=0;i<Tr.numAngles();i++)
    printf("goal %d value: %lf\n",i,
	   val[goal[i].coord.angle][goal[i].coord.row][goal[i].coord.col]);

  
  for(ang=0;ang<map->nAngles;ang++)
    for(row=0;row<map->height;row++)
      for(col=0;col<map->width;col++)
	if(map->rows[row][col]<64)
	  valuefunc[ang][row][col] = minval;

  for(i=0;i<Tr.numAngles();i+=9)
    {
      char filename[128];
      sprintf(filename,"value%02d.pgm",i);
      write_double_map(filename,valuefunc[i],map->width,map->height,1);
    }
  for(ang=0;ang<map->nAngles;ang++)
    {
      for(row=0;row<map->height;row++)
	{
	  for(col=0;col<map->width;col++)
	    printf("%d",pi[ang][row][col]);
	  printf("\n");
	}
      printf("\n");
    }    

    

  


  return 0;
}
  
