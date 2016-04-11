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

using namespace std;



#include <transmodel.h>

int main(int argc,char **argv)
{
  unsigned i;
  int ang,act;
  TransitionModel Tr;
  MapCoord s;
  vector<MapCell> list;
    
  Tr.read((char*)"transitions.dat");

  for(act=0;act<Tr.numActions();act++)
    {
      printf("action: %d  linear: %lf  angular: %lf\n",act,
	     Tr.getAction(act).linear_vel,
	     Tr.getAction(act).angular_vel
	     );
    }
  
  s.row = 100;
  s.col = 100;
  
  for(act=0;act<Tr.numActions();act++)
    for(ang=0;ang<Tr.numAngles();ang++)
      {
	printf("action: %d   angle: %d\n",act,ang);
	s.angle = ang;
	list = Tr.getTransitions(s,act);
	for(i=0;i<list.size();i++)
	  printf("%d %d %d %lf\n",
		 list[i].coord.row,
		 list[i].coord.col,
		 list[i].coord.angle,
		 list[i].value);
      }
    
  return 0;
}
  
