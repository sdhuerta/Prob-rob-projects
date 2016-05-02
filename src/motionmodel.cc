
#include <stdlib.h>
#include <stdio.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <unistd.h>
#include <math.h>
#include <gsl/gsl_rng.h>
#include <gsl/gsl_randist.h>
#include <map.h>
#include <motionmodel.h>

//double alpha[]={30.0,30.0,3.0,3.0,3.0,3.0};
//double alpha[]={60.0,60.0,3.0,3.0,3.0,3.0};

//double alpha[]={10.0,10.0,1.0,1.0,1.0,1.0};
//double alpha[]={2.5,2.5,0.1,0.1,0.1,0.1};
//double alpha[]={20.0,20.0,2.0,2.0,2.0,2.0};

//double alpha[]={10.0,10.0,1.0,1.0,1.0,1.0};
//double alpha[]={0.5,0.5,0.5,0.5,0.25,0.25};
double alpha[]={0.1,0.1,0.5,0.5,0.25,0.25};

//double alpha[]={0.5,0.5,10,10,0.25,0.25};

#define SQR(x) ((x)*(x))

#define prob(x,y) gsl_ran_gaussian_pdf(x,y)

double pnd(double a,double bsq)
{
  //printf("x=%f   variance= %f\n",a,bsq);
  return 1.0/sqrt(M_PI*2.0*bsq)*exp(-0.5*SQR(a)*bsq);
}

double ptd(double a,double bsq)
{
  static const double sqrt6 = sqrt(6.0);
  double tmp=1.0/(sqrt(bsq)*sqrt6) - fabs(a)/6.0*bsq;
  return (tmp>0.0?tmp:0.0);
}


float motionmodel(MapCoord &stp,    // state at time t
		  MapCoord &st,     // state at time t-1
		  Odometry &odom,   // odometry from t-1 to t
		  MapStruct *map,
		  float dt)
{
  double
    x,y,theta,
    xp,yp,thetap,
    v,w,vs,ws,mu,
    xstar,ystar,rstar,
    dtheta,costheta,sintheta,
    what,vhat,ghat,
    aprob,bprob,cprob,
    tmp,sign;

  // Copy stuff to local variables and convert from map row/col to global x,y
  
  Pose2D stpose = CellToPose(map,st);
  Pose2D stppose = CellToPose(map,stp);



  x = stpose.x;
  y = stpose.y;
  theta = stpose.theta;
  
  xp = stppose.x;
  yp = stppose.y;
  thetap = stppose.theta;

  // make sure theta is between +/- PI
  while(theta>M_PI)
    theta -= 2.0*M_PI;
  while(theta<-M_PI)
    theta += 2.0*M_PI;

  // transform stppose to a coordinate system where stpose is at origin
  // with zero angle.
  xp = xp-x;
  yp = yp-y;
  costheta=cos(theta);
  sintheta=sin(theta);
  tmp=xp;
  xp = xp*costheta - yp*sintheta;
  yp = tmp*sintheta + yp*costheta;  
  thetap = thetap-theta;

  // make sure thetap is within +/- PI 
  while(thetap > M_PI)
    thetap -= 2.0*M_PI;
  while(thetap < -M_PI)
    thetap += 2.0*M_PI;

  // while(thetap > 2.0 * M_PI)
  //   thetap -= 2.0*M_PI;
  // while(thetap < 0.0)
  //   thetap += 2.0 * M_PI;

  // Put it in the 1st or 2nd quadrant
  if(yp<0.0)
    {
      sign=-1;
      //thetap=-thetap;
      yp=-yp;
     }
  else
    sign=1;

  // now stpose is at origin with angle 0

  // If we started at state st and went to stp, then what would the
  // forward velocity and angular velocity have been?  It would
  // be (approximately) the length of an arc connecting the two points
  // and ending tangent to the direction in each state.

  // So, what is the radius of the arc and the angle between the
  // two points?  From that, we can get the arc length.
  // First, find the center of the circle.  d(x,y) = dtheta 8 r
  
  mu = 0.5 * xp / yp; 
     
  if(fabs(mu)>1.0e12 || isnan(mu) || isinf(mu))
     {
       // either moving in straght line, or not moving at all.
       xstar = ystar=0.0;
       dtheta=0.0;
       rstar=xp;
       vhat = rstar/dt;
       what = 0.0;
       ghat = thetap/dt;
     }
   else
     {
       // normal curved movement
      xstar = 0.5*xp + mu * -yp;
      ystar = 0.5*yp + mu * xp;
      rstar = sqrt(SQR(xstar)+SQR(ystar));
      dtheta = atan2(yp-ystar,xp-xstar) - atan2(-ystar,-xstar);
      //dtheta *= sign;
      if(sign<0)
      	what = dtheta/dt;
      else
      	what = -dtheta/dt;
      //      what = dtheta/dt*sign;
      vhat = rstar*dtheta/dt;  
      //ghat = fabs(thetap/dt) - what;
      //ghat= fabs(ghat);
      ghat = (thetap/dt) - what;
     }

  v = odom.twist.twist.linear.x;
  w = odom.twist.twist.angular.z;
  vs = fabs(v);
  ws = fabs(w);
  
  aprob = prob(v-vhat,alpha[0]*vs+alpha[1]*ws);
  bprob = prob(w-what,alpha[2]*vs+alpha[3]*ws);
  cprob = prob(ghat,alpha[4]*vs+alpha[5]*ws);
  
  // aprob = 1.0;
  // bprob = 1.0;
  // cprob = 1.0;
  
  return aprob*bprob*cprob;
}


static gsl_rng *rng1, *rng2, *rng3;

void motionmodel_init()
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
  rng1 = gsl_rng_alloc(gsl_rng_mt19937);
  rng2 = gsl_rng_alloc(gsl_rng_mt19937);
  rng3 = gsl_rng_alloc(gsl_rng_mt19937);
  gsl_rng_set(rng1,seeds[0]);
  gsl_rng_set(rng2,seeds[1]);
  gsl_rng_set(rng3,seeds[2]);
}


//double malpha[]={0.1,0.1,0.5,0.5,0.1,0.1};
//double malpha[]={0.25,0.1,0.1,0.25,0.1,0.1};
//double malpha[]={1.0,1.0,0.25,0.25,0.1,0.1};
double malpha[]={0.005,0.005,0.005,0.005,0.005,0.005};

Pose2D samplemotionmodel(Pose2D &st,    // state at time t
			 Odometry &odom, // odometry from t-1 to t
			 MapStruct *map,
			 float dt)
{
  double v,w,vs,ws,vhat,what,ghat;
  double sigma1,sigma2,sigma3,ratio;
  Pose2D result;
      
  v = odom.twist.twist.linear.x;  
  w = odom.twist.twist.angular.z;

  vs = fabs(v);
  ws = fabs(w);

  if(vs < 0.0001 && ws < 0.0001)
    {
      result.x = st.x;
      result.y = st.y;
      result.theta = st.theta;
      return result;
    }

  sigma1 = malpha[0]*vs + malpha[1]*ws;
  sigma2 = malpha[2]*vs + malpha[3]*ws;
  sigma3 = malpha[4]*vs + malpha[5]*ws;
  
  vhat = v + gsl_ran_gaussian_ziggurat(rng1,sigma1);
  what = w + gsl_ran_gaussian_ziggurat(rng2,sigma2);
  ghat = gsl_ran_gaussian_ziggurat(rng3,sigma3);

  ratio=vhat/what;
  if(isnan(ratio)||isinf(ratio)||(fabs(ratio)>1e6))
    {
      ratio = 1.0;
      result.x = st.x + v*dt*cos(st.theta);
      result.y = st.y + v*dt*sin(st.theta);
      result.theta = st.theta + ghat*dt;
    }
  else
    {
      result.x = st.x - ratio*sin(st.theta) + ratio*sin(st.theta+what*dt);
      // result.y = st.y - ratio*cos(st.theta) + ratio*cos(st.theta+what*dt);
      result.y = st.y - ratio*cos(st.theta) + ratio*cos(st.theta+what*dt);
      result.theta = st.theta + (what+ghat)*dt;
    }
      
  return result;
}

