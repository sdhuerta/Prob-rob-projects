// This program should perform localization using the sensor and
// motion models.

#include <sstream>
#include <ros/ros.h>
#include <turtlesim/Pose.h>
#include <iomanip>  //for std::setprecision and std::fixed
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/Range.h>
#include <sensor_msgs/LaserScan.h>
#include <std_msgs/String.h>
#include <geometry_msgs/Pose2D.h>

#include <map.h>
#include <motionmodel.h>
#include <sensormodel.h>

using namespace std;


// Remember: in ROS, Odometry twist is velocity, not distance...
typedef struct{
  long double prob ;
  geometry_msgs::Pose2D pose ;
} particle ;


// Define a set of particles ....
typedef vector<particle> ParticleSet;

// Declare a set of Particles ...
ParticleSet Particles;

// Declare an occupancy grid map
#define NUM_ANGLES 32

MapStruct *Map = NULL;

// A callback function. Executed each time a new odometry message
// arrives.  It applies the sample motion model to the set of
// particles stochastically based on their importance rankings and
// creates a new set of particles.
void handleOdometry(const nav_msgs::Odometry &msg)
{
  ROS_INFO("ODOM");
  nav_msgs::Odometry message = msg ;
  geometry_msgs::Pose2D pose ;
  int num_particles = Particles.size() ;

  for(int i = 0; i < num_particles; i++)
  {
    ROS_INFO("BEFORE FUNC");
    samplemotionmodel(Particles[i].pose, message, Map, 0.1) ;
    ROS_INFO("AFTER FUNC\n");
  }

}


// A callback function. Executed each time a new laser scan message
// arrives.  It gives an importance rank to each particle based on how
// well it matches the scan.
void handleScan(const sensor_msgs::LaserScan &msg)
{
  //ROS_INFO("ENTERING SCAN") ;

  int nscans;
  nscans=(msg.angle_max-msg.angle_min)/msg.angle_increment;
  ParticleSet New_Particles ;
  particle new_particle ;
  MapCoord cell ;
  int r ;
  double theta ;
  long double prob ;
  long double temp, pick ;
  long double particles_sum = 0.0 ;

  int num_particles = Particles.size() ;

  for(int i = 0; i < num_particles; i++)
  {
    cell.col = Particles[i].pose.x ;
    cell.row = Particles[i].pose.y ;
    cell.angle = Particles[i].pose.theta ;
    prob = 1.0 ;
    for(int j = 0; j < nscans; j = j + 40)
    {
      r = msg.ranges[j] ;
      theta = msg.angle_min + j * msg.angle_increment ;      
      temp = sensormodel( cell, theta, r, Map);
      if(temp > 0.0)
        prob *= temp;
      else
        prob = 0.0; 
    }
    if (prob > .00000005)
      ROS_INFO("HIT");
    particles_sum += prob ;
    Particles[i].prob = prob; 
  }

  //ROS_INFO("I DIED 2 PROB SUM: %lf", particles_sum) ;

  // Time for some gambling!
  // Let's pick a particle MONTE CARLO style
  // for(int i = 0; i < num_particles; i++)
  // {
  //   pick = (rand() % num_particles) / num_particles ;

  //   int j = 0 ;
    
  //   double sum = 0;

  //   while( sum < pick )
  //   {
  //     sum += (Particles[j].prob / particles_sum);
  //     j++ ;
  //   }

  //   new_particle.pose = Particles[j].pose ;
  //   new_particle.prob = Particles[j].prob ;

  //   New_Particles.push_back(new_particle) ;
  // }

  // Particles.clear();

  //   // Replace old set of particles with new set of particles.
  // for(int i = 0; i < num_particles; i++)
  // {
  //   Particles.push_back(New_Particles[i]);
  // }

}




// A callback function. Executed each time a new single range sensor
// message arrives.  It gives an importance rank to each particle
// based on how well it matches the range reading.
void handleRange(const sensor_msgs::Range &msg)
{
  // The robot has IR sensors, but the simulator has no data from them.
}


int main(int argc,char **argv)
{
  srand(time(NULL));
  //Initialize the ROS system and become a node.
  ros::init(argc,argv,"localize");
  ros::NodeHandle nh;
  ros::Rate loop_rate(5);

  //Create the subscriber objects.
  ros::Subscriber odom=nh.subscribe("odom",100,&handleOdometry);
  ros::Subscriber scan1=nh.subscribe("scan",100,&handleScan);

  // These show up in rvis, but don't seem to produce anything....
  // ros::Subscriber scan2=nh.subscribe("ir_scan",5,&handleScan);
  // ros::Subscriber scan3=nh.subscribe("virtual_sensor_scan",5,&handleScan);

  // Don't seem to have any single-beam range devices... 
  // ros::Subscriber range=nh.subscribe("range",5,&handleRange);

  // We are going to publish 2D Pose messages on the base_pose topic...
  ros::Publisher pose=nh.advertise<geometry_msgs::Pose2D>("base_pose",10);

  if(argc != 2)
    {
      fprintf(stderr,"usage: %s <mapfile>\n  I need a map file!\n",argv[0]);
      exit(1);
    }
  
  // read the occupancy grid map 
  Map = readmap(argv,1,NUM_ANGLES,0.1);

  // Initialize our particles
  for(int ang = 0; ang < Map->nAngles; ang = ang + 4)
    for(int row = 0; row < Map->height; row = row + 4)
      for(int col = 0; col < Map->width; col = col + 4)
        {
          if(Map->rows[row][col] > 128)
          {
            particle new_particle ;

            new_particle.pose.x = row ;
            new_particle.pose.y = col ;
            new_particle.pose.theta = ang ;
            new_particle.prob = 1.0 ;

            Particles.push_back(new_particle) ; 
          }

        }        
  
  ROS_INFO("sdsmt_localize: Starting.");

  // Enter event loop
  while(ros::ok())
    {
      // handle all pending messages
      ros::spinOnce();
      
      // create and write or display a new image
      //movie_next_frame();

      
      // create and publish the Pose2D message
      
      // sleep...
      loop_rate.sleep();
    }
    
  return 0;
}

