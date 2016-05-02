// This program should perform localization using the sensor and
// motion models.



/******************************************************************************
Author: Dr. Pyeatt

Modified: Steven Huerta

Some notes: This project is not producing the expected results of clumping
probabilites, collapsed from the randomly seeded agent possible locations. 
The images are printed to a folder Images. After a few iterations, the 
expected locations are reduced to a few locations.

My original plan was to produce a publisher to advertise the vector of 
particles for a python script to make a movie by writing the successive frames.
You can find the messages:
Particle.msg
Particle_vector.msg

The CMakeLists.txt and package.xml have been modified to add these messages.
THe next steps are to:
1. make a python package to subscribe to the vector and use the map as 
  parameter.
2. make a launch file that would start both this localize
  package and the python package to run the localization and 
  make a movie

/*****************************************************************************/
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

// This was my attempt to create a seperate message through ROS to pass to 
// what would be movie making node
#include <sdsmt_slam/Particle_vector.h>
#include <sdsmt_slam/Particle.h>


using namespace std;

#define NUM_ANGLES 32 // set the number of possible poses for our agent
#define SCAN_ANGLES 16 // set the number of scans per position we want to evaluate
#define NUM_PARTICLES 3000


// Particle structure will consist of 
// probability of belief of position
// and the pose of that agent
typedef struct {
  double prob ;
  Pose2D pose ;
} Particle ;

typedef vector<Particle> ParticleSet;

// Declare a set of Particles ...
ParticleSet Particles;

// Declare an occupancy grid map
MapStruct *Map = NULL;



// A callback function. Executed each time a new odometry message
// arrives.  It applies the sample motion model to the set of
// particles stochastically based on their importance rankings and
// creates a new set of particles.
void handleOdometry(const nav_msgs::Odometry &msg)
{
  Odometry message = msg ;
  int row, col ;
  int height, width ;

  height = Map->height ;
  width = Map->width ;

  // OK So here we have all the wonderful particles that we are going to output
  for(int i = 0; i < NUM_PARTICLES; i++)
  {
    // generate the new pose for the particle
    Particles[i].pose = samplemotionmodel(Particles[i].pose, message, Map, 0.1) ;

    row = Particles[i].pose.y ;
    col = Particles[i].pose.x ;


    // IF our new pose is not possible because it is off the map
    // , lets throw a random particle back in its place
    if( row < 0 || row >= height || col < 0 || col >= width )
    {
      Particles[i].pose.y = rand() % height ;
      Particles[i].pose.x = rand() % width ;
      Particles[i].pose.theta = rand() % NUM_ANGLES ;
    }

  }

}


// A callback function. Executed each time a new laser scan message
// arrives.  It gives an importance rank to each particle based on how
// well it matches the scan.
void handleScan(const sensor_msgs::LaserScan &msg)
{
  int r, best ;
  long double temp ;
  double prob, theta, pick;
  double best_prob = 0; 
  long double prob_sum = 0;
  vector<double> probabilities ;
  Particle temp_particle ;
  MapCoord cell ;

  // determine the number of scan angles
  int nscans=(msg.angle_max-msg.angle_min)/msg.angle_increment + 1;

  // determine the size of the increment to decimate
  // the scans to 
  int scan_incr = nscans / SCAN_ANGLES ;

  // create a new particle to store particles
  Particle new_particle ;

  // ...and a new vector to push these new particles into
  ParticleSet new_particles ;

  // For all particles in our list...
  for(int i = 0; i < NUM_PARTICLES; i++)
  {
    // ...calculate the posterior of belief for 
    // all the scans at this position
    for(int j = 0; j < nscans; j = j + scan_incr)
    {
      r = msg.ranges[j] ;
      theta = msg.angle_min + j * msg.angle_increment ;

      cell.row = Particles[i].pose.y ;
      cell.col = Particles[i].pose.x ;
      cell.angle = Particles[i].pose.theta ;

      // return the belief for this particular ray cast
      temp = sensormodel(cell, theta, r, Map);
      if(temp > 0.0)
        // we are going to use log to prevent underflow
        prob += log(temp) ;
      else
        prob = 0.0; 
    }
    if( prob != 0)
      // hey! we're bringing it back with exponent
      prob = exp(prob) ;
    // set our particle belief to the calc prob
    Particles[i].prob = prob ;

    // add the prob to sum of all probs
    prob_sum += prob ;
  }

  // We may have the condition where all possible 
  // locations have a 0 probability
  if(prob_sum > 0.0)
  {
    // Time for some gambling!
    // Let's pick a particle MONTE CARLO style
    for(int i = 0; i < NUM_PARTICLES; i++)
    {
      // SPIN THE WHEEL ;
      // We pick a number from [0,1]
      pick = ((double)rand()/(double)RAND_MAX);

      int j = 0 ;
      double wheel_sum = 0;

      while( pick >= wheel_sum )
      {
        // increment by (agent prob) / (total prob)
        // until we hit our prob pick
        wheel_sum += (Particles[j].prob / prob_sum);
        j++ ;
      }
      
      // While we're picking the agent likely locations
      // let's track which likely location is our
      // best guess
      if( Particles[j].prob > best_prob)
      {
        best = j ;
        best_prob = Particles[j].prob ; 
      }

      new_particle = Particles[j] ;

      // Put this particle in our list of new particles
      new_particles.push_back(new_particle) ;
    }

    // Let's take our best guess for the agent
    // and place that at the top of our vector 
    // so we can grab that off the top later
    temp_particle = Particles[0] ;
    Particles[0] = Particles[best] ;
    Particles[best] = temp_particle ;

      // Replace old set of particles with new set of particles.
    for(int i = 0; i < NUM_PARTICLES; i++)
    {
      Particles[i] = new_particles[i] ;
    }

  }

}

// This function writes the frame for the 
// map with the particles in the particle set
void write_frame(int frame_counter)
{
  int width = Map->width ;
  int height = Map->height ;
  int row, col ;
  char filename[80] ;

  sprintf(filename,"Images/Frame#%d.pgm",frame_counter);

  // Let's create an empty structure to copy our map into
  double** new_frame = allocate_double_map(width, height);

  //copy in the walls!
  for(row=0;row<height;row++)
    for(col=0;col<width;col++)
    {
      if(Map->rows[row][col]<128)
        new_frame[row][col] = 255;
      else
        new_frame[row][col] = 0;
    }

  // now let's add the particles
  for(int i = 0; i < NUM_PARTICLES; i++)
  {
    row = Particles[i].pose.y ;
    col = Particles[i].pose.x ;
    new_frame[row][col] = 128 ;
  }

  // output the frame
  write_double_map(filename, new_frame, width, height, 1) ;


  // release the created map copy
  free_double_map(new_frame, height) ;
}


int main(int argc,char **argv)
{

  srand(time(NULL));
  motionmodel_init();

  //Initialize the ROS system and become a node.
  ros::init(argc,argv,"localize");
  ros::NodeHandle nh;
  ros::Rate loop_rate(1);

  //Create the subscriber objects.
  ros::Subscriber odom=nh.subscribe("odom",100,&handleOdometry);
  ros::Subscriber scan1=nh.subscribe("scan",100,&handleScan);

  ros::Publisher pose=nh.advertise<geometry_msgs::Pose2D>("base_pose",10);

  Particle new_particle ;
  Pose2D new_pose ;

  if(argc != 2)
    {
      fprintf(stderr,"usage: %s <mapfile>\n  I need a map file!\n",argv[0]);
      exit(1);
    }
  
  // read the occupancy grid map 
  Map = readmap(argv,1,NUM_ANGLES,0.1);

  int width = Map->width ;
  int height = Map->height ;

  // Initialize our particles to some random
  // location on the map with some 
  // random given angle
  for(int i = 0; i < NUM_PARTICLES; i++)
  {
    new_pose.x = rand() % width ;
    new_pose.y = rand() % height ;
    new_pose.theta = rand() % NUM_ANGLES ;

    new_particle.pose = new_pose ;

    // push in the new particle to the list
    Particles.push_back( new_particle ) ;
  }
  
  ROS_INFO("sdsmt_localize: Starting.");

  int frame_counter = 0 ;

  // Enter event loop
  while(ros::ok())
    {
      // handle all pending messages
      ros::spinOnce();


      
      // create and write or display a new image
      write_frame(frame_counter) ;
      frame_counter++ ;



      
      // create and publish the Pose2D message
      
      // sleep...
      loop_rate.sleep();
    }
    
  return 0;
}

