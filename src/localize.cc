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
  double prob ;
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
  double prob ;
  double prob_sum =0;
  vector<double> probabilities ;
  Pose2D new_pose, old_pose ;
  // Some debugging code...
  //
  // Odometry has a Printer, so you can just pass it to an ostream.
  // ROS_INFO_STREAM(msg);
  // or print only the fields that you care about.
  ROS_INFO("dx=%lf dtheta=%lf\n",msg.twist.twist.linear.x,
	   msg.twist.twist.angular.z);

  // Normalize the importance rankings (must sum to one)
  // \forall p \in Particles do .... done

  // \forall p \in Particles do .... done
  for(int i = 0; i < Particles.size(); i++)
  {
    Particles[i].pose = new_pose = samplemotionmodel(Particles[i].pose, msg, Map, 0.1) ;
  }

  // Generate new particles stochastically based on their importance ranking.
  // \forall p \in Particles do .... done

  // Replace old set of particles with new set of particles.
}


// A callback function. Executed each time a new laser scan message
// arrives.  It gives an importance rank to each particle based on how
// well it matches the scan.
void handleScan(const sensor_msgs::LaserScan &msg)
{

  // Some debugging code...
  //
  // ROS_INFO("Scan Received: angle_min=%f angle_max=%f angle_increment=%f",
	 //   msg.angle_min,msg.angle_max,msg.angle_increment);
  //
  int nscans;
  // stringstream ss;
  nscans=(msg.angle_max-msg.angle_min)/msg.angle_increment;
  MapCoord cell ;
  int r ;
  double theta ;
  double prob, temp ;
  double sum = 0.0 ;
  // for(i=0;i<nscans;i++)
  // {
  //   //ss<<" "<<msg.ranges[i];

  // }
  // cout << nscans << endl;
  // ROS_INFO_STREAM(" "<<ss.str());
  // cout << endl ;

  // Remember to account for the offset of the sensor relative to the
  // center of the robot for each particle.
  // \forall p \in Particles do .... done
  for(int i = 0; i < Particles.size(); i++)
  {
    cell.row = Particles[i].pose.row;
    cell.col = Particles[i].pose.col;
    cell.angle = Particles[i].theta
    for(int j = 0; j < nscans; j++)
    {
      r = msg.ranges[j]
      theta = msg.angle_min + j * msg.angle_increment ;      
      temp = sensormodel(cell, theta, r, Map);
      if(temp > 0.0)
        prob *= 0.0;
      else
        prob = 0.0; 
    }
    sum += prob ;
    Particles[i].prob = prob; 
  }

  for(int i = 0; i < Particles.size(); i++)
  {
    while(Particles[i].prob == 0.0)
      Particles.erase(Particles.begin() + i) ;

    Particles[i].prob = prob / sum ;
  }
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
  //Initialize the ROS system and become a node.
  ros::init(argc,argv,"localize");
  ros::NodeHandle nh;
  ros::Rate loop_rate(10);

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
  for(int ang = 0; ang < Map->nAngles; ang++)
    for(int row = 0; row < Map->height; row++)
      for(int col = 0; col < Map->width; col++)
        {
          if(Map->rows[row][col] > 128)
          {
            particle new_particle ;

            new_particle.pose.row = row ;
            new_particle.pose.col = col ;
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

