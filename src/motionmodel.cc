#include <stdlib.h>
#include <stdio.h>

#include <map.h>
#include <motionmodel.h>


/* Odometry
std_msgs/Header header
	uint32 seq
	time stamp
	string frame_id
string child_frame_id
geometry_msgs/PoseWithCovariance pose
	geometry_msgs/Pose pose
		geometry_msgs/Point position
			float64 x
			float64 y
			float64 z		
		geometry_msgs/Quaternion orientation
			float64 x
			float64 y
			float64 z
			float64 w
	float64[36] covariance
geometry_msgs/TwistWithCovariance twist
	geometry_msgs/Twist twist
		geometry_msgs/Vector3 linear
			float64 x
			float64 y
			float64 z
		geometry_msgs/Vector3 angular
			float64 x
			float64 y
			float64 z			
	float64[36] covariance
*/

float motionmodel(MapCell &st,    // state at time t (row, col, Theta)
		  MapCell &stp,  // state at time t-1 (row, col, Theta)
		  Odometry &odom, // odometry from t-1 to t 
		  MapStruct *map,
		  float dt )
{
	MapCell curr, prev;

	curr = st ; 
	prev = stp ;

	return veloctity_motion_model( curr, prev, odom, map, dt );
}

// Markov motion model estimates the probability of the current position
// given our previous position, direction, and velocities
float veloctity_motion_model(MapCell &st,    // state at time t (row, col, Theta)
		  MapCell &stp,  // state at time t-1 (row, col, Theta)
		  Odometry &odom, // odometry from t-1 to t 
		  MapStruct *map,
		  float dt )
{
	Pose2D pose = CellToPose(map, stp) ;
	Pose2D pose_p = CellToPose(map, st) ;

	double x,y,theta; // original positions
	double x_p, y_p, theta_p; // possible next position
	double v, w ; // hold our angular velocities
	double v_hat, w_hat, g_hat ; // calculated expected values
	double mu, r, r_str, x_str, y_str, theta_del ;
	double prob_1, prob_2, prob_3 ;

	// Original State
	x = pose.x ;
	y = pose.y ;
	theta = pose.theta ;

	// Possible Next State
	x_p = pose_p.x ;
	y_p = pose_p.y ;
	theta_p = pose_p.theta ;

	// Get our velocity readings
	v = odom.twist.twist.linear.x ;
	w = odom.twist.twist.angular.z ;	

	// Calculate mu
	mu = (x - x_p) * cos(theta) + (y - y_p) * sin(theta) ;
	mu /= ((y - y_p) * cos(theta) - (x - x_p) * sin(theta)) ;
	mu *= 0.5 ;

	// if mu is either too large or isn't a number, we know that our 
	// angular velocity is effectively zero, so we just need to calculate
	// the velocity between poses and set angular to zero.
	if( isnan(mu) || isinf(mu) )
	{
		v_hat = sqrt(pow(x-x_p,2.0) + pow(y-y_p,2.0)) / dt ;
		w_hat = 0 ;
		g_hat = 0 ;
	}
	else
	{
		// Let's get the center of our turn
		x_str = (x + x_p) / 2.0 + mu * (y - y_p) ;
		y_str = (y + y_p) / 2.0 + mu * (x_p - x) ;

		// calc the radius of the the turning circle
		r_str = sqrt(pow(x - x_str, 2.0) + pow(y - y_str, 2.0)) ;

		// calculate our change in angle
		theta_del = atan2(y_p - y_str, x_p - x_str) - atan2(y - y_str, x - x_str) ;

		// calc our linear and angular velocities
		v_hat = theta_del / dt * r_str ;

		w_hat = theta_del / dt ;

		g_hat = (theta_p - theta) / dt - w_hat ;
	}

	// STILL HAVING PROBLEMS WITH PROBABILITIES. There is an exception 
	// not currently accounted for, causing severe distortion

	// grab the prob of that the our bot is in this current cell given 
	// our calculated linear and angular velocities
	prob_1 = prob_gauss(v-v_hat, ALPHA_1 * abs(v) + ALPHA_2 * abs(w));

	prob_2 = prob_gauss(w-w_hat, ALPHA_3 * abs(v) + ALPHA_4 * abs(w));

	prob_3 = prob_gauss(g_hat, ALPHA_5 * abs(v) + ALPHA_6 * abs(w));

	return prob_1 * prob_2 * prob_3 ;
}

// Grab the probability from a gaussian dist. 
double prob_gauss(double a, double b)
{
	return exp( -0.5 * ((a*a)/(b*b))) / sqrt( 2 * M_PI * b * b ) ;
}


// More quickly grab a probablility, NOT DIALED IN
double prob_triangular(double a, double b)
{
	a = sqrt(a) ;
	b = 6 * b ;
	double c = sqrt(b) ;

	double prob ;

	if( a > c )
		prob = 0;
	else
	{
		prob = (c - a)/b ;
	}

	return prob ;
}
