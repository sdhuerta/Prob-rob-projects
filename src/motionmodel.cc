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

float veloctity_motion_model(MapCell &st,    // state at time t (row, col, Theta)
		  MapCell &stp,  // state at time t-1 (row, col, Theta)
		  Odometry &odom, // odometry from t-1 to t 
		  MapStruct *map,
		  float dt )
{
	if( map->rows[st.row][st.col] == 0 )
		return 0 ;

	double mu ;
	double prev_theta = stp.theta ;
	double cur_theta = st.theta; 
	double v, w, v_hat, w_hat, g_hat;
	double x, y, r, d_theta ;
	double prob_one, prob_two, prob_three;

	v = odom.twist.twist.linear.x ;
	w = odom.twist.twist.angular.z ;

	mu = (stp.col - st.col) * cos(prev_theta) + (stp.row - st.row) * sin(prev_theta);
	mu /= (stp.row - st.row) * cos(prev_theta) + (stp.col - st.col) * sin(prev_theta);
	mu *= 0.5 ;

	// Coordinates of the center of turning radius
	x = 0.5 * (stp.col + st.col) + mu * (stp.row - st.row) ;
	y = 0.5 * (stp.row + st.row) + mu * (st.col - stp.col) ;

	// Distance from center of turning 
	r = sqrt(pow(stp.col - x, 2.0) + pow(stp.row - y, 2.0)) ;

	if( isnan(r) || isinf(r) )
	{
		v_hat = sqrt(pow(st.col - stp.col, 2.0) + pow(st.row - stp.row, 2.0)) / dt ;
		w_hat = 0 ;
		g_hat = 0 ;
	}
	else
	{
		d_theta = atan2(st.row - y, st.col - x) - atan2(stp.row - y, stp.col - x) ;

		v_hat = d_theta / dt * r ;
		w_hat = d_theta / dt ;

		g_hat = (cur_theta - prev_theta) / dt - w_hat ;
	}

	prob_one = prob_gauss(v-v_hat, 
						  ALPHA_1 * abs(v) + ALPHA_2 * abs(w));

	prob_two = prob_gauss(w-w_hat, 
		                  ALPHA_3 * abs(v) + ALPHA_4 * abs(w));

	prob_three = prob_gauss(g_hat, 
		      				ALPHA_5 * abs(v) + ALPHA_6 * abs(w));

	return prob_one * prob_two * prob_three ;
}



double prob_gauss(double linear_vel, double angular_vel)
{

	return exp( (-0.5) * (pow(linear_vel, 2) / angular_vel ) / (sqrt( 2 * M_PI) * angular_vel ) );
}



double prob_triangular(double linear_vel, double angular_vel)
{

	//printf("%10f\t%10f\n",linear_vel, angular_vel) ;
	double prob ;
	double eval_linear = abs(linear_vel);
	double eval_angular = pow(6 * angular_vel, 2);

	if( eval_linear > eval_angular )
		prob = 0;
	else
	{
		prob = (eval_angular - eval_linear) / (6 * angular_vel) ;
	}

	return prob ;
}
