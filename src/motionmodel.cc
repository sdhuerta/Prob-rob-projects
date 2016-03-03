s
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


	double v,w;
	double time_step;
	double mu, col, row, r;
	double dis, theta_del;
	double v_hat, w_hat, gamma_hat ;
	double prob_v, prob_w, prob_gamma;

	int x_prev = stp.col;
	int y_prev = stp.row;
	float theta_prev = -stp.theta;

	int x_curr = st.col;
	int y_curr = st.row;
	float theta_curr = -st.theta;

	time_step = dt ; // We need to change this to take the header

	v = odom.twist.twist.linear.x ;
	w = -odom.twist.twist.angular.z ;

	mu = 0.5 * ((x_prev - x_curr) * cos(theta_prev) + (y_prev - y_curr) * 
		 sin(theta_prev)) / ((y_prev - y_curr) * cos(theta_prev) - 
		 (x_prev - x_curr) * sin(theta_prev)) ;

	col = (x_curr + x_prev)/2.0 + mu * (y_prev - y_curr) ;
	row = (x_curr + x_prev)/2.0 + mu * (x_curr - x_prev) ;

	dis = sqrt(pow(x_prev - col, 2) + pow(y_prev - row, 2)) ;

	if( isnan(dis) || isinf(dis) || dis > 100000 )
	{
		v_hat = 0 ;
		w_hat = 0 ;
		gamma_hat = 0;
	}
	else
	{				
		theta_del = atan2(y_curr - row, x_curr - col) - 
					atan2(y_prev - row, x_prev - col);

		v_hat = theta_del / time_step * dis ;

		w_hat = theta_del / time_step ;

		gamma_hat =  (theta_curr - theta_prev) / time_step - w_hat ;

	}

	prob_v = prob_gauss((v-v_hat), (ALPHA_1 * abs(v) + ALPHA_2 * abs(w))) ; 

	prob_w = prob_gauss((w-w_hat), (ALPHA_3 * abs(v) + ALPHA_4 * abs(w))) ;

	prob_gamma = prob_gauss(gamma_hat, (ALPHA_5 * abs(v) + ALPHA_6 * abs(w)))	 ;

	//printf( "%6.4lf %6.4lf %6.4lf \n", prob_v, prob_w, prob_gamma);

	return prob_v * prob_w * prob_gamma ;
}



double prob_gauss(double linear_vel, double angular_vel)
{

	return exp( (-0.5) * pow(linear_vel/angular_vel, 2) ) / (sqrt( 2 * M_PI) * angular_vel ) ;
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
