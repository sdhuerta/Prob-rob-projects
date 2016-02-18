
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
		  MapStruct *map)
{
  Twist twist = odom.twist.twist; // make it easier to read...

  // this is here just to give it something to do.
  return (st.row*st.col)/(float)(map->width*map->height);
}

float motion_model_velocity(MapCell &st,    // state at time t (contains row, col, theta)
		  					MapCell &stp,  // state at time t-1 (contains row, col, theta)
		  					Odometry &odom, // odometry from t-1 to t 
		  					MapStruct *map) 
{
	time_step =  1;

	mu = 0.5 * ((st.col - stp.col)cos(st.theta) + (st.row - stp.row)sin(st.theta))
		 / ((st.row - stp.row)cos(st.theta) - (st.col - stp.col)sin(st.theta)) ;

	col = (st.col + stp.col)/2.0 + mu * (st.row - stp.row) ;

	row = (st.row + stp.row)/2.0 + mu * (stp.col - st.col) ;

	dis = sqrt(pow(st.col - col, 2) + pow(st.row - row, 2)) ;

	theta_del = atan2(stp.row - row, stp.col - col) - 
				atan2(st.row - row, st.col - col);

	v_hat = theta_del / time_step * dis ;

	w_hat = theta_del / time_step ;

	gamma_hat =  (stp.theta - st.theta) / time_step - w_hat ;

	v = prob(v-v_hat, alpha1 * abs(v) + alpha_2 * abs(w)) ;
}
