#include "ros/ros.h"
#include "geometry_msgs/Vector3Stamped.h"
#include "sensor_msgs/Imu.h"

#include <rc/math/vector.h>
#include <cstring>
#include <chrono>
#include <cmath>
#include <iostream>

using namespace std;

//Initialize new message object and publisher
geometry_msgs::Vector3Stamped ORIgyro;
ros::Publisher ORIgyro_pub;

//Initialize 'previous cycle' variables so the first cycle can use them
double AngPrevx = 0.0, AngPrevy = 0.0, AngPrevz = 0.0;
double TimePrev = 0.0;

//Callback from subscriber. Performs ARF on accelerometer data.
void ORIfinder(const sensor_msgs::Imu RAWmsg){
	//Find time elapsed since previous cycle and set this cycle's time 
	//as the new TimePrev.
	double TimeNow = ros::Time::now().toSec();
	double TimeElapsed = TimeNow - TimePrev;
	TimePrev = TimeNow;

	//Fill AngVelx/y/z with current gyroscope reading
	double AngVelx = RAWmsg.angular_velocity.x;
	double AngVely = RAWmsg.angular_velocity.y;
	double AngVelz = RAWmsg.angular_velocity.z;

	//Feed data into the ORIgyro message for publishing
	//Equation for angle value is [ AngVel*TimeElapsed + PrevAngle ]
	ORIgyro.header = RAWmsg.header;
	ORIgyro.vector.x = AngVelx * TimeElapsed + AngPrevx;
	ORIgyro.vector.y = AngVely * TimeElapsed + AngPrevy;
	ORIgyro.vector.z = AngVelz * TimeElapsed + AngPrevz;

	AngPrevx = ORIgyro.vector.x;
	AngPrevy = ORIgyro.vector.y;
	AngPrevz = ORIgyro.vector.z;


	ORIgyro_pub.publish(ORIgyro);

	std::cout << "\n AngleX = " << ORIgyro.vector.x 
			  << "\n AngleY = " << ORIgyro.vector.y 
			  << "\n AngleZ = " << ORIgyro.vector.z 
			  << "\n";
}



//Main ROS loop. Initializes pubs/subs, establishes loop rates.
int main(int argc, char ** argv){
	ros::init(argc, argv, "ORIgyro");
	ros::NodeHandle n;

	TimePrev = ros::Time::now().toSec();

	ORIgyro_pub =
		n.advertise<geometry_msgs::Vector3Stamped>("imu/ORIgyro", 1000);

	ros::Subscriber RAWgyro_sub =
		n.subscribe("/imu/data_raw", 1000, ORIfinder);

	ros::Rate loop_rate(100);

	ROS_INFO("Listening to raw gyroscope data and finding angles, then publishing on /imu/ORIgyro");

	while( ros::ok() ){
		ros::spinOnce();
		loop_rate.sleep();
	}

	return 0;

}
