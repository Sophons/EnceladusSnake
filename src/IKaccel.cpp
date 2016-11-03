#include "ros/ros.h"
#include "geometry_msgs/Vector3Stamped.h"

#include <rc/math/vector.h>
#include <cstring>
#include <chrono>
#include <cmath>
#include <iostream>

using namespace std;

//Initialize new message object.
geometry_msgs::Vector3Stamped ORIaccel;
ros::Publisher IKacc_pub;


//Callback from subscriber. Performs ARF on accelerometer data.
	//Initialize variables for stuff we want to keep between loops

void IKinematics(const geometry_msgs::Vector3Stamped msg){

	//Get filtered accelerations in x/y/z axes
	double Ax = msg.vector.x;
	double Ay = msg.vector.y;
	double Az = msg.vector.z;

	//Calculate angle from x/y/z axes from acceleration ratios
/*	double thetax = atan( Ax/sqrt( Ay*Ay + Az*Az ) ); //Angle between world xy plane and accelerometer x axis
	double   psiy = atan( Ay/sqrt( Ax*Ax + Az*Az ) ); //Angle between world xy plane and accelerometer y axis
	double   phiz = atan( Az/sqrt( Ax*Ax + Ay*Ay ) ); //Angle between gravity vector and accelerometer z axis

	//Feed data into the ORIaccel message for publishing
	ORIaccel.header = msg.header;
	ORIaccel.vector.x = thetax;
	ORIaccel.vector.y = psiy;
	ORIaccel.vector.z = phiz;
*/

	double roll = atan(Ay/sqrt( Ax*Ax + Az*Az ));
	double pitch = atan(-1.0*Ax/Az);
	ORIaccel.header = msg.header;
	ORIaccel.vector.x = roll;
	ORIaccel.vector.y = pitch;
	ORIaccel.vector.z = 0.0;

	IKacc_pub.publish(ORIaccel);

//	std::cout << " theta = " << thetax  << "\n  psi = " << psiy << "\n phi = " << phiz << "\n \n";
	std::cout << " roll = " << roll << "\n pitch = " << pitch << "\n \n";
}



//Main ROS loop. Initializes pubs/subs, establishes loop rates.
int main(int argc, char ** argv){
	ros::init(argc, argv, "IKaccel");
	ros::NodeHandle n;

	IKacc_pub =
		n.advertise<geometry_msgs::Vector3Stamped>("imu/IKaccel", 1000);

	ros::Subscriber ARFaccel_sub =
		n.subscribe("/imu/ARFaccel", 1000, IKinematics);

	ros::Rate loop_rate(100);

	ROS_INFO("Listening to filtered accelerometer data and applying IK, then publishing on /imu/IKacc");

	while( ros::ok() ){
		ros::spinOnce();
		loop_rate.sleep();
	}

	return 0;

}
