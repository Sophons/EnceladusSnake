#include "ros/ros.h"
#include "geometry_msgs/Vector3Stamped.h"
#include "sensor_msgs/Imu.h"

#include <rc/math/vector.h>
#include <cstring>
#include <chrono>
#include <cmath>
#include <iostream>

using namespace std;

//Initialize new message object.
geometry_msgs::Vector3Stamped ARFacc;
ros::Publisher ARFacc_pub;

//Testing to see if this works :D
//Callback from subscriber. Performs ARF on accelerometer data.
	//Initialize variables for stuff we want to keep between loops
	double Fxcurrent = 0.0, Fycurrent = 0.0, Fzcurrent = 0.0;
	double Fxprev = 0.0, Fyprev = 0.0, Fzprev = 0.0;
	int i = 0;
void ARFilter(const sensor_msgs::Imu RAWmsg){

	//Fill Z/x/y/z/current with current accelerometer reading
	double Zxcurrent = RAWmsg.linear_acceleration.x;
	double Zycurrent = RAWmsg.linear_acceleration.y;
	double Zzcurrent = RAWmsg.linear_acceleration.z;

	//Initialize empty arrays that are 63 cells long to hold the first 63 sensor inputs
	double ValueArrayx [63] = { };
	double ValueArrayy [63] = { };
	double ValueArrayz [63] = { };
	double sumx = 0.0, sumy = 0.0, sumz = 0.0;

	if(i < 64){ //if number of recieved readings < 64 (63'rd term still goes through)
		//Assign i'th term to corresponding part in array
		ValueArrayx[i] = Zxcurrent;
		ValueArrayy[i] = Zycurrent;
		ValueArrayz[i] = Zzcurrent;

		//Do a sum over i cells of the array
		for(int j=0; j<i; j++){
			sumx += ValueArrayx[j];
			sumy += ValueArrayy[j];
			sumz += ValueArrayz[j];
		}

		//Divide by i to get average of those i terms
		Fxcurrent = sumx / (i+1) ;
		Fycurrent = sumy / (i+1) ;
		Fzcurrent = sumz / (i+1) ;

	}else{  //if it's reading accel measurement number 64+
		//Save the previous output, then do the
		//(currentMeasurment - prefFilterResult) / (N + 1) thing
		Fxprev = Fxcurrent;
		Fxcurrent = (Zxcurrent - Fxprev)/64.0;
		Fyprev = Fycurrent;
		Fycurrent = (Zycurrent - Fyprev)/64.0;
		Fzprev = Fzcurrent;
		Fzcurrent = (Zzcurrent - Fzprev)/64.0;
	}
	i++; //increment i for the next sensor input


	//Feed data into the ARFacc message for publishing
	ARFacc.header = RAWmsg.header;
	ARFacc.vector.x = Fxcurrent;
	ARFacc.vector.y = Fycurrent;
	ARFacc.vector.z = Fzcurrent;

	ARFacc_pub.publish(ARFacc);

	std::cout << Fxcurrent <<" " << Fycurrent << " " << Fzcurrent << "\n";
}



//Main ROS loop. Initializes pubs/subs, establishes loop rates.
int main(int argc, char ** argv){
	ros::init(argc, argv, "ARFaccel");
	ros::NodeHandle n;

	ARFacc_pub =
		n.advertise<geometry_msgs::Vector3Stamped>("imu/ARFaccel", 1000);

	ros::Subscriber RAWacc_sub =
		n.subscribe("/imu/data_raw", 1000, ARFilter);

	ros::Rate loop_rate(100);

	ROS_INFO("Listening to raw accelerometer data and applying ARF, then publishing on /imu/ARFacc");

	while( ros::ok() ){
		ros::spinOnce();
		loop_rate.sleep();
	}

	return 0;

}
