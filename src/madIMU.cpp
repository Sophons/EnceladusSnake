#include "ros/ros.h"
#include "geometry_msgs/Quaternion.h"
#include "geometry_msgs/Vector3.h"
#include "sensor_msgs/Imu.h"
#include "sensor_msgs/MagneticField.h"

#include <rc/mpu.h>
#include <rc/math/vector.h>
#include <sstream>
#include <cstring>
#include <chrono>
#include <cmath>

using namespace std::chrono;

#define I2C_BUS 2
#define MICRO 10000000
#define PI 3.1415

/**
 * Sends IMU data over the ROS system for use with the Magdwick Filter
 */
int main(int argc, char ** argv) {
    //Initialize magXYZ stuff for comparisons
    double previousMagx = 0;
    double currentMagx;
    double significantx = 0;

    double previousMagy = 0;
    double currentMagy;
    double significanty = 0;

    double previousMagz = 0;
    double currentMagz;
    double significantz = 0;


    // Vectors to store x,y,z components
    rc_vector_t accel_v = RC_VECTOR_INITIALIZER;
    rc_vector_t gyro_v = RC_VECTOR_INITIALIZER;
    rc_vector_t mag_v = RC_VECTOR_INITIALIZER;

    // Initialize ROS
    ros::init(argc, argv, "madIMU");

    // Fully initialize this node
    ros::NodeHandle n;

    // Get parameters
    std::string key1;
    double linAcc_covar = 0.0;
    if (ros::param::search("linAcc_covar", key1))
    {
      ros::param::get(key1, linAcc_covar);
    }

    std::string key2;
    double angVel_covar = 0.0;
    if (ros::param::search("angVel_covar", key2))
    {
      ros::param::get(key2, angVel_covar);
    }

    std::string key3;
    double magFld_covar = 0.0;
    if (ros::param::search("magFld_covar", key3))
    {
      ros::param::get(key3, magFld_covar);
    }


    // Tell ROS you want to publish on topics "imu/data_raw" and "imu/mag"
    ros::Publisher imu_pub =
        n.advertise<sensor_msgs::Imu>("imu/data_raw", 1000);

    ros::Publisher mag_pub =
        n.advertise<sensor_msgs::MagneticField>("imu/mag", 1000);

    // Publish message to chatter 10 times a second
    ros::Rate loop_rate(10);

    while(ros::ok()) {
        // Message objects to fill with data and publish
        sensor_msgs::Imu imu;
	sensor_msgs::MagneticField mag;

        // Struct to hold new data
        rc_mpu_data_t data;

        // Use defaults, enable magnetometer
        rc_mpu_config_t conf = rc_mpu_default_config();
        conf.i2c_bus = I2C_BUS;
        conf.enable_magnetometer = 1;
        conf.show_warnings = 0;

        // Initialize MPU
        if(rc_mpu_initialize(&data, conf)) {
            fprintf(stderr, "rc_mpu_initialize_dmp failed\n");
            return -1;
        }

        //rc_mpu_initialize_dmp reads this data for you
        // Read sensor data for accelerometer, gyroscope, and magnetometer
        rc_mpu_read_accel(&data);
        rc_mpu_read_gyro(&data);
        rc_mpu_read_mag(&data);

        // Convert gyro data to rad/s
        for(int i = 0; i < 3; i++) {
            data.gyro[i] = data.gyro[i] * DEG_TO_RAD;
        }

	//Fill msg headers
	imu.header.stamp = ros::Time::now();
	mag.header.stamp = ros::Time::now();
	imu.header.frame_id = "imu2orientation";
	mag.header.frame_id = "imu2orientation";

	//Fill imu msg with LinAccel and AngVel and their covariance matrices
	imu.linear_acceleration.x = data.accel[0];
        imu.linear_acceleration.y = data.accel[1];
        imu.linear_acceleration.z = data.accel[2];
        imu.linear_acceleration_covariance[0] = linAcc_covar;
        imu.linear_acceleration_covariance[4] = linAcc_covar;
        imu.linear_acceleration_covariance[8] = linAcc_covar;

	imu.angular_velocity.x = data.gyro[0];
        imu.angular_velocity.y = data.gyro[1];
        imu.angular_velocity.z = data.gyro[2];
	imu.angular_velocity_covariance[0] = angVel_covar;
        imu.angular_velocity_covariance[4] = angVel_covar;
        imu.angular_velocity_covariance[8] = angVel_covar;


	//Fill mag msg with magnetic field and its covariance matrix
	currentMagx=pow(10,-6)*data.mag[0];
	currentMagy=pow(10,-6)*data.mag[1];
	currentMagz=pow(10,-6)*data.mag[2];

	if(abs(currentMagx - previousMagx) > abs(.3*previousMagx)){
	  mag.magnetic_field.x = currentMagx;
	  significantx = currentMagx;
	}else{
	  mag.magnetic_field.x = significantx;
	}

        if(abs(currentMagy - previousMagy) > abs(.3*previousMagy)){
          mag.magnetic_field.y = currentMagy;
	  significanty = currentMagy;
        }else{
	  mag.magnetic_field.y = significanty;
	}

        if(abs(currentMagz - previousMagz) > abs(.3*previousMagz)){
          mag.magnetic_field.z = currentMagz;
	  significantz = currentMagz;
        }else{
	  mag.magnetic_field.z = significantz;
	}


	previousMagx=currentMagx;
	previousMagy=currentMagy;
	previousMagz=currentMagz;

/*
	mag.magnetic_field.x = data.mag[0];
	mag.magnetic_field.y = data.mag[1];
	mag.magnetic_field.z = data.mag[2];
*/
	mag.magnetic_field_covariance[0] = magFld_covar;
	mag.magnetic_field_covariance[4] = magFld_covar;
	mag.magnetic_field_covariance[8] = magFld_covar;


        // ROS version of printf/cout
        ROS_INFO("publishing to imu/data_raw and imu/mag");


        // Broadcast the message to anyone who is connected
        imu_pub.publish(imu);
	mag_pub.publish(mag);

        ros::spinOnce();

        loop_rate.sleep();
    }

    rc_mpu_power_off();

    return 0;
}
