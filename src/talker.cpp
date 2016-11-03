#include "ros/ros.h"
#include "geometry_msgs/Quaternion.h"
#include "geometry_msgs/Vector3.h"
#include "sensor_msgs/Imu.h"

#include <rc/mpu.h>
#include <rc/math/vector.h>
#include <sstream>
#include <cstring>
#include <chrono>
using namespace std::chrono;

#define I2C_BUS 2
#define MICRO 10000000
#define PI 3.1415

/**
 * Sends IMU data over the ROS system
 */
int main(int argc, char ** argv) {
    // Vectors to store x,y,z components
    rc_vector_t accel_v = RC_VECTOR_INITIALIZER;
    rc_vector_t gyro_v = RC_VECTOR_INITIALIZER;
    rc_vector_t mag_v = RC_VECTOR_INITIALIZER;

    // Initialize ROS
    ros::init(argc, argv, "talker");

    // Fully initialize this node
    ros::NodeHandle n;

    // Tell ROS you want to publish on topic "chatter"
    ros::Publisher imu_pub =
        n.advertise<sensor_msgs::Imu>("imu", 1000);

    // Publish message to chatter 10 times a second
    ros::Rate loop_rate(10);

    // Create starting point for clock
    //auto start = high_resolution_clock::now();

    while(ros::ok()) {
        // Message objects to fill with data and publish
        sensor_msgs::Imu imu;

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

        /*
        quat.x = data.fused_quat[QUAT_X];
        quat.y = data.fused_quat[QUAT_Y];
        quat.z = data.fused_quat[QUAT_Z];
        quat.w = data.fused_quat[QUAT_W];
        */

        // Get amount of time passed
        //auto finish = high_resolution_clock::now();
        //auto duration = duration_cast<milliseconds>(finish-start);
        imu.header.stamp = ros::Time::now();

	//Calculate Orientation: First chunk reads data, Second chunk calculates roll pitch yaw, Third chunk quaternionizes
	//Chunk1
	double accelX = data.accel[0];
	double accelY = data.accel[1];
	double accelZ = data.accel[2];
	double magReadX = data.mag[0];
	double magReadY = data.mag[1];
	double magReadZ = data.mag[2];

	//Chunk2
	double roll = 180 * atan2(accelX, sqrt(accelY*accelY + accelZ*accelZ))/PI;
	double pitch = 180 * atan2(accelY, sqrt(accelX*accelX + accelZ*accelZ))/PI;

	double mag_x = magReadX*cos(pitch) + magReadY*sin(roll)*sin(pitch) + magReadZ*cos(roll)*sin(pitch);
        double mag_y = magReadY * cos(roll) - magReadZ*sin(roll);
	double yaw = 180 * atan2(-mag_y,mag_x)/M_PI;

	//Chunk3
	double cy = cos(yaw * 0.5);
	double sy = sin(yaw * 0.5);
	double cp = cos(pitch * 0.5);
	double sp = sin(pitch * 0.5);
	double cr = cos(roll * 0.5);
	double sr = sin(roll * 0.5);

	imu.orientation.w = cy * cp * cr + sy * sp * sr;
	imu.orientation.x = cy * cp * sr - sy * sp * cr;
	imu.orientation.y = sy * cp * sr + cy * sp * cr;
	imu.orientation.z = sy * cp * cr - cy * sp * sr;

        imu.orientation_covariance[0] = 1;
	imu.orientation_covariance[4] = 1;
	imu.orientation_covariance[8] = 1;


	//Fills in AngVel's and AngVelCovariance
        imu.angular_velocity.x = data.gyro[0];
        imu.angular_velocity.y = data.gyro[1];
        imu.angular_velocity.z = data.gyro[2];
        //memset(imu.angular_velocity_covariance, 0, 9*8); <- failed attempt at making covariance
        imu.angular_velocity_covariance[0] = 1;
        imu.angular_velocity_covariance[4] = 1;
        imu.angular_velocity_covariance[8] = 1;


	//Fills in LinAccels and LinAccelCovariance
        imu.linear_acceleration.x = data.accel[0];
        imu.linear_acceleration.y = data.accel[1];
        imu.linear_acceleration.z = data.accel[2];
        imu.linear_acceleration_covariance[0] = 1;
        imu.linear_acceleration_covariance[4] = 1;
        imu.linear_acceleration_covariance[8] = 1;

        // Store data from arrays into vectors
        rc_vector_from_array(&accel_v, data.accel, 3);
        rc_vector_from_array(&gyro_v, data.gyro, 2);
        rc_vector_from_array(&mag_v, data.mag, 3);


        /*std::stringstream ss;
        ss << "Time: " << double(duration.count())/1000 << " sec | ";
        ss << "Accel: " << rc_vector_norm(accel_v, 2) << " m/s^2 | ";
        ss << "Gyro: " << rc_vector_norm(gyro_v, 2) << " rad/s | ";
        ss << "Mag: " << rc_vector_norm(mag_v, 2) << " uT";
        imu.data = ss.str();*/

        // ROS version of printf/cout
        ROS_INFO("publishing\n");

        // Broadcast the message to anyone who is connected
        imu_pub.publish(imu);

        ros::spinOnce();

        loop_rate.sleep();
    }

    rc_mpu_power_off();

    return 0;
}
