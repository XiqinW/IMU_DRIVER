#include "ros/ros.h"
#include <serial/serial.h>
#include "std_msgs/String.h"
#include <sensor_msgs/Imu.h>
#include <geometry_msgs/Vector3.h>
#include <geometry_msgs/Quaternion.h>
#include <tf/transform_datatypes.h>
#include <math.h>
#include "imu_ah100b.h"

int main(int argc, char **argv)
{
	ros::init(argc, argv, "imu_ah100b");
	ros::NodeHandle n("~");
	//Serial Driver
	std::string serial_port;
	int framerate;
	n.param("serial_port", serial_port, std::string("/dev/ttyUSB0"));
	n.param("framerate", framerate, 100);
	ROS_INFO("serial_port: %s", serial_port.c_str());
	serial::Timeout to = serial::Timeout::simpleTimeout(10);
	serial::Serial i_sp(serial_port, 115200, to/*, //Default Value
						serial::eightbits,	
						serial::parity_none,
						serial::stopbits_one,
						serial::flowcontrol_none*/
						);

	ros::Publisher imu_msg_pub = n.advertise<sensor_msgs::Imu>("imu_raw", 20);
	ros::Rate loop_rate(framerate);
	sDataLink imu_DataPack;
	while (ros::ok())
	{
		if (!i_sp.isOpen()) {
			try {
				i_sp.open();
				ROS_INFO("Open Serial port: %s",serial_port.c_str());
			} catch (serial::IOException e) {
				ROS_ERROR("Unable to open port: %s",serial_port.c_str());
				return -1;
			} 
		}

		if (i_sp.waitReadable()) {
			i_sp.read((uint8 *)(&imu_DataPack), 2);
			if (('T'==imu_DataPack.Header1) && ('M'==imu_DataPack.Header2)) { //Find header
				i_sp.read((uint8 *)(&imu_DataPack.Class), sizeof(imu_DataPack)-2);
			} else 
				continue;			
		} else 
			continue;
		//Check data
		uint16 tmpCheckSum = CRC16((uint8*) &imu_DataPack.Class, imu_DataPack.Length+4);
		if (tmpCheckSum != imu_DataPack.CheckSum) 
			continue;
		sensor_msgs::Imu imu_msg;
		imu_msg.header.stamp = ros::Time::now();
		imu_msg.header.frame_id = "imu_frame";
		tf::Quaternion tmpQuatOrien = tf::createQuaternionFromRPY(
														imu_DataPack.Payload.RPY[0],
														imu_DataPack.Payload.RPY[1],
														imu_DataPack.Payload.RPY[2]);
		imu_msg.orientation.x = tmpQuatOrien.x();
		imu_msg.orientation.y = tmpQuatOrien.y();
		imu_msg.orientation.z = tmpQuatOrien.z();
		imu_msg.orientation.w = tmpQuatOrien.w();
		#define ANG_2_RAD M_PI/180
		imu_msg.angular_velocity.x = imu_DataPack.Payload.Gyro[0] * ANG_2_RAD;
		imu_msg.angular_velocity.y = imu_DataPack.Payload.Gyro[1] * ANG_2_RAD;
		imu_msg.angular_velocity.z = imu_DataPack.Payload.Gyro[2] * ANG_2_RAD;
		#define GRAV_NJ	9.7949
		imu_msg.linear_acceleration.x = imu_DataPack.Payload.Acc[0] * GRAV_NJ;
		imu_msg.linear_acceleration.y = imu_DataPack.Payload.Acc[1] * GRAV_NJ;
		imu_msg.linear_acceleration.z = imu_DataPack.Payload.Acc[2] * GRAV_NJ;
		//Pub msg
		imu_msg_pub.publish(imu_msg);
		ros::spinOnce();
		loop_rate.sleep();
	}
	i_sp.close();
	return 0;
}
