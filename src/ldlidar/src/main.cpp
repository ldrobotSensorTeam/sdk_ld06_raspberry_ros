#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <iostream>
#include <wiringPi.h>
#include <softPwm.h>
#include "cmd_interface_linux.h"
#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include "lipkg.h"
#include "tofbf.h"
#include "signal.h"
#include "pid.h"

int main(int argc, char **argv)
{
	ros::init(argc, argv, "product");
	ros::NodeHandle nh;                    /* create a ROS Node */
	std::string productName("LD06");
	std::string product;

	nh.getParam("product",product);

	if (product != productName)
	{
		std::cout << "Please use our product ld06 !" << std::endl;      /* print Usage */
		return -1;
	}

	
	int32_t pwm_out = 0;
	PIDObjTyp LidarMotorPID(500, 50, 200, 0, 100, 0);
	wiringPiSetup();
	//set pwm out frequency 24kHz
	pinMode(LIDAR_PWM, PWM_OUTPUT);
	pwmSetClock(8);
	pwmSetMode(PWM_MODE_MS);
	pwmSetRange(100);
	pwmWrite(LIDAR_PWM, 70);


	int32_t ver = 6;
	LiPkg * pkg = new LD06_LiPkg;

	CmdInterfaceLinux cmd_port(ver);
	std::string port_name("/dev/ttyAMA0");
	cmd_port.SetReadCallback([&pkg](const char *byte, size_t len) {
		if (pkg->Parse((uint8_t*)byte, len))
		{
			pkg->AssemblePacket();
		}
		});
	cmd_port.Open(port_name);

	sensor_msgs::LaserScan scan;
	scan.header.stamp = ros::Time::now();
	scan.header.frame_id = "lidar_frame";
	scan.range_min = 0.0;
	scan.range_max = 100.0;
	
	ros::Publisher lidar_pub = nh.advertise<sensor_msgs::LaserScan>("LD06/LDLiDAR", 1); /*create a ROS topic */

	while (ros::ok())
	{
		if (pkg->IsFrameReady())
		{
			FrameData data = pkg->GetFrameData();

			int32_t speed = pkg->GetSpeed() * 1000;
			pwm_out = LidarMotorPID.PIDRegulatorS32(10000, speed, pwm_out);
			if (pwm_out < 65)
			{
				pwm_out = 65;
			}
			std::cout << "speed:" << speed << "pwm out:" << pwm_out << std::endl;
			pwmWrite(LIDAR_PWM, pwm_out);

			scan.angle_min = ANGLE_TO_RADIAN(data.angle_min);
			scan.angle_max = ANGLE_TO_RADIAN(data.angle_max);
			scan.angle_increment = (scan.angle_max - scan.angle_min) / data.len;
			scan.ranges.resize(data.len);
			scan.intensities.resize(data.len);
			for (int i = 0; i < data.len; i++)
			{
				scan.ranges[i] = data.distance[i] / 1000.f;
				scan.intensities[i] = data.intensities[i];
			}
			scan.header.stamp = ros::Time::now();
			lidar_pub.publish(scan);
			printf("min:%f,max:%f,len:%d\n", data.angle_min, data.angle_max, data.len);
		}

	}
	return 0;
}
