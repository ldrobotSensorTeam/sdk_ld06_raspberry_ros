/**
 * @file main.cpp
 * @author LDRobot (marketing1@ldrobot.com)
 * @brief  main process App
 *         This code is only applicable to LDROBOT LiDAR LD06 products 
 * sold by Shenzhen LDROBOT Co., LTD    
 * @version 0.1
 * @date 2021-10-28
 *
 * @copyright Copyright (c) 2021  SHENZHEN LDROBOT CO., LTD. All rights
 * reserved.
 * Licensed under the MIT License (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License in the file LICENSE
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#include <ros/ros.h>
#include <stdio.h>
#include <string.h>

#include <iostream>
#include <string>

#include "cmd_interface_linux.h"
#include "lipkg.h"

int main(int argc, char **argv) {
  ros::init(argc, argv, "ldLiDAR_LD06");
  ros::NodeHandle nh;  // create a ROS Node
  ros::NodeHandle n("~");
	std::string topic_name;
	std::string port_name;
	std::string frame_id;

	n.getParam("topic_name", topic_name);
	n.getParam("port_name", port_name);
	n.getParam("frame_id", frame_id);

  std::cout << "[ldrobot] SDK Pack Version is " << "v2.0.0" << std::endl;
	std::cout << "[ldrobot] "
            << "<topic_name>: " << topic_name
            << " <port_name>: "  << port_name
            << " <frame_id>: "   << frame_id << std::endl;
  
   ros::Publisher lidar_pub = nh.advertise<sensor_msgs::LaserScan>(topic_name, 1);  // create a ROS topic

  if (port_name.empty()) {
    std::cout << "[ldrobot] Can't find LiDAR LD06" << std::endl;
    exit(EXIT_FAILURE);
  }else {
    std::cout << "[ldrobot] FOUND LiDAR_LD06" << std::endl;
  }

  CmdInterfaceLinux cmd_port;
  LiPkg *lidar = new LiPkg(frame_id);
  cmd_port.SetReadCallback([&lidar](const char *byte, size_t len) {
    if (lidar->Parse((uint8_t *)byte, len)) {
      lidar->AssemblePacket();
    }
  });

  if (cmd_port.Open(port_name)) {
    std::cout << "[ldrobot] LiDAR_LD06 started successfully " << std::endl;
  }else {
    std::cout << "[ldrobot] LiDAR_LD06 started failed " << std::endl;
  }
  
  ros::Rate r(10); //10hz

  while (ros::ok()) {
    if (lidar->IsFrameReady()) {
      lidar_pub.publish(lidar->GetLaserScan());  // Fixed Frame:  lidar_frame
      lidar->ResetFrameReady();
#if 0 
			sensor_msgs::LaserScan data = lidar->GetLaserScan();
			unsigned int lens = (data.angle_max - data.angle_min) / data.angle_increment;  
			std::cout << "[ldrobot] current_speed: " << lidar->GetSpeed() << " " 
			          << "len: " << lens << " "
					  << "angle_min: " << RADIAN_TO_ANGLED(data.angle_min) << " "
					  << "angle_max: " << RADIAN_TO_ANGLED(data.angle_max) << std::endl; 
			std::cout << "----------------------------" << std::endl;
			for (int i = 0; i < lens; i++)
			{
				std::cout << "[ldrobot] range: " <<  data.ranges[i] << " " 
						  << "intensites: " <<  data.intensities[i] << std::endl;
			}
			std::cout << "----------------------------" << std::endl;
#endif
    }
    r.sleep();
  }
  return 0;
}

/********************* (C) COPYRIGHT SHENZHEN LDROBOT CO., LTD *******END OF
 * FILE ********/