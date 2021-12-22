- [中文](#操作指南)
- [EN](#Instructions)
# 操作指南

> 此SDK仅适用于深圳乐动机器人有限公司销售的激光雷达产品，产品型号为LDROBOT LiDAR LD06. 应用环境为ROS.

## 0.获取源码

```bash
cd ~
git clone https://github.com/ldrobotSensorTeam/sdk_ld06_raspberry_ros.git
```

## 1. 设置

  - 将雷达连接到你的系统主板，设置雷达在系统中挂载的串口设备-x权限(以/dev/ttyS0为例)
	- 实际使用时，根据雷达在你的系统中的实际挂载情况来设置，可以使用`ls -l /dev`命令查看.

``` bash
sudo chmod 777 /dev/ttyS0
```

  - 修改`~/sdk_ld06_raspberry_ros/src/ldlidar/launch/ld06.launch`文件中的port_name值，以`/dev/ttyS0`为例，如下所示.

```xml
<launch>
<node name="LD06" pkg="ldlidar" type="ldlidar" output="screen" >
<param name="topic_name" value="LiDAR/LD06"/>
<param name="port_name" value ="/dev/ttyS0"/>
<param name="frame_id" value="lidar_frame"/>
</node>
</launch>
```

## 2. 编译与运行

- 编译

``` bash
cd ~/sdk_ld06_raspberry_ros/
catkin_make
```

- 运行

```bash
source devel/setup.bash
roslaunch ldlidar ld06.launch
```

## 3. 数据可视化

- 在确保编译与运行成功后，请另外打开一个终端，运行`rosrun rviz rviz`打开ROS的RVIZ工具

- 在RVIZ工具中打开`~/sdk_ld06_raspberry_ros/rviz/ldlidar.rviz`即可显示雷达数据，或者利用`~/sdk_ld06_raspberry_ros/src/ldlidar/launch/ld06.launch`中的`topic_name`与`frame_id`的值在RVIZ工具中自行配置

- 如果使用Ubuntu16.04 kinetic 版本以上的ROS,建议自行配置rviz文件  

---

# Instructions

> This SDK is only applicable to the LiDAR products sold by Shenzhen LDROBOT Co., LTD. The product models are  LDROBOT LiDAR LD06. The application environment is ROS.

## step 0: Access to the source code

```bash
cd ~
git clone https://github.com/ldrobotSensorTeam/sdk_ld06_raspberry_ros.git
```

## step 1: setup

  - Set the permission of serial port device mounted by LiDAR in the system(example:device name is /dev/ttyS0)
    - The actual use of the radar is based on the actual mounted on your system, you can use the `ls -l /dev` command to view. 

``` bash
sudo chmod 777 /dev/ttyS0
```
  -  Modify port_name value in the ~/sdk_ld06_raspberry_ros/src/ldlidar/launch/ld06.launch  files,

   > for example `/dev/ttyS0`.

``` xml
<launch>
<node name="LD06" pkg="ldlidar" type="ldlidar" output="screen" >
<param name="topic_name" value="LiDAR/LD06"/>
<param name="port_name" value ="/dev/ttyS0"/>
<param name="frame_id" value="lidar_frame"/>
</node>
</launch>
```

## step 2: build and run
-  build

``` bash
cd ~/sdk_ld06_raspberry_ros/
catkin_make
```

- run

```bash
source devel/setup.bash
roslaunch ldlidar ld06.launch
```

## step 3: Data visualization

-  After making sure that the compilation and operation are successful, please open another terminal and run `rosrun rviz rviz` to open the ROS rviz tool.

-  Open `~/sdk_ld06_raspberry_ros/rviz/ldlidar.rviz` in the RVIZ tool to display radar data. 

   Or using `~/sdk_ld06_raspberry_ros/src/ldlidar/launch/ld06.launch` , The launch file of `topic_name` and `frame_id ` values in RVIZ tool to configure. 

-  It is recommended that you configure your own rviz file if you use Ubuntu16.04 kinetic or higher 

  
