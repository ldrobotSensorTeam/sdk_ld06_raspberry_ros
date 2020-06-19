## LOGIN ROOT (passwd:ubuntu)
```sh
su  

```
## COMPILE

```sh
cd ld06_ws
catkin_make

```

## RUN

```sh
source devel/setup.bash
roslaunch ldlidar ld06.launch
rosrun rviz rviz

```
## or RUN
```sh
bash run.sh

```

## TEST

Code in ubuntun16.04 It is tested under the version of kinetic and visualized with rviz.