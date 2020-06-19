#! /bin/sh

workdir=$(cd $(dirname $0); pwd)

workdir+=/devel/setup.bash

source ${workdir}

rosrun rviz rviz &

roslaunch ldlidar ld06.launch

