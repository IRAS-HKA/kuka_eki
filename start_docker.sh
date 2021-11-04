#!/bin/sh
uid=$(eval "id -u")
gid=$(eval "id -g")
docker build --build-arg UID="$uid" --build-arg GID="$gid" --build-arg ROS_DISTRO=foxy -t iras/moveit2:foxy .
echo "Run Container"
docker run --privileged -it -e DISPLAY=$DISPLAY -v $PWD/src:/home/robot/ros_ws/src:rw --net host --rm --ipc host --gpus all iras/moveit2:foxy
