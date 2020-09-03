#!/bin/bash
docker run -it --name=f1tenth_gym_mult_container --rm -v $(pwd)/waypoints:/catkin_ws/src/f1tenth_gym_ros/waypoints --net=host f1tenth_gym_mult
