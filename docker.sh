#!/bin/bash
docker run -it --name=f1tenth_gym_container -v $(pwd)/waypoints:/catkin_ws/src/f1tenth_gym_ros/waypoints --rm --net=host f1tenth_gym
