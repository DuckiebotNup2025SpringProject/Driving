#!/bin/bash

# Here put your commands to launch your ROS2 nodes
ros2 launch barriers_node barriers_node.launch
ros2 launch driving_node driving_node.launch
ros2 launch apriltag_node apriltag_node.launch
ros2 launch traffic_lights_node traffic_lights_node.launch
ros2 launch map_node map_node.launch
ros2 launch master_node master_node.launch
