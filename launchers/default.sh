#!/bin/bash

ros2 launch wheels_node wheels_node.launch &
ros2 launch wheels_encoder_reader_node wheels_encoder_reader_node.launch &