#!/bin/sh

rosbag record /jaco_trajectory_controller/trajectory/goal /vector/right_gripper/cmd /detected_intent /kinect/qhd/image_color/compressed -o $1

