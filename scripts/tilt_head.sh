#!/bin/bash

rostopic pub -1 /tilt_controller/command std_msgs/Float64 "data: 0.5"
