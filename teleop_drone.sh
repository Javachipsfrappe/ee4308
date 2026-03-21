#!/bin/bash
source install/setup.bash

ros2 run teleop_twist_keyboard teleop_twist_keyboard --ros-args -r __ns:=/drone