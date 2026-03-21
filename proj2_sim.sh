#!/bin/bash

source install/setup.bash
# ros2 daemon stop
ros2 launch ee4308_bringup proj2_sim.launch.py libgl:=False
./kill_gz.sh