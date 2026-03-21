#!/bin/bash

echo "Killing all active ruby scripts, including those running Gazebo"
pkill -9 ruby # kills ruby scripts running Gz, and all other ruby scripts.
pkill -9 rviz # kills rviz