#!/bin/bash

cd "$(dirname "$0")"

source /opt/ros/kinetic/setup.sh

./clean.sh

catkin_make install
