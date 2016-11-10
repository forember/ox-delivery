#!/bin/bash

cd "$(dirname "$0")"

source /opt/ros/kinetic/setup.sh

catkin_make clean
echo rm -rf build devel install
rm -rf build devel install
