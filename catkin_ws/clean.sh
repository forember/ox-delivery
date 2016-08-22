#!/bin/sh

cd "$(dirname "$0")"

source /opt/ros/indigo/setup.sh

catkin_make clean
echo rm -rf build devel install
rm -rf build devel install
