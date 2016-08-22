#!/bin/bash

cd "$(dirname "$0")"

source /opt/ros/indigo/setup.sh

./clean.sh

catkin_make install
