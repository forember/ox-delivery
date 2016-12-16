#!/bin/bash

cd "$(dirname "$0")"

source /opt/ros/kinetic/setup.sh

if [ "$1" != 'build' ]; then
    ./clean.sh 2>&1 | tee .clean.log
else
    echo 'NO CLEAN PERFORMED' | tee .clean.log
fi

catkin_make install
