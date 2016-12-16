#!/bin/bash

cd "$(dirname "$0")"

from_leo_to () {
    diff Leo_libs/run.sh "$1/"
    diff Leo_libs/clean.sh "$1/"
    cp Leo_libs/{run,clean}.sh "$1/"
}

from_leo_to WayPoints_libs
from_leo_to BCD_libs
from_leo_to CPP_libs
from_leo_to KCPP
