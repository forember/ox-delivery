#!/bin/sh
enter_to_continue () {
    read -p 'Press ENTER to continue (or ^C to exit).' _
}
base="$(readlink -e "$(dirname "$0")")"
compile_lib () {
    echo "Compiling ${1}"
    cd "${base}/${1}"
    ./run.sh
    enter_to_continue
}
compile_lib Leo_libs
compile_lib WayPoints_libs
compile_lib BCD_libs
compile_lib CPP_libs
compile_lib KCPP
compile_lib UseLibs
compile_lib catkin_ws
