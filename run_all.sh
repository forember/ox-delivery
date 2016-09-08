#!/bin/bash
enter_to_continue () {
    read -p 'Press ENTER to continue (or ^C to exit).' _
}
base="$(readlink -e "$(dirname "$0")")"
compile_lib () {
    echo "Compiling ${1}"
    cd "${base}/${1}"
    ./run.sh
    s="$?"
    if [ "$s" -ne 0 ]; then
        echo "$0: ERROR"
        exit $s
    fi
}
compile_lib Leo_libs
compile_lib WayPoints_libs
compile_lib BCD_libs
compile_lib CPP_libs
compile_lib KCPP
compile_lib UseLibs
compile_lib catkin_ws
