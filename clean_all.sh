#!/bin/bash
enter_to_continue () {
    read -p 'Press ENTER to continue (or ^C to exit).' _
}
base="$(readlink -e "$(dirname "$0")")"
clean_lib () {
    echo "Cleaning ${1}"
    cd "${base}/${1}"
    ./clean.sh
}
clean_lib Leo_libs
clean_lib WayPoints_libs
clean_lib BCD_libs
clean_lib CPP_libs
clean_lib KCPP
clean_lib UseLibs
clean_lib catkin_ws
cd "$base"
echo rm -f frames.gv frames.pdf
rm -f frames.gv frames.pdf
echo rm -f *.WayGraph.*.png tourLines.txt
rm -f *.WayGraph.*.png tourLines.txt
#./remove_gedit_files.sh
