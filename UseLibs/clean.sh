#!/bin/sh

cd "$(dirname "$0")"

[ -e Makefile ] && make clean
echo rm -f *.o
rm -f *.o
echo rm -f CMakeCache.txt Makefile cmake_install.cmake
rm -f CMakeCache.txt Makefile cmake_install.cmake
echo rm -rf CMakeFiles
rm -rf CMakeFiles

echo rm -f *.pro UseLibs
rm -f *.pro UseLibs
