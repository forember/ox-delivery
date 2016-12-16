#!/bin/bash

cd "$(dirname "$0")"

./fix_links.sh

echo git submodule update --init --recursive
git submodule update --init --recursive
echo cd src/navigation_2d-git/src/nav2d_karto
cd src/navigation_2d-git/src/nav2d_karto && \
    echo git checkout -- CMakeLists.txt && git checkout -- CMakeLists.txt && \
    cd ../../../..
echo cd ../../../..
echo \<src/navigation_2d-git/src/nav2d_karto/CMakeLists.txt python -c 'import sys,re;sys.stdout.write(re.sub(r"(set\(Eigen3_INCLUDE_DIRS \$\{EIGEN3_INCLUDE_DIR\}\)\n)?catkin_package",r"set(Eigen3_INCLUDE_DIRS ${EIGEN3_INCLUDE_DIR})\ncatkin_package",sys.stdin.read(),1,re.M))' \>.sn2gsnkc.txt
<src/navigation_2d-git/src/nav2d_karto/CMakeLists.txt python -c 'import sys,re;sys.stdout.write(re.sub(r"(set\(Eigen3_INCLUDE_DIRS \$\{EIGEN3_INCLUDE_DIR\}\)\n)?catkin_package",r"set(Eigen3_INCLUDE_DIRS ${EIGEN3_INCLUDE_DIR})\ncatkin_package",sys.stdin.read(),1,re.M))' >.sn2gsnkc.txt
echo mv .sn2gsnkc.txt src/navigation_2d-git/src/nav2d_karto/CMakeLists.txt
mv .sn2gsnkc.txt src/navigation_2d-git/src/nav2d_karto/CMakeLists.txt

source /opt/ros/kinetic/setup.sh

catkin_make clean
echo rm -rf build devel install
rm -rf build devel install
