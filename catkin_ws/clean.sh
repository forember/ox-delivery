#!/bin/bash

cd "$(dirname "$0")"

./fix_links.sh

echo git submodule update --init --recursive
git submodule update --init --recursive
echo cd src/navigation_2d-git/src/nav2d_karto
cd src/navigation_2d-git/src/nav2d_karto && pwd &&\
    echo git checkout -- CMakeLists.txt && git checkout -- CMakeLists.txt && \
    echo cd ../../../.. && cd ../../../..
echo \<src/navigation_2d-git/src/nav2d_karto/CMakeLists.txt python -c 'import sys,re;sys.stdout.write(re.sub(r"(set\(Eigen3_INCLUDE_DIRS \$\{EIGEN3_INCLUDE_DIR\}\)\n)?catkin_package",r"set(Eigen3_INCLUDE_DIRS ${EIGEN3_INCLUDE_DIR})\ncatkin_package",sys.stdin.read(),1,re.M))' \>.sn2gsnkc.txt
<src/navigation_2d-git/src/nav2d_karto/CMakeLists.txt python -c 'import sys,re;sys.stdout.write(re.sub(r"(set\(Eigen3_INCLUDE_DIRS \$\{EIGEN3_INCLUDE_DIR\}\)\n)?catkin_package",r"set(Eigen3_INCLUDE_DIRS ${EIGEN3_INCLUDE_DIR})\ncatkin_package",sys.stdin.read(),1,re.M))' >.sn2gsnkc.txt
echo mv .sn2gsnkc.txt src/navigation_2d-git/src/nav2d_karto/CMakeLists.txt
mv .sn2gsnkc.txt src/navigation_2d-git/src/nav2d_karto/CMakeLists.txt

if [ -f /opt/ros/indigo/setup.sh ]; then
    echo source /opt/ros/indigo/setup.sh
    source /opt/ros/indigo/setup.sh
elif [ -f /opt/ros/kinetic/setup.sh ]; then
    echo source /opt/ros/kinetic/setup.sh
    source /opt/ros/kinetic/setup.sh
fi

echo rm -f src/navigation_2d-git/src/{nav2d,nav2d_exploration,nav2d_karto,nav2d_localizer,nav2d_msgs,nav2d_navigator,nav2d_operator,nav2d_remote,nav2d_tutorials}/CATKIN_IGNORE
rm -f src/navigation_2d-git/src/{nav2d,nav2d_exploration,nav2d_karto,nav2d_localizer,nav2d_msgs,nav2d_navigator,nav2d_operator,nav2d_remote,nav2d_tutorials}/CATKIN_IGNORE
echo catkin_make clean
catkin_make clean
echo touch src/navigation_2d-git/src/{nav2d,nav2d_exploration,nav2d_karto,nav2d_localizer,nav2d_msgs,nav2d_navigator,nav2d_operator,nav2d_remote,nav2d_tutorials}/CATKIN_IGNORE
touch src/navigation_2d-git/src/{nav2d,nav2d_exploration,nav2d_karto,nav2d_localizer,nav2d_msgs,nav2d_navigator,nav2d_operator,nav2d_remote,nav2d_tutorials}/CATKIN_IGNORE
echo rm -rf build devel install
rm -rf build devel install
