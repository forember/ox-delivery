#!/bin/bash

cd "$(dirname "$0")"

if [ -f /opt/ros/indigo/setup.sh ]; then
    source /opt/ros/indigo/setup.sh
elif [ -f /opt/ros/kinetic/setup.sh ]; then
    source /opt/ros/kinetic/setup.sh
fi
[ -f 'devel/setup.sh' ] && source devel/setup.sh

rospack_noncwd () {
    rospack find "$1" && ! ( rospack find "$1" | egrep "^$(readlink -e . | sed 's/\D/\\&/g')" )
}

rn4 () {
    rospack_noncwd "nav2d_$1"
}

if rn4 exploration && rn4 karto && rn4 localizer && rn4 msgs && rn4 navigator \
        && rn4 operator && rn4 remote && rn4 tutorials ; then
    whitelist='efficient_coverage_py'
else
    echo 'WARNING: COULD NOT FIND INSTALLED NAV2D.'
    echo '      ATTEMPTING TO BUILD FROM SOURCE.'
    whitelist=''
fi

if [ "$1" = 'init' -o "$1" = 'initins' ]; then
    ./clean.sh 2>&1 | tee .clean.log
else
    echo 'NO CLEAN PERFORMED' | tee .clean.log
fi

if [ "$whitelist" = '' ]; then
    echo rm -f src/navigation_2d-git/src/{nav2d,nav2d_exploration,nav2d_karto,nav2d_localizer,nav2d_msgs,nav2d_navigator,nav2d_operator,nav2d_remote,nav2d_tutorials}/CATKIN_IGNORE
    rm -f src/navigation_2d-git/src/{nav2d,nav2d_exploration,nav2d_karto,nav2d_localizer,nav2d_msgs,nav2d_navigator,nav2d_operator,nav2d_remote,nav2d_tutorials}/CATKIN_IGNORE
fi

if [ "$1" = 'install' -o "$1" = 'initins' ]; then
    catkin_make -DCATKIN_WHITELIST_PACKAGES="$whitelist" install
else
    catkin_make -DCATKIN_WHITELIST_PACKAGES="$whitelist"
fi

if [ "$whitelist" != '' ]; then
    echo touch src/navigation_2d-git/src/{nav2d,nav2d_exploration,nav2d_karto,nav2d_localizer,nav2d_msgs,nav2d_navigator,nav2d_operator,nav2d_remote,nav2d_tutorials}/CATKIN_IGNORE
    touch src/navigation_2d-git/src/{nav2d,nav2d_exploration,nav2d_karto,nav2d_localizer,nav2d_msgs,nav2d_navigator,nav2d_operator,nav2d_remote,nav2d_tutorials}/CATKIN_IGNORE
fi
