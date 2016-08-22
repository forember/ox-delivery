cmake_minimum_required(VERSION 2.8.3)
project(efficient_coverage)
find_package(PkgConfig REQUIRED)
pkg_check_modules(BULLET bullet)
include_directories(${BULLET_INCLUDE_DIRS})

SET ( PROJECT_NAME "efficient_coverage" )
SET ( EXECUTABLE_NAME "efficient_coverage" )

find_package(catkin REQUIRED COMPONENTS 
  message_generation 
  roscpp 
  rospy 
  std_msgs 
  tf 
  geometry_msgs 
  nav_msgs 
  visualization_msgs 
  message_filters
  sensor_msgs
)


catkin_package(CATKIN_DEPENDS 
  message_runtime 
  roscpp 
  rospy
  std_msgs 
  tf 
  geometry_msgs 
  nav_msgs 
  visualization_msgs
  sensor_msgs
) 

### === 5. CHECK FOR LIBRARIES === ###
# Check and load Boost

#1
SET ( Boost_USE_MULTITHREADED OFF )
FIND_PACKAGE( Boost COMPONENTS date_time graph filesystem system thread REQUIRED )
IF ( NOT Boost_FOUND )
  MESSAGE ( STATUS "Boost library not found." )
  MESSAGE ( FATAL_ERROR "Please install libboost and the following modules: date_time, graph, filesystem, system, thread." )
ENDIF ()

#2
# Check and load OpenCV
FIND_PACKAGE ( OpenCV 2.2 REQUIRED )
IF ( NOT OpenCV_FOUND )
  MESSAGE ( STATUS "OpenCV library not found." )
  MESSAGE ( FATAL_ERROR "Please install opencv v2.2+ (strongly recommended from source)." )
ENDIF ()

#3
# Check and load Qt
SET ( QT_MAX_VERSION "4.9.9" )
FIND_PACKAGE ( Qt4 REQUIRED )
IF ( NOT QT_FOUND )
  MESSAGE ( STATUS "Qt4 library not found." )
  MESSAGE ( FATAL_ERROR "Please install libqt4." )
ENDIF ()
SET ( QT_USE_QTOPENGL TRUE )
INCLUDE( ${QT_USE_FILE} )

#4
# Check and load Leo library
FIND_PACKAGE ( Leo REQUIRED )
IF ( NOT Leo_FOUND )
  MESSAGE ( STATUS "Leo not found." )
  MESSAGE ( FATAL_ERROR "Please install libLeo." )
ENDIF ()

#4
# Check and load ChinesePostman library
FIND_PACKAGE ( Boustrophedon REQUIRED )
IF ( NOT Boustrophedon_FOUND )
  MESSAGE ( STATUS "Boustrophedon not found." )
  MESSAGE ( FATAL_ERROR "Please install libBoustrophedon." )
ENDIF ()

#5
# Check and load ChinesePostman library
FIND_PACKAGE ( ChinesePostman REQUIRED )
IF ( NOT ChinesePostman_FOUND )
  MESSAGE ( STATUS "ChinesePostman not found." )
  MESSAGE ( FATAL_ERROR "Please install libChinesePostman." )
ENDIF ()

#5
# Check and load ChinesePostman library
FIND_PACKAGE ( WayPoints REQUIRED )
IF ( NOT WayPoints_FOUND )
  MESSAGE ( STATUS "WayPoints not found." )
  MESSAGE ( FATAL_ERROR "Please install libWayPoints." )
ENDIF ()

#6
# Check and load KCPP library
FIND_PACKAGE ( KCPP REQUIRED )
IF ( NOT KCPP_FOUND )
  MESSAGE ( STATUS "KCPP not found." )
  MESSAGE ( FATAL_ERROR "Please install libKCPP." )
ENDIF ()

include_directories(${catkin_INCLUDE_DIRS})
include_directories(include)

include_directories(${OpenCV_INCLUDE_DIRS})
include_directories(${Boost_INCLUDE_DIRS})
include_directories(${QT_INCLUDE_DIRS})
include_directories(${Leo_INCLUDE_DIRS})
include_directories(${ChinesePostman_INCLUDE_DIRS})
include_directories(${Boustrophedon_INCLUDE_DIRS})
include_directories(${WayPoints_INCLUDE_DIRS})
include_directories(${KCPP_INCLUDE_DIRS})


add_executable(efficient_coverage  src/efficient_coverage.cpp)
target_link_libraries(efficient_coverage ${catkin_LIBRARIES})
add_dependencies(efficient_coverage ${PROJECT_NAME}_generate_messages_cpp)


# set the path to the library folder for the custom ChinesePostman lib
target_link_libraries(${PROJECT_NAME} ${OpenCV_LIBS})
target_link_libraries(${PROJECT_NAME} ${Boost_LIBRARIES})
target_link_libraries(${PROJECT_NAME} ${QT_LIBRARIES})
target_link_libraries(${PROJECT_NAME} ${Leo_LIBRARIES} -lpthread )
target_link_libraries(${PROJECT_NAME} ${ChinesePostman_LIBRARIES} -lpthread )
target_link_libraries(${PROJECT_NAME} ${Boustrophedon_LIBRARIES} -lpthread )
target_link_libraries(${PROJECT_NAME} ${WayPoints_LIBRARIES} -lpthread )
target_link_libraries(${PROJECT_NAME} ${KCPP_LIBRARIES} -lpthread )