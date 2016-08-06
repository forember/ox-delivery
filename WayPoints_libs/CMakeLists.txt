### === 1. CMAKE-RELATED SETUP === ###
# Project name
PROJECT ( WayPoints )

# Needed so that CMAKE won't complain about it
CMAKE_MINIMUM_REQUIRED ( VERSION 2.6 )

# Include manually-created CMake 'FIND_PACKAGE' modules
SET ( CMAKE_MODULE_PATH "${CMAKE_SOURCE_DIR}/CMAKE_MODULES/" )

# ${PROJECT_SOURCE_DIR}

### === 2. USER-CONFIGURABLE OPTIONS === ###
# Set project compilation flags
SET ( PROJECT_NAME_COVERAGE "cpp" CACHE STRING "Name (filename header) for coverage application" )

# Specify static or shared library-building option
OPTION ( BUILD_SHARED_LIBS "Build shared libraries" ON )

### === 3. NON-CONFIGURABLE OPTIONS === ###
# Specify WinXP target
IF ( MSVC )
  ADD_DEFINITIONS ( -D_WIN32_WINNT=0x0501 -D__MSVC__ )
ENDIF ()

# Set debug mode (-g) and enable all warnings (-Wall) for GNU compilers
IF ( CMAKE_COMPILER_IS_GNUCC OR CMAKE_COMPILER_IS_GNUCXX )
  SET ( CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -g -Wall -Wno-deprecated" )
  SET ( CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -g -Wall -Wno-deprecated" )
ENDIF ()


### === 4. SOURCE CODE DECLARATIONS === ###
SET ( SOURCES
  ${CMAKE_SOURCE_DIR}/src/WayPoints.cpp
  ${CMAKE_SOURCE_DIR}/src/DrawImage.cpp
)

SET ( INCLUDE_FILES
  ${CMAKE_SOURCE_DIR}/include/WayPoints.h
  ${CMAKE_SOURCE_DIR}/include/DrawImage.h
)

# Set other parameters
SET ( LIBRARY_NAME "WayPoints" )
SET ( EXECUTABLE_NAME "WayPoints" )
SET ( CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR} )

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

#5
# Check and load Leo library
FIND_PACKAGE ( Leo REQUIRED )
IF ( NOT Leo_FOUND )
  MESSAGE ( STATUS "Leo not found." )
  MESSAGE ( FATAL_ERROR "Please install libLeo." )
ENDIF ()

### === 6. CREATE LIBRARIES === ###
include_directories(${CMAKE_SOURCE_DIR}/include)
include_directories(${OpenCV_INCLUDE_DIRS})
include_directories(${Boost_INCLUDE_DIRS})
include_directories(${QT_INCLUDE_DIRS})
include_directories(${Leo_INCLUDE_DIRS})

add_library( ${LIBRARY_NAME} SHARED ${SOURCES})
#add_library( ${LIBRARY_NAME}_static STATIC ${SOURCES})

target_link_libraries(${LIBRARY_NAME} ${OpenCV_LIBS})
target_link_libraries(${LIBRARY_NAME} ${Boost_LIBRARIES})
target_link_libraries(${LIBRARY_NAME} ${QT_LIBRARIES})
target_link_libraries(${LIBRARY_NAME} ${Leo_LIBRARIES} -lpthread )

install(FILES "libWayPoints.so" DESTINATION /usr/local/lib/WayPoints)

install(DIRECTORY include/ DESTINATION /usr/local/include/WayPoints FILES_MATCHING PATTERN "*.h")

install(FILES "WayPointsConfig.cmake" DESTINATION /usr/lib/cmake/WayPoints)

install(FILES "BensonKE_WayPoints.conf" DESTINATION /etc/ld.so.conf.d)
