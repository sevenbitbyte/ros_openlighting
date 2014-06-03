cmake_minimum_required(VERSION 2.8.3)
project(ola_ros)

find_package(catkin REQUIRED COMPONENTS nodelet roscpp std_msgs lighting_msgs)

## Find protobuf
include(FindProtobuf)
find_package(Protobuf REQUIRED)

## Find Qt
find_package(Qt5Core REQUIRED)
find_package(Qt5Gui REQUIRED)
find_package(Qt5Sql REQUIRED)
find_package(Qt5Network REQUIRED)

set(CMAKE_BUILD_TYPE Debug)
set(ROS_BUILD_TYPE Reslease)
set(CMAKE_CXX_FLAGS_DEBUG "${CMAKE_CXX_FLAGS_DEBUG} -Wall -ggdb")



catkin_package(
  INCLUDE_DIRS include
  LIBRARIES nodelet_ola_bridge
  CATKIN_DEPENDS nodelet roscpp std_msgs lighting_msgs
)

## Setup include directories
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${Qt5Core_INCLUDE_DIRS}
  ${Qt5Gui_INCLUDE_DIRS}
  ${Qt5Network_INCLUDE_DIRS}
  ${Qt5Sql_INCLUDE_DIRS}
  ${PROTOBUF_INCLUDE_DIR}
)

add_definitions(${Qt5Widgets_DEFINITIONS})

set(OLA_BRIDGE_SOURCE
  src/utils.cpp
  src/ledrun.cpp
  src/olamanager.cpp
  src/ola_bridge.cpp
)

set(OLA_BRIDGE_HEADERS
  src/utils.h
  src/ledrun.h
  src/olamanager.h
  include/ola_bridge.h
)


## Create the nodelet tutorial library
add_library(nodelet_ola_bridge ${OLA_BRIDGE_SOURCE})

target_link_libraries(nodelet_ola_bridge
  ${catkin_LIBRARIES}
  ola
  olacommon
  ${PROTOBUF_LIBRARY}
)

qt5_use_modules(nodelet_ola_bridge Core Gui Widgets Sql Network)

if(catkin_EXPORTED_LIBRARIES)
  add_dependencies(nodelet_ola_bridge ${catkin_EXPORTED_LIBRARIES})
endif()

## Mark the nodelet library for installations
install(TARGETS nodelet_ola_bridge
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

## Mark other files for installation (e.g. launch and bag files, etc.)
install(FILES nodelet_ola_bridge.xml
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
