cmake_minimum_required(VERSION 2.8.3)
project(ola_ros)

find_package(catkin REQUIRED COMPONENTS lighting_msgs roscpp std_msgs genmsg)

#set(CMAKE_BUILD_TYPE Debug)
set(ROS_BUILD_TYPE Debug)
#set(CMAKE_CXX_FLAGS_DEBUG "${CMAKE_CXX_FLAGS_DEBUG} -Wall -ggdb")


catkin_package(
  INCLUDE_DIRS include
 # LIBRARIES nodelet_ola_bridge
  CATKIN_DEPENDS lighting_msgs roscpp std_msgs
)


## Find protobuf
find_package(Protobuf REQUIRED)

## Find Qt
find_package(Qt5Core REQUIRED)
find_package(Qt5Gui REQUIRED)
find_package(Qt5Sql REQUIRED)
find_package(Qt5Network REQUIRED)

set(CMAKE_AUTOMOC ON)

set(CMAKE_INCLUDE_CURRENT_DIR ON)
INCLUDE_DIRECTORIES( ${CMAKE_CURRENT_BINARY_DIR} )



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

set(OLA_BRIDGE_NODELET_SOURCE
  src/utils.cpp
  src/ledrun.cpp
  src/olamanager.cpp
  src/ola_bridge_nodelet.cpp
)

set(OLA_BRIDGE_NODELET_HEADERS
  include/utils.h
  include/ledrun.h
  include/olamanager.h
  include/ola_bridge_nodelet.h
)

set(OLA_BRIDGE_NODE_SOURCE
  src/ola_bridge_node.cpp
  src/utils.cpp
  src/ledrun.cpp
  src/pixel_mapper.cpp
  src/olamanager.cpp
  src/ola_bridge.cpp
)

set(OLA_BRIDGE_NODE_HEADERS
  include/utils.h
  include/ledrun.h
  include/pixel_mapper.h
  include/olamanager.h
  include/ola_bridge.h
)

set(DMX_TEST_NODE_SOURCE
  src/dmx_test.cpp
)

## Create node binaries
add_executable(ola_bridge_node ${OLA_BRIDGE_NODE_SOURCE} ${OLA_BRIDGE_NODE_HEADERS})

target_link_libraries(ola_bridge_node
  ${catkin_LIBRARIES}
  ola
  olacommon
  ${PROTOBUF_LIBRARY}
)

add_dependencies(ola_bridge_node lighting_msgs_gencpp)

qt5_use_modules(ola_bridge_node Core Gui Widgets Sql Network)

## Create test binary
add_executable(dmx_test ${DMX_TEST_NODE_SOURCE})

target_link_libraries(dmx_test
  ${catkin_LIBRARIES}
)

add_dependencies(dmx_test lighting_msgs_gencpp)


## Mark the executable
install(TARGETS ola_bridge_node
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

