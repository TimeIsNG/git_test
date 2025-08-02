cmake_minimum_required(VERSION 2.8.3)
project(gnss_base)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)

# cmake module path
list(APPEND CMAKE_MODULE_PATH "${CMAKE_CURRENT_SOURCE_DIR}/cmake")

find_package(catkin REQUIRED COMPONENTS    
    roslaunch 
    roslint 
    roscpp 
    sensor_msgs
    std_msgs
    geometry_msgs
    gnss_sdk
    tf2 
    tf2_ros
)
    
# find_package(Boost REQUIRED COMPONENTS chrono)

###################################
## catkin specific configuration ##
###################################

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES gnss_messenger
  CATKIN_DEPENDS roscpp sensor_msgs gnss_sdk
#   DEPENDS Boost
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
    include
    ${catkin_INCLUDE_DIRS}
)

add_library(gnss_messenger STATIC src/gnss_messenger.cpp)
target_link_libraries(gnss_messenger ${catkin_LIBRARIES})
set_property(TARGET gnss_messenger PROPERTY POSITION_INDEPENDENT_CODE ON)

add_executable(gnss_base_node src/gnss_base_node.cpp)
target_link_libraries(gnss_base_node gnss_messenger ${catkin_LIBRARIES})


#############
## Install ##
#############

roslaunch_add_file_check(launch)

install(TARGETS gnss_messenger gnss_base_node
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION})

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION})

install(DIRECTORY launch urdf
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
