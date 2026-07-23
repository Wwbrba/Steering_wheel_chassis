cmake_minimum_required(VERSION 3.10)
project(robosense_sim_ros2)

## Use C++14
set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

## Export compile commands for clangd
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

#######################
## Find dependencies ##
#######################
find_package(ament_cmake_auto REQUIRED)
find_package(rclcpp REQUIRED)
find_package(gazebo_dev REQUIRED)
find_package(gazebo_ros REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
ament_auto_find_build_dependencies()

###########
## Build ##
###########

include_directories(
    include
    ${GAZEBO_INCLUDE_DIRS}
)

link_directories(${GAZEBO_LIBRARY_DIRS})

ament_auto_add_library(robosense_sim SHARED
    src/robosense_points_plugin.cpp
    src/robosense_ode_multiray_shape.cpp
)

target_link_libraries(robosense_sim
    RayPlugin
    GpuRayPlugin
    libprotobuf.so.9
)

#############
## Testing ##
#############

#############
## Install ##
#############

ament_auto_package(
  INSTALL_TO_SHARE
  scan_mode
  urdf
)
