cmake_minimum_required(VERSION 3.8)
project(a_loam_ros2)

if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE "Release")
endif()
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_FLAGS_RELEASE "-O3 -Wall -g")

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2 REQUIRED)
find_package(pcl_conversions REQUIRED)

find_package(Eigen3 REQUIRED)
find_package(PCL REQUIRED)
find_package(OpenCV REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(rosbag2_cpp REQUIRED)
find_package(rosbag2_storage REQUIRED)
find_package(Ceres REQUIRED)

include_directories(
  include
  ${rclcpp_INCLUDE_DIRS}
  ${sensor_msgs_INCLUDE_DIRS}
  ${nav_msgs_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
  ${cv_bridge_INCLUDE_DIRS}
  ${CERES_INCLUDE_DIRS}
)

# --- Executables ---

add_executable(scan_registration_node src/scan_registration_node.cpp)
target_link_libraries(scan_registration_node
  ${rclcpp_LIBRARIES}
  ${sensor_msgs_LIBRARIES}
  ${PCL_LIBRARIES}
)
ament_target_dependencies(scan_registration_node
  rclcpp sensor_msgs nav_msgs geometry_msgs std_msgs
  tf2_ros tf2 pcl_conversions
)

add_executable(laser_odometry_node src/laser_odometry_node.cpp)
target_link_libraries(laser_odometry_node
  ${rclcpp_LIBRARIES}
  ${sensor_msgs_LIBRARIES}
  ${PCL_LIBRARIES}
  ${CERES_LIBRARIES}
)
ament_target_dependencies(laser_odometry_node
  rclcpp sensor_msgs nav_msgs geometry_msgs std_msgs
  tf2_ros tf2 pcl_conversions
)

add_executable(laser_mapping_node src/laser_mapping_node.cpp)
target_link_libraries(laser_mapping_node
  ${rclcpp_LIBRARIES}
  ${sensor_msgs_LIBRARIES}
  ${PCL_LIBRARIES}
  ${CERES_LIBRARIES}
)
ament_target_dependencies(laser_mapping_node
  rclcpp sensor_msgs nav_msgs geometry_msgs std_msgs
  tf2_ros tf2 pcl_conversions
)

add_executable(kitti_helper_node src/kitti_helper_node.cpp)
target_link_libraries(kitti_helper_node
  ${rclcpp_LIBRARIES}
  ${sensor_msgs_LIBRARIES}
  ${PCL_LIBRARIES}
  ${OpenCV_LIBS}
)
ament_target_dependencies(kitti_helper_node
  rclcpp sensor_msgs nav_msgs geometry_msgs std_msgs
  tf2_ros tf2 pcl_conversions cv_bridge rosbag2_cpp rosbag2_storage
)

install(DIRECTORY launch config rviz
  DESTINATION share/${PROJECT_NAME}
)

install(TARGETS
  scan_registration_node
  laser_odometry_node
  laser_mapping_node
  kitti_helper_node
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
