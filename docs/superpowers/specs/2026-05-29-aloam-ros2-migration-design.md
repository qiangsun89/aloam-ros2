# A-LOAM ROS2 Migration Design

## Overview

Migrate A-LOAM (Advanced LOAM) from ROS1 to ROS2, preserving all algorithm functionality while adapting the communication layer, build system, and node lifecycle management.

**Source:** `aloam_noted/` (ROS1 catkin package `aloam_velodyne`)
**Target:** `a_loam_ros2/` (ROS2 ament package)

## Approach: Direct 1:1 Port

Each ROS1 executable becomes a ROS2 `rclcpp::Node` subclass. Core algorithm logic (curvature computation, feature extraction, Ceres optimization, KD-tree search, cube management) is copied verbatim. Only the ROS communication layer changes.

## Directory Structure

```
a_loam_ros2/
├── CMakeLists.txt            # ament_cmake build
├── package.xml               # ROS2 package manifest
├── include/
│   └── a_loam_ros2/
│       ├── common.h          # Copied from aloam_velodyne/common.h (no ROS deps)
│       ├── tic_toc.h         # Copied from aloam_velodyne/tic_toc.h (no ROS deps)
│       └── lidarFactor.hpp   # Copied from src/lidarFactor.hpp (pure Ceres)
├── src/
│   ├── scan_registration_node.cpp
│   ├── laser_odometry_node.cpp
│   ├── laser_mapping_node.cpp
│   └── kitti_helper_node.cpp
├── launch/
│   ├── aloam_velodyne_VLP_16.launch.py
│   ├── aloam_velodyne_HDL_32.launch.py
│   ├── aloam_velodyne_HDL_64.launch.py
│   └── kitti_helper.launch.py
├── config/
│   └── aloam_params.yaml
└── rviz/
    └── aloam_velodyne.rviz
```

## Node Mapping

| ROS1 Executable | ROS2 Node Class | Node Name |
|---|---|---|
| ascanRegistration | ScanRegistrationNode | scan_registration |
| alaserOdometry | LaserOdometryNode | laser_odometry |
| alaserMapping | LaserMappingNode | laser_mapping |
| kittiHelper | KittiHelperNode | kitti_helper |

## ROS Communication Topology

```
/velodyne_points → ScanRegistrationNode
  → /velodyne_cloud_2 (full cloud)
  → /laser_cloud_sharp (corner features)
  → /laser_cloud_less_sharp (downsampled corners)
  → /laser_cloud_flat (surface features)
  → /laser_cloud_less_flat (downsampled surfaces)

                  → LaserOdometryNode
  → /laser_odom_to_init (odometry pose)
  → /laser_odom_path (odometry path)
  → /velodyne_cloud_3 (full cloud pass-through)
  → /laser_cloud_corner_last (corner features for mapping)
  → /laser_cloud_surf_last (surface features for mapping)

                  → LaserMappingNode
  → /aft_mapped_to_init (refined pose)
  → /aft_mapped_to_init_high_frec (high-freq refined pose)
  → /aft_mapped_path (refined path)
  → /velodyne_cloud_registered (registered full cloud)
  → /laser_cloud_surround (local map)
  → /laser_cloud_map (global map)
```

All topic names and frame IDs are preserved from the ROS1 version.

## API Migration Details

### Headers
- `ros/ros.h` → `rclcpp/rclcpp.hpp`
- `sensor_msgs/PointCloud2.h` → `sensor_msgs/msg/point_cloud2.hpp`
- `nav_msgs/Odometry.h` → `nav_msgs/msg/odometry.hpp`
- `nav_msgs/Path.h` → `nav_msgs/msg/path.hpp`
- `tf/transform_broadcaster.h` → `tf2_ros/transform_broadcaster.h`

### Types
- `sensor_msgs::PointCloud2` → `sensor_msgs::msg::PointCloud2`
- `nav_msgs::Odometry` → `nav_msgs::msg::Odometry`
- `ros::Time` → `rclcpp::Time`
- `ros::Publisher` → `rclcpp::Publisher<msg>::SharedPtr`
- `ros::Subscriber` → `rclcpp::Subscription<msg>::SharedPtr`

### Node Patterns
- `ros::init() + ros::NodeHandle` → `rclcpp::Node` subclass + `rclcpp::init() + rclcpp::spin()`
- `nh.param<T>(name, var, default)` → `declare_parameter<T>(name, default)` + `get_parameter(name).as<T>()`
- `pub.publish(msg)` → `pub->publish(msg)`
- `pcl::fromROSMsg` / `pcl::toROSMsg` → unchanged (PCL API compatible with both)

### Logging
- `ROS_WARN(...)` → `RCLCPP_WARN(this->get_logger(), ...)`
- `ROS_BREAK()` → `rclcpp::shutdown()` + early return

### Spin Patterns
- LaserOdometry: `ros::spinOnce()` loop → `rclcpp::spin_some()` loop (preserved)
- LaserMapping: `std::thread + ros::spin()` → `std::thread + rclcpp::spin()` (preserved)
- ScanRegistration: `ros::spin()` → `rclcpp::spin()` (preserved)

### TF
- `tf::TransformBroadcaster::sendTransform()` → `tf2_ros::TransformBroadcaster::sendTransform()`

## Build System

- catkin → ament_cmake
- `find_package(catkin ...)` → `find_package(ament_cmake)` + individual `find_package(rclcpp)`, etc.
- `catkin_package()` removed, replaced by `ament_package()`
- Target names: `ascanRegistration` → `scan_registration_node`, etc.

## Parameter Configuration

Parameters extracted from launch files into `config/aloam_params.yaml`:
- `scan_line` (int, default 16)
- `minimum_range` (double, default 0.1)
- `mapping_skip_frame` (int, default 5)
- `mapping_line_resolution` (float, default 0.4)
- `mapping_plane_resolution` (float, default 0.8)

## Files Requiring No Changes

- `lidarFactor.hpp` — pure Ceres/Eigen, zero ROS dependencies
- `common.h` — only `<cmath>` and `<pcl/point_types.h>`
- `tic_toc.h` — only `<chrono>` standard library

## Non-Functional Requirements

- Real-time: processing frequency >= 10Hz
- Compatibility: ROS2 Humble (primary), Iron
- Sensor: Velodyne VLP-16, HDL-32, HDL-64
- PCL >= 1.10
- Dependency versions: Ceres 2.0+, Eigen 3.4+, OpenCV 4.5+
