# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## Project Overview

A-LOAM ROS2 — a direct 1:1 port of the A-LOAM (Advanced Lidar Odometry and Mapping) algorithm from ROS1 to ROS2. This is a 3D lidar SLAM system described in:
J. Zhang and S. Singh. *LOAM: Lidar Odometry and Mapping in Real-time.* RSS, 2014.

## Build & Run

```bash
# Build (from workspace root)
colcon build --packages-select a_loam_ros2 --symlink-install
source install/setup.bash

# Run with live sensor
ros2 launch a_loam_ros2 aloam_velodyne_VLP_16.launch.py

# Run with KITTI dataset playback
ros2 launch a_loam_ros2 kitti_helper.launch.py \
  dataset_folder:=/path/to/kitti \
  sequence_number:=00
```

## Architecture

Three-node pipeline, all topic names and frame IDs preserved from the ROS1 version:

```
/velodyne_points
    → scan_registration (ScanRegistrationNode)
          → /laser_cloud_sharp, /laser_cloud_flat (features)
          → /laser_cloud_less_sharp, /laser_cloud_less_flat (downsampled)
    → laser_odometry (LaserOdometryNode) [scan-to-scan matching via Ceres]
          → /laser_odom_to_init, /laser_cloud_corner_last, /laser_cloud_surf_last
    → laser_mapping (LaserMappingNode) [scan-to-submap matching via Ceres]
          → /aft_mapped_to_init, /laser_cloud_surround, /laser_cloud_map
```

A fourth node, `kitti_helper`, replays KITTI `.bin` point clouds onto `/velodyne_points` for offline testing.

## Key source files

| File | Purpose |
|---|---|
| `include/a_loam_ros2/common.h` | Point type alias (`PointType` = `pcl::PointXYZI`), `rad2deg`/`deg2rad` helpers. Zero ROS deps. |
| `include/a_loam_ros2/lidarFactor.hpp` | Ceres cost functions: `LidarEdgeFactor`, `LidarPlaneFactor`, `LidarPlaneNormFactor`, `LidarDistanceFactor`. Pure Ceres/Eigen, zero ROS deps. |
| `include/a_loam_ros2/tic_toc.h` | Simple scoped timer using `std::chrono`. |
| `src/scan_registration_node.cpp` | Feature extraction: curvature computation, corner/planar classification per scan line. Single subscription callback, `rclcpp::spin()`. |
| `src/laser_odometry_node.cpp` | Frame-to-frame pose estimation via Ceres optimization. Uses `rclcpp::spin_some()` loop pattern (non-blocking). |
| `src/laser_mapping_node.cpp` | Backend: scan-to-submap registration, cube-based local map management, global map accumulation. Runs `rclcpp::spin()` on a dedicated `std::thread`. |
| `src/kitti_helper_node.cpp` | KITTI raw data player publishing `/velodyne_points`, stereo images, and ground-truth odometry. |
| `config/aloam_params.yaml` | Shared parameters: `scan_line`, `minimum_range`, `mapping_skip_frame`, `mapping_line_resolution`, `mapping_plane_resolution`. |

## Parameters

All runtime parameters declared in `config/aloam_params.yaml`:

- `scan_line` (int, default 16) — lidar line count: 16, 32, or 64
- `minimum_range` (double, default 0.1) — min distance to keep points
- `mapping_skip_frame` (int, default 5) — frames skipped between mapping updates
- `mapping_line_resolution` (float, default 0.4) — corner feature map downsampling
- `mapping_plane_resolution` (float, default 0.8) — planar feature map downsampling

## Key dependencies

- ROS2 Humble (rclcpp, nav_msgs, geometry_msgs, std_msgs, tf2_ros)
- PCL (pcl_conversions, filters, kdtree, voxel_grid)
- Eigen3
- Ceres Solver 2.0+
- OpenCV 4.5+ (only for kitti_helper)

## Design decisions

- The three header files (`common.h`, `tic_toc.h`, `lidarFactor.hpp`) required no changes from the ROS1 version — they have no ROS dependencies.
- ScanRegistration uses a flat function-local callback model (not a class method) with global arrays for performance — consistent with the original.
- LaserOdometry uses `rclcpp::spin_some()` in a timed loop to approximate the ROS1 `ros::spinOnce()` pattern.
- LaserMapping spawns a separate `std::thread` for `rclcpp::spin()` because it must process callbacks while its main thread runs the mapping loop.
- `DISTORTION` is set to 0 (point cloud distortion correction disabled) to match the original A-LOAM behavior.

## Migration design document

Full migration spec at `docs/superpowers/specs/2026-05-29-aloam-ros2-migration-design.md` — covers node mapping, API migration details (`ros::Time` → `rclcpp::Time`, header changes, `declare_parameter` pattern), and the communication topology diagram.
