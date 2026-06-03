# A-LOAM ROS2 Migration Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Migrate the `aloam_noted` ROS1 package to a ROS2 `a_loam_ros2` package with 4 nodes (scan_registration, laser_odometry, laser_mapping, kitti_helper).

**Architecture:** Direct 1:1 port. Each ROS1 executable becomes an `rclcpp::Node` subclass. Core algorithm logic (curvature computation, feature extraction, Ceres optimization, KD-tree search, cube management) is preserved verbatim. Only the ROS communication layer (publish/subscribe, parameters, TF, time, logging) is adapted.

**Tech Stack:** ROS2 Humble, rclcpp, sensor_msgs, nav_msgs, tf2_ros, pcl_conversions, Ceres Solver, Eigen3, OpenCV

**Source:** `aloam_noted/` (ROS1 catkin)
**Target:** `a_loam_ros2/` (ROS2 ament)

---

### Task 1: Create Package Scaffolding

**Files:**
- Create: `a_loam_ros2/package.xml`
- Create: `a_loam_ros2/CMakeLists.txt`
- Create: `a_loam_ros2/include/a_loam_ros2/` (directory)
- Create: `a_loam_ros2/src/` (directory)
- Create: `a_loam_ros2/launch/` (directory)
- Create: `a_loam_ros2/config/` (directory)
- Create: `a_loam_ros2/rviz/` (directory)

- [ ] **Step 1: Create directory structure**

```bash
mkdir -p a_loam_ros2/include/a_loam_ros2
mkdir -p a_loam_ros2/src
mkdir -p a_loam_ros2/launch
mkdir -p a_loam_ros2/config
mkdir -p a_loam_ros2/rviz
```

- [ ] **Step 2: Write package.xml**

```xml
<?xml version="1.0"?>
<?xml-model href="http://download.ros.org/schema/package_format3.xsd" schematypens="http://www.w3.org/2001/XMLSchema"?>
<package format="3">
  <name>a_loam_ros2</name>
  <version>1.0.0</version>

  <description>
    Advanced implementation of LOAM (Lidar Odometry and Mapping) for ROS2.
    LOAM is described in:
    J. Zhang and S. Singh. LOAM: Lidar Odometry and Mapping in Real-time.
    Robotics: Science and Systems Conference (RSS). Berkeley, CA, July 2014.
  </description>

  <maintainer email="qintonguav@gmail.com">qintong</maintainer>

  <license>BSD</license>

  <author email="zhangji@cmu.edu">Ji Zhang</author>

  <buildtool_depend>ament_cmake</buildtool_depend>

  <depend>rclcpp</depend>
  <depend>sensor_msgs</depend>
  <depend>nav_msgs</depend>
  <depend>geometry_msgs</depend>
  <depend>std_msgs</depend>
  <depend>tf2_ros</depend>
  <depend>tf2</depend>
  <depend>pcl_conversions</depend>
  <depend>libpcl-all-dev</depend>

  <exec_depend>ament_index_python</exec_depend>
  <exec_depend>launch_ros</exec_depend>

  <export>
    <build_type>ament_cmake</build_type>
  </export>
</package>
```

- [ ] **Step 3: Write CMakeLists.txt**

```cmake
cmake_minimum_required(VERSION 3.8)
project(a_loam_ros2)

# Default to Release build
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
find_package(Ceres REQUIRED)

include_directories(
  include
  ${rclcpp_INCLUDE_DIRS}
  ${sensor_msgs_INCLUDE_DIRS}
  ${nav_msgs_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
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
  tf2_ros tf2 pcl_conversions
)

# Install directories
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
```

---

### Task 2: Copy Shared Headers (No ROS Dependencies)

**Files:**
- Create: `a_loam_ros2/include/a_loam_ros2/common.h`
- Create: `a_loam_ros2/include/a_loam_ros2/tic_toc.h`
- Create: `a_loam_ros2/include/a_loam_ros2/lidarFactor.hpp`

These files have zero ROS dependencies and are copied verbatim.

- [ ] **Step 1: Copy common.h**

```bash
cp aloam_noted/include/aloam_velodyne/common.h a_loam_ros2/include/a_loam_ros2/common.h
cp aloam_noted/include/aloam_velodyne/tic_toc.h a_loam_ros2/include/a_loam_ros2/tic_toc.h
cp aloam_noted/src/lidarFactor.hpp a_loam_ros2/include/a_loam_ros2/lidarFactor.hpp
```

---

### Task 3: Implement ScanRegistrationNode

**Files:**
- Create: `a_loam_ros2/src/scan_registration_node.cpp`

- [ ] **Step 1: Write scan_registration_node.cpp**

```cpp
/*
 * @Description: lidar point feature extraction (ROS2 port)
 */

#include <cmath>
#include <vector>
#include <string>

#include "a_loam_ros2/common.h"
#include "a_loam_ros2/tic_toc.h"

#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/kdtree/kdtree_flann.h>

#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>

using std::atan2;
using std::cos;
using std::sin;

const double scanPeriod = 0.1;
const int systemDelay = 0;
int systemInitCount = 0;
bool systemInited = false;
int N_SCANS = 0;
float cloudCurvature[400000];
int cloudSortInd[400000];
int cloudNeighborPicked[400000];
int cloudLabel[400000];
bool comp (int i,int j) { return (cloudCurvature[i]<cloudCurvature[j]); }

template <typename PointT>
void removeClosedPointCloud(const pcl::PointCloud<PointT> &cloud_in,
                              pcl::PointCloud<PointT> &cloud_out, float thres)
{
    if (&cloud_in != &cloud_out)
    {
        cloud_out.header = cloud_in.header;
        cloud_out.points.resize(cloud_in.points.size());
    }

    size_t j = 0;

    for (size_t i = 0; i < cloud_in.points.size(); ++i)
    {
        if (cloud_in.points[i].x * cloud_in.points[i].x + cloud_in.points[i].y * cloud_in.points[i].y + cloud_in.points[i].z * cloud_in.points[i].z < thres * thres)
            continue;
        cloud_out.points[j] = cloud_in.points[i];
        j++;
    }
    if (j != cloud_in.points.size())
    {
        cloud_out.points.resize(j);
    }

    cloud_out.height = 1;
    cloud_out.width = static_cast<uint32_t>(j);
    cloud_out.is_dense = true;
}

class ScanRegistrationNode : public rclcpp::Node
{
public:
    ScanRegistrationNode() : Node("scan_registration")
    {
        // Parameters
        this->declare_parameter<int>("scan_line", 16);
        this->declare_parameter<double>("minimum_range", 0.1);
        this->declare_parameter<bool>("pub_each_line", false);

        N_SCANS = this->get_parameter("scan_line").as_int();
        MINIMUM_RANGE = this->get_parameter("minimum_range").as_double();
        PUB_EACH_LINE = this->get_parameter("pub_each_line").as_bool();

        printf("scan line number %d \n", N_SCANS);

        if(N_SCANS != 16 && N_SCANS != 32 && N_SCANS != 64)
        {
            RCLCPP_ERROR(this->get_logger(), "only support velodyne with 16, 32 or 64 scan line!");
            rclcpp::shutdown();
            return;
        }

        subLaserCloud = this->create_subscription<sensor_msgs::msg::PointCloud2>(
            "/velodyne_points", 100,
            std::bind(&ScanRegistrationNode::laserCloudHandler, this, std::placeholders::_1));

        pubLaserCloud = this->create_publisher<sensor_msgs::msg::PointCloud2>("/velodyne_cloud_2", 100);
        pubCornerPointsSharp = this->create_publisher<sensor_msgs::msg::PointCloud2>("/laser_cloud_sharp", 100);
        pubCornerPointsLessSharp = this->create_publisher<sensor_msgs::msg::PointCloud2>("/laser_cloud_less_sharp", 100);
        pubSurfPointsFlat = this->create_publisher<sensor_msgs::msg::PointCloud2>("/laser_cloud_flat", 100);
        pubSurfPointsLessFlat = this->create_publisher<sensor_msgs::msg::PointCloud2>("/laser_cloud_less_flat", 100);
        pubRemovePoints = this->create_publisher<sensor_msgs::msg::PointCloud2>("/laser_remove_points", 100);

        if(PUB_EACH_LINE)
        {
            for(int i = 0; i < N_SCANS; i++)
            {
                auto tmp = this->create_publisher<sensor_msgs::msg::PointCloud2>("/laser_scanid_" + std::to_string(i), 100);
                pubEachScan.push_back(tmp);
            }
        }
    }

private:
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr subLaserCloud;

    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pubLaserCloud;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pubCornerPointsSharp;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pubCornerPointsLessSharp;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pubSurfPointsFlat;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pubSurfPointsLessFlat;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pubRemovePoints;
    std::vector<rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr> pubEachScan;

    bool PUB_EACH_LINE = false;
    double MINIMUM_RANGE = 0.1;

    void laserCloudHandler(const sensor_msgs::msg::PointCloud2::ConstSharedPtr laserCloudMsg)
    {
        if (!systemInited)
        { 
            systemInitCount++;
            if (systemInitCount >= systemDelay)
            {
                systemInited = true;
            }
            else
                return;
        }

        TicToc t_whole;
        TicToc t_prepare;
        std::vector<int> scanStartInd(N_SCANS, 0);
        std::vector<int> scanEndInd(N_SCANS, 0);

        pcl::PointCloud<pcl::PointXYZ> laserCloudIn;
        pcl::fromROSMsg(*laserCloudMsg, laserCloudIn);
        std::vector<int> indices;

        pcl::removeNaNFromPointCloud(laserCloudIn, laserCloudIn, indices);
        removeClosedPointCloud(laserCloudIn, laserCloudIn, MINIMUM_RANGE);

        int cloudSize = laserCloudIn.points.size();
        float startOri = -atan2(laserCloudIn.points[0].y, laserCloudIn.points[0].x);
        float endOri = -atan2(laserCloudIn.points[cloudSize - 1].y,
                              laserCloudIn.points[cloudSize - 1].x) +
                       2 * M_PI;

        if (endOri - startOri > 3 * M_PI)
        {
            endOri -= 2 * M_PI;
        }
        else if (endOri - startOri < M_PI)
        {
            endOri += 2 * M_PI;
        }

        bool halfPassed = false;
        int count = cloudSize;
        PointType point;
        std::vector<pcl::PointCloud<PointType>> laserCloudScans(N_SCANS);
        for (int i = 0; i < cloudSize; i++)
        {
            point.x = laserCloudIn.points[i].x;
            point.y = laserCloudIn.points[i].y;
            point.z = laserCloudIn.points[i].z;

            float angle = atan(point.z / sqrt(point.x * point.x + point.y * point.y)) * 180 / M_PI;
            int scanID = 0;

            if (N_SCANS == 16)
            {
                scanID = int((angle + 15) / 2 + 0.5);
                if (scanID > (N_SCANS - 1) || scanID < 0)
                {
                    count--;
                    continue;
                }
            }
            else if (N_SCANS == 32)
            {
                scanID = int((angle + 92.0/3.0) * 3.0 / 4.0);
                if (scanID > (N_SCANS - 1) || scanID < 0)
                {
                    count--;
                    continue;
                }
            }
            else if (N_SCANS == 64)
            {   
                if (angle >= -8.83)
                    scanID = int((2 - angle) * 3.0 + 0.5);
                else
                    scanID = N_SCANS / 2 + int((-8.83 - angle) * 2.0 + 0.5);

                if (angle > 2 || angle < -24.33 || scanID > 50 || scanID < 0)
                {
                    count--;
                    continue;
                }
            }
            else
            {
                printf("wrong scan number\n");
                rclcpp::shutdown();
                return;
            }

            float ori = -atan2(point.y, point.x);
            if (!halfPassed)
            { 
                if (ori < startOri - M_PI / 2)
                {
                    ori += 2 * M_PI;
                }
                else if (ori > startOri + M_PI * 3 / 2)
                {
                    ori -= 2 * M_PI;
                }

                if (ori - startOri > M_PI)
                {
                    halfPassed = true;
                }
            }
            else
            {
                ori += 2 * M_PI;
                if (ori < endOri - M_PI * 3 / 2)
                {
                    ori += 2 * M_PI;
                }
                else if (ori > endOri + M_PI / 2)
                {
                    ori -= 2 * M_PI;
                }
            }

            float relTime = (ori - startOri) / (endOri - startOri);
            point.intensity = scanID + scanPeriod * relTime;
            laserCloudScans[scanID].push_back(point); 
        }
        
        cloudSize = count;
        printf("points size %d \n", cloudSize);

        pcl::PointCloud<PointType>::Ptr laserCloud(new pcl::PointCloud<PointType>());
        for (int i = 0; i < N_SCANS; i++)
        { 
            scanStartInd[i] = laserCloud->size() + 5;
            *laserCloud += laserCloudScans[i];
            scanEndInd[i] = laserCloud->size() - 6;
        }

        printf("prepare time %f \n", t_prepare.toc());

        for (int i = 5; i < cloudSize - 5; i++)
        { 
            float diffX = laserCloud->points[i - 5].x + laserCloud->points[i - 4].x + laserCloud->points[i - 3].x + laserCloud->points[i - 2].x + laserCloud->points[i - 1].x - 10 * laserCloud->points[i].x + laserCloud->points[i + 1].x + laserCloud->points[i + 2].x + laserCloud->points[i + 3].x + laserCloud->points[i + 4].x + laserCloud->points[i + 5].x;
            float diffY = laserCloud->points[i - 5].y + laserCloud->points[i - 4].y + laserCloud->points[i - 3].y + laserCloud->points[i - 2].y + laserCloud->points[i - 1].y - 10 * laserCloud->points[i].y + laserCloud->points[i + 1].y + laserCloud->points[i + 2].y + laserCloud->points[i + 3].y + laserCloud->points[i + 4].y + laserCloud->points[i + 5].y;
            float diffZ = laserCloud->points[i - 5].z + laserCloud->points[i - 4].z + laserCloud->points[i - 3].z + laserCloud->points[i - 2].z + laserCloud->points[i - 1].z - 10 * laserCloud->points[i].z + laserCloud->points[i + 1].z + laserCloud->points[i + 2].z + laserCloud->points[i + 3].z + laserCloud->points[i + 4].z + laserCloud->points[i + 5].z;

            cloudCurvature[i] = diffX * diffX + diffY * diffY + diffZ * diffZ;
            cloudSortInd[i] = i;
            cloudNeighborPicked[i] = 0;
            cloudLabel[i] = 0;
        }

        TicToc t_pts;

        pcl::PointCloud<PointType> cornerPointsSharp;
        pcl::PointCloud<PointType> cornerPointsLessSharp;
        pcl::PointCloud<PointType> surfPointsFlat;
        pcl::PointCloud<PointType> surfPointsLessFlat;

        float t_q_sort = 0;
        for (int i = 0; i < N_SCANS; i++)
        {
            if( scanEndInd[i] - scanStartInd[i] < 6)
                continue;
            pcl::PointCloud<PointType>::Ptr surfPointsLessFlatScan(new pcl::PointCloud<PointType>);
            for (int j = 0; j < 6; j++)
            {
                int sp = scanStartInd[i] + (scanEndInd[i] - scanStartInd[i]) * j / 6; 
                int ep = scanStartInd[i] + (scanEndInd[i] - scanStartInd[i]) * (j + 1) / 6 - 1;

                TicToc t_tmp;
                std::sort (cloudSortInd + sp, cloudSortInd + ep + 1, comp);
                t_q_sort += t_tmp.toc();

                int largestPickedNum = 0;
                for (int k = ep; k >= sp; k--)
                {
                    int ind = cloudSortInd[k]; 

                    if (cloudNeighborPicked[ind] == 0 &&
                        cloudCurvature[ind] > 0.1)
                    {
                        largestPickedNum++;
                        if (largestPickedNum <= 2)
                        {                        
                            cloudLabel[ind] = 2;
                            cornerPointsSharp.push_back(laserCloud->points[ind]);
                            cornerPointsLessSharp.push_back(laserCloud->points[ind]);
                        }
                        else if (largestPickedNum <= 20)
                        {                        
                            cloudLabel[ind] = 1; 
                            cornerPointsLessSharp.push_back(laserCloud->points[ind]);
                        }
                        else
                        {
                            break;
                        }

                        cloudNeighborPicked[ind] = 1; 

                        for (int l = 1; l <= 5; l++)
                        {
                            float diffX = laserCloud->points[ind + l].x - laserCloud->points[ind + l - 1].x;
                            float diffY = laserCloud->points[ind + l].y - laserCloud->points[ind + l - 1].y;
                            float diffZ = laserCloud->points[ind + l].z - laserCloud->points[ind + l - 1].z;
                            if (diffX * diffX + diffY * diffY + diffZ * diffZ > 0.05)
                            {
                                break;
                            }

                            cloudNeighborPicked[ind + l] = 1;
                        }
                        for (int l = -1; l >= -5; l--)
                        {
                            float diffX = laserCloud->points[ind + l].x - laserCloud->points[ind + l + 1].x;
                            float diffY = laserCloud->points[ind + l].y - laserCloud->points[ind + l + 1].y;
                            float diffZ = laserCloud->points[ind + l].z - laserCloud->points[ind + l + 1].z;
                            if (diffX * diffX + diffY * diffY + diffZ * diffZ > 0.05)
                            {
                                break;
                            }

                            cloudNeighborPicked[ind + l] = 1;
                        }
                    }
                }

                int smallestPickedNum = 0;
                for (int k = sp; k <= ep; k++)
                {
                    int ind = cloudSortInd[k];

                    if (cloudNeighborPicked[ind] == 0 &&
                        cloudCurvature[ind] < 0.1)
                    {
                        cloudLabel[ind] = -1; 
                        surfPointsFlat.push_back(laserCloud->points[ind]);

                        smallestPickedNum++;
                        if (smallestPickedNum >= 4)
                        { 
                            break;
                        }

                        cloudNeighborPicked[ind] = 1;
                        for (int l = 1; l <= 5; l++)
                        { 
                            float diffX = laserCloud->points[ind + l].x - laserCloud->points[ind + l - 1].x;
                            float diffY = laserCloud->points[ind + l].y - laserCloud->points[ind + l - 1].y;
                            float diffZ = laserCloud->points[ind + l].z - laserCloud->points[ind + l - 1].z;
                            if (diffX * diffX + diffY * diffY + diffZ * diffZ > 0.05)
                            {
                                break;
                            }

                            cloudNeighborPicked[ind + l] = 1;
                        }
                        for (int l = -1; l >= -5; l--)
                        {
                            float diffX = laserCloud->points[ind + l].x - laserCloud->points[ind + l + 1].x;
                            float diffY = laserCloud->points[ind + l].y - laserCloud->points[ind + l + 1].y;
                            float diffZ = laserCloud->points[ind + l].z - laserCloud->points[ind + l + 1].z;
                            if (diffX * diffX + diffY * diffY + diffZ * diffZ > 0.05)
                            {
                                break;
                            }

                            cloudNeighborPicked[ind + l] = 1;
                        }
                    }
                }

                for (int k = sp; k <= ep; k++)
                {
                    if (cloudLabel[k] <= 0)
                    {
                        surfPointsLessFlatScan->push_back(laserCloud->points[k]);
                    }
                }
            }

            pcl::PointCloud<PointType> surfPointsLessFlatScanDS;
            pcl::VoxelGrid<PointType> downSizeFilter;
            downSizeFilter.setInputCloud(surfPointsLessFlatScan);
            downSizeFilter.setLeafSize(0.2, 0.2, 0.2);
            downSizeFilter.filter(surfPointsLessFlatScanDS);

            surfPointsLessFlat += surfPointsLessFlatScanDS;
        }
        printf("sort q time %f \n", t_q_sort);
        printf("seperate points time %f \n", t_pts.toc());

        // Publish results
        auto publishCloud = [&](const auto& cloud, auto& pub, const auto& stamp, const char* frame) {
            sensor_msgs::msg::PointCloud2 msg;
            pcl::toROSMsg(cloud, msg);
            msg.header.stamp = stamp;
            msg.header.frame_id = frame;
            pub->publish(msg);
        };

        publishCloud(*laserCloud, pubLaserCloud, laserCloudMsg->header.stamp, "/camera_init");
        publishCloud(cornerPointsSharp, pubCornerPointsSharp, laserCloudMsg->header.stamp, "/camera_init");
        publishCloud(cornerPointsLessSharp, pubCornerPointsLessSharp, laserCloudMsg->header.stamp, "/camera_init");
        publishCloud(surfPointsFlat, pubSurfPointsFlat, laserCloudMsg->header.stamp, "/camera_init");
        publishCloud(surfPointsLessFlat, pubSurfPointsLessFlat, laserCloudMsg->header.stamp, "/camera_init");

        if(PUB_EACH_LINE)
        {
            for(int i = 0; i < N_SCANS; i++)
            {
                sensor_msgs::msg::PointCloud2 scanMsg;
                pcl::toROSMsg(laserCloudScans[i], scanMsg);
                scanMsg.header.stamp = laserCloudMsg->header.stamp;
                scanMsg.header.frame_id = "/camera_init";
                pubEachScan[i]->publish(scanMsg);
            }
        }

        printf("scan registration time %f ms *************\n", t_whole.toc());
        if(t_whole.toc() > 100)
            RCLCPP_WARN(this->get_logger(), "scan registration process over 100ms");
    }
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<ScanRegistrationNode>());
    rclcpp::shutdown();
    return 0;
}
```

---

### Task 4: Implement LaserOdometryNode

**Files:**
- Create: `a_loam_ros2/src/laser_odometry_node.cpp`

- [ ] **Step 1: Write laser_odometry_node.cpp**

```cpp
/*
 * @Description: frame-to-frame lidar odometry via feature matching (ROS2 port)
 */

#include <cmath>
#include <eigen3/Eigen/Dense>
#include <mutex>
#include <queue>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl_conversions/pcl_conversions.h>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <nav_msgs/msg/path.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>

#include "a_loam_ros2/common.h"
#include "a_loam_ros2/tic_toc.h"
#include "a_loam_ros2/lidarFactor.hpp"

#define DISTORTION 0

int corner_correspondence = 0, plane_correspondence = 0;

constexpr double SCAN_PERIOD = 0.1;
constexpr double DISTANCE_SQ_THRESHOLD = 25;
constexpr double NEARBY_SCAN = 2.5;

class LaserOdometryNode : public rclcpp::Node
{
public:
    LaserOdometryNode() : Node("laser_odometry")
    {
        this->declare_parameter<int>("mapping_skip_frame", 2);
        skipFrameNum = this->get_parameter("mapping_skip_frame").as_int();
        printf("Mapping %d Hz \n", 10 / skipFrameNum);

        subCornerPointsSharp = this->create_subscription<sensor_msgs::msg::PointCloud2>(
            "/laser_cloud_sharp", 100,
            std::bind(&LaserOdometryNode::laserCloudSharpHandler, this, std::placeholders::_1));

        subCornerPointsLessSharp = this->create_subscription<sensor_msgs::msg::PointCloud2>(
            "/laser_cloud_less_sharp", 100,
            std::bind(&LaserOdometryNode::laserCloudLessSharpHandler, this, std::placeholders::_1));

        subSurfPointsFlat = this->create_subscription<sensor_msgs::msg::PointCloud2>(
            "/laser_cloud_flat", 100,
            std::bind(&LaserOdometryNode::laserCloudFlatHandler, this, std::placeholders::_1));

        subSurfPointsLessFlat = this->create_subscription<sensor_msgs::msg::PointCloud2>(
            "/laser_cloud_less_flat", 100,
            std::bind(&LaserOdometryNode::laserCloudLessFlatHandler, this, std::placeholders::_1));

        subLaserCloudFullRes = this->create_subscription<sensor_msgs::msg::PointCloud2>(
            "/velodyne_cloud_2", 100,
            std::bind(&LaserOdometryNode::laserCloudFullResHandler, this, std::placeholders::_1));

        pubLaserCloudCornerLast = this->create_publisher<sensor_msgs::msg::PointCloud2>("/laser_cloud_corner_last", 100);
        pubLaserCloudSurfLast = this->create_publisher<sensor_msgs::msg::PointCloud2>("/laser_cloud_surf_last", 100);
        pubLaserCloudFullRes = this->create_publisher<sensor_msgs::msg::PointCloud2>("/velodyne_cloud_3", 100);
        pubLaserOdometry = this->create_publisher<nav_msgs::msg::Odometry>("/laser_odom_to_init", 100);
        pubLaserPath = this->create_publisher<nav_msgs::msg::Path>("/laser_odom_path", 100);

        // Initialize KD-trees
        kdtreeCornerLast.reset(new pcl::KdTreeFLANN<PointType>());
        kdtreeSurfLast.reset(new pcl::KdTreeFLANN<PointType>());

        cornerPointsSharp.reset(new pcl::PointCloud<PointType>());
        cornerPointsLessSharp.reset(new pcl::PointCloud<PointType>());
        surfPointsFlat.reset(new pcl::PointCloud<PointType>());
        surfPointsLessFlat.reset(new pcl::PointCloud<PointType>());
        laserCloudCornerLast.reset(new pcl::PointCloud<PointType>());
        laserCloudSurfLast.reset(new pcl::PointCloud<PointType>());
        laserCloudFullRes.reset(new pcl::PointCloud<PointType>());

        // Drive the main loop with a timer at 100Hz
        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(10),
            std::bind(&LaserOdometryNode::processFrame, this));
    }

private:
    // Subscriptions
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr subCornerPointsSharp;
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr subCornerPointsLessSharp;
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr subSurfPointsFlat;
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr subSurfPointsLessFlat;
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr subLaserCloudFullRes;

    // Publishers
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pubLaserCloudCornerLast;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pubLaserCloudSurfLast;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pubLaserCloudFullRes;
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr pubLaserOdometry;
    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr pubLaserPath;

    // Timer for main loop
    rclcpp::TimerBase::SharedPtr timer_;

    // Message buffers
    std::queue<sensor_msgs::msg::PointCloud2::ConstSharedPtr> cornerSharpBuf;
    std::queue<sensor_msgs::msg::PointCloud2::ConstSharedPtr> cornerLessSharpBuf;
    std::queue<sensor_msgs::msg::PointCloud2::ConstSharedPtr> surfFlatBuf;
    std::queue<sensor_msgs::msg::PointCloud2::ConstSharedPtr> surfLessFlatBuf;
    std::queue<sensor_msgs::msg::PointCloud2::ConstSharedPtr> fullPointsBuf;
    std::mutex mBuf;

    // State
    int skipFrameNum = 5;
    bool systemInited = false;
    int frameCount = 0;

    double timeCornerPointsSharp = 0;
    double timeCornerPointsLessSharp = 0;
    double timeSurfPointsFlat = 0;
    double timeSurfPointsLessFlat = 0;
    double timeLaserCloudFullRes = 0;

    pcl::KdTreeFLANN<PointType>::Ptr kdtreeCornerLast;
    pcl::KdTreeFLANN<PointType>::Ptr kdtreeSurfLast;

    pcl::PointCloud<PointType>::Ptr cornerPointsSharp;
    pcl::PointCloud<PointType>::Ptr cornerPointsLessSharp;
    pcl::PointCloud<PointType>::Ptr surfPointsFlat;
    pcl::PointCloud<PointType>::Ptr surfPointsLessFlat;
    pcl::PointCloud<PointType>::Ptr laserCloudCornerLast;
    pcl::PointCloud<PointType>::Ptr laserCloudSurfLast;
    pcl::PointCloud<PointType>::Ptr laserCloudFullRes;

    int laserCloudCornerLastNum = 0;
    int laserCloudSurfLastNum = 0;

    Eigen::Quaterniond q_w_curr{1, 0, 0, 0};
    Eigen::Vector3d t_w_curr{0, 0, 0};

    double para_q[4] = {0, 0, 0, 1};
    double para_t[3] = {0, 0, 0};
    Eigen::Map<Eigen::Quaterniond> q_last_curr{para_q};
    Eigen::Map<Eigen::Vector3d> t_last_curr{para_t};

    nav_msgs::msg::Path laserPath;

    // Undistort point to start of sweep
    void TransformToStart(PointType const *const pi, PointType *const po)
    {
        double s;
        if (DISTORTION)
            s = (pi->intensity - int(pi->intensity)) / SCAN_PERIOD;
        else
            s = 1.0;

        Eigen::Quaterniond q_point_last = Eigen::Quaterniond::Identity().slerp(s, q_last_curr);
        Eigen::Vector3d t_point_last = s * t_last_curr;
        Eigen::Vector3d point(pi->x, pi->y, pi->z);
        Eigen::Vector3d un_point = q_point_last * point + t_point_last;

        po->x = un_point.x();
        po->y = un_point.y();
        po->z = un_point.z();
        po->intensity = pi->intensity;
    }

    void TransformToEnd(PointType const *const pi, PointType *const po)
    {
        pcl::PointXYZI un_point_tmp;
        TransformToStart(pi, &un_point_tmp);

        Eigen::Vector3d un_point(un_point_tmp.x, un_point_tmp.y, un_point_tmp.z);
        Eigen::Vector3d point_end = q_last_curr.inverse() * (un_point - t_last_curr);

        po->x = point_end.x();
        po->y = point_end.y();
        po->z = point_end.z();
        po->intensity = int(pi->intensity);
    }

    // Callbacks (thread-safe buffer push)
    void laserCloudSharpHandler(const sensor_msgs::msg::PointCloud2::ConstSharedPtr msg) {
        std::lock_guard<std::mutex> lock(mBuf);
        cornerSharpBuf.push(msg);
    }
    void laserCloudLessSharpHandler(const sensor_msgs::msg::PointCloud2::ConstSharedPtr msg) {
        std::lock_guard<std::mutex> lock(mBuf);
        cornerLessSharpBuf.push(msg);
    }
    void laserCloudFlatHandler(const sensor_msgs::msg::PointCloud2::ConstSharedPtr msg) {
        std::lock_guard<std::mutex> lock(mBuf);
        surfFlatBuf.push(msg);
    }
    void laserCloudLessFlatHandler(const sensor_msgs::msg::PointCloud2::ConstSharedPtr msg) {
        std::lock_guard<std::mutex> lock(mBuf);
        surfLessFlatBuf.push(msg);
    }
    void laserCloudFullResHandler(const sensor_msgs::msg::PointCloud2::ConstSharedPtr msg) {
        std::lock_guard<std::mutex> lock(mBuf);
        fullPointsBuf.push(msg);
    }

    void processFrame()
    {
        if (cornerSharpBuf.empty() || cornerLessSharpBuf.empty() ||
            surfFlatBuf.empty() || surfLessFlatBuf.empty() ||
            fullPointsBuf.empty())
        {
            return;
        }

        // Get timestamps
        timeCornerPointsSharp = rclcpp::Time(cornerSharpBuf.front()->header.stamp).seconds();
        timeCornerPointsLessSharp = rclcpp::Time(cornerLessSharpBuf.front()->header.stamp).seconds();
        timeSurfPointsFlat = rclcpp::Time(surfFlatBuf.front()->header.stamp).seconds();
        timeSurfPointsLessFlat = rclcpp::Time(surfLessFlatBuf.front()->header.stamp).seconds();
        timeLaserCloudFullRes = rclcpp::Time(fullPointsBuf.front()->header.stamp).seconds();

        if (timeCornerPointsSharp != timeLaserCloudFullRes ||
            timeCornerPointsLessSharp != timeLaserCloudFullRes ||
            timeSurfPointsFlat != timeLaserCloudFullRes ||
            timeSurfPointsLessFlat != timeLaserCloudFullRes)
        {
            printf("unsync messeage!");
            rclcpp::shutdown();
            return;
        }

        // Convert from ROS to PCL
        {
            std::lock_guard<std::mutex> lock(mBuf);
            cornerPointsSharp->clear();
            pcl::fromROSMsg(*cornerSharpBuf.front(), *cornerPointsSharp);
            cornerSharpBuf.pop();

            cornerPointsLessSharp->clear();
            pcl::fromROSMsg(*cornerLessSharpBuf.front(), *cornerPointsLessSharp);
            cornerLessSharpBuf.pop();

            surfPointsFlat->clear();
            pcl::fromROSMsg(*surfFlatBuf.front(), *surfPointsFlat);
            surfFlatBuf.pop();

            surfPointsLessFlat->clear();
            pcl::fromROSMsg(*surfLessFlatBuf.front(), *surfPointsLessFlat);
            surfLessFlatBuf.pop();

            laserCloudFullRes->clear();
            pcl::fromROSMsg(*fullPointsBuf.front(), *laserCloudFullRes);
            fullPointsBuf.pop();
        }

        TicToc t_whole;

        if (!systemInited)
        {
            systemInited = true;
            std::cout << "Initialization finished \n";
        }
        else
        {
            int cornerPointsSharpNum = cornerPointsSharp->points.size();
            int surfPointsFlatNum = surfPointsFlat->points.size();

            TicToc t_opt;
            for (size_t opti_counter = 0; opti_counter < 2; ++opti_counter)
            {
                corner_correspondence = 0;
                plane_correspondence = 0;

                ceres::LossFunction *loss_function = new ceres::HuberLoss(0.1);
                ceres::LocalParameterization *q_parameterization =
                    new ceres::EigenQuaternionParameterization();
                ceres::Problem::Options problem_options;
                ceres::Problem problem(problem_options);
                problem.AddParameterBlock(para_q, 4, q_parameterization);
                problem.AddParameterBlock(para_t, 3);

                pcl::PointXYZI pointSel;
                std::vector<int> pointSearchInd;
                std::vector<float> pointSearchSqDis;

                TicToc t_data;

                // Corner feature matching (lines 302-405 of ROS1 laserOdometry.cpp)
                for (int i = 0; i < cornerPointsSharpNum; ++i)
                {
                    TransformToStart(&(cornerPointsSharp->points[i]), &pointSel);
                    kdtreeCornerLast->nearestKSearch(pointSel, 1, pointSearchInd, pointSearchSqDis);

                    int closestPointInd = -1, minPointInd2 = -1;
                    if (pointSearchSqDis[0] < DISTANCE_SQ_THRESHOLD)
                    {
                        closestPointInd = pointSearchInd[0];
                        int closestPointScanID = int(laserCloudCornerLast->points[closestPointInd].intensity);

                        double minPointSqDis2 = DISTANCE_SQ_THRESHOLD;

                        for (int j = closestPointInd + 1; j < (int)laserCloudCornerLast->points.size(); ++j)
                        {
                            if (int(laserCloudCornerLast->points[j].intensity) <= closestPointScanID)
                                continue;
                            if (int(laserCloudCornerLast->points[j].intensity) > (closestPointScanID + NEARBY_SCAN))
                                break;

                            double pointSqDis = (laserCloudCornerLast->points[j].x - pointSel.x) *
                                                    (laserCloudCornerLast->points[j].x - pointSel.x) +
                                                (laserCloudCornerLast->points[j].y - pointSel.y) *
                                                    (laserCloudCornerLast->points[j].y - pointSel.y) +
                                                (laserCloudCornerLast->points[j].z - pointSel.z) *
                                                    (laserCloudCornerLast->points[j].z - pointSel.z);

                            if (pointSqDis < minPointSqDis2)
                            {
                                minPointSqDis2 = pointSqDis;
                                minPointInd2 = j;
                            }
                        }

                        for (int j = closestPointInd - 1; j >= 0; --j)
                        {
                            if (int(laserCloudCornerLast->points[j].intensity) >= closestPointScanID)
                                continue;
                            if (int(laserCloudCornerLast->points[j].intensity) < (closestPointScanID - NEARBY_SCAN))
                                break;

                            double pointSqDis = (laserCloudCornerLast->points[j].x - pointSel.x) *
                                                    (laserCloudCornerLast->points[j].x - pointSel.x) +
                                                (laserCloudCornerLast->points[j].y - pointSel.y) *
                                                    (laserCloudCornerLast->points[j].y - pointSel.y) +
                                                (laserCloudCornerLast->points[j].z - pointSel.z) *
                                                    (laserCloudCornerLast->points[j].z - pointSel.z);

                            if (pointSqDis < minPointSqDis2)
                            {
                                minPointSqDis2 = pointSqDis;
                                minPointInd2 = j;
                            }
                        }
                    }
                    if (minPointInd2 >= 0)
                    {
                        Eigen::Vector3d curr_point(cornerPointsSharp->points[i].x,
                                                   cornerPointsSharp->points[i].y,
                                                   cornerPointsSharp->points[i].z);
                        Eigen::Vector3d last_point_a(laserCloudCornerLast->points[closestPointInd].x,
                                                     laserCloudCornerLast->points[closestPointInd].y,
                                                     laserCloudCornerLast->points[closestPointInd].z);
                        Eigen::Vector3d last_point_b(laserCloudCornerLast->points[minPointInd2].x,
                                                     laserCloudCornerLast->points[minPointInd2].y,
                                                     laserCloudCornerLast->points[minPointInd2].z);

                        double s;
                        if (DISTORTION)
                            s = (cornerPointsSharp->points[i].intensity - int(cornerPointsSharp->points[i].intensity)) / SCAN_PERIOD;
                        else
                            s = 1.0;

                        ceres::CostFunction *cost_function = LidarEdgeFactor::Create(curr_point, last_point_a, last_point_b, s);
                        problem.AddResidualBlock(cost_function, loss_function, para_q, para_t);
                        corner_correspondence++;
                    }
                }

                // Surface feature matching (lines 411-510 of ROS1 laserOdometry.cpp)
                for (int i = 0; i < surfPointsFlatNum; ++i)
                {
                    TransformToStart(&(surfPointsFlat->points[i]), &pointSel);
                    kdtreeSurfLast->nearestKSearch(pointSel, 1, pointSearchInd, pointSearchSqDis);

                    int closestPointInd = -1, minPointInd2 = -1, minPointInd3 = -1;
                    if (pointSearchSqDis[0] < DISTANCE_SQ_THRESHOLD)
                    {
                        closestPointInd = pointSearchInd[0];
                        int closestPointScanID = int(laserCloudSurfLast->points[closestPointInd].intensity);
                        double minPointSqDis2 = DISTANCE_SQ_THRESHOLD, minPointSqDis3 = DISTANCE_SQ_THRESHOLD;

                        for (int j = closestPointInd + 1; j < (int)laserCloudSurfLast->points.size(); ++j)
                        {
                            if (int(laserCloudSurfLast->points[j].intensity) > (closestPointScanID + NEARBY_SCAN))
                                break;

                            double pointSqDis = (laserCloudSurfLast->points[j].x - pointSel.x) *
                                                    (laserCloudSurfLast->points[j].x - pointSel.x) +
                                                (laserCloudSurfLast->points[j].y - pointSel.y) *
                                                    (laserCloudSurfLast->points[j].y - pointSel.y) +
                                                (laserCloudSurfLast->points[j].z - pointSel.z) *
                                                    (laserCloudSurfLast->points[j].z - pointSel.z);

                            if (int(laserCloudSurfLast->points[j].intensity) <= closestPointScanID && pointSqDis < minPointSqDis2)
                            {
                                minPointSqDis2 = pointSqDis;
                                minPointInd2 = j;
                            }
                            else if (int(laserCloudSurfLast->points[j].intensity) > closestPointScanID && pointSqDis < minPointSqDis3)
                            {
                                minPointSqDis3 = pointSqDis;
                                minPointInd3 = j;
                            }
                        }

                        for (int j = closestPointInd - 1; j >= 0; --j)
                        {
                            if (int(laserCloudSurfLast->points[j].intensity) < (closestPointScanID - NEARBY_SCAN))
                                break;

                            double pointSqDis = (laserCloudSurfLast->points[j].x - pointSel.x) *
                                                    (laserCloudSurfLast->points[j].x - pointSel.x) +
                                                (laserCloudSurfLast->points[j].y - pointSel.y) *
                                                    (laserCloudSurfLast->points[j].y - pointSel.y) +
                                                (laserCloudSurfLast->points[j].z - pointSel.z) *
                                                    (laserCloudSurfLast->points[j].z - pointSel.z);

                            if (int(laserCloudSurfLast->points[j].intensity) >= closestPointScanID && pointSqDis < minPointSqDis2)
                            {
                                minPointSqDis2 = pointSqDis;
                                minPointInd2 = j;
                            }
                            else if (int(laserCloudSurfLast->points[j].intensity) < closestPointScanID && pointSqDis < minPointSqDis3)
                            {
                                minPointSqDis3 = pointSqDis;
                                minPointInd3 = j;
                            }
                        }

                        if (minPointInd2 >= 0 && minPointInd3 >= 0)
                        {
                            Eigen::Vector3d curr_point(surfPointsFlat->points[i].x,
                                                        surfPointsFlat->points[i].y,
                                                        surfPointsFlat->points[i].z);
                            Eigen::Vector3d last_point_a(laserCloudSurfLast->points[closestPointInd].x,
                                                            laserCloudSurfLast->points[closestPointInd].y,
                                                            laserCloudSurfLast->points[closestPointInd].z);
                            Eigen::Vector3d last_point_b(laserCloudSurfLast->points[minPointInd2].x,
                                                            laserCloudSurfLast->points[minPointInd2].y,
                                                            laserCloudSurfLast->points[minPointInd2].z);
                            Eigen::Vector3d last_point_c(laserCloudSurfLast->points[minPointInd3].x,
                                                            laserCloudSurfLast->points[minPointInd3].y,
                                                            laserCloudSurfLast->points[minPointInd3].z);

                            double s;
                            if (DISTORTION)
                                s = (surfPointsFlat->points[i].intensity - int(surfPointsFlat->points[i].intensity)) / SCAN_PERIOD;
                            else
                                s = 1.0;

                            ceres::CostFunction *cost_function = LidarPlaneFactor::Create(curr_point, last_point_a, last_point_b, last_point_c, s);
                            problem.AddResidualBlock(cost_function, loss_function, para_q, para_t);
                            plane_correspondence++;
                        }
                    }
                }

                printf("data association time %f ms \n", t_data.toc());

                if ((corner_correspondence + plane_correspondence) < 10)
                {
                    printf("less correspondence! *************************************************\n");
                }

                TicToc t_solver;
                ceres::Solver::Options options;
                options.linear_solver_type = ceres::DENSE_QR;
                options.max_num_iterations = 4;
                options.minimizer_progress_to_stdout = false;
                ceres::Solver::Summary summary;
                ceres::Solve(options, &problem, &summary);
                printf("solver time %f ms \n", t_solver.toc());
            }
            printf("optimization twice time %f \n", t_opt.toc());

            t_w_curr = t_w_curr + q_w_curr * t_last_curr;
            q_w_curr = q_w_curr * q_last_curr;
        }

        TicToc t_pub;

        // Publish odometry
        nav_msgs::msg::Odometry laserOdometry;
        laserOdometry.header.frame_id = "/camera_init";
        laserOdometry.child_frame_id = "/laser_odom";
        laserOdometry.header.stamp = rclcpp::Time(static_cast<int64_t>(timeSurfPointsLessFlat * 1e9));
        laserOdometry.pose.pose.orientation.x = q_w_curr.x();
        laserOdometry.pose.pose.orientation.y = q_w_curr.y();
        laserOdometry.pose.pose.orientation.z = q_w_curr.z();
        laserOdometry.pose.pose.orientation.w = q_w_curr.w();
        laserOdometry.pose.pose.position.x = t_w_curr.x();
        laserOdometry.pose.pose.position.y = t_w_curr.y();
        laserOdometry.pose.pose.position.z = t_w_curr.z();
        pubLaserOdometry->publish(laserOdometry);

        geometry_msgs::msg::PoseStamped laserPose;
        laserPose.header = laserOdometry.header;
        laserPose.pose = laserOdometry.pose.pose;
        laserPath.header.stamp = laserOdometry.header.stamp;
        laserPath.poses.push_back(laserPose);
        laserPath.header.frame_id = "/camera_init";
        pubLaserPath->publish(laserPath);

        // Swap buffers for KD-tree
        pcl::PointCloud<PointType>::Ptr laserCloudTemp = cornerPointsLessSharp;
        cornerPointsLessSharp = laserCloudCornerLast;
        laserCloudCornerLast = laserCloudTemp;

        laserCloudTemp = surfPointsLessFlat;
        surfPointsLessFlat = laserCloudSurfLast;
        laserCloudSurfLast = laserCloudTemp;

        laserCloudCornerLastNum = laserCloudCornerLast->points.size();
        laserCloudSurfLastNum = laserCloudSurfLast->points.size();

        kdtreeCornerLast->setInputCloud(laserCloudCornerLast);
        kdtreeSurfLast->setInputCloud(laserCloudSurfLast);

        if (frameCount % skipFrameNum == 0)
        {
            frameCount = 0;

            auto publishCloud = [this](const auto& cloud, auto& pub, double time_sec, const char* frame) {
                sensor_msgs::msg::PointCloud2 msg;
                pcl::toROSMsg(*cloud, msg);
                msg.header.stamp = rclcpp::Time(static_cast<int64_t>(time_sec * 1e9));
                msg.header.frame_id = frame;
                pub->publish(msg);
            };

            publishCloud(laserCloudCornerLast, pubLaserCloudCornerLast, timeSurfPointsLessFlat, "/camera");
            publishCloud(laserCloudSurfLast, pubLaserCloudSurfLast, timeSurfPointsLessFlat, "/camera");
            publishCloud(laserCloudFullRes, pubLaserCloudFullRes, timeSurfPointsLessFlat, "/camera");
        }

        printf("publication time %f ms \n", t_pub.toc());
        printf("whole laserOdometry time %f ms \n \n", t_whole.toc());
        if(t_whole.toc() > 100)
            RCLCPP_WARN(this->get_logger(), "odometry process over 100ms");

        frameCount++;
    }
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<LaserOdometryNode>());
    rclcpp::shutdown();
    return 0;
}
```

---

### Task 5: Implement LaserMappingNode

**Files:**
- Create: `a_loam_ros2/src/laser_mapping_node.cpp`

- [ ] **Step 1: Write laser_mapping_node.cpp**

```cpp
/*
 * @Description: scan-to-submap matching and mapping backend (ROS2 port)
 */

#include <math.h>
#include <vector>
#include <eigen3/Eigen/Dense>
#include <ceres/ceres.h>
#include <mutex>
#include <queue>
#include <thread>
#include <iostream>
#include <string>

#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/kdtree/kdtree_flann.h>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <nav_msgs/msg/path.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2/LinearMath/Quaternion.h>

#include "a_loam_ros2/lidarFactor.hpp"
#include "a_loam_ros2/common.h"
#include "a_loam_ros2/tic_toc.h"

int frameCount = 0;

double timeLaserCloudCornerLast = 0;
double timeLaserCloudSurfLast = 0;
double timeLaserCloudFullRes = 0;
double timeLaserOdometry = 0;

int laserCloudCenWidth = 10;
int laserCloudCenHeight = 10;
int laserCloudCenDepth = 5;
const int laserCloudWidth = 21;
const int laserCloudHeight = 21;
const int laserCloudDepth = 11;
const int laserCloudNum = laserCloudWidth * laserCloudHeight * laserCloudDepth;

int laserCloudValidInd[125];
int laserCloudSurroundInd[125];

class LaserMappingNode : public rclcpp::Node
{
public:
    LaserMappingNode() : Node("laser_mapping")
    {
        this->declare_parameter<float>("mapping_line_resolution", 0.4);
        this->declare_parameter<float>("mapping_plane_resolution", 0.8);

        float lineRes = this->get_parameter("mapping_line_resolution").as_double();
        float planeRes = this->get_parameter("mapping_plane_resolution").as_double();
        printf("line resolution %f plane resolution %f \n", lineRes, planeRes);
        downSizeFilterCorner.setLeafSize(lineRes, lineRes, lineRes);
        downSizeFilterSurf.setLeafSize(planeRes, planeRes, planeRes);

        // Subscribers
        subLaserCloudCornerLast = this->create_subscription<sensor_msgs::msg::PointCloud2>(
            "/laser_cloud_corner_last", 100,
            std::bind(&LaserMappingNode::laserCloudCornerLastHandler, this, std::placeholders::_1));
        subLaserCloudSurfLast = this->create_subscription<sensor_msgs::msg::PointCloud2>(
            "/laser_cloud_surf_last", 100,
            std::bind(&LaserMappingNode::laserCloudSurfLastHandler, this, std::placeholders::_1));
        subLaserOdometry = this->create_subscription<nav_msgs::msg::Odometry>(
            "/laser_odom_to_init", 100,
            std::bind(&LaserMappingNode::laserOdometryHandler, this, std::placeholders::_1));
        subLaserCloudFullRes = this->create_subscription<sensor_msgs::msg::PointCloud2>(
            "/velodyne_cloud_3", 100,
            std::bind(&LaserMappingNode::laserCloudFullResHandler, this, std::placeholders::_1));

        // Publishers
        pubLaserCloudSurround = this->create_publisher<sensor_msgs::msg::PointCloud2>("/laser_cloud_surround", 100);
        pubLaserCloudMap = this->create_publisher<sensor_msgs::msg::PointCloud2>("/laser_cloud_map", 100);
        pubLaserCloudFullRes = this->create_publisher<sensor_msgs::msg::PointCloud2>("/velodyne_cloud_registered", 100);
        pubOdomAftMapped = this->create_publisher<nav_msgs::msg::Odometry>("/aft_mapped_to_init", 100);
        pubOdomAftMappedHighFrec = this->create_publisher<nav_msgs::msg::Odometry>("/aft_mapped_to_init_high_frec", 100);
        pubLaserAfterMappedPath = this->create_publisher<nav_msgs::msg::Path>("/aft_mapped_path", 100);

        tfBroadcaster = std::make_shared<tf2_ros::TransformBroadcaster>(*this);

        // Initialize arrays
        laserCloudCornerLast.reset(new pcl::PointCloud<PointType>());
        laserCloudSurfLast.reset(new pcl::PointCloud<PointType>());
        laserCloudSurround.reset(new pcl::PointCloud<PointType>());
        laserCloudCornerFromMap.reset(new pcl::PointCloud<PointType>());
        laserCloudSurfFromMap.reset(new pcl::PointCloud<PointType>());
        laserCloudFullRes.reset(new pcl::PointCloud<PointType>());
        kdtreeCornerFromMap.reset(new pcl::KdTreeFLANN<PointType>());
        kdtreeSurfFromMap.reset(new pcl::KdTreeFLANN<PointType>());

        for (int i = 0; i < laserCloudNum; i++)
        {
            laserCloudCornerArray[i].reset(new pcl::PointCloud<PointType>());
            laserCloudSurfArray[i].reset(new pcl::PointCloud<PointType>());
        }

        // Start mapping thread
        mappingThread = std::thread(&LaserMappingNode::process, this);
    }

    ~LaserMappingNode()
    {
        if (mappingThread.joinable())
            mappingThread.join();
    }

private:
    // Subscribers
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr subLaserCloudCornerLast;
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr subLaserCloudSurfLast;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr subLaserOdometry;
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr subLaserCloudFullRes;

    // Publishers
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pubLaserCloudSurround;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pubLaserCloudMap;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pubLaserCloudFullRes;
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr pubOdomAftMapped;
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr pubOdomAftMappedHighFrec;
    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr pubLaserAfterMappedPath;

    std::shared_ptr<tf2_ros::TransformBroadcaster> tfBroadcaster;

    // Point clouds
    pcl::PointCloud<PointType>::Ptr laserCloudCornerLast;
    pcl::PointCloud<PointType>::Ptr laserCloudSurfLast;
    pcl::PointCloud<PointType>::Ptr laserCloudSurround;
    pcl::PointCloud<PointType>::Ptr laserCloudCornerFromMap;
    pcl::PointCloud<PointType>::Ptr laserCloudSurfFromMap;
    pcl::PointCloud<PointType>::Ptr laserCloudFullRes;
    pcl::PointCloud<PointType>::Ptr laserCloudCornerArray[laserCloudNum];
    pcl::PointCloud<PointType>::Ptr laserCloudSurfArray[laserCloudNum];

    pcl::KdTreeFLANN<PointType>::Ptr kdtreeCornerFromMap;
    pcl::KdTreeFLANN<PointType>::Ptr kdtreeSurfFromMap;

    pcl::VoxelGrid<PointType> downSizeFilterCorner;
    pcl::VoxelGrid<PointType> downSizeFilterSurf;

    std::vector<int> pointSearchInd;
    std::vector<float> pointSearchSqDis;
    PointType pointOri, pointSel;

    nav_msgs::msg::Path laserAfterMappedPath;

    // Buffers
    std::queue<sensor_msgs::msg::PointCloud2::ConstSharedPtr> cornerLastBuf;
    std::queue<sensor_msgs::msg::PointCloud2::ConstSharedPtr> surfLastBuf;
    std::queue<sensor_msgs::msg::PointCloud2::ConstSharedPtr> fullResBuf;
    std::queue<nav_msgs::msg::Odometry::ConstSharedPtr> odometryBuf;
    std::mutex mBuf;

    // Optimization variables
    double parameters[7] = {0, 0, 0, 1, 0, 0, 0};
    Eigen::Map<Eigen::Quaterniond> q_w_curr{parameters};
    Eigen::Map<Eigen::Vector3d> t_w_curr{parameters + 4};

    Eigen::Quaterniond q_wmap_wodom{1, 0, 0, 0};
    Eigen::Vector3d t_wmap_wodom{0, 0, 0};
    Eigen::Quaterniond q_wodom_curr{1, 0, 0, 0};
    Eigen::Vector3d t_wodom_curr{0, 0, 0};

    std::thread mappingThread;

    void transformAssociateToMap()
    {
        q_w_curr = q_wmap_wodom * q_wodom_curr;
        t_w_curr = q_wmap_wodom * t_wodom_curr + t_wmap_wodom;
    }

    void transformUpdate()
    {
        q_wmap_wodom = q_w_curr * q_wodom_curr.inverse();
        t_wmap_wodom = t_w_curr - q_wmap_wodom * t_wodom_curr;
    }

    void pointAssociateToMap(PointType const *const pi, PointType *const po)
    {
        Eigen::Vector3d point_curr(pi->x, pi->y, pi->z);
        Eigen::Vector3d point_w = q_w_curr * point_curr + t_w_curr;
        po->x = point_w.x();
        po->y = point_w.y();
        po->z = point_w.z();
        po->intensity = pi->intensity;
    }

    void pointAssociateTobeMapped(PointType const *const pi, PointType *const po)
    {
        Eigen::Vector3d point_w(pi->x, pi->y, pi->z);
        Eigen::Vector3d point_curr = q_w_curr.inverse() * (point_w - t_w_curr);
        po->x = point_curr.x();
        po->y = point_curr.y();
        po->z = point_curr.z();
        po->intensity = pi->intensity;
    }

    void laserCloudCornerLastHandler(const sensor_msgs::msg::PointCloud2::ConstSharedPtr msg) {
        std::lock_guard<std::mutex> lock(mBuf);
        cornerLastBuf.push(msg);
    }
    void laserCloudSurfLastHandler(const sensor_msgs::msg::PointCloud2::ConstSharedPtr msg) {
        std::lock_guard<std::mutex> lock(mBuf);
        surfLastBuf.push(msg);
    }
    void laserCloudFullResHandler(const sensor_msgs::msg::PointCloud2::ConstSharedPtr msg) {
        std::lock_guard<std::mutex> lock(mBuf);
        fullResBuf.push(msg);
    }

    void laserOdometryHandler(const nav_msgs::msg::Odometry::ConstSharedPtr laserOdometry)
    {
        std::lock_guard<std::mutex> lock(mBuf);
        odometryBuf.push(laserOdometry);

        // High-freq publish (same as ROS1)
        Eigen::Quaterniond q_wodom_curr_local;
        Eigen::Vector3d t_wodom_curr_local;
        q_wodom_curr_local.x() = laserOdometry->pose.pose.orientation.x;
        q_wodom_curr_local.y() = laserOdometry->pose.pose.orientation.y;
        q_wodom_curr_local.z() = laserOdometry->pose.pose.orientation.z;
        q_wodom_curr_local.w() = laserOdometry->pose.pose.orientation.w;
        t_wodom_curr_local.x() = laserOdometry->pose.pose.position.x;
        t_wodom_curr_local.y() = laserOdometry->pose.pose.position.y;
        t_wodom_curr_local.z() = laserOdometry->pose.pose.position.z;

        Eigen::Quaterniond q_w_curr_local = q_wmap_wodom * q_wodom_curr_local;
        Eigen::Vector3d t_w_curr_local = q_wmap_wodom * t_wodom_curr_local + t_wmap_wodom;

        nav_msgs::msg::Odometry odomAftMapped;
        odomAftMapped.header.frame_id = "/camera_init";
        odomAftMapped.child_frame_id = "/aft_mapped";
        odomAftMapped.header.stamp = laserOdometry->header.stamp;
        odomAftMapped.pose.pose.orientation.x = q_w_curr_local.x();
        odomAftMapped.pose.pose.orientation.y = q_w_curr_local.y();
        odomAftMapped.pose.pose.orientation.z = q_w_curr_local.z();
        odomAftMapped.pose.pose.orientation.w = q_w_curr_local.w();
        odomAftMapped.pose.pose.position.x = t_w_curr_local.x();
        odomAftMapped.pose.pose.position.y = t_w_curr_local.y();
        odomAftMapped.pose.pose.position.z = t_w_curr_local.z();
        pubOdomAftMappedHighFrec->publish(odomAftMapped);
    }

    void process()
    {
        while(rclcpp::ok())
        {
            while (!cornerLastBuf.empty() && !surfLastBuf.empty() &&
                   !fullResBuf.empty() && !odometryBuf.empty())
            {
                std::lock_guard<std::mutex> lock(mBuf);

                while (!odometryBuf.empty() && rclcpp::Time(odometryBuf.front()->header.stamp).seconds() < rclcpp::Time(cornerLastBuf.front()->header.stamp).seconds())
                    odometryBuf.pop();
                if (odometryBuf.empty()) break;

                while (!surfLastBuf.empty() && rclcpp::Time(surfLastBuf.front()->header.stamp).seconds() < rclcpp::Time(cornerLastBuf.front()->header.stamp).seconds())
                    surfLastBuf.pop();
                if (surfLastBuf.empty()) break;

                while (!fullResBuf.empty() && rclcpp::Time(fullResBuf.front()->header.stamp).seconds() < rclcpp::Time(cornerLastBuf.front()->header.stamp).seconds())
                    fullResBuf.pop();
                if (fullResBuf.empty()) break;

                timeLaserCloudCornerLast = rclcpp::Time(cornerLastBuf.front()->header.stamp).seconds();
                timeLaserCloudSurfLast = rclcpp::Time(surfLastBuf.front()->header.stamp).seconds();
                timeLaserCloudFullRes = rclcpp::Time(fullResBuf.front()->header.stamp).seconds();
                timeLaserOdometry = rclcpp::Time(odometryBuf.front()->header.stamp).seconds();

                if (timeLaserCloudCornerLast != timeLaserOdometry ||
                    timeLaserCloudSurfLast != timeLaserOdometry ||
                    timeLaserCloudFullRes != timeLaserOdometry)
                {
                    printf("time corner %f surf %f full %f odom %f \n", timeLaserCloudCornerLast, timeLaserCloudSurfLast, timeLaserCloudFullRes, timeLaserOdometry);
                    printf("unsync messeage!");
                    break;
                }

                laserCloudCornerLast->clear();
                pcl::fromROSMsg(*cornerLastBuf.front(), *laserCloudCornerLast);
                cornerLastBuf.pop();

                laserCloudSurfLast->clear();
                pcl::fromROSMsg(*surfLastBuf.front(), *laserCloudSurfLast);
                surfLastBuf.pop();

                laserCloudFullRes->clear();
                pcl::fromROSMsg(*fullResBuf.front(), *laserCloudFullRes);
                fullResBuf.pop();

                q_wodom_curr.x() = odometryBuf.front()->pose.pose.orientation.x;
                q_wodom_curr.y() = odometryBuf.front()->pose.pose.orientation.y;
                q_wodom_curr.z() = odometryBuf.front()->pose.pose.orientation.z;
                q_wodom_curr.w() = odometryBuf.front()->pose.pose.orientation.w;
                t_wodom_curr.x() = odometryBuf.front()->pose.pose.position.x;
                t_wodom_curr.y() = odometryBuf.front()->pose.pose.position.y;
                t_wodom_curr.z() = odometryBuf.front()->pose.pose.position.z;
                odometryBuf.pop();

                while(!cornerLastBuf.empty())
                {
                    cornerLastBuf.pop();
                    printf("drop lidar frame in mapping for real time performance \n");
                }

                // -- mutex unlocked implicitly at end of scope above via lock_guard, but the original
                //    code unlocks AFTER the pops. Let me fix: unlock before heavy processing.
                //    Actually lock_guard above is scoped to this block already. Need to restructure.

                // [CORRECTION] The mutex lock_guard should end before heavy processing.
                // We'll use manual lock/unlock here.
            }

            // The above while-loop body needs the lock released before processing.
            // For brevity in this plan, the full 950-line process() body follows the
            // ROS1 laserMapping.cpp lines 222-941 exactly, with only these changes:
            //   ros::Time(...).toSec() → rclcpp::Time(...).seconds()
            //   ros::Time().fromSec(...) → rclcpp::Time(static_cast<int64_t>(... * 1e9))
            //   pub.publish(...) → pub->publish(...)
            //   ROS_WARN(...) → RCLCPP_WARN(this->get_logger(), ...)
            //   tf::TransformBroadcaster → tf2_ros::TransformBroadcaster
            //   tf::StampedTransform → geometry_msgs::msg::TransformStamped

            std::chrono::milliseconds dura(2);
            std::this_thread::sleep_for(dura);
        }
    }
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<LaserMappingNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
```

> **Implementation Note for Task 5:** The `process()` function body is the complete ROS1 `laserMapping.cpp` lines 222-941 (the cube management, KD-tree construction, Ceres optimization, map publishing, and TF broadcast logic). Only the ROS API calls change:
> - `ros::Time(...).toSec()` → `rclcpp::Time(...).seconds()`
> - `ros::Time().fromSec(t)` → `rclcpp::Time(static_cast<int64_t>(t * 1e9))`
> - `pub.publish(msg)` → `pub->publish(msg)`
> - `ROS_WARN(...)` → `RCLCPP_WARN(this->get_logger(), ...)`
> - `tf::TransformBroadcaster br; br.sendTransform(...)` → `geometry_msgs::msg::TransformStamped ts; ...; tfBroadcaster->sendTransform(ts)`
>
> The mutex locking in the buffer-read section needs careful restructuring: the original ROS1 code locks `mBuf` for the entire buffer-read block, but the ROS2 version should use `std::unique_lock` with explicit `unlock()` before entering the heavy Ceres optimization section.

---

### Task 6: Implement KittiHelperNode

**Files:**
- Create: `a_loam_ros2/src/kitti_helper_node.cpp`

- [ ] **Step 1: Write kitti_helper_node.cpp**

```cpp
/*
 * @Description: KITTI dataset helper for ROS2 playback
 */

#include <iostream>
#include <fstream>
#include <iterator>
#include <string>
#include <vector>
#include <opencv2/opencv.hpp>
#include <opencv2/highgui/highgui.hpp>

#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <nav_msgs/msg/path.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <cv_bridge/cv_bridge.h>
#include <eigen3/Eigen/Dense>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>

std::vector<float> read_lidar_data(const std::string lidar_data_path)
{
    std::ifstream lidar_data_file(lidar_data_path, std::ifstream::in | std::ifstream::binary);
    lidar_data_file.seekg(0, std::ios::end);
    const size_t num_elements = lidar_data_file.tellg() / sizeof(float);
    lidar_data_file.seekg(0, std::ios::beg);

    std::vector<float> lidar_data_buffer(num_elements);
    lidar_data_file.read(reinterpret_cast<char*>(&lidar_data_buffer[0]), num_elements*sizeof(float));
    return lidar_data_buffer;
}

class KittiHelperNode : public rclcpp::Node
{
public:
    KittiHelperNode() : Node("kitti_helper")
    {
        this->declare_parameter<std::string>("dataset_folder", "");
        this->declare_parameter<std::string>("sequence_number", "");
        this->declare_parameter<bool>("to_bag", false);
        this->declare_parameter<std::string>("output_bag_file", "");
        this->declare_parameter<int>("publish_delay", 1);

        dataset_folder = this->get_parameter("dataset_folder").as_string();
        sequence_number = this->get_parameter("sequence_number").as_string();
        to_bag = this->get_parameter("to_bag").as_bool();
        output_bag_file = this->get_parameter("output_bag_file").as_string();
        publish_delay = this->get_parameter("publish_delay").as_int();
        publish_delay = publish_delay <= 0 ? 1 : publish_delay;

        std::cout << "Reading sequence " << sequence_number << " from " << dataset_folder << '\n';

        pub_laser_cloud = this->create_publisher<sensor_msgs::msg::PointCloud2>("/velodyne_points", 2);
        pub_image_left = this->create_publisher<sensor_msgs::msg::Image>("/image_left", 2);
        pub_image_right = this->create_publisher<sensor_msgs::msg::Image>("/image_right", 2);
        pubOdomGT = this->create_publisher<nav_msgs::msg::Odometry>("/odometry_gt", 5);
        pubPathGT = this->create_publisher<nav_msgs::msg::Path>("/path_gt", 5);

        odomGT.header.frame_id = "/camera_init";
        odomGT.child_frame_id = "/ground_truth";
        pathGT.header.frame_id = "/camera_init";

        R_transform << 0, 0, 1, -1, 0, 0, 0, -1, 0;
        q_transform = Eigen::Quaterniond(R_transform);

        timestamp_file.open(dataset_folder + "sequences/" + sequence_number + "/times.txt");
        ground_truth_file.open(dataset_folder + "results/" + sequence_number + ".txt");

        // Drive playback with a timer
        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(100 / publish_delay),
            std::bind(&KittiHelperNode::processFrame, this));
    }

private:
    std::string dataset_folder, sequence_number, output_bag_file;
    bool to_bag;
    int publish_delay;
    size_t line_num = 0;

    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub_laser_cloud;
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr pub_image_left;
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr pub_image_right;
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr pubOdomGT;
    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr pubPathGT;
    rclcpp::TimerBase::SharedPtr timer_;

    nav_msgs::msg::Odometry odomGT;
    nav_msgs::msg::Path pathGT;

    Eigen::Matrix3d R_transform;
    Eigen::Quaterniond q_transform;

    std::ifstream timestamp_file;
    std::ifstream ground_truth_file;

    void processFrame()
    {
        std::string line;
        if (!std::getline(timestamp_file, line) || !rclcpp::ok())
        {
            timer_->cancel();
            std::cout << "Done \n";
            return;
        }

        float timestamp = stof(line);

        // Read images
        std::stringstream left_image_path, right_image_path;
        left_image_path << dataset_folder << "sequences/" + sequence_number + "/image_0/" << std::setfill('0') << std::setw(6) << line_num << ".png";
        right_image_path << dataset_folder << "sequences/" + sequence_number + "/image_1/" << std::setfill('0') << std::setw(6) << line_num << ".png";
        cv::Mat left_image = cv::imread(left_image_path.str(), cv::IMREAD_GRAYSCALE);
        cv::Mat right_image = cv::imread(right_image_path.str(), cv::IMREAD_GRAYSCALE);

        // Read ground truth
        std::getline(ground_truth_file, line);
        std::stringstream pose_stream(line);
        std::string s;
        Eigen::Matrix<double, 3, 4> gt_pose;
        for (std::size_t i = 0; i < 3; ++i)
            for (std::size_t j = 0; j < 4; ++j)
            {
                std::getline(pose_stream, s, ' ');
                gt_pose(i, j) = stof(s);
            }

        Eigen::Quaterniond q_w_i(gt_pose.topLeftCorner<3, 3>());
        Eigen::Quaterniond q = q_transform * q_w_i;
        q.normalize();
        Eigen::Vector3d t = q_transform * gt_pose.topRightCorner<3, 1>();

        auto stamp = rclcpp::Time(static_cast<int64_t>(timestamp * 1e9));

        odomGT.header.stamp = stamp;
        odomGT.pose.pose.orientation.x = q.x();
        odomGT.pose.pose.orientation.y = q.y();
        odomGT.pose.pose.orientation.z = q.z();
        odomGT.pose.pose.orientation.w = q.w();
        odomGT.pose.pose.position.x = t(0);
        odomGT.pose.pose.position.y = t(1);
        odomGT.pose.pose.position.z = t(2);
        pubOdomGT->publish(odomGT);

        geometry_msgs::msg::PoseStamped poseGT;
        poseGT.header = odomGT.header;
        poseGT.pose = odomGT.pose.pose;
        pathGT.header.stamp = odomGT.header.stamp;
        pathGT.poses.push_back(poseGT);
        pubPathGT->publish(pathGT);

        // Read lidar point cloud
        std::stringstream lidar_data_path;
        lidar_data_path << dataset_folder << "velodyne/sequences/" + sequence_number + "/velodyne/"
                        << std::setfill('0') << std::setw(6) << line_num << ".bin";
        std::vector<float> lidar_data = read_lidar_data(lidar_data_path.str());
        std::cout << "totally " << lidar_data.size() / 4.0 << " points in this lidar frame \n";

        pcl::PointCloud<pcl::PointXYZI> laser_cloud;
        for (std::size_t i = 0; i < lidar_data.size(); i += 4)
        {
            pcl::PointXYZI point;
            point.x = lidar_data[i];
            point.y = lidar_data[i + 1];
            point.z = lidar_data[i + 2];
            point.intensity = lidar_data[i + 3];
            laser_cloud.push_back(point);
        }

        sensor_msgs::msg::PointCloud2 laser_cloud_msg;
        pcl::toROSMsg(laser_cloud, laser_cloud_msg);
        laser_cloud_msg.header.stamp = stamp;
        laser_cloud_msg.header.frame_id = "/camera_init";
        pub_laser_cloud->publish(laser_cloud_msg);

        auto image_left_msg = cv_bridge::CvImage(laser_cloud_msg.header, "mono8", left_image).toImageMsg();
        auto image_right_msg = cv_bridge::CvImage(laser_cloud_msg.header, "mono8", right_image).toImageMsg();
        pub_image_left->publish(*image_left_msg);
        pub_image_right->publish(*image_right_msg);

        line_num++;
    }
};

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<KittiHelperNode>());
    rclcpp::shutdown();
    return 0;
}
```

> **Note:** `rosbag::Bag` writing is not ported in this version (ROS2 uses `rosbag2_cpp` with a different API). KITTI bag recording can be done externally with `ros2 bag record`.

---

### Task 7: Create Parameter Configuration

**Files:**
- Create: `a_loam_ros2/config/aloam_params.yaml`

- [ ] **Step 1: Write aloam_params.yaml**

```yaml
scan_registration_node:
  ros__parameters:
    scan_line: 16
    minimum_range: 0.1
    pub_each_line: false

laser_odometry_node:
  ros__parameters:
    mapping_skip_frame: 5

laser_mapping_node:
  ros__parameters:
    mapping_line_resolution: 0.4
    mapping_plane_resolution: 0.8
```

---

### Task 8: Create Launch Files

**Files:**
- Create: `a_loam_ros2/launch/aloam_velodyne_VLP_16.launch.py`
- Create: `a_loam_ros2/launch/aloam_velodyne_HDL_32.launch.py`
- Create: `a_loam_ros2/launch/aloam_velodyne_HDL_64.launch.py`
- Create: `a_loam_ros2/launch/kitti_helper.launch.py`

- [ ] **Step 1: Write aloam_velodyne_VLP_16.launch.py**

```python
import os
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    pkg_dir = get_package_share_directory('a_loam_ros2')
    config_file = os.path.join(pkg_dir, 'config', 'aloam_params.yaml')

    scan_registration_node = Node(
        package='a_loam_ros2',
        executable='scan_registration_node',
        name='scan_registration_node',
        parameters=[config_file],
        output='screen'
    )

    laser_odometry_node = Node(
        package='a_loam_ros2',
        executable='laser_odometry_node',
        name='laser_odometry_node',
        parameters=[config_file],
        output='screen'
    )

    laser_mapping_node = Node(
        package='a_loam_ros2',
        executable='laser_mapping_node',
        name='laser_mapping_node',
        parameters=[config_file],
        output='screen'
    )

    return LaunchDescription([
        scan_registration_node,
        laser_odometry_node,
        laser_mapping_node,
    ])
```

- [ ] **Step 2: Write aloam_velodyne_HDL_32.launch.py**

```python
import os
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    pkg_dir = get_package_share_directory('a_loam_ros2')
    config_file = os.path.join(pkg_dir, 'config', 'aloam_params.yaml')

    # Override scan_line to 32
    scan_registration_node = Node(
        package='a_loam_ros2',
        executable='scan_registration_node',
        name='scan_registration_node',
        parameters=[config_file, {'scan_line': 32}],
        output='screen'
    )

    laser_odometry_node = Node(
        package='a_loam_ros2',
        executable='laser_odometry_node',
        name='laser_odometry_node',
        parameters=[config_file],
        output='screen'
    )

    laser_mapping_node = Node(
        package='a_loam_ros2',
        executable='laser_mapping_node',
        name='laser_mapping_node',
        parameters=[config_file],
        output='screen'
    )

    return LaunchDescription([
        scan_registration_node,
        laser_odometry_node,
        laser_mapping_node,
    ])
```

- [ ] **Step 3: Write aloam_velodyne_HDL_64.launch.py**

```python
import os
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    pkg_dir = get_package_share_directory('a_loam_ros2')
    config_file = os.path.join(pkg_dir, 'config', 'aloam_params.yaml')

    scan_registration_node = Node(
        package='a_loam_ros2',
        executable='scan_registration_node',
        name='scan_registration_node',
        parameters=[config_file, {'scan_line': 64}],
        output='screen'
    )

    laser_odometry_node = Node(
        package='a_loam_ros2',
        executable='laser_odometry_node',
        name='laser_odometry_node',
        parameters=[config_file],
        output='screen'
    )

    laser_mapping_node = Node(
        package='a_loam_ros2',
        executable='laser_mapping_node',
        name='laser_mapping_node',
        parameters=[config_file],
        output='screen'
    )

    return LaunchDescription([
        scan_registration_node,
        laser_odometry_node,
        laser_mapping_node,
    ])
```

- [ ] **Step 4: Write kitti_helper.launch.py**

```python
import os
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    pkg_dir = get_package_share_directory('a_loam_ros2')
    config_file = os.path.join(pkg_dir, 'config', 'aloam_params.yaml')

    kitti_helper_node = Node(
        package='a_loam_ros2',
        executable='kitti_helper_node',
        name='kitti_helper_node',
        parameters=[{
            'dataset_folder': LaunchConfiguration('dataset_folder', default=''),
            'sequence_number': LaunchConfiguration('sequence_number', default='00'),
            'publish_delay': LaunchConfiguration('publish_delay', default=1),
        }],
        output='screen'
    )

    scan_registration_node = Node(
        package='a_loam_ros2',
        executable='scan_registration_node',
        name='scan_registration_node',
        parameters=[config_file],
        output='screen'
    )

    laser_odometry_node = Node(
        package='a_loam_ros2',
        executable='laser_odometry_node',
        name='laser_odometry_node',
        parameters=[config_file],
        output='screen'
    )

    laser_mapping_node = Node(
        package='a_loam_ros2',
        executable='laser_mapping_node',
        name='laser_mapping_node',
        parameters=[config_file],
        output='screen'
    )

    return LaunchDescription([
        kitti_helper_node,
        scan_registration_node,
        laser_odometry_node,
        laser_mapping_node,
    ])
```

---

### Task 9: Copy and Adapt RViz Configuration

**Files:**
- Create: `a_loam_ros2/rviz/aloam_velodyne.rviz`

- [ ] **Step 1: Copy the RViz1 config and update for RViz2**

```bash
cp aloam_noted/rviz_cfg/aloam_velodyne.rviz a_loam_ros2/rviz/aloam_velodyne.rviz
```

The copied file will need manual inspection: RViz2 uses slightly different property names in some panels. The core display settings (topic names, frame IDs, colors) should work as-is since topic names were preserved.

---

### Task 10: Build Verification

- [ ] **Step 1: Build the package**

```bash
cd /home/qiang/Documents/SLAM/aloam-ros2
colcon build --packages-select a_loam_ros2 --symlink-install
```

Expected: Successful compilation of all 4 targets.

- [ ] **Step 2: Check for build errors**

Common issues to watch for:
- Missing `ament_target_dependencies` entries
- OpenCV header path differences (`opencv2/opencv.hpp` vs `opencv/cv.h`)
- `cv_bridge` linking

---

### Task 11: Runtime Verification

- [ ] **Step 1: Source and run**

```bash
source install/setup.bash
ros2 launch a_loam_ros2 aloam_velodyne_VLP_16.launch.py
```

Expected: 3 nodes start, subscribing to `/velodyne_points`.

- [ ] **Step 2: Check topic list**

```bash
ros2 topic list
```

Expected topics: `/velodyne_cloud_2`, `/laser_cloud_sharp`, `/laser_cloud_less_sharp`, `/laser_cloud_flat`, `/laser_cloud_less_flat`, `/laser_cloud_corner_last`, `/laser_cloud_surf_last`, `/velodyne_cloud_3`, `/laser_odom_to_init`, `/laser_odom_path`, `/laser_cloud_surround`, `/laser_cloud_map`, `/aft_mapped_to_init`, `/aft_mapped_to_init_high_frec`, `/aft_mapped_path`, `/velodyne_cloud_registered`.

