A-LOAM ROS2 迁移项目需求文档

1. 项目概述

1.1 项目背景

A-LOAM（Advanced LOAM）是LOAM（Lidar Odometry and Mapping）算法的简化版本，采用Eigen和Ceres库替代了原始LOAM中的手动实现，是学习3D激光SLAM的经典入门项目。原项目基于ROS1开发，随着ROS2生态的成熟，将其迁移至ROS2框架具有重要的工程实践意义。

1.2 项目目标

将A-LOAM从ROS1完整迁移到ROS2，保持算法功能与原版完全一致，同时利用ROS2的现代化特性提升系统的可靠性和可维护性。

1.3 功能一致性要求

• 算法核心功能不变：特征点提取、scan-to-scan里程计、scan-to-map后端优化流程保持不变

• 输入输出接口一致：保持相同的点云输入格式和位姿输出格式

• 性能指标对齐：确保ROS2版本的实时性、精度与原版相当

2. 系统架构分析

2.1 原ROS1版本架构

A-LOAM采用三节点架构：

节点名称 对应文件 主要功能

scanRegistration scanRegistration.cpp 处理原始LiDAR点云，提取角点和平面点特征

laserOdometry laserOdometry.cpp 接收特征点，计算帧间位姿（前端里程计）

laserMapping laserMapping.cpp 接收点云和里程计位姿，进行后端优化建图

2.2 ROS通信拓扑


原始点云 → scanRegistration → 特征点云 → laserOdometry → 粗粒度位姿 → laserMapping
      ↓                            ↓                          ↓
发布完整点云                   发布前端里程计位姿           发布精化位姿和地图


3. ROS1到ROS2迁移关键技术点

3.1 通信机制重构

• DDS替代集中式Master：ROS2采用真正的分布式通信架构

• 消息接口变更：

  • ROS1: sensor_msgs::PointCloud2

  • ROS2: sensor_msgs::msg::PointCloud2

  • 自定义消息头文件从.h变为.hpp

3.2 节点生命周期管理

• 从roscpp到rclcpp：采用基于生命周期的节点管理

• 智能指针使用：ROS2大量使用智能指针，类初始化方式需要调整

3.3 参数系统升级

• 参数声明机制：使用ROS2新的参数声明与获取接口

• 动态参数配置：支持运行时参数调整

3.4 时间系统适配

• 时间接口统一：ros::Time → rclcpp::Time

• 时间单位处理：注意ROS1与ROS2时间单位的差异

4. 详细迁移方案

4.1 文件结构重构


a_loam_ros2/
├── CMakeLists.txt          # ament构建系统配置
├── package.xml            # ROS2包描述文件
├── include/               # 头文件
├── src/
│   ├── scan_registration_node.cpp
│   ├── laser_odometry_node.cpp
│   └── laser_mapping_node.cpp
├── launch/                # ROS2 launch文件
├── config/                # 参数配置文件
└── rviz/                  # RViz2配置文件


4.2 核心代码修改要点

4.2.1 头文件包含

// ROS1
#include "ros/ros.h"
#include "sensor_msgs/PointCloud2.h"

// ROS2
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"


4.2.2 节点类定义

// ROS1风格
int main(int argc, char** argv) {
    ros::init(argc, argv, "scan_registration");
    ros::NodeHandle nh;
    // ...
}

// ROS2风格
class ScanRegistrationNode : public rclcpp::Node {
public:
    ScanRegistrationNode() : Node("scan_registration") {
        // 初始化
    }
};


4.2.3 话题发布/订阅

// ROS1发布
ros::Publisher pub = nh.advertise<sensor_msgs::PointCloud2>("topic", 10);
pub.publish(msg);

// ROS2发布
auto pub = this->create_publisher<sensor_msgs::msg::PointCloud2>("topic", 10);
pub->publish(msg);


4.2.4 参数获取

// ROS1参数
ros::NodeHandle private_nh("~");
private_nh.param<int>("skip_frame_num", skipFrameNum, 1);

// ROS2参数
this->declare_parameter<int>("skip_frame_num", 1);
skipFrameNum = this->get_parameter("skip_frame_num").as_int();


4.3 构建系统迁移

4.3.1 CMakeLists.txt关键配置

# ROS1 (catkin)
find_package(catkin REQUIRED COMPONENTS
  roscpp
  sensor_msgs
  pcl_ros
)

# ROS2 (ament)
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(pcl_conversions REQUIRED)  # 替代pcl_ros


4.3.2 package.xml依赖声明

<!-- ROS2依赖 -->
<depend>rclcpp</depend>
<depend>sensor_msgs</depend>
<depend>tf2_ros</depend>
<depend>pcl_conversions</depend>
<depend>Eigen3</depend>
<depend>libceres-dev</depend>


5. 功能模块详细需求

5.1 scanRegistration模块

• 输入：原始Velodyne点云（/velodyne_points）

• 输出：

  • 角点特征（/laser_cloud_sharp）

  • 平面点特征（/laser_cloud_flat）

  • 降采样完整点云（/velodyne_cloud_3）

• 核心算法：曲率计算、特征点提取、点云去畸变

5.2 laserOdometry模块

• 输入：角点和平面点特征

• 输出：

  • 前端里程计位姿（/laser_odom_to_init）

  • 前端路径（/laser_odom_path）

• 核心算法：Ceres优化、帧间匹配、运动估计

5.3 laserMapping模块

• 输入：完整点云、特征点、前端位姿

• 输出：

  • 精化位姿（/aft_mapped_to_init）

  • 精化路径（/aft_mapped_path）

  • 局部地图（/laser_cloud_surround）

  • 全局地图（/laser_cloud_map）

• 核心算法：scan-to-map匹配、位姿优化、地图更新

6. 非功能性需求

6.1 性能要求

• 实时性：处理频率≥10Hz（与原始版本一致）

• 内存使用：与ROS1版本相当

• CPU占用：优化后的ROS2版本不应显著增加计算负担

6.2 兼容性要求

• ROS2版本：支持Humble、Iron等主流LTS版本

• 传感器兼容：保持对Velodyne、Ouster等主流激光雷达的支持

• 点云库：PCL 1.10+兼容

6.3 可维护性

• 代码结构：模块化设计，便于后续扩展

• 文档完整：提供完整的API文档和迁移说明

• 测试覆盖：单元测试覆盖核心算法模块

7. 测试验证方案

7.1 功能测试

1. 数据回放测试：使用标准bag文件验证算法输出一致性
2. 实时数据测试：连接真实激光雷达验证实时处理能力
3. 精度对比测试：与ROS1版本在相同数据集上对比轨迹精度

7.2 性能测试

1. 处理延迟测试：测量从点云输入到位姿输出的端到端延迟
2. 资源占用测试：监控CPU、内存使用情况
3. 稳定性测试：长时间运行测试系统稳定性

7.3 兼容性测试

1. 多ROS2版本测试：在不同ROS2发行版上验证兼容性
2. 多传感器测试：验证对不同激光雷达型号的兼容性

8. 开发与部署计划

8.1 开发环境

• 操作系统：Ubuntu 22.04 LTS

• ROS2版本：ROS2 Humble Hawksbill

• 依赖库：

  • Ceres Solver 2.0+

  • PCL 1.12

  • Eigen 3.4

  • OpenCV 4.5+

9. 参考资料

1. A-LOAM原项目仓库：https://github.com/HKUST-Aerial-Robotics/A-LOAM
2. ROS2迁移最佳实践：从ROS1到ROS2的架构差异分析
3. 已有ROS2移植实践：A-LOAM项目ROS2移植的技术方案
4. ROS2官方文档：节点、话题、参数等核心概念