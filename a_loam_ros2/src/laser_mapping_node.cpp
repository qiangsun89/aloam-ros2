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
#include <geometry_msgs/msg/transform_stamped.hpp>

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

        pubLaserCloudSurround = this->create_publisher<sensor_msgs::msg::PointCloud2>("/laser_cloud_surround", 100);
        pubLaserCloudMap = this->create_publisher<sensor_msgs::msg::PointCloud2>("/laser_cloud_map", 100);
        pubLaserCloudFullRes = this->create_publisher<sensor_msgs::msg::PointCloud2>("/velodyne_cloud_registered", 100);
        pubOdomAftMapped = this->create_publisher<nav_msgs::msg::Odometry>("/aft_mapped_to_init", 100);
        pubOdomAftMappedHighFrec = this->create_publisher<nav_msgs::msg::Odometry>("/aft_mapped_to_init_high_frec", 100);
        pubLaserAfterMappedPath = this->create_publisher<nav_msgs::msg::Path>("/aft_mapped_path", 100);

        tfBroadcaster = std::make_shared<tf2_ros::TransformBroadcaster>(*this);

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

        mappingThread = std::thread(&LaserMappingNode::process, this);
    }

    ~LaserMappingNode()
    {
        if (mappingThread.joinable())
            mappingThread.join();
    }

private:
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr subLaserCloudCornerLast;
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr subLaserCloudSurfLast;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr subLaserOdometry;
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr subLaserCloudFullRes;

    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pubLaserCloudSurround;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pubLaserCloudMap;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pubLaserCloudFullRes;
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr pubOdomAftMapped;
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr pubOdomAftMappedHighFrec;
    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr pubLaserAfterMappedPath;

    std::shared_ptr<tf2_ros::TransformBroadcaster> tfBroadcaster;

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

    std::queue<sensor_msgs::msg::PointCloud2::ConstSharedPtr> cornerLastBuf;
    std::queue<sensor_msgs::msg::PointCloud2::ConstSharedPtr> surfLastBuf;
    std::queue<sensor_msgs::msg::PointCloud2::ConstSharedPtr> fullResBuf;
    std::queue<nav_msgs::msg::Odometry::ConstSharedPtr> odometryBuf;
    std::mutex mBuf;

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
                mBuf.lock();
                while (!odometryBuf.empty() && rclcpp::Time(odometryBuf.front()->header.stamp).seconds() < rclcpp::Time(cornerLastBuf.front()->header.stamp).seconds())
                    odometryBuf.pop();
                if (odometryBuf.empty())
                {
                    mBuf.unlock();
                    break;
                }

                while (!surfLastBuf.empty() && rclcpp::Time(surfLastBuf.front()->header.stamp).seconds() < rclcpp::Time(cornerLastBuf.front()->header.stamp).seconds())
                    surfLastBuf.pop();
                if (surfLastBuf.empty())
                {
                    mBuf.unlock();
                    break;
                }

                while (!fullResBuf.empty() && rclcpp::Time(fullResBuf.front()->header.stamp).seconds() < rclcpp::Time(cornerLastBuf.front()->header.stamp).seconds())
                    fullResBuf.pop();
                if (fullResBuf.empty())
                {
                    mBuf.unlock();
                    break;
                }

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
                    mBuf.unlock();
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

                mBuf.unlock();

                TicToc t_whole;

                transformAssociateToMap();

                TicToc t_shift;
                int centerCubeI = int((t_w_curr.x() + 25.0) / 50.0) + laserCloudCenWidth;
                int centerCubeJ = int((t_w_curr.y() + 25.0) / 50.0) + laserCloudCenHeight;
                int centerCubeK = int((t_w_curr.z() + 25.0) / 50.0) + laserCloudCenDepth;

                if (t_w_curr.x() + 25.0 < 0)
                    centerCubeI--;
                if (t_w_curr.y() + 25.0 < 0)
                    centerCubeJ--;
                if (t_w_curr.z() + 25.0 < 0)
                    centerCubeK--;

                while (centerCubeI < 3)
                {
                    for (int j = 0; j < laserCloudHeight; j++)
                    {
                        for (int k = 0; k < laserCloudDepth; k++)
                        {
                            int i = laserCloudWidth - 1;
                            pcl::PointCloud<PointType>::Ptr laserCloudCubeCornerPointer =
                                laserCloudCornerArray[i + laserCloudWidth * j + laserCloudWidth * laserCloudHeight * k];
                            pcl::PointCloud<PointType>::Ptr laserCloudCubeSurfPointer =
                                laserCloudSurfArray[i + laserCloudWidth * j + laserCloudWidth * laserCloudHeight * k];
                            for (; i >= 1; i--)
                            {
                                laserCloudCornerArray[i + laserCloudWidth * j + laserCloudWidth * laserCloudHeight * k] =
                                    laserCloudCornerArray[i - 1 + laserCloudWidth * j + laserCloudWidth * laserCloudHeight * k];
                                laserCloudSurfArray[i + laserCloudWidth * j + laserCloudWidth * laserCloudHeight * k] =
                                    laserCloudSurfArray[i - 1 + laserCloudWidth * j + laserCloudWidth * laserCloudHeight * k];
                            }
                            laserCloudCornerArray[i + laserCloudWidth * j + laserCloudWidth * laserCloudHeight * k] =
                                laserCloudCubeCornerPointer;
                            laserCloudSurfArray[i + laserCloudWidth * j + laserCloudWidth * laserCloudHeight * k] =
                                laserCloudCubeSurfPointer;
                            laserCloudCubeCornerPointer->clear();
                            laserCloudCubeSurfPointer->clear();
                        }
                    }

                    centerCubeI++;
                    laserCloudCenWidth++;
                }

                while (centerCubeI >= laserCloudWidth - 3)
                {
                    for (int j = 0; j < laserCloudHeight; j++)
                    {
                        for (int k = 0; k < laserCloudDepth; k++)
                        {
                            int i = 0;
                            pcl::PointCloud<PointType>::Ptr laserCloudCubeCornerPointer =
                                laserCloudCornerArray[i + laserCloudWidth * j + laserCloudWidth * laserCloudHeight * k];
                            pcl::PointCloud<PointType>::Ptr laserCloudCubeSurfPointer =
                                laserCloudSurfArray[i + laserCloudWidth * j + laserCloudWidth * laserCloudHeight * k];
                            for (; i < laserCloudWidth - 1; i++)
                            {
                                laserCloudCornerArray[i + laserCloudWidth * j + laserCloudWidth * laserCloudHeight * k] =
                                    laserCloudCornerArray[i + 1 + laserCloudWidth * j + laserCloudWidth * laserCloudHeight * k];
                                laserCloudSurfArray[i + laserCloudWidth * j + laserCloudWidth * laserCloudHeight * k] =
                                    laserCloudSurfArray[i + 1 + laserCloudWidth * j + laserCloudWidth * laserCloudHeight * k];
                            }
                            laserCloudCornerArray[i + laserCloudWidth * j + laserCloudWidth * laserCloudHeight * k] =
                                laserCloudCubeCornerPointer;
                            laserCloudSurfArray[i + laserCloudWidth * j + laserCloudWidth * laserCloudHeight * k] =
                                laserCloudCubeSurfPointer;
                            laserCloudCubeCornerPointer->clear();
                            laserCloudCubeSurfPointer->clear();
                        }
                    }

                    centerCubeI--;
                    laserCloudCenWidth--;
                }

                while (centerCubeJ < 3)
                {
                    for (int i = 0; i < laserCloudWidth; i++)
                    {
                        for (int k = 0; k < laserCloudDepth; k++)
                        {
                            int j = laserCloudHeight - 1;
                            pcl::PointCloud<PointType>::Ptr laserCloudCubeCornerPointer =
                                laserCloudCornerArray[i + laserCloudWidth * j + laserCloudWidth * laserCloudHeight * k];
                            pcl::PointCloud<PointType>::Ptr laserCloudCubeSurfPointer =
                                laserCloudSurfArray[i + laserCloudWidth * j + laserCloudWidth * laserCloudHeight * k];
                            for (; j >= 1; j--)
                            {
                                laserCloudCornerArray[i + laserCloudWidth * j + laserCloudWidth * laserCloudHeight * k] =
                                    laserCloudCornerArray[i + laserCloudWidth * (j - 1) + laserCloudWidth * laserCloudHeight * k];
                                laserCloudSurfArray[i + laserCloudWidth * j + laserCloudWidth * laserCloudHeight * k] =
                                    laserCloudSurfArray[i + laserCloudWidth * (j - 1) + laserCloudWidth * laserCloudHeight * k];
                            }
                            laserCloudCornerArray[i + laserCloudWidth * j + laserCloudWidth * laserCloudHeight * k] =
                                laserCloudCubeCornerPointer;
                            laserCloudSurfArray[i + laserCloudWidth * j + laserCloudWidth * laserCloudHeight * k] =
                                laserCloudCubeSurfPointer;
                            laserCloudCubeCornerPointer->clear();
                            laserCloudCubeSurfPointer->clear();
                        }
                    }

                    centerCubeJ++;
                    laserCloudCenHeight++;
                }

                while (centerCubeJ >= laserCloudHeight - 3)
                {
                    for (int i = 0; i < laserCloudWidth; i++)
                    {
                        for (int k = 0; k < laserCloudDepth; k++)
                        {
                            int j = 0;
                            pcl::PointCloud<PointType>::Ptr laserCloudCubeCornerPointer =
                                laserCloudCornerArray[i + laserCloudWidth * j + laserCloudWidth * laserCloudHeight * k];
                            pcl::PointCloud<PointType>::Ptr laserCloudCubeSurfPointer =
                                laserCloudSurfArray[i + laserCloudWidth * j + laserCloudWidth * laserCloudHeight * k];
                            for (; j < laserCloudHeight - 1; j++)
                            {
                                laserCloudCornerArray[i + laserCloudWidth * j + laserCloudWidth * laserCloudHeight * k] =
                                    laserCloudCornerArray[i + laserCloudWidth * (j + 1) + laserCloudWidth * laserCloudHeight * k];
                                laserCloudSurfArray[i + laserCloudWidth * j + laserCloudWidth * laserCloudHeight * k] =
                                    laserCloudSurfArray[i + laserCloudWidth * (j + 1) + laserCloudWidth * laserCloudHeight * k];
                            }
                            laserCloudCornerArray[i + laserCloudWidth * j + laserCloudWidth * laserCloudHeight * k] =
                                laserCloudCubeCornerPointer;
                            laserCloudSurfArray[i + laserCloudWidth * j + laserCloudWidth * laserCloudHeight * k] =
                                laserCloudCubeSurfPointer;
                            laserCloudCubeCornerPointer->clear();
                            laserCloudCubeSurfPointer->clear();
                        }
                    }

                    centerCubeJ--;
                    laserCloudCenHeight--;
                }

                while (centerCubeK < 3)
                {
                    for (int i = 0; i < laserCloudWidth; i++)
                    {
                        for (int j = 0; j < laserCloudHeight; j++)
                        {
                            int k = laserCloudDepth - 1;
                            pcl::PointCloud<PointType>::Ptr laserCloudCubeCornerPointer =
                                laserCloudCornerArray[i + laserCloudWidth * j + laserCloudWidth * laserCloudHeight * k];
                            pcl::PointCloud<PointType>::Ptr laserCloudCubeSurfPointer =
                                laserCloudSurfArray[i + laserCloudWidth * j + laserCloudWidth * laserCloudHeight * k];
                            for (; k >= 1; k--)
                            {
                                laserCloudCornerArray[i + laserCloudWidth * j + laserCloudWidth * laserCloudHeight * k] =
                                    laserCloudCornerArray[i + laserCloudWidth * j + laserCloudWidth * laserCloudHeight * (k - 1)];
                                laserCloudSurfArray[i + laserCloudWidth * j + laserCloudWidth * laserCloudHeight * k] =
                                    laserCloudSurfArray[i + laserCloudWidth * j + laserCloudWidth * laserCloudHeight * (k - 1)];
                            }
                            laserCloudCornerArray[i + laserCloudWidth * j + laserCloudWidth * laserCloudHeight * k] =
                                laserCloudCubeCornerPointer;
                            laserCloudSurfArray[i + laserCloudWidth * j + laserCloudWidth * laserCloudHeight * k] =
                                laserCloudCubeSurfPointer;
                            laserCloudCubeCornerPointer->clear();
                            laserCloudCubeSurfPointer->clear();
                        }
                    }

                    centerCubeK++;
                    laserCloudCenDepth++;
                }

                while (centerCubeK >= laserCloudDepth - 3)
                {
                    for (int i = 0; i < laserCloudWidth; i++)
                    {
                        for (int j = 0; j < laserCloudHeight; j++)
                        {
                            int k = 0;
                            pcl::PointCloud<PointType>::Ptr laserCloudCubeCornerPointer =
                                laserCloudCornerArray[i + laserCloudWidth * j + laserCloudWidth * laserCloudHeight * k];
                            pcl::PointCloud<PointType>::Ptr laserCloudCubeSurfPointer =
                                laserCloudSurfArray[i + laserCloudWidth * j + laserCloudWidth * laserCloudHeight * k];
                            for (; k < laserCloudDepth - 1; k++)
                            {
                                laserCloudCornerArray[i + laserCloudWidth * j + laserCloudWidth * laserCloudHeight * k] =
                                    laserCloudCornerArray[i + laserCloudWidth * j + laserCloudWidth * laserCloudHeight * (k + 1)];
                                laserCloudSurfArray[i + laserCloudWidth * j + laserCloudWidth * laserCloudHeight * k] =
                                    laserCloudSurfArray[i + laserCloudWidth * j + laserCloudWidth * laserCloudHeight * (k + 1)];
                            }
                            laserCloudCornerArray[i + laserCloudWidth * j + laserCloudWidth * laserCloudHeight * k] =
                                laserCloudCubeCornerPointer;
                            laserCloudSurfArray[i + laserCloudWidth * j + laserCloudWidth * laserCloudHeight * k] =
                                laserCloudCubeSurfPointer;
                            laserCloudCubeCornerPointer->clear();
                            laserCloudCubeSurfPointer->clear();
                        }
                    }

                    centerCubeK--;
                    laserCloudCenDepth--;
                }

                int laserCloudValidNum = 0;
                int laserCloudSurroundNum = 0;

                for (int i = centerCubeI - 2; i <= centerCubeI + 2; i++)
                {
                    for (int j = centerCubeJ - 2; j <= centerCubeJ + 2; j++)
                    {
                        for (int k = centerCubeK - 1; k <= centerCubeK + 1; k++)
                        {
                            if (i >= 0 && i < laserCloudWidth &&
                                j >= 0 && j < laserCloudHeight &&
                                k >= 0 && k < laserCloudDepth)
                            {
                                laserCloudValidInd[laserCloudValidNum] = i + laserCloudWidth * j + laserCloudWidth * laserCloudHeight * k;
                                laserCloudValidNum++;
                                laserCloudSurroundInd[laserCloudSurroundNum] = i + laserCloudWidth * j + laserCloudWidth * laserCloudHeight * k;
                                laserCloudSurroundNum++;
                            }
                        }
                    }
                }

                laserCloudCornerFromMap->clear();
                laserCloudSurfFromMap->clear();
                for (int i = 0; i < laserCloudValidNum; i++)
                {
                    *laserCloudCornerFromMap += *laserCloudCornerArray[laserCloudValidInd[i]];
                    *laserCloudSurfFromMap += *laserCloudSurfArray[laserCloudValidInd[i]];
                }
                int laserCloudCornerFromMapNum = laserCloudCornerFromMap->points.size();
                int laserCloudSurfFromMapNum = laserCloudSurfFromMap->points.size();

                pcl::PointCloud<PointType>::Ptr laserCloudCornerStack(new pcl::PointCloud<PointType>());
                downSizeFilterCorner.setInputCloud(laserCloudCornerLast);
                downSizeFilterCorner.filter(*laserCloudCornerStack);
                int laserCloudCornerStackNum = laserCloudCornerStack->points.size();

                pcl::PointCloud<PointType>::Ptr laserCloudSurfStack(new pcl::PointCloud<PointType>());
                downSizeFilterSurf.setInputCloud(laserCloudSurfLast);
                downSizeFilterSurf.filter(*laserCloudSurfStack);
                int laserCloudSurfStackNum = laserCloudSurfStack->points.size();

                printf("map prepare time %f ms\n", t_shift.toc());
                printf("map corner num %d  surf num %d \n", laserCloudCornerFromMapNum, laserCloudSurfFromMapNum);
                if (laserCloudCornerFromMapNum > 10 && laserCloudSurfFromMapNum > 50)
                {
                    TicToc t_opt;
                    TicToc t_tree;
                    kdtreeCornerFromMap->setInputCloud(laserCloudCornerFromMap);
                    kdtreeSurfFromMap->setInputCloud(laserCloudSurfFromMap);
                    printf("build tree time %f ms \n", t_tree.toc());

                    for (int iterCount = 0; iterCount < 2; iterCount++)
                    {
                        ceres::LossFunction *loss_function = new ceres::HuberLoss(0.1);
                        ceres::LocalParameterization *q_parameterization =
                            new ceres::EigenQuaternionParameterization();
                        ceres::Problem::Options problem_options;

                        ceres::Problem problem(problem_options);
                        problem.AddParameterBlock(parameters, 4, q_parameterization);
                        problem.AddParameterBlock(parameters + 4, 3);

                        TicToc t_data;
                        int corner_num = 0;

                        for (int i = 0; i < laserCloudCornerStackNum; i++)
                        {
                            pointOri = laserCloudCornerStack->points[i];
                            pointAssociateToMap(&pointOri, &pointSel);
                            kdtreeCornerFromMap->nearestKSearch(pointSel, 5, pointSearchInd, pointSearchSqDis);

                            if (pointSearchSqDis[4] < 1.0)
                            {
                                std::vector<Eigen::Vector3d> nearCorners;
                                Eigen::Vector3d center(0, 0, 0);
                                for (int j = 0; j < 5; j++)
                                {
                                    Eigen::Vector3d tmp(laserCloudCornerFromMap->points[pointSearchInd[j]].x,
                                                        laserCloudCornerFromMap->points[pointSearchInd[j]].y,
                                                        laserCloudCornerFromMap->points[pointSearchInd[j]].z);
                                    center = center + tmp;
                                    nearCorners.push_back(tmp);
                                }
                                center = center / 5.0;

                                Eigen::Matrix3d covMat = Eigen::Matrix3d::Zero();
                                for (int j = 0; j < 5; j++)
                                {
                                    Eigen::Matrix<double, 3, 1> tmpZeroMean = nearCorners[j] - center;
                                    covMat = covMat + tmpZeroMean * tmpZeroMean.transpose();
                                }

                                Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d> saes(covMat);

                                Eigen::Vector3d unit_direction = saes.eigenvectors().col(2);
                                Eigen::Vector3d curr_point(pointOri.x, pointOri.y, pointOri.z);
                                if (saes.eigenvalues()[2] > 3 * saes.eigenvalues()[1])
                                {
                                    Eigen::Vector3d point_on_line = center;
                                    Eigen::Vector3d point_a, point_b;
                                    point_a = 0.1 * unit_direction + point_on_line;
                                    point_b = -0.1 * unit_direction + point_on_line;

                                    ceres::CostFunction *cost_function = LidarEdgeFactor::Create(curr_point, point_a, point_b, 1.0);
                                    problem.AddResidualBlock(cost_function, loss_function, parameters, parameters + 4);
                                    corner_num++;
                                }
                            }
                        }

                        int surf_num = 0;
                        for (int i = 0; i < laserCloudSurfStackNum; i++)
                        {
                            pointOri = laserCloudSurfStack->points[i];
                            pointAssociateToMap(&pointOri, &pointSel);
                            kdtreeSurfFromMap->nearestKSearch(pointSel, 5, pointSearchInd, pointSearchSqDis);

                            Eigen::Matrix<double, 5, 3> matA0;
                            Eigen::Matrix<double, 5, 1> matB0 = -1 * Eigen::Matrix<double, 5, 1>::Ones();
                            if (pointSearchSqDis[4] < 1.0)
                            {
                                for (int j = 0; j < 5; j++)
                                {
                                    matA0(j, 0) = laserCloudSurfFromMap->points[pointSearchInd[j]].x;
                                    matA0(j, 1) = laserCloudSurfFromMap->points[pointSearchInd[j]].y;
                                    matA0(j, 2) = laserCloudSurfFromMap->points[pointSearchInd[j]].z;
                                }

                                Eigen::Vector3d norm = matA0.colPivHouseholderQr().solve(matB0);
                                double negative_OA_dot_norm = 1 / norm.norm();
                                norm.normalize();

                                bool planeValid = true;
                                for (int j = 0; j < 5; j++)
                                {
                                    if (fabs(norm(0) * laserCloudSurfFromMap->points[pointSearchInd[j]].x +
                                             norm(1) * laserCloudSurfFromMap->points[pointSearchInd[j]].y +
                                             norm(2) * laserCloudSurfFromMap->points[pointSearchInd[j]].z + negative_OA_dot_norm) > 0.2)
                                    {
                                        planeValid = false;
                                        break;
                                    }
                                }
                                Eigen::Vector3d curr_point(pointOri.x, pointOri.y, pointOri.z);
                                if (planeValid)
                                {
                                    ceres::CostFunction *cost_function = LidarPlaneNormFactor::Create(curr_point, norm, negative_OA_dot_norm);
                                    problem.AddResidualBlock(cost_function, loss_function, parameters, parameters + 4);
                                    surf_num++;
                                }
                            }
                        }

                        printf("mapping data assosiation time %f ms \n", t_data.toc());

                        TicToc t_solver;
                        ceres::Solver::Options options;
                        options.linear_solver_type = ceres::DENSE_QR;
                        options.max_num_iterations = 4;
                        options.minimizer_progress_to_stdout = false;
                        options.check_gradients = false;
                        options.gradient_check_relative_precision = 1e-4;
                        ceres::Solver::Summary summary;
                        ceres::Solve(options, &problem, &summary);
                        printf("mapping solver time %f ms \n", t_solver.toc());
                    }
                    printf("mapping optimization time %f \n", t_opt.toc());
                }
                else
                {
                    RCLCPP_WARN(this->get_logger(), "time Map corner and surf num are not enough");
                }
                transformUpdate();

                TicToc t_add;
                for (int i = 0; i < laserCloudCornerStackNum; i++)
                {
                    pointAssociateToMap(&laserCloudCornerStack->points[i], &pointSel);

                    int cubeI = int((pointSel.x + 25.0) / 50.0) + laserCloudCenWidth;
                    int cubeJ = int((pointSel.y + 25.0) / 50.0) + laserCloudCenHeight;
                    int cubeK = int((pointSel.z + 25.0) / 50.0) + laserCloudCenDepth;

                    if (pointSel.x + 25.0 < 0)
                        cubeI--;
                    if (pointSel.y + 25.0 < 0)
                        cubeJ--;
                    if (pointSel.z + 25.0 < 0)
                        cubeK--;

                    if (cubeI >= 0 && cubeI < laserCloudWidth &&
                        cubeJ >= 0 && cubeJ < laserCloudHeight &&
                        cubeK >= 0 && cubeK < laserCloudDepth)
                    {
                        int cubeInd = cubeI + laserCloudWidth * cubeJ + laserCloudWidth * laserCloudHeight * cubeK;
                        laserCloudCornerArray[cubeInd]->push_back(pointSel);
                    }
                }

                for (int i = 0; i < laserCloudSurfStackNum; i++)
                {
                    pointAssociateToMap(&laserCloudSurfStack->points[i], &pointSel);

                    int cubeI = int((pointSel.x + 25.0) / 50.0) + laserCloudCenWidth;
                    int cubeJ = int((pointSel.y + 25.0) / 50.0) + laserCloudCenHeight;
                    int cubeK = int((pointSel.z + 25.0) / 50.0) + laserCloudCenDepth;

                    if (pointSel.x + 25.0 < 0)
                        cubeI--;
                    if (pointSel.y + 25.0 < 0)
                        cubeJ--;
                    if (pointSel.z + 25.0 < 0)
                        cubeK--;

                    if (cubeI >= 0 && cubeI < laserCloudWidth &&
                        cubeJ >= 0 && cubeJ < laserCloudHeight &&
                        cubeK >= 0 && cubeK < laserCloudDepth)
                    {
                        int cubeInd = cubeI + laserCloudWidth * cubeJ + laserCloudWidth * laserCloudHeight * cubeK;
                        laserCloudSurfArray[cubeInd]->push_back(pointSel);
                    }
                }
                printf("add points time %f ms\n", t_add.toc());

                TicToc t_filter;
                for (int i = 0; i < laserCloudValidNum; i++)
                {
                    int ind = laserCloudValidInd[i];

                    pcl::PointCloud<PointType>::Ptr tmpCorner(new pcl::PointCloud<PointType>());
                    downSizeFilterCorner.setInputCloud(laserCloudCornerArray[ind]);
                    downSizeFilterCorner.filter(*tmpCorner);
                    laserCloudCornerArray[ind] = tmpCorner;

                    pcl::PointCloud<PointType>::Ptr tmpSurf(new pcl::PointCloud<PointType>());
                    downSizeFilterSurf.setInputCloud(laserCloudSurfArray[ind]);
                    downSizeFilterSurf.filter(*tmpSurf);
                    laserCloudSurfArray[ind] = tmpSurf;
                }
                printf("filter time %f ms \n", t_filter.toc());

                TicToc t_pub;
                if (frameCount % 5 == 0)
                {
                    laserCloudSurround->clear();
                    for (int i = 0; i < laserCloudSurroundNum; i++)
                    {
                        int ind = laserCloudSurroundInd[i];
                        *laserCloudSurround += *laserCloudCornerArray[ind];
                        *laserCloudSurround += *laserCloudSurfArray[ind];
                    }

                    sensor_msgs::msg::PointCloud2 laserCloudSurround3;
                    pcl::toROSMsg(*laserCloudSurround, laserCloudSurround3);
                    laserCloudSurround3.header.stamp = rclcpp::Time(static_cast<int64_t>(timeLaserOdometry * 1e9));
                    laserCloudSurround3.header.frame_id = "/camera_init";
                    pubLaserCloudSurround->publish(laserCloudSurround3);
                }

                if (frameCount % 20 == 0)
                {
                    pcl::PointCloud<PointType> laserCloudMap;
                    for (int i = 0; i < 4851; i++)
                    {
                        laserCloudMap += *laserCloudCornerArray[i];
                        laserCloudMap += *laserCloudSurfArray[i];
                    }
                    sensor_msgs::msg::PointCloud2 laserCloudMsg;
                    pcl::toROSMsg(laserCloudMap, laserCloudMsg);
                    laserCloudMsg.header.stamp = rclcpp::Time(static_cast<int64_t>(timeLaserOdometry * 1e9));
                    laserCloudMsg.header.frame_id = "/camera_init";
                    pubLaserCloudMap->publish(laserCloudMsg);
                }

                int laserCloudFullResNum = laserCloudFullRes->points.size();
                for (int i = 0; i < laserCloudFullResNum; i++)
                {
                    pointAssociateToMap(&laserCloudFullRes->points[i], &laserCloudFullRes->points[i]);
                }

                sensor_msgs::msg::PointCloud2 laserCloudFullRes3;
                pcl::toROSMsg(*laserCloudFullRes, laserCloudFullRes3);
                laserCloudFullRes3.header.stamp = rclcpp::Time(static_cast<int64_t>(timeLaserOdometry * 1e9));
                laserCloudFullRes3.header.frame_id = "/camera_init";
                pubLaserCloudFullRes->publish(laserCloudFullRes3);

                printf("mapping pub time %f ms \n", t_pub.toc());

                printf("whole mapping time %f ms +++++\n", t_whole.toc());

                nav_msgs::msg::Odometry odomAftMapped;
                odomAftMapped.header.frame_id = "/camera_init";
                odomAftMapped.child_frame_id = "/aft_mapped";
                odomAftMapped.header.stamp = rclcpp::Time(static_cast<int64_t>(timeLaserOdometry * 1e9));
                odomAftMapped.pose.pose.orientation.x = q_w_curr.x();
                odomAftMapped.pose.pose.orientation.y = q_w_curr.y();
                odomAftMapped.pose.pose.orientation.z = q_w_curr.z();
                odomAftMapped.pose.pose.orientation.w = q_w_curr.w();
                odomAftMapped.pose.pose.position.x = t_w_curr.x();
                odomAftMapped.pose.pose.position.y = t_w_curr.y();
                odomAftMapped.pose.pose.position.z = t_w_curr.z();
                pubOdomAftMapped->publish(odomAftMapped);

                geometry_msgs::msg::PoseStamped laserAfterMappedPose;
                laserAfterMappedPose.header = odomAftMapped.header;
                laserAfterMappedPose.pose = odomAftMapped.pose.pose;
                laserAfterMappedPath.header.stamp = odomAftMapped.header.stamp;
                laserAfterMappedPath.header.frame_id = "/camera_init";
                laserAfterMappedPath.poses.push_back(laserAfterMappedPose);
                pubLaserAfterMappedPath->publish(laserAfterMappedPath);

                geometry_msgs::msg::TransformStamped transformStamped;
                transformStamped.header.stamp = odomAftMapped.header.stamp;
                transformStamped.header.frame_id = "/camera_init";
                transformStamped.child_frame_id = "/aft_mapped";
                transformStamped.transform.translation.x = t_w_curr(0);
                transformStamped.transform.translation.y = t_w_curr(1);
                transformStamped.transform.translation.z = t_w_curr(2);
                transformStamped.transform.rotation.x = q_w_curr.x();
                transformStamped.transform.rotation.y = q_w_curr.y();
                transformStamped.transform.rotation.z = q_w_curr.z();
                transformStamped.transform.rotation.w = q_w_curr.w();
                tfBroadcaster->sendTransform(transformStamped);

                frameCount++;
            }
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
