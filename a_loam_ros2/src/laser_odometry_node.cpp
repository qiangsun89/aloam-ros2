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

        kdtreeCornerLast.reset(new pcl::KdTreeFLANN<PointType>());
        kdtreeSurfLast.reset(new pcl::KdTreeFLANN<PointType>());

        cornerPointsSharp.reset(new pcl::PointCloud<PointType>());
        cornerPointsLessSharp.reset(new pcl::PointCloud<PointType>());
        surfPointsFlat.reset(new pcl::PointCloud<PointType>());
        surfPointsLessFlat.reset(new pcl::PointCloud<PointType>());
        laserCloudCornerLast.reset(new pcl::PointCloud<PointType>());
        laserCloudSurfLast.reset(new pcl::PointCloud<PointType>());
        laserCloudFullRes.reset(new pcl::PointCloud<PointType>());

        // Main processing loop at 100Hz
        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(10),
            std::bind(&LaserOdometryNode::processFrame, this));
    }

private:
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr subCornerPointsSharp;
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr subCornerPointsLessSharp;
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr subSurfPointsFlat;
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr subSurfPointsLessFlat;
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr subLaserCloudFullRes;

    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pubLaserCloudCornerLast;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pubLaserCloudSurfLast;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pubLaserCloudFullRes;
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr pubLaserOdometry;
    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr pubLaserPath;

    rclcpp::TimerBase::SharedPtr timer_;

    std::queue<sensor_msgs::msg::PointCloud2::ConstSharedPtr> cornerSharpBuf;
    std::queue<sensor_msgs::msg::PointCloud2::ConstSharedPtr> cornerLessSharpBuf;
    std::queue<sensor_msgs::msg::PointCloud2::ConstSharedPtr> surfFlatBuf;
    std::queue<sensor_msgs::msg::PointCloud2::ConstSharedPtr> surfLessFlatBuf;
    std::queue<sensor_msgs::msg::PointCloud2::ConstSharedPtr> fullPointsBuf;
    std::mutex mBuf;

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

                // Corner feature matching
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

                // Surface feature matching
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

        // Transform corner features and plane features to the scan end point
        if (0)
        {
            int cornerPointsLessSharpNum = cornerPointsLessSharp->points.size();
            for (int i = 0; i < cornerPointsLessSharpNum; i++)
            {
                TransformToEnd(&cornerPointsLessSharp->points[i], &cornerPointsLessSharp->points[i]);
            }

            int surfPointsLessFlatNum = surfPointsLessFlat->points.size();
            for (int i = 0; i < surfPointsLessFlatNum; i++)
            {
                TransformToEnd(&surfPointsLessFlat->points[i], &surfPointsLessFlat->points[i]);
            }

            int laserCloudFullResNum = laserCloudFullRes->points.size();
            for (int i = 0; i < laserCloudFullResNum; i++)
            {
                TransformToEnd(&laserCloudFullRes->points[i], &laserCloudFullRes->points[i]);
            }
        }

        // Swap buffers for next frame KD-tree
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

            sensor_msgs::msg::PointCloud2 laserCloudCornerLast2;
            pcl::toROSMsg(*laserCloudCornerLast, laserCloudCornerLast2);
            laserCloudCornerLast2.header.stamp = rclcpp::Time(static_cast<int64_t>(timeSurfPointsLessFlat * 1e9));
            laserCloudCornerLast2.header.frame_id = "/camera";
            pubLaserCloudCornerLast->publish(laserCloudCornerLast2);

            sensor_msgs::msg::PointCloud2 laserCloudSurfLast2;
            pcl::toROSMsg(*laserCloudSurfLast, laserCloudSurfLast2);
            laserCloudSurfLast2.header.stamp = rclcpp::Time(static_cast<int64_t>(timeSurfPointsLessFlat * 1e9));
            laserCloudSurfLast2.header.frame_id = "/camera";
            pubLaserCloudSurfLast->publish(laserCloudSurfLast2);

            sensor_msgs::msg::PointCloud2 laserCloudFullRes3;
            pcl::toROSMsg(*laserCloudFullRes, laserCloudFullRes3);
            laserCloudFullRes3.header.stamp = rclcpp::Time(static_cast<int64_t>(timeSurfPointsLessFlat * 1e9));
            laserCloudFullRes3.header.frame_id = "/camera";
            pubLaserCloudFullRes->publish(laserCloudFullRes3);
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
