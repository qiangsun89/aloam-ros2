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
#include <rosbag2_cpp/writer.hpp>
#include <rosbag2_storage/serialized_bag_message.hpp>
#include <rosbag2_storage/topic_metadata.hpp>
#include <rclcpp/serialization.hpp>
#include <rclcpp/serialized_message.hpp>

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

        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(100 / publish_delay),
            std::bind(&KittiHelperNode::processFrame, this));

        if (to_bag)
        {
            if (output_bag_file.empty())
            {
                output_bag_file = dataset_folder + "kitti_" + sequence_number;
            }

            rosbag2_storage::StorageOptions storage_options;
            storage_options.uri = output_bag_file;
            storage_options.storage_id = "sqlite3";

            rosbag2_cpp::ConverterOptions converter_options;
            converter_options.input_serialization_format = "cdr";
            converter_options.output_serialization_format = "cdr";

            bag_writer_ = std::make_unique<rosbag2_cpp::Writer>();
            bag_writer_->open(storage_options, converter_options);

            bag_writer_->create_topic({"/velodyne_points", "sensor_msgs/msg/PointCloud2", "cdr", ""});
            bag_writer_->create_topic({"/image_left", "sensor_msgs/msg/Image", "cdr", ""});
            bag_writer_->create_topic({"/image_right", "sensor_msgs/msg/Image", "cdr", ""});
            bag_writer_->create_topic({"/path_gt", "nav_msgs/msg/Path", "cdr", ""});
            bag_writer_->create_topic({"/odometry_gt", "nav_msgs/msg/Odometry", "cdr", ""});

            std::cout << "Recording to bag: " << output_bag_file << '\n';
        }
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
    std::unique_ptr<rosbag2_cpp::Writer> bag_writer_;

    nav_msgs::msg::Odometry odomGT;
    nav_msgs::msg::Path pathGT;

    Eigen::Matrix3d R_transform;
    Eigen::Quaterniond q_transform;

    std::ifstream timestamp_file;
    std::ifstream ground_truth_file;

    template<typename MessageT>
    void writeToBag(const MessageT& msg, const std::string& topic, const rclcpp::Time& stamp)
    {
        if (!to_bag) return;

        rclcpp::Serialization<MessageT> serialization;
        rclcpp::SerializedMessage serialized_msg;
        serialization.serialize_message(&msg, &serialized_msg);

        auto bag_msg = std::make_shared<rosbag2_storage::SerializedBagMessage>();
        bag_msg->topic_name = topic;
        bag_msg->time_stamp = stamp.nanoseconds();

        auto& rcl_msg = serialized_msg.get_rcl_serialized_message();
        auto data = std::shared_ptr<rcutils_uint8_array_t>(
            new rcutils_uint8_array_t,
            [](rcutils_uint8_array_t* p) {
                rcutils_ret_t fini_ret = rcutils_uint8_array_fini(p);
                (void)fini_ret;
                delete p;
            });
        *data = rcutils_get_zero_initialized_uint8_array();

        rcutils_allocator_t allocator = rcutils_get_default_allocator();
        rcutils_ret_t init_ret = rcutils_uint8_array_init(
            data.get(), rcl_msg.buffer_length, &allocator);
        if (init_ret != RCUTILS_RET_OK) {
            return;
        }
        if (rcl_msg.buffer_length > 0)
        {
            memcpy(data->buffer, rcl_msg.buffer, rcl_msg.buffer_length);
            data->buffer_length = rcl_msg.buffer_length;
        }
        bag_msg->serialized_data = data;

        bag_writer_->write(bag_msg);
    }

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

        std::stringstream left_image_path, right_image_path;
        left_image_path << dataset_folder << "sequences/" + sequence_number + "/image_0/" << std::setfill('0') << std::setw(6) << line_num << ".png";
        right_image_path << dataset_folder << "sequences/" + sequence_number + "/image_1/" << std::setfill('0') << std::setw(6) << line_num << ".png";
        cv::Mat left_image = cv::imread(left_image_path.str(), cv::IMREAD_GRAYSCALE);
        cv::Mat right_image = cv::imread(right_image_path.str(), cv::IMREAD_GRAYSCALE);

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
        writeToBag(odomGT, "/odometry_gt", stamp);

        geometry_msgs::msg::PoseStamped poseGT;
        poseGT.header = odomGT.header;
        poseGT.pose = odomGT.pose.pose;
        pathGT.header.stamp = odomGT.header.stamp;
        pathGT.poses.push_back(poseGT);
        pubPathGT->publish(pathGT);
        writeToBag(pathGT, "/path_gt", stamp);

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
        writeToBag(laser_cloud_msg, "/velodyne_points", stamp);

        auto image_left_msg = cv_bridge::CvImage(laser_cloud_msg.header, "mono8", left_image).toImageMsg();
        auto image_right_msg = cv_bridge::CvImage(laser_cloud_msg.header, "mono8", right_image).toImageMsg();
        pub_image_left->publish(*image_left_msg);
        pub_image_right->publish(*image_right_msg);
        writeToBag(*image_left_msg, "/image_left", stamp);
        writeToBag(*image_right_msg, "/image_right", stamp);

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
