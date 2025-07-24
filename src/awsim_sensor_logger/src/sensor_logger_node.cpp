#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <autoware_vehicle_msgs/msg/velocity_report.hpp>
#include <autoware_vehicle_msgs/msg/gear_report.hpp>
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <cv_bridge/cv_bridge.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <opencv2/opencv.hpp>
#include <chrono>
#include <memory>
#include <fstream>
#include <filesystem>
#include <iomanip>
#include <sstream>

class SynchronizedSensorLogger : public rclcpp::Node
{
public:
    SynchronizedSensorLogger() : Node("synchronized_sensor_logger")
    {
        // Declare parameters with default values
        this->declare_parameter<std::string>("output_directory", "/tmp/awsim_sensor_data");
        this->declare_parameter<int>("logging_duration_sec", 300); // 5 minutes default
        this->declare_parameter<double>("sync_tolerance_sec", 0.1); // 100ms tolerance
        this->declare_parameter<bool>("save_pointclouds", true);
        this->declare_parameter<bool>("save_images", true);
        this->declare_parameter<bool>("save_raw_data", true);
        this->declare_parameter<int>("queue_size", 10);
        
        // Get parameters
        output_dir_ = this->get_parameter("output_directory").as_string();
        logging_duration_ = std::chrono::seconds(this->get_parameter("logging_duration_sec").as_int());
        sync_tolerance_ = this->get_parameter("sync_tolerance_sec").as_double();
        save_pointclouds_ = this->get_parameter("save_pointclouds").as_bool();
        save_images_ = this->get_parameter("save_images").as_bool();
        save_raw_data_ = this->get_parameter("save_raw_data").as_bool();
        int queue_size = this->get_parameter("queue_size").as_int();

        // Initialize counters and state
        synchronized_count_ = 0;
        is_logging_ = true;
        camera_info_saved_ = false;
        start_time_ = this->get_clock()->now();

        // Create output directory structure
        create_output_directories();

        // Initialize data files
        initialize_data_files();

        // Create message filter subscribers (5 sensors: core + ground truth pose)
        lidar_sub_.subscribe(this, "/sensing/lidar/top/pointcloud_raw", 
                           rclcpp::QoS(queue_size).reliability(RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT).get_rmw_qos_profile());
        camera_sub_.subscribe(this, "/sensing/camera/traffic_light/image_raw",
                            rclcpp::QoS(queue_size).reliability(RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT).get_rmw_qos_profile());
        imu_sub_.subscribe(this, "/sensing/imu/tamagawa/imu_raw",
                         rclcpp::QoS(queue_size).reliability(RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT).get_rmw_qos_profile());
        gnss_sub_.subscribe(this, "/sensing/gnss/pose_with_covariance",
                          rclcpp::QoS(queue_size).reliability(RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT).get_rmw_qos_profile());
        ground_truth_pose_sub_.subscribe(this, "/awsim/ground_truth/vehicle/pose",
                                       rclcpp::QoS(queue_size).reliability(RMW_QOS_POLICY_RELIABILITY_RELIABLE).get_rmw_qos_profile());

        // Create separate subscriber for camera info (static data, saved once)
        camera_info_sub_static_ = this->create_subscription<sensor_msgs::msg::CameraInfo>(
            "/sensing/camera/traffic_light/camera_info",
            rclcpp::QoS(1).reliability(RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT),
            std::bind(&SynchronizedSensorLogger::camera_info_callback, this, std::placeholders::_1));

        // Create approximate time synchronizer (5 sensors: core + ground truth pose)
        sync_ = std::make_shared<message_filters::Synchronizer<SyncPolicy>>(
            SyncPolicy(queue_size), lidar_sub_, camera_sub_, imu_sub_, gnss_sub_, ground_truth_pose_sub_);
        
        // Set synchronization tolerance
        sync_->setAgePenalty(sync_tolerance_);
        
        // Register synchronized callback (5 sensors)
        sync_->registerCallback(std::bind(&SynchronizedSensorLogger::synchronized_callback, this,
                                        std::placeholders::_1, std::placeholders::_2, std::placeholders::_3,
                                        std::placeholders::_4, std::placeholders::_5));

        // Create timer for logging control and status
        control_timer_ = this->create_wall_timer(
            std::chrono::seconds(1),
            std::bind(&SynchronizedSensorLogger::control_callback, this));

        RCLCPP_INFO(this->get_logger(), "Synchronized Sensor Logger initialized");
        RCLCPP_INFO(this->get_logger(), "Output directory: %s", output_dir_.c_str());
        RCLCPP_INFO(this->get_logger(), "Logging duration: %ld seconds", logging_duration_.count());
        RCLCPP_INFO(this->get_logger(), "Sync tolerance: %.3f seconds", sync_tolerance_);
    }

    void camera_info_callback(const sensor_msgs::msg::CameraInfo::ConstSharedPtr& msg)
    {
        // Save camera info once (static calibration data)
        if (!camera_info_saved_) {
            std::ofstream camera_info_file(session_dir_ + "/metadata/camera_info.json");
            camera_info_file << "{\n";
            camera_info_file << "  \"camera_matrix\": {\n";
            camera_info_file << "    \"fx\": " << msg->k[0] << ",\n";
            camera_info_file << "    \"fy\": " << msg->k[4] << ",\n";
            camera_info_file << "    \"cx\": " << msg->k[2] << ",\n";
            camera_info_file << "    \"cy\": " << msg->k[5] << "\n";
            camera_info_file << "  },\n";
            camera_info_file << "  \"distortion_model\": \"" << msg->distortion_model << "\",\n";
            camera_info_file << "  \"distortion_coefficients\": [";
            for (size_t i = 0; i < msg->d.size(); ++i) {
                camera_info_file << msg->d[i];
                if (i < msg->d.size() - 1) camera_info_file << ", ";
            }
            camera_info_file << "],\n";
            camera_info_file << "  \"image_width\": " << msg->width << ",\n";
            camera_info_file << "  \"image_height\": " << msg->height << "\n";
            camera_info_file << "}";
            camera_info_file.close();
            camera_info_saved_ = true;
            RCLCPP_INFO(this->get_logger(), "Camera calibration saved to metadata/camera_info.json");
        }
    }

    ~SynchronizedSensorLogger()
    {
        finalize_logging();
    }

private:
    // Type definitions for message filters (5 sensors: core + ground truth pose only)
    using SyncPolicy = message_filters::sync_policies::ApproximateTime<
        sensor_msgs::msg::PointCloud2,
        sensor_msgs::msg::Image,
        sensor_msgs::msg::Imu,
        geometry_msgs::msg::PoseWithCovarianceStamped,
        geometry_msgs::msg::PoseStamped>;

    void create_output_directories()
    {
        // Create timestamped session directory
        auto now = std::chrono::system_clock::now();
        auto time_t = std::chrono::system_clock::to_time_t(now);
        std::stringstream ss;
        ss << std::put_time(std::localtime(&time_t), "%Y%m%d_%H%M%S");
        session_dir_ = output_dir_ + "/session_" + ss.str();

        std::filesystem::create_directories(session_dir_);
        std::filesystem::create_directories(session_dir_ + "/pointclouds");
        std::filesystem::create_directories(session_dir_ + "/images");
        std::filesystem::create_directories(session_dir_ + "/metadata");

        RCLCPP_INFO(this->get_logger(), "Created session directory: %s", session_dir_.c_str());
    }

    void initialize_data_files()
    {
        if (save_raw_data_) {
            // Initialize CSV files for sensor data (5 sensors: core + ground truth pose)
            synchronized_data_file_.open(session_dir_ + "/synchronized_data.csv");
            synchronized_data_file_ << "timestamp_ns,frame_id,lidar_points,image_width,image_height,"
                                   << "imu_linear_x,imu_linear_y,imu_linear_z,"
                                   << "imu_angular_x,imu_angular_y,imu_angular_z,"
                                   << "gnss_x,gnss_y,gnss_z,gnss_orientation_w,gnss_orientation_x,"
                                   << "gnss_orientation_y,gnss_orientation_z,"
                                   << "gt_pose_x,gt_pose_y,gt_pose_z,gt_pose_qw,gt_pose_qx,gt_pose_qy,gt_pose_qz" << std::endl;

            timing_analysis_file_.open(session_dir_ + "/timing_analysis.csv");
            timing_analysis_file_ << "sync_id,lidar_timestamp_ns,camera_timestamp_ns,"
                                 << "imu_timestamp_ns,gnss_timestamp_ns,gt_pose_timestamp_ns,"
                                 << "max_time_diff_ms" << std::endl;
        }
    }

    void synchronized_callback(
        const sensor_msgs::msg::PointCloud2::ConstSharedPtr& lidar_msg,
        const sensor_msgs::msg::Image::ConstSharedPtr& camera_msg,
        const sensor_msgs::msg::Imu::ConstSharedPtr& imu_msg,
        const geometry_msgs::msg::PoseWithCovarianceStamped::ConstSharedPtr& gnss_msg,
        const geometry_msgs::msg::PoseStamped::ConstSharedPtr& ground_truth_pose_msg)
    {
        if (!is_logging_) return;

        synchronized_count_++;
        
        // Calculate timing analysis using raw timestamp values (5 sensors: core + ground truth pose)
        auto lidar_time = lidar_msg->header.stamp.sec * 1000000000LL + lidar_msg->header.stamp.nanosec;
        auto camera_time = camera_msg->header.stamp.sec * 1000000000LL + camera_msg->header.stamp.nanosec;
        auto imu_time = imu_msg->header.stamp.sec * 1000000000LL + imu_msg->header.stamp.nanosec;
        auto gnss_time = gnss_msg->header.stamp.sec * 1000000000LL + gnss_msg->header.stamp.nanosec;
        auto gt_pose_time = ground_truth_pose_msg->header.stamp.sec * 1000000000LL + ground_truth_pose_msg->header.stamp.nanosec;
        
        auto max_time = std::max({lidar_time, camera_time, imu_time, gnss_time, gt_pose_time});
        auto min_time = std::min({lidar_time, camera_time, imu_time, gnss_time, gt_pose_time});
        auto max_diff_ms = (max_time - min_time) / 1e6; // Convert to milliseconds

        RCLCPP_INFO(this->get_logger(), 
                   "Synchronized frame %d - Max time diff: %.2f ms", 
                   synchronized_count_, max_diff_ms);

        // Save timing analysis (5 sensors: core + ground truth pose)
        if (save_raw_data_) {
            timing_analysis_file_ << synchronized_count_ << ","
                                 << lidar_time << "," << camera_time << ","
                                 << imu_time << "," << gnss_time << "," << gt_pose_time << ","
                                 << max_diff_ms << std::endl;
        }

        // Save synchronized data (5 sensors: core + ground truth pose)
        save_synchronized_frame(lidar_msg, camera_msg, imu_msg, gnss_msg, ground_truth_pose_msg);
    }

    void save_synchronized_frame(
        const sensor_msgs::msg::PointCloud2::ConstSharedPtr& lidar_msg,
        const sensor_msgs::msg::Image::ConstSharedPtr& camera_msg,
        const sensor_msgs::msg::Imu::ConstSharedPtr& imu_msg,
        const geometry_msgs::msg::PoseWithCovarianceStamped::ConstSharedPtr& gnss_msg,
        const geometry_msgs::msg::PoseStamped::ConstSharedPtr& ground_truth_pose_msg)
    {
        try {
            // Save point cloud data
            if (save_pointclouds_) {
                save_pointcloud(lidar_msg, synchronized_count_);
            }

            // Save camera image
            if (save_images_) {
                save_image(camera_msg, synchronized_count_);
            }

            // Save synchronized CSV data (5 sensors: core + ground truth pose)
            if (save_raw_data_) {
                save_csv_data(lidar_msg, camera_msg, imu_msg, gnss_msg, ground_truth_pose_msg);
            }

        } catch (const std::exception& e) {
            RCLCPP_ERROR(this->get_logger(), "Error saving frame %d: %s", synchronized_count_, e.what());
        }
    }

    void save_pointcloud(const sensor_msgs::msg::PointCloud2::ConstSharedPtr& msg, int frame_id)
    {
        // Convert ROS2 PointCloud2 to PCL
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
        pcl::fromROSMsg(*msg, *cloud);

        // Save as PCD file
        std::string filename = session_dir_ + "/pointclouds/frame_" + 
                              std::to_string(frame_id) + ".pcd";
        pcl::io::savePCDFileBinary(filename, *cloud);
    }

    void save_image(const sensor_msgs::msg::Image::ConstSharedPtr& msg, int frame_id)
    {
        try {
            // Convert ROS2 Image to OpenCV
            cv_bridge::CvImagePtr cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
            
            // Save as PNG file
            std::string filename = session_dir_ + "/images/frame_" + 
                                  std::to_string(frame_id) + ".png";
            cv::imwrite(filename, cv_ptr->image);
        } catch (cv_bridge::Exception& e) {
            RCLCPP_ERROR(this->get_logger(), "cv_bridge exception: %s", e.what());
        }
    }

    void save_csv_data(
        const sensor_msgs::msg::PointCloud2::ConstSharedPtr& lidar_msg,
        const sensor_msgs::msg::Image::ConstSharedPtr& camera_msg,
        const sensor_msgs::msg::Imu::ConstSharedPtr& imu_msg,
        const geometry_msgs::msg::PoseWithCovarianceStamped::ConstSharedPtr& gnss_msg,
        const geometry_msgs::msg::PoseStamped::ConstSharedPtr& ground_truth_pose_msg)
    {
        // Use raw timestamp values to avoid time source compatibility issues (5 sensors: core + ground truth pose)
        auto lidar_time_ns = lidar_msg->header.stamp.sec * 1000000000LL + lidar_msg->header.stamp.nanosec;
        auto camera_time_ns = camera_msg->header.stamp.sec * 1000000000LL + camera_msg->header.stamp.nanosec;
        auto imu_time_ns = imu_msg->header.stamp.sec * 1000000000LL + imu_msg->header.stamp.nanosec;
        auto gnss_time_ns = gnss_msg->header.stamp.sec * 1000000000LL + gnss_msg->header.stamp.nanosec;
        auto gt_pose_time_ns = ground_truth_pose_msg->header.stamp.sec * 1000000000LL + ground_truth_pose_msg->header.stamp.nanosec;
        
        // Calculate average timestamp using raw values (5 sensors)
        auto avg_timestamp = (lidar_time_ns + camera_time_ns + imu_time_ns + gnss_time_ns + gt_pose_time_ns) / 5;

        // Calculate number of points in point cloud
        size_t point_count = lidar_msg->width * lidar_msg->height;

        synchronized_data_file_ << avg_timestamp << ","
                               << synchronized_count_ << ","
                               << point_count << ","
                               << camera_msg->width << ","
                               << camera_msg->height << ","
                               << imu_msg->linear_acceleration.x << ","
                               << imu_msg->linear_acceleration.y << ","
                               << imu_msg->linear_acceleration.z << ","
                               << imu_msg->angular_velocity.x << ","
                               << imu_msg->angular_velocity.y << ","
                               << imu_msg->angular_velocity.z << ","
                               << gnss_msg->pose.pose.position.x << ","
                               << gnss_msg->pose.pose.position.y << ","
                               << gnss_msg->pose.pose.position.z << ","
                               << gnss_msg->pose.pose.orientation.w << ","
                               << gnss_msg->pose.pose.orientation.x << ","
                               << gnss_msg->pose.pose.orientation.y << ","
                               << gnss_msg->pose.pose.orientation.z << ","
                               << ground_truth_pose_msg->pose.position.x << ","
                               << ground_truth_pose_msg->pose.position.y << ","
                               << ground_truth_pose_msg->pose.position.z << ","
                               << ground_truth_pose_msg->pose.orientation.w << ","
                               << ground_truth_pose_msg->pose.orientation.x << ","
                               << ground_truth_pose_msg->pose.orientation.y << ","
                               << ground_truth_pose_msg->pose.orientation.z << std::endl;
    }

    void control_callback()
    {
        auto current_time = this->get_clock()->now();
        auto elapsed = current_time - start_time_;

        if (elapsed >= rclcpp::Duration(logging_duration_)) {
            if (is_logging_) {
                RCLCPP_INFO(this->get_logger(), 
                           "Logging duration reached (%ld sec). Stopping data collection.", 
                           logging_duration_.count());
                is_logging_ = false;
                finalize_logging();
            }
        } else {
            // Status update every 10 seconds during logging
            if (synchronized_count_ > 0 && synchronized_count_ % 10 == 0) {
                auto remaining = rclcpp::Duration(logging_duration_) - elapsed;
                RCLCPP_INFO(this->get_logger(), 
                           "Synchronized frames: %d, Time remaining: %ld sec", 
                           synchronized_count_, 
                           std::chrono::duration_cast<std::chrono::seconds>(
                               std::chrono::nanoseconds(remaining.nanoseconds())).count());
            }
        }
    }

    void finalize_logging()
    {
        if (synchronized_data_file_.is_open()) {
            synchronized_data_file_.close();
        }
        if (timing_analysis_file_.is_open()) {
            timing_analysis_file_.close();
        }

        // Write session summary
        std::ofstream summary_file(session_dir_ + "/session_summary.txt");
        summary_file << "AWSIM Sensor Data Collection Session Summary\n";
        summary_file << "===========================================\n";
        summary_file << "Session Directory: " << session_dir_ << "\n";
        summary_file << "Total Synchronized Frames: " << synchronized_count_ << "\n";
        summary_file << "Synchronization Tolerance: " << sync_tolerance_ << " seconds\n";
        summary_file << "Point Clouds Saved: " << (save_pointclouds_ ? "Yes" : "No") << "\n";
        summary_file << "Images Saved: " << (save_images_ ? "Yes" : "No") << "\n";
        summary_file << "Raw Data CSV Saved: " << (save_raw_data_ ? "Yes" : "No") << "\n";
        summary_file.close();

        RCLCPP_INFO(this->get_logger(), "Logging finalized. Session saved to: %s", session_dir_.c_str());
        RCLCPP_INFO(this->get_logger(), "Total synchronized frames collected: %d", synchronized_count_);
    }

    // Message filter subscribers (5 sensors: core + ground truth pose)
    message_filters::Subscriber<sensor_msgs::msg::PointCloud2> lidar_sub_;
    message_filters::Subscriber<sensor_msgs::msg::Image> camera_sub_;
    message_filters::Subscriber<sensor_msgs::msg::Imu> imu_sub_;
    message_filters::Subscriber<geometry_msgs::msg::PoseWithCovarianceStamped> gnss_sub_;
    message_filters::Subscriber<geometry_msgs::msg::PoseStamped> ground_truth_pose_sub_;

    // Static camera info subscriber (separate from synchronization)
    rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr camera_info_sub_static_;

    // Synchronizer
    std::shared_ptr<message_filters::Synchronizer<SyncPolicy>> sync_;

    // Timers
    rclcpp::TimerBase::SharedPtr control_timer_;

    // Parameters
    std::string output_dir_;
    std::string session_dir_;
    std::chrono::seconds logging_duration_;
    double sync_tolerance_;
    bool save_pointclouds_;
    bool save_images_;
    bool save_raw_data_;

    // State variables
    int synchronized_count_;
    bool is_logging_;
    bool camera_info_saved_;
    rclcpp::Time start_time_;

    // File handles
    std::ofstream synchronized_data_file_;
    std::ofstream timing_analysis_file_;
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<SynchronizedSensorLogger>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
