#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <chrono>
#include <memory>

class SensorLoggerNode : public rclcpp::Node
{
public:
    SensorLoggerNode() : Node("sensor_logger_node")
    {
        // Initialize counters
        lidar_count_ = 0;
        camera_count_ = 0;
        imu_count_ = 0;
        gnss_count_ = 0;
        ground_truth_count_ = 0;

        // Create subscribers with appropriate QoS
        auto sensor_qos = rclcpp::QoS(10).reliability(RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT);
        auto default_qos = rclcpp::QoS(10);

        lidar_sub_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
            "/sensing/lidar/top/pointcloud_raw", sensor_qos,
            std::bind(&SensorLoggerNode::lidar_callback, this, std::placeholders::_1));

        camera_sub_ = this->create_subscription<sensor_msgs::msg::Image>(
            "/sensing/camera/traffic_light/image_raw", sensor_qos,
            std::bind(&SensorLoggerNode::camera_callback, this, std::placeholders::_1));

        imu_sub_ = this->create_subscription<sensor_msgs::msg::Imu>(
            "/sensing/imu/tamagawa/imu_raw", default_qos,
            std::bind(&SensorLoggerNode::imu_callback, this, std::placeholders::_1));

        gnss_sub_ = this->create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>(
            "/sensing/gnss/pose_with_covariance", default_qos,
            std::bind(&SensorLoggerNode::gnss_callback, this, std::placeholders::_1));

        ground_truth_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
            "/awsim/ground_truth/localization/kinematic_state", default_qos,
            std::bind(&SensorLoggerNode::ground_truth_callback, this, std::placeholders::_1));

        // Create timer for status reporting
        timer_ = this->create_wall_timer(
            std::chrono::seconds(5),
            std::bind(&SensorLoggerNode::status_callback, this));

        RCLCPP_INFO(this->get_logger(), "AWSIM Sensor Logger initialized");
        RCLCPP_INFO(this->get_logger(), "Subscribing to sensor topics...");
    }

private:
    void lidar_callback(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
    {
        lidar_count_++;
        last_lidar_time_ = this->get_clock()->now();
        
        if (lidar_count_ % 50 == 0) {
            RCLCPP_INFO(this->get_logger(), "LiDAR: %d messages received", lidar_count_);
        }
    }

    void camera_callback(const sensor_msgs::msg::Image::SharedPtr msg)
    {
        camera_count_++;
        last_camera_time_ = this->get_clock()->now();
        
        if (camera_count_ % 50 == 0) {
            RCLCPP_INFO(this->get_logger(), "Camera: %d messages received", camera_count_);
        }
    }

    void imu_callback(const sensor_msgs::msg::Imu::SharedPtr msg)
    {
        imu_count_++;
        last_imu_time_ = this->get_clock()->now();
        
        if (imu_count_ % 100 == 0) {
            RCLCPP_INFO(this->get_logger(), "IMU: %d messages received", imu_count_);
        }
    }

    void gnss_callback(const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg)
    {
        gnss_count_++;
        last_gnss_time_ = this->get_clock()->now();
        
        RCLCPP_INFO(this->get_logger(), "GNSS: %d messages received", gnss_count_);
    }

    void ground_truth_callback(const nav_msgs::msg::Odometry::SharedPtr msg)
    {
        ground_truth_count_++;
        last_ground_truth_time_ = this->get_clock()->now();
        
        if (ground_truth_count_ % 20 == 0) {
            RCLCPP_INFO(this->get_logger(), "Ground Truth: %d messages received (pos: %.2f, %.2f)", 
                       ground_truth_count_, msg->pose.pose.position.x, msg->pose.pose.position.y);
        }
    }

    void status_callback()
    {
        RCLCPP_INFO(this->get_logger(), "=== Sensor Status ===");
        RCLCPP_INFO(this->get_logger(), "LiDAR: %d msgs", lidar_count_);
        RCLCPP_INFO(this->get_logger(), "Camera: %d msgs", camera_count_);
        RCLCPP_INFO(this->get_logger(), "IMU: %d msgs", imu_count_);
        RCLCPP_INFO(this->get_logger(), "GNSS: %d msgs", gnss_count_);
        RCLCPP_INFO(this->get_logger(), "Ground Truth: %d msgs", ground_truth_count_);
    }

    // Subscribers
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr lidar_sub_;
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr camera_sub_;
    rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub_;
    rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr gnss_sub_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr ground_truth_sub_;
    
    // Timer
    rclcpp::TimerBase::SharedPtr timer_;
    
    // Counters
    int lidar_count_;
    int camera_count_;
    int imu_count_;
    int gnss_count_;
    int ground_truth_count_;
    
    // Timestamps
    rclcpp::Time last_lidar_time_;
    rclcpp::Time last_camera_time_;
    rclcpp::Time last_imu_time_;
    rclcpp::Time last_gnss_time_;
    rclcpp::Time last_ground_truth_time_;
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<SensorLoggerNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
