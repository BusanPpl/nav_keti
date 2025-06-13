#ifndef TOPIC_HANDLER_H
#define TOPIC_HANDLER_H

#include <rclcpp/rclcpp.hpp>
#include <functional>
#include <nlohmann/json.hpp>

// 메시지 타입들
#include <std_msgs/msg/string.hpp>
#include <sensor_msgs/msg/nav_sat_fix.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <nav2_msgs/msg/costmap.hpp>
#include <nav_msgs/msg/path.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <visualization_msgs/msg/marker.hpp>

#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>

using json = nlohmann::json;

class TopicHandler {
public:
    explicit TopicHandler(rclcpp::Node* node)
    : node_(node), qos_(10)
    {
        pub_path_ = node_->create_publisher<nav_msgs::msg::Path>("planned_path", qos_);

        gps_callback_           = [](auto) {};
        target_gps_callback_    = [](auto) {};
        imu_callback_           = [](auto) {};
        costmap_callback_       = [](auto) {};
        global_path_callback_   = [](auto) {};
        goal_pose_callback_     = [](auto) {};
        pointcloud_callback_    = [](auto) {};
        laserscan_callback_     = [](auto) {};
        detect_callback_        = [](auto) {};
        vel_callback_           = [](auto) {};
    }

    void initSubscriptions() {
        gps_sub_ = node_->create_subscription<sensor_msgs::msg::NavSatFix>(
            "/gps", qos_, std::bind(&TopicHandler::handleGpsMsg, this, std::placeholders::_1));

        target_sub_ = node_->create_subscription<sensor_msgs::msg::NavSatFix>(
            "/target_gps", qos_, std::bind(&TopicHandler::handleTargetGpsMsg, this, std::placeholders::_1));

        imu_sub_ = node_->create_subscription<sensor_msgs::msg::Imu>(
            "/imu", rclcpp::SensorDataQoS(), std::bind(&TopicHandler::handleImuMsg, this, std::placeholders::_1));

        costmap_sub_ = node_->create_subscription<nav2_msgs::msg::Costmap>(
            "/costmap_raw", qos_, std::bind(&TopicHandler::handleCostmapMsg, this, std::placeholders::_1));

        goal_pose_sub_ = node_->create_subscription<geometry_msgs::msg::PoseStamped>(
            "/goal_pose", qos_, std::bind(&TopicHandler::handleGoalPoseMsg, this, std::placeholders::_1));

        global_path_sub_ = node_->create_subscription<nav_msgs::msg::Path>(
            "/planned_path", rclcpp::SensorDataQoS(), std::bind(&TopicHandler::handleGlobalPathMsg, this, std::placeholders::_1));

        point_cloud_sub_ = node_->create_subscription<sensor_msgs::msg::PointCloud2>(
            "/ouster/points", rclcpp::SensorDataQoS(), std::bind(&TopicHandler::handlePointCloudMsg, this, std::placeholders::_1));

        laser_scan_sub_ = node_->create_subscription<sensor_msgs::msg::LaserScan>(
            "/scan", rclcpp::SensorDataQoS(), std::bind(&TopicHandler::handleLaserScanMsg, this, std::placeholders::_1));

        detect_sub_ = node_->create_subscription<std_msgs::msg::String>(
            "/detection_result", qos_, std::bind(&TopicHandler::handleDetectMsg, this, std::placeholders::_1));

        vel_sub_ = node_->create_subscription<geometry_msgs::msg::Twist>(
            "/cmd_vel", qos_, std::bind(&TopicHandler::handleCmdvelMsg, this, std::placeholders::_1));
    }

    void initPublishers() {
        auto reliable = rclcpp::QoS(rclcpp::KeepLast(1)).transient_local().reliable();
        pub_cmd_ = node_->create_publisher<geometry_msgs::msg::Twist>("cmd_vel", qos_);
        pub_path_ = node_->create_publisher<nav_msgs::msg::Path>("planned_path", qos_);
        pub_costmap_raw_ = node_->create_publisher<nav2_msgs::msg::Costmap>("/costmap_raw", reliable);
        pub_occupancy_grid_ = node_->create_publisher<nav_msgs::msg::OccupancyGrid>("/costmap", reliable);
        pub_marker_ = node_->create_publisher<visualization_msgs::msg::Marker>("/obstacle_marker", reliable);
    }

    void setGpsCallback(std::function<void(sensor_msgs::msg::NavSatFix::SharedPtr)> cb)          { gps_callback_ = cb; }
    void setTargetGpsCallback(std::function<void(sensor_msgs::msg::NavSatFix::SharedPtr)> cb)    { target_gps_callback_ = cb; }
    void setImuCallback(std::function<void(sensor_msgs::msg::Imu::SharedPtr)> cb)                { imu_callback_ = cb; }
    void setCostmapCallback(std::function<void(nav2_msgs::msg::Costmap::SharedPtr)> cb)          { costmap_callback_ = cb; }
    void setGlobalPathCallback(std::function<void(nav_msgs::msg::Path::SharedPtr)> cb)           { global_path_callback_ = cb; }
    void setGoalPoseCallback(std::function<void(geometry_msgs::msg::PoseStamped::SharedPtr)> cb) { goal_pose_callback_ = cb; }
    void setPointCloudCallback(std::function<void(sensor_msgs::msg::PointCloud2::SharedPtr)> cb) { pointcloud_callback_ = cb; }
    void setLaserScanCallback(std::function<void(sensor_msgs::msg::LaserScan::SharedPtr)> cb)    { laserscan_callback_ = cb; }
    void setDetectCallback(std::function<void(std_msgs::msg::String::SharedPtr)> cb)             { detect_callback_ = cb; }
    void setCmdvelCallback(std::function<void(geometry_msgs::msg::Twist::SharedPtr)> cb)         { vel_callback_ = cb; }

    //GPS 좌표계 상에서의 방향값을 반환
    double getYaw() const {
        tf2::Quaternion quat(
            imu_msg_.orientation.x,
            imu_msg_.orientation.y,
            imu_msg_.orientation.z,
            imu_msg_.orientation.w
        );
        double roll, pitch, yaw;
        tf2::Matrix3x3(quat).getRPY(roll, pitch, yaw);
        yaw += 0.0;  // declination (편각) 필요 시 설정
        if (yaw < 0) yaw += 2 * M_PI;
        return yaw;
    }

    void convertGPSToXY(double lat, double lon, double& x, double& y) const {
        double lat_rad = lat * M_PI / 180.0;
        double lon_rad = lon * M_PI / 180.0;
        const double R = 6378137.0;
        x = lon_rad * R * cos(lat_rad);
        y = lat_rad * R;
    }

    void publishCmd(const geometry_msgs::msg::Twist& vel) {
        pub_cmd_->publish(vel);
    }

    void publishPath(const nav_msgs::msg::Path& path) {
        pub_path_->publish(path);
    }

    void publishRawCostmap(const nav2_msgs::msg::Costmap& costmap) {
        pub_costmap_raw_->publish(costmap);
    }

    void publishOccupancyGrid(const nav_msgs::msg::OccupancyGrid& grid) {
        pub_occupancy_grid_->publish(grid);
    }

    void publishMarker(const visualization_msgs::msg::Marker& marker) {
        pub_marker_->publish(marker);
    }

    const sensor_msgs::msg::NavSatFix& gps() const          { return gps_msg_; }
    const sensor_msgs::msg::NavSatFix& targetGps() const    { return target_gps_msg_; }
    const sensor_msgs::msg::Imu& imu() const                { return imu_msg_; }
    const nav2_msgs::msg::Costmap& costmap() const          { return costmap_msg_; }
    const nav_msgs::msg::Path& globalPath() const           { return global_path_msg_; }
    const geometry_msgs::msg::PoseStamped& goalPose() const { return goal_pose_msg_; }
    const sensor_msgs::msg::PointCloud2& pointCloud() const { return pointcloud_msg_; }
    const sensor_msgs::msg::LaserScan& laserScan() const    { return laser_scan_msg_; }
    const geometry_msgs::msg::Twist& cmdVel() const    { return Vel_msg; }

    bool gps_ready = false, laser_ready = false, costmap_ready = false, global_path_ready = false;

protected:
    void handleGpsMsg(sensor_msgs::msg::NavSatFix::SharedPtr msg) {
        gps_msg_ = *msg;
        gps_callback_(msg);
        gps_ready = true;
    }

    void handleTargetGpsMsg(sensor_msgs::msg::NavSatFix::SharedPtr msg) {
        target_gps_msg_ = *msg;
        target_gps_callback_(msg);
    }

    void handleImuMsg(sensor_msgs::msg::Imu::SharedPtr msg) {
        imu_msg_ = *msg;
        imu_callback_(msg);
    }

    void handleCostmapMsg(nav2_msgs::msg::Costmap::SharedPtr msg) {
        costmap_msg_ = *msg;
        costmap_callback_(msg);
        costmap_ready = true;
    }

    void handleGoalPoseMsg(geometry_msgs::msg::PoseStamped::SharedPtr msg) {
        goal_pose_msg_ = *msg;
        goal_pose_callback_(msg);
    }

    void handleGlobalPathMsg(nav_msgs::msg::Path::SharedPtr msg) {
        global_path_msg_ = *msg;
        global_path_callback_(msg);
        global_path_ready = true;
    }

    void handlePointCloudMsg(sensor_msgs::msg::PointCloud2::SharedPtr msg) {
        pointcloud_msg_ = *msg;
        pointcloud_callback_(msg);
    }

    void handleLaserScanMsg(sensor_msgs::msg::LaserScan::SharedPtr msg) {
        laser_scan_msg_ = *msg;
        laserscan_callback_(msg);
    }

    void handleDetectMsg(std_msgs::msg::String::SharedPtr msg) {
        detect_msg_ = *msg;
        detect_callback_(msg);
    }

    void handleCmdvelMsg(geometry_msgs::msg::Twist::SharedPtr msg) {
        Vel_msg = *msg;
        vel_callback_(msg);
    }

    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr pub_cmd_;
    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr pub_path_;
    rclcpp::Publisher<nav2_msgs::msg::Costmap>::SharedPtr pub_costmap_raw_;
    rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr pub_occupancy_grid_;
    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr pub_marker_;

private:

    rclcpp::Node* node_;
    rclcpp::QoS qos_;

    sensor_msgs::msg::NavSatFix gps_msg_;
    sensor_msgs::msg::NavSatFix target_gps_msg_;
    sensor_msgs::msg::Imu imu_msg_;
    nav2_msgs::msg::Costmap costmap_msg_;
    nav_msgs::msg::Path global_path_msg_;
    geometry_msgs::msg::PoseStamped goal_pose_msg_;
    sensor_msgs::msg::PointCloud2 pointcloud_msg_;
    sensor_msgs::msg::LaserScan laser_scan_msg_;
    std_msgs::msg::String detect_msg_;
    geometry_msgs::msg::Twist Vel_msg;

    rclcpp::Subscription<sensor_msgs::msg::NavSatFix>::SharedPtr gps_sub_;
    rclcpp::Subscription<sensor_msgs::msg::NavSatFix>::SharedPtr target_sub_;
    rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub_;
    rclcpp::Subscription<nav2_msgs::msg::Costmap>::SharedPtr costmap_sub_;
    rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr global_path_sub_;
    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr goal_pose_sub_;
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr point_cloud_sub_;
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr laser_scan_sub_;
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr detect_sub_;
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr vel_sub_;

    std::function<void(sensor_msgs::msg::NavSatFix::SharedPtr)> gps_callback_;
    std::function<void(sensor_msgs::msg::NavSatFix::SharedPtr)> target_gps_callback_;
    std::function<void(sensor_msgs::msg::Imu::SharedPtr)> imu_callback_;
    std::function<void(nav2_msgs::msg::Costmap::SharedPtr)> costmap_callback_;
    std::function<void(nav_msgs::msg::Path::SharedPtr)> global_path_callback_;
    std::function<void(geometry_msgs::msg::PoseStamped::SharedPtr)> goal_pose_callback_;
    std::function<void(sensor_msgs::msg::PointCloud2::SharedPtr)> pointcloud_callback_;
    std::function<void(sensor_msgs::msg::LaserScan::SharedPtr)> laserscan_callback_;
    std::function<void(std_msgs::msg::String::SharedPtr)> detect_callback_;
    std::function<void(geometry_msgs::msg::Twist::SharedPtr)> vel_callback_;
};

#endif // TOPIC_HANDLER_H
