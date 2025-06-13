#ifndef COSTMAP_GENERATOR_HPP
#define COSTMAP_GENERATOR_HPP

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "sensor_msgs/point_cloud2_iterator.hpp"
#include "rcl_interfaces/msg/set_parameters_result.hpp"
#include "nav2_msgs/msg/costmap.hpp"
#include "visualization_msgs/msg/marker.hpp"

#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

#include <sensor_msgs/msg/imu.hpp>
#include "sensor_msgs/msg/nav_sat_fix.hpp"

#include <vector>
#include <cmath>
#include <algorithm>
#include <thread>
#include <functional>

#include "cost_values.h"
#include "obstacle_tracking.h"
#include "dynamic_costmap_updater.h"


class CostmapGenerator : public rclcpp::Node {
public:
    CostmapGenerator();
    ~CostmapGenerator();
    ObstacleTracker Tracker;
    std::unique_ptr<DynamicCostmapUpdater> Updater;

private:
    int width, height;
    int GRADIENT_SIZE, GRADIENT_FACTOR;
    double resolution, max_obstacle_height, sensor_rotation_angle, robot_radius, robot_x = 0.0, robot_y = 0.0, robot_yaw = 0.0;
    double initial_x, initial_y, initial_lat, initial_lon;
    bool gps_initialized_ = false;
    std::atomic<double> publish_interval_{0.1};
    std::atomic<bool> stop_publishing_{false};
    std::vector<int8_t> latest_data_;
    std::thread publish_thread_;
    std::mutex costmap_mutex_;

    rcl_interfaces::msg::SetParametersResult parameterCallback(const std::vector<rclcpp::Parameter>& parameters);

    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr marker_pub_;
    rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr parameter_event_sub_;
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr point_cloud_sub_;
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr laser_scan_sub_;
    rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr costmap_pub;
    rclcpp::Publisher<nav2_msgs::msg::Costmap>::SharedPtr costmap_pub_raw;
    rclcpp::Subscription<sensor_msgs::msg::NavSatFix>::SharedPtr gps_sub_;
    rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub;
    std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
    std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

    void publishCostmapLoop();
    void updateParameters();
    void pointCloudCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg);
    void laserScanCallback(const sensor_msgs::msg::LaserScan::SharedPtr msg);
    void publishObstacleMarker(const Eigen::Vector4f& state);
    void updateCostmap(const sensor_msgs::msg::PointCloud2::SharedPtr& cloud);
    void updateCostmap(const sensor_msgs::msg::LaserScan::SharedPtr& scan);

    void applyPadding(std::vector<int8_t>& data, int grid_x, int grid_y);
    std::vector<int8_t> normalizeCostmap(const std::vector<int8_t>& data);
    void generateCostmap(const std::vector<int8_t>& data, int width, int height, double resolution);
    void publishRawCostmap(const std::shared_ptr<nav_msgs::msg::OccupancyGrid>& costmap_msg);
    void publishCostmap(const std::shared_ptr<nav_msgs::msg::OccupancyGrid>& costmap_msg);
    void convertGPSToXY(double latitude, double longitude, double &x, double &y);
    void gpsCallback(const sensor_msgs::msg::NavSatFix::SharedPtr msg);
    void imuCallback(const sensor_msgs::msg::Imu::SharedPtr msg);
    void publishMapToOdomTF();
    void publishOdomToBaseLinkTF();
    void publishTF();
};

#endif // COSTMAP_GENERATOR_HPP
