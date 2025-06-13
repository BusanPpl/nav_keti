#ifndef COSTMAP_GENERATOR_HPP
#define COSTMAP_GENERATOR_HPP

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "sensor_msgs/point_cloud2_iterator.hpp"
#include "rcl_interfaces/msg/set_parameters_result.hpp"
#include "nav2_msgs/msg/costmap.hpp"

#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/nav_sat_fix.hpp>
#include <std_msgs/msg/float32_multi_array.hpp>

#include <vector>
#include <cmath>
#include <algorithm>
#include <thread>
#include <functional>

#include <opencv2/video/tracking.hpp>

#include "cost_values.h"
#include "rviz2_color.h"
#include "obstacle_tracking.h"
#include "dynamic_costmap_updater.h"
#include "operation_costmap_updater.h"
#include "topic_handler.h"


class TrackedObject {
public:
    int id;
    int label;
    double radius, yaw;
    int age;
    int hit_counter;

    cv::KalmanFilter kf;
    cv::Mat state;
    cv::Mat measurement;
    std::vector<float> size;

    TrackedObject(int id_)
        : id(id_), label(-1), radius(0.0), age(0), hit_counter(5), yaw(0.0),
          kf(4, 2, 0), state(4, 1, CV_32F), measurement(2, 1, CV_32F), size(std::vector<float>{0.0, 0.0, 0.0})
    {
        kf.transitionMatrix = (cv::Mat_<float>(4, 4) <<
            1, 0, 1, 0,
            0, 1, 0, 1,
            0, 0, 1, 0,
            0, 0, 0, 1);

        setIdentity(kf.measurementMatrix);
        setIdentity(kf.processNoiseCov, cv::Scalar::all(1e-2));
        setIdentity(kf.measurementNoiseCov, cv::Scalar::all(1e-1));
        setIdentity(kf.errorCovPost, cv::Scalar::all(1));

        state.setTo(0);
        measurement.setTo(0);
    }

    void predict(double dt) {
        kf.transitionMatrix.at<float>(0, 2) = dt;
        kf.transitionMatrix.at<float>(1, 3) = dt;
        state = kf.predict();
    }

    void update(float x, float y) {
        measurement.at<float>(0) = x;
        measurement.at<float>(1) = y;
        kf.correct(measurement);
    }

    std::vector<float> getState() const {
        return {
            state.at<float>(0),  // x
            state.at<float>(1),  // y
            state.at<float>(2),  // vx
            state.at<float>(3)   // vy
        };
    }
};


class CostmapGenerator : public rclcpp::Node, public TopicHandler {
public:
    CostmapGenerator();
    ~CostmapGenerator();
    ObstacleTracker Tracker;
    std::unique_ptr<DynamicCostmapUpdater> Updater;
    std::unique_ptr<OperationCostmapUpdater> Operation;

private:
    int width, height;
    int GRADIENT_SIZE, GRADIENT_FACTOR;
    double resolution, max_obstacle_height, sensor_rotation_angle, robot_radius, robot_x = 0.0, robot_y = 0.0, robot_yaw = 0.0;
    double initial_x, initial_y, initial_lat, initial_lon;
    bool gps_initialized_ = false;
    std::atomic<double> publish_interval_{0.1};
    std::atomic<bool> stop_publishing_{false};
    std::thread publish_thread_;
    std::mutex costmap_mutex_;
    std::vector<Obstacle> obstacles;
    std::vector<Object> objectes;
    std::vector<TrackedObject> tracked_objects_;
    std::chrono::steady_clock::time_point last_update_time_;

    rcl_interfaces::msg::SetParametersResult parameterCallback(const std::vector<rclcpp::Parameter>& parameters);
    rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr parameter_event_sub_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr tracked_info_pub_;
    
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr point_cloud_sub_;
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr laser_scan_sub_;
    rclcpp::Subscription<std_msgs::msg::Float32MultiArray>::SharedPtr tracker_states_sub_;
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr detect_sub_;
    
    std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
    std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

    void publishCostmapLoop();
    void updateParameters();
    void pointCloudCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg);
    void laserScanCallback(const sensor_msgs::msg::LaserScan::SharedPtr msg);
    void detectCallback(const std_msgs::msg::String::SharedPtr msg);
    void publishObstacleMarker(const TrackedObject& obj);
    void updateCostmap(const sensor_msgs::msg::PointCloud2::SharedPtr& cloud);
    void updateCostmap(const sensor_msgs::msg::LaserScan::SharedPtr& scan);
    void trackerStatesCallback(const std_msgs::msg::Float32MultiArray::SharedPtr msg);

    void applyPadding(std::vector<int8_t>& data, int grid_x, int grid_y);
    std::vector<int8_t> normalizeCostmap(const std::vector<int8_t>& data);
    void generateCostmap(const std::vector<int8_t>& data, int width, int height, double resolution);
    void publishCostmap(const std::shared_ptr<nav_msgs::msg::OccupancyGrid>& costmap_msg);
    void publishGridmap(const std::shared_ptr<nav_msgs::msg::OccupancyGrid>& costmap_msg);

};

#endif // COSTMAP_GENERATOR_HPP