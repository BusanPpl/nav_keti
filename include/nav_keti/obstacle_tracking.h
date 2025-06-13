#ifndef OBSTACLE_TRACKING_H_
#define OBSTACLE_TRACKING_H_
#include "nav_msgs/msg/occupancy_grid.hpp"
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <Eigen/Dense>
#include <vector>
#include <chrono>

#include "cost_values.h"

class ObstacleTracker : public rclcpp::Node {
public:
    ObstacleTracker();
    bool costmapCallback(const nav_msgs::msg::OccupancyGrid::SharedPtr msg);
    void logObstacleData();
    Eigen::Vector4f getState() const;
    double getObstacleRadius() const;

private:
    struct Point {
        double x, y;
    };
    std::chrono::steady_clock::time_point last_update_time_;
    rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr parameter_event_sub_;
    rcl_interfaces::msg::SetParametersResult parameterCallback(const std::vector<rclcpp::Parameter>& parameters);
    void initializeKalmanFilter();
    void updateKalmanFilter(const Eigen::Vector2f& measurement, double dt);
    Point calculateCentroid(const std::vector<Point>& points);
    void resetState();
    void updateParameters();
    void calculateRadius(const Point& centroid, const std::vector<Point>& points);
    double obstacle_radius_;
    float process_noise_factor, measurement_noise_factor;
    Eigen::Matrix<float, 4, 1> state_;
    Eigen::Matrix<float, 4, 4> state_covariance_, transition_matrix_, process_noise_;
    Eigen::Matrix<float, 2, 4> measurement_matrix_;
    Eigen::Matrix<float, 2, 2> measurement_noise_;
    rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr subscription_;
};

#endif  
