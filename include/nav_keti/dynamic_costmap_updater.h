#ifndef DYNAMIC_COSTMAP_UPDATER_HPP
#define DYNAMIC_COSTMAP_UPDATER_HPP

#include "rclcpp/rclcpp.hpp"
#include "rcl_interfaces/msg/set_parameters_result.hpp"

#include <nav_msgs/msg/occupancy_grid.hpp>
#include <Eigen/Dense>
#include <cmath>
#include <algorithm>
#include <vector>
#include <iostream>
#include <unordered_set>

#include "cost_values.h"

struct Obstacle {
    Eigen::Vector2f position;
    Eigen::Vector2f velocity;
    double radius;
    int label;
    Obstacle(float x, float y, float vx, float vy, double r, int l = -1)
        : position(x, y), velocity(vx, vy), radius(r), label(l) {}
};

class DynamicCostmapUpdater : public rclcpp::Node{
public:
    DynamicCostmapUpdater(double resolution, double robot_radius, double inscribed_radius, double cost_scaling_factor);
    void ADIC(nav_msgs::msg::OccupancyGrid &costmap_msg, const std::vector<Obstacle> &obstacles, double time_step);
    void DADIM(nav_msgs::msg::OccupancyGrid &costmap_msg, const std::vector<Obstacle> &obstacles, double time_step);
    void TruncatedVelocityObstacles(nav_msgs::msg::OccupancyGrid &costmap_msg, const std::vector<Obstacle> &obstacles, double truncation_time);
    void VelocityObstacle(nav_msgs::msg::OccupancyGrid &costmap_msg, const std::vector<Obstacle> &obstacles);
    void Circle(nav_msgs::msg::OccupancyGrid &costmap_msg, const std::vector<Obstacle> &obstacles);
    void getVel(double v, double yaw, double angle);

private:
    double robot_vel, robot_yaw, robot_angle;
    double robot_radius_, robot_max_speed, inscribed_radius_, cost_scaling_factor_, resolution_;
    rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr parameter_event_sub_;

    double computeCost(double adjust_distance, double distance, double combined_radius) const;
    rcl_interfaces::msg::SetParametersResult parameterCallback(const std::vector<rclcpp::Parameter>& parameters);
    void resetCostmap(nav_msgs::msg::OccupancyGrid &costmap_msg);
    void updateParameters();
    
};

#endif // DYNAMIC_COSTMAP_UPDATER_HPP
