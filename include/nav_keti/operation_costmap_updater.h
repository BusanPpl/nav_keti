#ifndef OPERATION_COSTMAP_UPDATER_HPP
#define OPERATION_COSTMAP_UPDATER_HPP

#include "rclcpp/rclcpp.hpp"
#include "rcl_interfaces/msg/set_parameters_result.hpp"

#include <nav_msgs/msg/occupancy_grid.hpp>
#include <Eigen/Dense>
#include <cmath>
#include <algorithm>
#include <vector>
#include <unordered_set>

#include "cost_values.h"

struct Object {
    Eigen::Vector2f position;
    Eigen::Vector2f velocity;
    double radius;
    int id, label, state;
    Object(float x, float y, float vx, float vy, double r, int id_ = -1, int l = -1, int s = -1)
        : position(x, y), velocity(vx, vy), radius(r), id(id_), label(l), state(s) {}
};

class OperationCostmapUpdater : public rclcpp::Node {
public:
    OperationCostmapUpdater(double resolution, double robot_radius, double inscribed_radius, double cost_scaling_factor);

    void update(nav_msgs::msg::OccupancyGrid &costmap_msg, const std::vector<Object> &objects);
    void getVel(double v, double yaw, double angle);

private:
    double robot_vel, robot_yaw, robot_angle;
    double resolution_;

    void applyCircularPadding(nav_msgs::msg::OccupancyGrid &costmap_msg, const Object &object, double padding_factor);
    void applyDefaultPadding(nav_msgs::msg::OccupancyGrid &costmap_msg, const Object &object);
    double getPaddingFactor(int label);
};

#endif
