#ifndef GLOBAL_PATH_PLANNER_THETA_H
#define GLOBAL_PATH_PLANNER_THETA_H

#include <memory>
#include <thread>
#include <mutex>
#include <vector>
#include <atomic>
#include <cmath>

#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/path.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <nav2_msgs/msg/costmap.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <sensor_msgs/msg/nav_sat_fix.hpp>

#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <geometry_msgs/msg/transform_stamped.hpp>

#include "theta_star.h"

class GlobalPathPlannerTheta : public rclcpp::Node
{
public:
    GlobalPathPlannerTheta();
    ~GlobalPathPlannerTheta();

private:
    // 콜백 함수
    void costmapCallback(const nav2_msgs::msg::Costmap::SharedPtr msg);
    void gpsCallback(const sensor_msgs::msg::NavSatFix::SharedPtr msg);
    void targetGpsCallback(const sensor_msgs::msg::NavSatFix::SharedPtr msg);

    // 경로 생성 함수
    void navGoalHandler();
    void convertGPSToXY(double latitude, double longitude, double &x, double &y);
    bool clipToCostmapBoundary(double robot_x, double robot_y, double &target_x, double &target_y);

    // Theta* 경로 생성기
    std::shared_ptr<theta_star::ThetaStar> theta_star_planner_;

    // ROS2 통신 객체
    rclcpp::Subscription<nav2_msgs::msg::Costmap>::SharedPtr costmap_sub_;
    rclcpp::Subscription<sensor_msgs::msg::NavSatFix>::SharedPtr gps_sub_;
    rclcpp::Subscription<sensor_msgs::msg::NavSatFix>::SharedPtr target_gps_sub_;
    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr pub_robot_path_;

    // 맵 및 좌표 상태
    nav2_msgs::msg::Costmap current_costmap_;
    double robot_x, robot_y, target_x, target_y;

    std::atomic<bool> map_exist_;
    std::mutex costmap_mutex_;
};

#endif  // GLOBAL_PATH_PLANNER_THETA_H
