#ifndef GLOBAL_RRT_PLANNER_H
#define GLOBAL_RRT_PLANNER_H

#include "rrt_planner.h"  // 기존 알고리즘 헤더

#include <memory>
#include <thread>
#include <mutex>
#include <atomic>

#include <rclcpp/rclcpp.hpp>
#include <nav2_msgs/msg/costmap.hpp>
#include <sensor_msgs/msg/nav_sat_fix.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <nav_msgs/msg/path.hpp>

class GlobalRRTPlanner : public rclcpp::Node
{
public:
  GlobalRRTPlanner();
  ~GlobalRRTPlanner();

private:
  // ROS2 통신
  rclcpp::Subscription<nav2_msgs::msg::Costmap>::SharedPtr costmap_sub_;
  rclcpp::Subscription<sensor_msgs::msg::NavSatFix>::SharedPtr gps_sub_;
  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub_;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_pub_;

  // 콜백
  void costmapCallback(const nav2_msgs::msg::Costmap::SharedPtr msg);
  void gpsCallback(const sensor_msgs::msg::NavSatFix::SharedPtr msg);
  void imuCallback(const sensor_msgs::msg::Imu::SharedPtr msg);

  // 메인 루프
  std::thread planning_thread_;
  std::atomic<bool> stop_thread_;
  void planningLoop();
  void convertGPSToXY(double latitude, double longitude, double &x, double &y);
  // 내부 상태
  std::mutex mutex_;
  nav2_msgs::msg::Costmap latest_costmap_;
  double robot_x_, robot_y_, robot_yaw_;
  double target_x_, target_y_;
  bool costmap_ready_, gps_ready_, target_initialized_;
};

#endif  // GLOBAL_RRT_PLANNER_H
