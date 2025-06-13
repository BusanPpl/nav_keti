#ifndef GLOBAL_PATH_PLANNER_RRT_H
#define GLOBAL_PATH_PLANNER_RRT_H

#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/path.hpp>
#include <nav2_msgs/msg/costmap.hpp>
#include <sensor_msgs/msg/nav_sat_fix.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>

#include <mutex>
#include <thread>
#include "rrt.h"
#include "cost_values.h"

class GlobalPathPlannerRRT : public rclcpp::Node
{
public:
    GlobalPathPlannerRRT();
    ~GlobalPathPlannerRRT();

private:
    // 콜백 함수들
    void gpsCallback(const sensor_msgs::msg::NavSatFix::SharedPtr msg);
    void targetGpsCallback(const sensor_msgs::msg::NavSatFix::SharedPtr msg);
    void costmapCallback(const nav2_msgs::msg::Costmap::SharedPtr msg);
    void imuCallback(const sensor_msgs::msg::Imu::SharedPtr msg);
    bool clipToCostmapBoundary(double robot_x, double robot_y, double &target_x, double &target_y);
    // 내부 처리 함수
    void costmapProcessingLoop();
    void updateRRTMap();
    void navGoalHandler();
    void convertGPSToXY(double latitude, double longitude, double &x, double &y);


    // ROS 통신 객체
    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr pub_robot_path_;
    rclcpp::Subscription<nav2_msgs::msg::Costmap>::SharedPtr costmap_sub_;
    rclcpp::Subscription<sensor_msgs::msg::NavSatFix>::SharedPtr gps_sub_;
    rclcpp::Subscription<sensor_msgs::msg::NavSatFix>::SharedPtr target_gps_sub_;
    rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_subscriber_;

    // 상태 변수
    std::shared_ptr<nav2_msgs::msg::Costmap> latest_costmap_msg_;
    nav2_msgs::msg::Costmap current_costmap_;
    std::thread costmap_thread_;
    std::mutex costmap_mutex_;
    std::atomic<bool> stop_thread_;

    bool map_exist_, pose_flag_, f1_;
    double robot_x_, robot_y_, robot_yaw_, target_x_, target_y_;

    // RRT 알고리즘 객체
    RRT::Planner rrt_planner_;
};

#endif // GLOBAL_PATH_PLANNER_RRT_H
