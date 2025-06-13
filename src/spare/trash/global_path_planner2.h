#ifndef GLOBAL_PATH_PLANNER_H
#define GLOBAL_PATH_PLANNER_H


#include <memory>
#include <thread>
#include <mutex>
#include <chrono>
#include <cmath>

#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/path.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <nav2_msgs/msg/costmap.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include "sensor_msgs/msg/nav_sat_fix.hpp"

#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

#include "astar_v2.h"
#include "cost_values.h"
class GlobalPathPlanner : public rclcpp::Node
{
public:
    GlobalPathPlanner();
    ~GlobalPathPlanner();

private:

    void costmapCallback(const nav2_msgs::msg::Costmap::SharedPtr msg);
    void updateAStarMap();
    void costmapProcessingLoop();
    
    // 경로 생성 및 처리
    void navGoalHandler();
    void robotPose();
    void pathCallback(const nav_msgs::msg::Path::SharedPtr msg);

    // GPS 관련
    void gpsCallback(const sensor_msgs::msg::NavSatFix::SharedPtr msg);
    void targetGpsCallback(const sensor_msgs::msg::NavSatFix::SharedPtr msg);
    void convertGPSToXY(double latitude, double longitude, double &x, double &y);
    bool clipToCostmapBoundary(double robot_x, double robot_y, double &target_x, double &target_y);
    
    // A* 알고리즘 객체
    AStar::Generator map_generator_;
    
    // ROS2 통신 객체
    rclcpp::Subscription<nav2_msgs::msg::Costmap>::SharedPtr costmap_sub_;
    std::shared_ptr<nav2_msgs::msg::Costmap> latest_costmap_msg_;
    rclcpp::Subscription<sensor_msgs::msg::NavSatFix>::SharedPtr gps_sub_;
    rclcpp::Subscription<sensor_msgs::msg::NavSatFix>::SharedPtr target_gps_sub_;
    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr pub_robot_path_;

    // 맵 정보 및 로봇 상태
    nav2_msgs::msg::Costmap current_costmap_;
    std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
    std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;

    std::atomic<bool> stop_thread_;
    std::thread costmap_thread_;
    std::mutex costmap_mutex_;
    std::atomic<double> publish_interval_{0.1};
    bool map_exist_, pose_flag;
    double r_x, r_y;
    double robot_x, robot_y, target_x, target_y;
};

#endif  // GLOBAL_PATH_PLANNER_H
