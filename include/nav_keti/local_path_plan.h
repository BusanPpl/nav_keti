#ifndef LocalPathPlanner3_H
#define LocalPathPlanner3_H

#include <rclcpp/rclcpp.hpp>

#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>

#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2_ros/transform_listener.h>

#include <nav_msgs/msg/path.hpp>
#include <nav2_msgs/msg/costmap.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>

#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/nav_sat_fix.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>

#include <memory>
#include <cmath>
#include <iostream>
#include <vector>
#include <limits>
#include <tuple>
#include <mutex>

#include "dwa.h"
#include "cost_values.h"

#include "nav_keti/topic_handler.h"

class LocalPathPlanner : public rclcpp::Node, public TopicHandler {
public:
    LocalPathPlanner();
    ~LocalPathPlanner();

private:
    std::unique_ptr<DWA> dwa;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_pub_;
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_sub_;
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Time last_time;
    rclcpp::Rate r;
    rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_subscriber_;
    rclcpp::Subscription<sensor_msgs::msg::NavSatFix>::SharedPtr gps_sub_;
    rclcpp::Subscription<sensor_msgs::msg::NavSatFix>::SharedPtr target_gps_sub_;
    rclcpp::Subscription<nav2_msgs::msg::Costmap>::SharedPtr costmap_sub_;
    rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr global_path_sub_;
    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr goal_pose_sub_;

    nav2_msgs::msg::Costmap::SharedPtr latest_costmap_msg_; 
    geometry_msgs::msg::Pose pose_;
    geometry_msgs::msg::Twist speed_;
    
    std::atomic<bool> running_;
    std::mutex gps_mutex;
    std::thread thread_obj;
    // bool laser_ready = false, costmap_ready = false, global_path_ready = false, gps_ready_=false;
    double boundary, dt;
    double thresh_dist, thresh_ang;
    double trans_spd, ang_spd;
    bool f1, f2, detect;
    double robot_x, robot_y, robot_yaw, vx, w;
    double target_x, target_y, target_yaw;

    void timer_callback();
    void laserCallback(const sensor_msgs::msg::LaserScan::SharedPtr msg);
    void stopSpeed();
    std::tuple<double, double> getDir();
    void LocalPathPlannerGoal();

    double constrain(double input, double low, double high);
    double checkLinearLimitVelocity(double vel);
    double checkAngularLimitVelocity(double vel);

    void costmapCallback(const nav2_msgs::msg::Costmap::SharedPtr msg);
    void globalPathCallback(const nav_msgs::msg::Path::SharedPtr msg);

    void onGpsMsg(sensor_msgs::msg::NavSatFix::SharedPtr msg);
    void onTargetGpsMsg(sensor_msgs::msg::NavSatFix::SharedPtr msg);
    void onGoalposeMsg(geometry_msgs::msg::PoseStamped::SharedPtr msg);
};

#endif  
