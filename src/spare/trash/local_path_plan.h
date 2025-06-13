#ifndef MOVE_H
#define MOVE_H

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

class LPP : public rclcpp::Node {
public:
    LPP(double max_v, double v_resolution, double max_omega, double omega_resolution, double direction_gain_, double speed_gain_, double obstacle_gain_, double dt, 
            double boundary_, double thresh_d, double thresh_a);
    ~LPP();

    void run();

private:
    DWA dwa;

    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_pub_;
    // rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr target_sub_;
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_sub_;
    rclcpp::Subscription<sensor_msgs::msg::NavSatFix>::SharedPtr target_gps_sub_;
    rclcpp::Subscription<sensor_msgs::msg::NavSatFix>::SharedPtr gps_sub_;
    rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_subscriber_;
    rclcpp::Subscription<nav2_msgs::msg::Costmap>::SharedPtr costmap_sub_;
    rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr global_path_sub_;

    rclcpp::Rate r;
    rclcpp::Time last_time;
    std::mutex data_mutex;
    geometry_msgs::msg::Pose pose_;
    geometry_msgs::msg::Twist speed_;
    nav2_msgs::msg::Costmap::SharedPtr latest_costmap_msg_; 
    std::thread thread_obj;

    double boundary, dt;
    double thresh_dist, thresh_ang;
    double trans_spd, ang_spd;
    bool f1, f2, detect;
    double robot_x, robot_y, robot_yaw, vx, w;
    double goal_x, goal_y, goal_yaw;

    // void targetPose(const nav_msgs::msg::Odometry::SharedPtr msg);

    void timer_callback();
    void laserCallback(const sensor_msgs::msg::LaserScan::SharedPtr msg);
    void stopSpeed();
    std::tuple<double, double> getDir();
    void LPPGoal();

    double constrain(double input, double low, double high);
    double checkLinearLimitVelocity(double vel);
    double checkAngularLimitVelocity(double vel);

    void costmapCallback(const nav2_msgs::msg::Costmap::SharedPtr msg);
    void gpsCallback(const sensor_msgs::msg::NavSatFix::SharedPtr msg);
    void globalPathCallback(const nav_msgs::msg::Path::SharedPtr msg);
    void imuCallback(const sensor_msgs::msg::Imu::SharedPtr msg);
    void targetGpsCallback(const sensor_msgs::msg::NavSatFix::SharedPtr msg);
    void convertGPSToXY(double latitude, double longitude, double &x, double &y);
    

};

#endif  
