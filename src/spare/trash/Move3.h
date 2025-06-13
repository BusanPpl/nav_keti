#ifndef MOVE3_H
#define MOVE3_H

#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <tf2_ros/transform_listener.h>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/nav_sat_fix.hpp>

#include <memory>
#include <cmath>
#include <iostream>
#include <vector>
#include <limits>
#include <tuple>
#include <thread>
#include "dynamic_window_approach.h"



class Move : public rclcpp::Node {
public:
    Move(double max_v, double v_resolution, double max_omega, double omega_resolution, double direction_gain_, double speed_gain_, double obstacle_gain_, double dt, 
            double boundary_, double thresh_d, double thresh_a);
    ~Move();
    void moveGoal();

private:
    DWA dwa;

    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_pub_;
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_sub_;
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Time last_time;
    rclcpp::Rate rate;
    std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
    rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_subscriber_;
    rclcpp::Subscription<sensor_msgs::msg::NavSatFix>::SharedPtr gps_sub_;
    rclcpp::Subscription<sensor_msgs::msg::NavSatFix>::SharedPtr target_gps_sub_;
    geometry_msgs::msg::Pose pose_;
    geometry_msgs::msg::Twist speed_;
    
    std::mutex gps_mutex;
    std::thread thread_obj;
    std::thread spin_thread_;
    
    void timer_callback();
    void robotPose();
    void laserCallback(const sensor_msgs::msg::LaserScan::SharedPtr msg);
    void stopSpeed();
    std::tuple<double, double> getDir();
    
    double constrain(double input, double low, double high);
    double checkLinearLimitVelocity(double vel);
    double checkAngularLimitVelocity(double vel);
    void gpsCallback(const sensor_msgs::msg::NavSatFix::SharedPtr msg);
    void targetGpsCallback(const sensor_msgs::msg::NavSatFix::SharedPtr msg);
    void imuCallback(const sensor_msgs::msg::Imu::SharedPtr msg);
    void convertGPSToXY(double latitude, double longitude, double &x, double &y);
    double boundary, dt;
    double thresh_dist, thresh_ang;
    double trans_spd, ang_spd;
    bool f1, f2, detect;
    double x, y, yaw, vx, w;
    double goal_x, goal_y, goal_yaw;

    
};

#endif  
