#ifndef PATH_GENERATOR_H
#define PATH_GENERATOR_H

#include "nav_keti/a_star.h"
#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/path.hpp>
#include <nav_msgs/srv/get_map.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>

#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/transform_listener.h>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

class PathGenerator : public rclcpp::Node
{
public:
    PathGenerator();
    ~PathGenerator();

private:
    void requestMap();
    void handleMapResponse(rclcpp::Client<nav_msgs::srv::GetMap>::SharedFuture future);
    void navGoalHandler();
    void robotPose();
    void pathCallback(const nav_msgs::msg::Path::SharedPtr msg);

private:

    AStar::Generator map_generator_;
    
    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr sub_nav_goal_;
    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr pub_robot_path_;
    rclcpp::Client<nav_msgs::srv::GetMap>::SharedPtr client_;
    
    nav_msgs::msg::MapMetaData map_info_;
    std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
    std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
    bool map_exist_, pose_flag;
    double r_x, r_y;
};

#endif
