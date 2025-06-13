#include "nav_keti/theta_path_planner.h"

using namespace std::placeholders;

GlobalPathPlannerTheta::GlobalPathPlannerTheta() : Node("global_path_planner_theta"), map_exist_(false)
{
    RCLCPP_INFO(this->get_logger(), "Theta* Global Path Planner initialized.");

    // ROS2 토픽 설정
    costmap_sub_ = this->create_subscription<nav2_msgs::msg::Costmap>(
        "/costmap_raw", 10, std::bind(&GlobalPathPlannerTheta::costmapCallback, this, _1));
    gps_sub_ = this->create_subscription<sensor_msgs::msg::NavSatFix>(
        "/gps", 10, std::bind(&GlobalPathPlannerTheta::gpsCallback, this, _1));
    target_gps_sub_ = this->create_subscription<sensor_msgs::msg::NavSatFix>(
        "/target_gps", 10, std::bind(&GlobalPathPlannerTheta::targetGpsCallback, this, _1));
    pub_robot_path_ = this->create_publisher<nav_msgs::msg::Path>("planned_path", 10);

    // Theta* 경로 생성기 초기화
    theta_star_planner_ = std::make_shared<theta_star::ThetaStar>();
}

GlobalPathPlannerTheta::~GlobalPathPlannerTheta() {}

void GlobalPathPlannerTheta::costmapCallback(const nav2_msgs::msg::Costmap::SharedPtr msg)
{
    std::lock_guard<std::mutex> lock(costmap_mutex_);
    current_costmap_ = *msg;
    map_exist_ = true;
}

void GlobalPathPlannerTheta::gpsCallback(const sensor_msgs::msg::NavSatFix::SharedPtr msg)
{
    convertGPSToXY(msg->latitude, msg->longitude, robot_x, robot_y);
}

void GlobalPathPlannerTheta::targetGpsCallback(const sensor_msgs::msg::NavSatFix::SharedPtr msg)
{
    convertGPSToXY(msg->latitude, msg->longitude, target_x, target_y);
    navGoalHandler();
}

void GlobalPathPlannerTheta::navGoalHandler()
{
    if (!map_exist_) {
        RCLCPP_WARN(this->get_logger(), "Costmap data is not ready yet.");
        return;
    }

    // 목표 및 로봇 위치를 코스트맵 기준 좌표계로 변환
    target_x = (target_x - robot_x - current_costmap_.metadata.origin.position.x) / current_costmap_.metadata.resolution;
    target_y = (target_y - robot_y - current_costmap_.metadata.origin.position.y) / current_costmap_.metadata.resolution;
    robot_x = (-current_costmap_.metadata.origin.position.x) / current_costmap_.metadata.resolution;
    robot_y = (-current_costmap_.metadata.origin.position.y) / current_costmap_.metadata.resolution;

    if (!clipToCostmapBoundary(robot_x, robot_y, target_x, target_y)) {
        RCLCPP_WARN(this->get_logger(), "Target position adjusted to fit within costmap.");
    }

    theta_star::CoordsM source = {static_cast<int>(robot_x), static_cast<int>(robot_y)};
    theta_star::CoordsM target = {static_cast<int>(target_x), static_cast<int>(target_y)};

    std::vector<theta_star::CoordsW> raw_path;
    if (!theta_star_planner_->generatePath(current_costmap_, source, target, raw_path)) {
        RCLCPP_WARN(this->get_logger(), "No path found.");
        return;
    }

    nav_msgs::msg::Path path_msg;
    for (const auto& point : raw_path) {
        geometry_msgs::msg::PoseStamped pose;
        pose.pose.position.x = point.x * current_costmap_.metadata.resolution + current_costmap_.metadata.origin.position.x;
        pose.pose.position.y = point.y * current_costmap_.metadata.resolution + current_costmap_.metadata.origin.position.y;
        path_msg.poses.push_back(pose);
    }
    path_msg.header.frame_id = "map";
    pub_robot_path_->publish(path_msg);

    RCLCPP_INFO(this->get_logger(), "Path successfully published.");
}

void GlobalPathPlannerTheta::convertGPSToXY(double latitude, double longitude, double &x, double &y)
{
    const double earth_radius = 6378137.0;
    double lat_rad = latitude * M_PI / 180.0;
    double lon_rad = longitude * M_PI / 180.0;
    x = lon_rad * earth_radius * cos(lat_rad);
    y = lat_rad * earth_radius;
}

bool GlobalPathPlannerTheta::clipToCostmapBoundary(double robot_x, double robot_y, double &target_x, double &target_y)
{
    int map_min_x = 0;
    int map_min_y = 0;
    int map_max_x = current_costmap_.metadata.size_x - 1;
    int map_max_y = current_costmap_.metadata.size_y - 1;

    int x0 = static_cast<int>(std::round(robot_x));
    int y0 = static_cast<int>(std::round(robot_y));
    int x1 = static_cast<int>(std::round(target_x));
    int y1 = static_cast<int>(std::round(target_y));

    int dx = abs(x1 - x0);
    int dy = abs(y1 - y0);
    int sx = (x0 < x1) ? 1 : -1;
    int sy = (y0 < y1) ? 1 : -1;
    int err = dx - dy;

    while (true) {
        if (x0 < map_min_x || x0 > map_max_x || y0 < map_min_y || y0 > map_max_y) {
            target_x = x0 - sx;
            target_y = y0 - sy;
            return false;
        }

        if (x0 == x1 && y0 == y1) break;

        int e2 = 2 * err;
        if (e2 > -dy) { err -= dy; x0 += sx; }
        if (e2 < dx) { err += dx; y0 += sy; }
    }
    target_x = x1;
    target_y = y1;
    return true;
}
