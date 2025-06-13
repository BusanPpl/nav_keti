#include "nav_keti/global_rrt_planner.h"
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <cmath>

using rmp::path_planner::RRTPathPlanner;

GlobalRRTPlanner::GlobalRRTPlanner()
: Node("global_rrt_planner"),
  costmap_ready_(false),
  gps_ready_(false),
  target_initialized_(false),
  stop_thread_(false)
{
  costmap_sub_ = this->create_subscription<nav2_msgs::msg::Costmap>(
    "/costmap_raw", 10, std::bind(&GlobalRRTPlanner::costmapCallback, this, std::placeholders::_1));

  gps_sub_ = this->create_subscription<sensor_msgs::msg::NavSatFix>(
    "/gps", 10, std::bind(&GlobalRRTPlanner::gpsCallback, this, std::placeholders::_1));

  imu_sub_ = this->create_subscription<sensor_msgs::msg::Imu>(
    "/imu", rclcpp::SensorDataQoS(), std::bind(&GlobalRRTPlanner::imuCallback, this, std::placeholders::_1));

  path_pub_ = this->create_publisher<nav_msgs::msg::Path>("global_path", 10);

  planning_thread_ = std::thread(&GlobalRRTPlanner::planningLoop, this);

  RCLCPP_INFO(this->get_logger(), "Global RRT Planner initialized.");
}

GlobalRRTPlanner::~GlobalRRTPlanner()
{
  stop_thread_ = true;
  if (planning_thread_.joinable())
    planning_thread_.join();
}

void GlobalRRTPlanner::costmapCallback(const nav2_msgs::msg::Costmap::SharedPtr msg)
{
  std::lock_guard<std::mutex> lock(mutex_);
  latest_costmap_ = *msg;
  costmap_ready_ = true;
}

void GlobalRRTPlanner::gpsCallback(const sensor_msgs::msg::NavSatFix::SharedPtr msg)
{
  std::lock_guard<std::mutex> lock(mutex_);
  // 좌표 변환 (실험용)
  robot_x_ = -msg->latitude;
  robot_y_ = -msg->longitude;
  gps_ready_ = true;

  if (!target_initialized_) {
    convertGPSToXY(-msg->latitude, -msg->longitude + 0.002, target_x_, target_y_);
    target_initialized_ = true;
    RCLCPP_INFO(this->get_logger(), "Target set by GPS offset.");
  }
}

void GlobalRRTPlanner::imuCallback(const sensor_msgs::msg::Imu::SharedPtr msg)
{
  tf2::Quaternion q(msg->orientation.x, msg->orientation.y, msg->orientation.z, msg->orientation.w);
  double roll, pitch, yaw;
  tf2::Matrix3x3(q).getRPY(roll, pitch, yaw);
  robot_yaw_ = (yaw >= 0) ? yaw : yaw + 2 * M_PI;
}

void GlobalRRTPlanner::planningLoop()
{
  rclcpp::Rate rate(1.0);
  while (rclcpp::ok() && !stop_thread_)
  {
    nav2_msgs::msg::Costmap costmap_copy;
    double robot_x, robot_y, target_x, target_y;
    bool ready = false;
    {
      std::lock_guard<std::mutex> lock(mutex_);
      RCLCPP_INFO(this->get_logger(), "Ready state: costmap=%d gps=%d target=%d",
    costmap_ready_, gps_ready_, target_initialized_);
      if (costmap_ready_ && gps_ready_ && target_initialized_) {
        costmap_copy = latest_costmap_;
        robot_x = robot_x_;
        robot_y = robot_y_;
        target_x = target_x_;
        target_y = target_y_;
        ready = true;
      }
    }

    if (!ready) {
      rate.sleep();
      continue;
    }

    // Costmap 메타 정보
    auto meta = costmap_copy.metadata;
    double resolution = meta.resolution;
    double origin_x = meta.origin.position.x;
    double origin_y = meta.origin.position.y;

    // 월드 → 맵 좌표로 변환
    double sx = (robot_x - origin_x) / resolution;
    double sy = (robot_y - origin_y) / resolution;

    // 월드 좌표계 → 로봇 기준 상대 좌표
    double dx = target_x - robot_x;
    double dy = target_y - robot_y;

    // 회전 적용 (로봇이 바라보는 방향만큼 목표 회전)
    double rotated_dx = dx * cos(-robot_yaw_) - dy * sin(-robot_yaw_);
    double rotated_dy = dx * sin(-robot_yaw_) + dy * cos(-robot_yaw_);

    // 회전된 상대 위치 → 로컬 맵 상 목표 좌표로 변환
    double gx = (robot_x + rotated_dx - origin_x) / resolution;
    double gy = (robot_y + rotated_dy - origin_y) / resolution;

    RRTPathPlanner planner(1.0, 300, 15.0);
    planner.setCostmapData({
      static_cast<int>(meta.size_x),
      static_cast<int>(meta.size_y),
      resolution,
      origin_x,
      origin_y,
      costmap_copy.data
    });
    RCLCPP_INFO(this->get_logger(), "Costmap set. Starting to plan...");
    std::vector<rmp::common::geometry::Point3d> path, expand;
    if (!planner.plan({sx, sy, 0.0}, {gx, gy, 0.0}, path, expand)) {
      RCLCPP_WARN(this->get_logger(), "RRT failed to find path.");
      rate.sleep();
      continue;
    }

    nav_msgs::msg::Path path_msg;
    path_msg.header.frame_id = "map";
    path_msg.header.stamp = this->now();

    for (auto& pt : path) {
      geometry_msgs::msg::PoseStamped pose;
      pose.pose.position.x = pt.x() * resolution + origin_x;
      pose.pose.position.y = pt.y() * resolution + origin_y;
      pose.pose.position.z = 0.0;
      path_msg.poses.push_back(pose);
    }
    RCLCPP_INFO(this->get_logger(), "RRT published path.");
    path_pub_->publish(path_msg);
    rate.sleep();
  }
}


// 실험용 변환 함수 정의
void GlobalRRTPlanner::convertGPSToXY(double latitude, double longitude, double& x, double& y)
{
  const double earth_radius = 6378137.0;
  double lat_rad = latitude * M_PI / 180.0;
  double lon_rad = longitude * M_PI / 180.0;
  x = lon_rad * earth_radius * std::cos(lat_rad);
  y = lat_rad * earth_radius;
}

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<GlobalRRTPlanner>();
    
    rclcpp::spin(node);
    
    rclcpp::shutdown();
    return 0;
}