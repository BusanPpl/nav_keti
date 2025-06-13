#include "nav_keti/global_path_planner_rrt.h"
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>

using namespace std::chrono_literals;
using namespace std;

GlobalPathPlannerRRT::GlobalPathPlannerRRT()
    : Node("Global_path_generator_rrt"),
      stop_thread_(false), map_exist_(false), pose_flag_(false), f1_(false),
      robot_yaw_(0.0), rrt_planner_(3.0, 0.5, 30, 500000)
{
    RCLCPP_INFO(this->get_logger(), "Global_path_generator_rrt node initialized.");

    pub_robot_path_ = this->create_publisher<nav_msgs::msg::Path>("planned_path", 10);

    costmap_sub_ = this->create_subscription<nav2_msgs::msg::Costmap>(
        "/costmap_raw", 10, bind(&GlobalPathPlannerRRT::costmapCallback, this, placeholders::_1));

    gps_sub_ = this->create_subscription<sensor_msgs::msg::NavSatFix>(
        "/gps", 10, bind(&GlobalPathPlannerRRT::gpsCallback, this, placeholders::_1));

    target_gps_sub_ = this->create_subscription<sensor_msgs::msg::NavSatFix>(
        "/target_gps", 10, bind(&GlobalPathPlannerRRT::targetGpsCallback, this, placeholders::_1));

    imu_subscriber_ = this->create_subscription<sensor_msgs::msg::Imu>(
        "imu", rclcpp::SensorDataQoS(), std::bind(&GlobalPathPlannerRRT::imuCallback, this, placeholders::_1));

    costmap_thread_ = std::thread(&GlobalPathPlannerRRT::costmapProcessingLoop, this);
}

GlobalPathPlannerRRT::~GlobalPathPlannerRRT()
{
    stop_thread_ = true;
    if (costmap_thread_.joinable())
        costmap_thread_.join();
}

void GlobalPathPlannerRRT::gpsCallback(const sensor_msgs::msg::NavSatFix::SharedPtr msg)
{
    convertGPSToXY(-msg->latitude, -msg->longitude, robot_x_, robot_y_);
    if (!f1_)
    {
        f1_ = true;
        convertGPSToXY(-msg->latitude, -msg->longitude + 0.002, target_x_, target_y_);
        RCLCPP_INFO(this->get_logger(), "Robot position set to x=%.2f, y=%.2f", robot_x_, robot_y_);
        RCLCPP_INFO(this->get_logger(), "Target position set to x=%.2f, y=%.2f", target_x_, target_y_);
    }
}

void GlobalPathPlannerRRT::targetGpsCallback(const sensor_msgs::msg::NavSatFix::SharedPtr msg)
{
    convertGPSToXY(msg->latitude, msg->longitude, target_x_, target_y_);
    navGoalHandler();
}

void GlobalPathPlannerRRT::costmapCallback(const nav2_msgs::msg::Costmap::SharedPtr msg)
{
    std::lock_guard<std::mutex> lock(costmap_mutex_);
    latest_costmap_msg_ = msg;
}

void GlobalPathPlannerRRT::imuCallback(const sensor_msgs::msg::Imu::SharedPtr msg)
{
    tf2::Quaternion quat(msg->orientation.x, msg->orientation.y, msg->orientation.z, msg->orientation.w);
    double roll, pitch;
    tf2::Matrix3x3(quat).getRPY(roll, pitch, robot_yaw_);
    if (robot_yaw_ < 0)
        robot_yaw_ += 2 * M_PI;
}

void GlobalPathPlannerRRT::costmapProcessingLoop()
{
    while (!stop_thread_ && rclcpp::ok())
    {
        {
            std::lock_guard<std::mutex> lock(costmap_mutex_);
            if (latest_costmap_msg_)
            {
                current_costmap_ = *latest_costmap_msg_;
                updateRRTMap();
                if (map_exist_)
                    navGoalHandler();

                latest_costmap_msg_.reset();
            }
        }
    }
}

void GlobalPathPlannerRRT::updateRRTMap()
{
    std::vector<std::tuple<double, double, double>> obstacle_list;

    int width = current_costmap_.metadata.size_x;
    int height = current_costmap_.metadata.size_y;
    double resolution = current_costmap_.metadata.resolution;

    for (size_t i = 0; i < current_costmap_.data.size(); ++i)
    {
        if (current_costmap_.data[i] > 30) // 장애물로 간주할 threshold
        {
            int x = i % width;
            int y = i / width;
            double world_x = x * resolution + current_costmap_.metadata.origin.position.x;
            double world_y = y * resolution + current_costmap_.metadata.origin.position.y;
            obstacle_list.emplace_back(world_x, world_y, 0.5); // 반지름 0.5m로 가정
        }
    }

    rrt_planner_.setObstacles(obstacle_list);
    //RCLCPP_INFO(this->get_logger(), "Obstacle count: %zu", obstacle_list.size());
    map_exist_ = true;
}

void GlobalPathPlannerRRT::navGoalHandler()
{
    // 먼저 grid index 계산
    // double local_robot_x = (robot_x_ - origin_x) / resolution;
    // double local_robot_y = (robot_y_ - origin_y) / resolution;

    double local_robot_x = (- current_costmap_.metadata.origin.position.x) / current_costmap_.metadata.resolution;
    double local_robot_y = (- current_costmap_.metadata.origin.position.y) / current_costmap_.metadata.resolution;

    double costmap_target_x = (target_x_ - robot_x_ - current_costmap_.metadata.origin.position.x) / current_costmap_.metadata.resolution;
    double costmap_target_y = (target_y_ - robot_y_ - current_costmap_.metadata.origin.position.y) / current_costmap_.metadata.resolution;

    double dx = costmap_target_x - local_robot_x;
    double dy = costmap_target_y - local_robot_y; 

    double rotated_dx = dx * cos(robot_yaw_) + dy * sin(robot_yaw_);
    double rotated_dy = -dx * sin(robot_yaw_) + dy * cos(robot_yaw_);

    double rotated_target_x = local_robot_x + rotated_dx;
    double rotated_target_y = local_robot_y + rotated_dy;
    
    // RCLCPP_INFO(this->get_logger(), "Target position set to x=%.2f, y=%.2f", rotated_target_x, rotated_target_y);

    if (!clipToCostmapBoundary(local_robot_x, local_robot_y, rotated_target_x, rotated_target_y))
    {
        //RCLCPP_WARN(this->get_logger(), "Target clipped to costmap boundary.");
    }

    RCLCPP_INFO(this->get_logger(), "Target position set to x=%.2f, y=%.2f", rotated_target_x - local_robot_x, rotated_target_y - local_robot_y);

    rrt_planner_.setStart({0, 0}); // 시작지점이 0,0
    rrt_planner_.setGoal({rotated_target_x - local_robot_x, rotated_target_y - local_robot_y}); //따라서 로컬 원점을 뺴서 계산
    
    double xmin = current_costmap_.metadata.origin.position.x;
    double xmax = xmin + current_costmap_.metadata.size_x * current_costmap_.metadata.resolution;
    double ymin = current_costmap_.metadata.origin.position.y;
    double ymax = ymin + current_costmap_.metadata.size_y * current_costmap_.metadata.resolution;
    rrt_planner_.setBounds(xmin, xmax, ymin, ymax);
    //RCLCPP_INFO(this->get_logger(), "Start (%.2f, %.2f), Goal (%.2f, %.2f)", robot_x_, robot_y_, target_x_, target_y_);
    
    auto path = rrt_planner_.plan();
    if (path.empty())
    {
        RCLCPP_WARN(this->get_logger(), "No path found.");
        return;
    }

    nav_msgs::msg::Path path_msg;
    path_msg.header.frame_id = "map";
    path_msg.header.stamp = this->now();

    for (const auto &pt : path)
    {
        geometry_msgs::msg::PoseStamped pose;
        pose.pose.position.x = pt.x;
        pose.pose.position.y = pt.y;
        pose.pose.position.z = 0.0;
        path_msg.poses.push_back(pose);
    }

    pub_robot_path_->publish(path_msg);
}

void GlobalPathPlannerRRT::convertGPSToXY(double latitude, double longitude, double &x, double &y)
{
    double lat_rad = latitude * M_PI / 180.0;
    double lon_rad = longitude * M_PI / 180.0;
    const double earth_radius = 6378137.0;
    x = lon_rad * earth_radius * cos(lat_rad);
    y = lat_rad * earth_radius;
}

bool GlobalPathPlannerRRT::clipToCostmapBoundary(double robot_x, double robot_y, double &target_x, double &target_y)
{
    int map_min_x = 0;
    int map_min_y = 0;
    int map_max_x = current_costmap_.metadata.size_x - 1;
    int map_max_y = current_costmap_.metadata.size_y - 1;

    int x0 = static_cast<int>(std::round(robot_x));
    int y0 = static_cast<int>(std::round(robot_y));
    int x1 = static_cast<int>(std::round(target_x));
    int y1 = static_cast<int>(std::round(target_y));
    // Bresenham's line algorithm
    int dx = abs(x1 - x0);
    int dy = abs(y1 - y0);
    int sx = (x0 < x1) ? 1 : -1;
    int sy = (y0 < y1) ? 1 : -1;
    int err = dx - dy;

    int count = 0;
    while (true)
    {
        if (x0 < map_min_x || x0 >= map_max_x || y0 < map_min_y || y0 >= map_max_y)
        {
            target_x = x0 - sx;
            target_y = y0 - sy;
        
            return false;
        }

        if (x0 == x1 && y0 == y1)
            break;

        int e2 = 2 * err;
        if (e2 > -dy) {
            err -= dy;
            x0 += sx;
        } else if (e2 < dx) {
            err += dx;
            y0 += sy;
        }

        if (++count > 10000)
        {
            RCLCPP_ERROR(this->get_logger(), "Loop exceeded limit, exiting to avoid infinite loop.");
            return false;
        }
    }

    target_x = x1;
    target_y = y1;
    // RCLCPP_INFO(this->get_logger(), "Target position adjusted to x=%.2f, y=%.2f", target_x, target_y);

    return true;
}


int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<GlobalPathPlannerRRT>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}