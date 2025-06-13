#include "nav_keti/global_path_planner.h"

using namespace std::chrono_literals;
using namespace rclcpp;
using namespace std;

GlobalPathPlanner::GlobalPathPlanner() : Node("Global_path_generator"), map_exist_(false), pose_flag(false), robot_yaw(0.0)
{
    RCLCPP_INFO(this->get_logger(), "Global_path_generator node initialized.");

    pub_robot_path_ = this->create_publisher<nav_msgs::msg::Path>("planned_path", 10);

    costmap_sub_ = this->create_subscription<nav2_msgs::msg::Costmap>(
        "/costmap_raw", 10, bind(&GlobalPathPlanner::costmapCallback, this, placeholders::_1));

    gps_sub_ = this->create_subscription<sensor_msgs::msg::NavSatFix>(
        "/gps", 10, bind(&GlobalPathPlanner::gpsCallback, this, placeholders::_1));

    target_gps_sub_ = this->create_subscription<sensor_msgs::msg::NavSatFix>(
        "/target_gps", 10, bind(&GlobalPathPlanner::targetGpsCallback, this, placeholders::_1));
    
    imu_subscriber_ = this->create_subscription<sensor_msgs::msg::Imu>(
        "imu", rclcpp::SensorDataQoS(), std::bind(&GlobalPathPlanner::imuCallback, this, placeholders::_1));

    costmap_thread_ = std::thread(&GlobalPathPlanner::costmapProcessingLoop, this);
}

GlobalPathPlanner::~GlobalPathPlanner()
{
    stop_thread_ = true;
    if (costmap_thread_.joinable())
    {
        costmap_thread_.join();
    }
    RCLCPP_INFO(this->get_logger(), "Global_path_generator node terminated.");
}

void GlobalPathPlanner::gpsCallback(const sensor_msgs::msg::NavSatFix::SharedPtr msg)
{   
    convertGPSToXY(-msg->latitude, -msg->longitude, robot_x, robot_y);

    if(!f1) {
        f1 = true;
        convertGPSToXY(-msg->latitude, -msg->longitude + 0.002, target_x, target_y); 
        RCLCPP_INFO(this->get_logger(), "Robot position set to x=%.2f, y=%.2f", robot_x, robot_y);
        RCLCPP_INFO(this->get_logger(), "Target position set to x=%.2f, y=%.2f", target_x, target_y);
    }
}

void GlobalPathPlanner::targetGpsCallback(const sensor_msgs::msg::NavSatFix::SharedPtr msg)
{
    // target_latitude_ = msg->latitude;
    // target_longitude_ = msg->longitude;
    convertGPSToXY(msg->latitude, msg->longitude, target_x, target_y);
    navGoalHandler();
}

void GlobalPathPlanner::costmapCallback(const nav2_msgs::msg::Costmap::SharedPtr msg)
{
    std::lock_guard<std::mutex> lock(costmap_mutex_);
    latest_costmap_msg_ = msg;
}

void GlobalPathPlanner::imuCallback(const sensor_msgs::msg::Imu::SharedPtr msg) {
    tf2::Quaternion quat(
        msg->orientation.x,
        msg->orientation.y,
        msg->orientation.z,
        msg->orientation.w
    );

    double roll, pitch;
    tf2::Matrix3x3(quat).getRPY(roll, pitch, robot_yaw);
    if (robot_yaw < 0) {
        robot_yaw += 2 * M_PI;
    }

}

void GlobalPathPlanner::costmapProcessingLoop()
{
    while (!stop_thread_ && rclcpp::ok())
    {
        {
            std::lock_guard<std::mutex> lock(costmap_mutex_);
            if (latest_costmap_msg_)
            {
                current_costmap_ = *latest_costmap_msg_;
                updateAStarMap();
                // RCLCPP_INFO(this->get_logger(), "Processing done.");
                if (map_exist_) navGoalHandler();
                //데이터 초기화
                map_generator_.clearCollisions();
                latest_costmap_msg_.reset();
                // std::this_thread::sleep_for(std::chrono::duration<double>(publish_interval_));

            }
        }
    }
}

void GlobalPathPlanner::updateAStarMap()
{
    map_generator_.setWorldSize({(int)current_costmap_.metadata.size_x, (int)current_costmap_.metadata.size_y});
    map_generator_.setHeuristic(AStar::Heuristic::manhattan);
    map_generator_.setDiagonalMovement(true);

    std::vector<uint8_t> costmap_data(current_costmap_.data.begin(), current_costmap_.data.end());
    map_generator_.setCostmap(costmap_data);

    // for (size_t i = 0; i < current_costmap_.data.size(); ++i) {
    //     int x = i % current_costmap_.metadata.size_x;
    //     int y = i / current_costmap_.metadata.size_x;
    //     if(current_costmap_.data[i] != cost::NO_INFORMATION){
    //         if (current_costmap_.data[i] > 1000) { 
    //             map_generator_.addCollision({x, y}, 0);
    //             if (current_costmap_.data[i] > 250) RCLCPP_INFO(this->get_logger(), "Obstacle added at x=%d, y=%d", x, y);
    //         }
    //     }
    // }

    map_exist_ = true;
}

void GlobalPathPlanner::navGoalHandler()
{
    if (!map_exist_) return;
    //좌표 변환 수식 다시 체크해야
    double local_robot_x = (- current_costmap_.metadata.origin.position.x) / current_costmap_.metadata.resolution;
    double local_robot_y = (- current_costmap_.metadata.origin.position.y) / current_costmap_.metadata.resolution;

    double costmap_target_x = (target_x - robot_x - current_costmap_.metadata.origin.position.x) / current_costmap_.metadata.resolution;
    double costmap_target_y = (target_y - robot_y - current_costmap_.metadata.origin.position.y) / current_costmap_.metadata.resolution;

    double dx = costmap_target_x - local_robot_x;
    double dy = costmap_target_y - local_robot_y;

    double rotated_dx = dx * cos(robot_yaw) + dy * sin(robot_yaw);
    double rotated_dy = -dx * sin(robot_yaw) + dy * cos(robot_yaw);

    double rotated_target_x = local_robot_x + rotated_dx;
    double rotated_target_y = local_robot_y + rotated_dy;

    if (!clipToCostmapBoundary(local_robot_x, local_robot_y, rotated_target_x, rotated_target_y))
    {
        //RCLCPP_WARN(this->get_logger(), "Target clipped to costmap boundary.");
    }

    AStar::Vec2i source = {static_cast<int>(local_robot_x), static_cast<int>(local_robot_y)};
    AStar::Vec2i target = {static_cast<int>(rotated_target_x), static_cast<int>(rotated_target_y)};
    auto path = map_generator_.findPath(source, target);

    if (path.empty()) {
        RCLCPP_WARN(this->get_logger(), "No path found.");
        return;
    }

    nav_msgs::msg::Path path_msg;
    path_msg.header.frame_id = "map";
    path_msg.header.stamp = this->now();

    for (const auto& coordinate : path) {
        geometry_msgs::msg::PoseStamped point_pose;
        point_pose.pose.position.x = coordinate.x * current_costmap_.metadata.resolution + current_costmap_.metadata.origin.position.x;
        point_pose.pose.position.y = coordinate.y * current_costmap_.metadata.resolution + current_costmap_.metadata.origin.position.y;
        point_pose.pose.position.z = 0.0;

        path_msg.poses.push_back(point_pose);
    }
    pub_robot_path_->publish(path_msg);
}


void GlobalPathPlanner::convertGPSToXY(double latitude, double longitude, double &x, double &y)
{

    double lat_rad = latitude * M_PI / 180.0;
    double lon_rad = longitude * M_PI / 180.0;

    // 위도, 경도를 미터 단위로 변환 (지구 반경 6378137m, 평면 근사)
    const double earth_radius = 6378137.0;  // 미터 단위 (WGS-84)

    // 위도 및 경도 차이를 거리(m)로 변환 (위도 차이는 항상 일정)
    x = lon_rad * earth_radius * cos(lat_rad);  // 경도는 위도에 따라 조정
    y = lat_rad * earth_radius;
}

bool GlobalPathPlanner::clipToCostmapBoundary(double robot_x, double robot_y, double &target_x, double &target_y)
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
    init(argc, argv);
    auto PG = std::make_shared<GlobalPathPlanner>();
    spin(PG);
    shutdown();
    return 0;
}