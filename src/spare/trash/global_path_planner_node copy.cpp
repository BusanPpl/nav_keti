#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/path.hpp>
#include <nav2_msgs/msg/costmap.hpp>
#include <sensor_msgs/msg/nav_sat_fix.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>

#include "nav_keti/path_selector.h"

#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>

#include <memory>
#include <cmath>

class GlobalPathPlannerNode : public rclcpp::Node {
public:
    GlobalPathPlannerNode() : Node("global_path_planner_node"), planner_("rrt_star"), map_ready_(false), plan_start(false), robot_yaw_(0.0){
        this->declare_parameter<std::string>("planner_algorithm", "rrt_star");
        this->get_parameter("planner_algorithm", algorithm_type_);

        pub_path_ = this->create_publisher<nav_msgs::msg::Path>("planned_path", 10);

        costmap_sub_ = this->create_subscription<nav2_msgs::msg::Costmap>(
            "/costmap_raw", 10,
            std::bind(&GlobalPathPlannerNode::costmapCallback, this, std::placeholders::_1));

        gps_sub_ = this->create_subscription<sensor_msgs::msg::NavSatFix>(
            "/gps", 10,
            std::bind(&GlobalPathPlannerNode::gpsCallback, this, std::placeholders::_1));

        target_sub_ = this->create_subscription<sensor_msgs::msg::NavSatFix>(
            "/target_gps", 10,
            std::bind(&GlobalPathPlannerNode::targetCallback, this, std::placeholders::_1));

        imu_sub_ = this->create_subscription<sensor_msgs::msg::Imu>(
            "/imu", rclcpp::SensorDataQoS(),
            std::bind(&GlobalPathPlannerNode::imuCallback, this, std::placeholders::_1));

        goal_pose_sub_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
            "/goal_pose", 10,
            std::bind(&GlobalPathPlannerNode::goalPoseCallback, this, std::placeholders::_1));

        planner_.setMode(algorithm_type_); // 설정 잘못하면 코스트맵 등 입력이 꼬여버림
    }

private:
    std::string algorithm_type_;
    bool map_ready_, plan_start;
    double robot_x_, robot_y_, robot_yaw_;
    double target_x_, target_y_;

    nav2_msgs::msg::Costmap current_costmap_;

    Planner planner_;
    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr pub_path_;
    rclcpp::Subscription<nav2_msgs::msg::Costmap>::SharedPtr costmap_sub_;
    rclcpp::Subscription<sensor_msgs::msg::NavSatFix>::SharedPtr gps_sub_;
    rclcpp::Subscription<sensor_msgs::msg::NavSatFix>::SharedPtr target_sub_;
    rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub_;
    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr goal_pose_sub_;

    void gpsCallback(const sensor_msgs::msg::NavSatFix::SharedPtr msg) {
        convertGPSToXY(-msg->latitude, -msg->longitude, robot_x_, robot_y_);
        if(!plan_start)
        {
            plan_start = true;
            //RCLCPP_INFO(this->get_logger(), "Robot position set to x=%.2f, y=%.2f", robot_x_, robot_y_);
            convertGPSToXY(-msg->latitude, -msg->longitude + 0.0002, target_x_, target_y_);
            //RCLCPP_INFO(this->get_logger(), "Target position set to x=%.2f, y=%.2f", target_x_, target_y_);
        }
    }

    void targetCallback(const sensor_msgs::msg::NavSatFix::SharedPtr msg) {
        convertGPSToXY(msg->latitude, msg->longitude, target_x_, target_y_);
        if (map_ready_) {
            RCLCPP_INFO(this->get_logger(), "Robot position set to x=%.2f, y=%.2f", robot_x_, robot_y_);
        }
    }

    void costmapCallback(const nav2_msgs::msg::Costmap::SharedPtr msg) {
        current_costmap_ = *msg;
        planner_.setCostmapData(msg->data, msg->metadata.size_x, msg->metadata.size_y, msg->metadata.resolution);
        planPath();
    }

    void imuCallback(const sensor_msgs::msg::Imu::SharedPtr msg) {
        tf2::Quaternion quat(
            msg->orientation.x,
            msg->orientation.y,
            msg->orientation.z,
            msg->orientation.w
        );
        double roll, pitch;
        tf2::Matrix3x3(quat).getRPY(roll, pitch, robot_yaw_);
        if (robot_yaw_ < 0) robot_yaw_ += 2 * M_PI;
    }

    void goalPoseCallback(const geometry_msgs::msg::PoseStamped::SharedPtr msg) {
        // goal_pose는 이미 map 좌표 기준
        target_x_ = msg->pose.position.x;
        target_y_ = msg->pose.position.y;
        RCLCPP_INFO(this->get_logger(), "Goal frame: %s, x=%.2f, y=%.2f", msg->header.frame_id.c_str(), target_x_, target_y_);
        if (map_ready_) {
            planPath();
        }
    }




    
    void planPath() {
        
        double local_robot_x = ( - current_costmap_.metadata.origin.position.x) / current_costmap_.metadata.resolution;
        double local_robot_y = ( - current_costmap_.metadata.origin.position.y) / current_costmap_.metadata.resolution;

        double costmap_target_x = (target_x_ - robot_x_ - current_costmap_.metadata.origin.position.x) / current_costmap_.metadata.resolution;
        double costmap_target_y = (target_y_ - robot_y_ - current_costmap_.metadata.origin.position.y) / current_costmap_.metadata.resolution;

        double dx = costmap_target_x - local_robot_x;
        double dy = costmap_target_y - local_robot_y;

        double rotated_dx = dx * cos(robot_yaw_) + dy * sin(robot_yaw_);
        double rotated_dy = -dx * sin(robot_yaw_) + dy * cos(robot_yaw_);

        double rotated_target_x = local_robot_x + rotated_dx;
        double rotated_target_y = local_robot_y + rotated_dy;

        if (!clipToCostmapBoundary(local_robot_x, local_robot_y,rotated_target_x, rotated_target_y))
        {
            // RCLCPP_WARN(this->get_logger(), "Target clipped to costmap boundary.");
        }
        
        //RCLCPP_INFO(this->get_logger(), "Local robot position: x=%.2f, y=%.2f", rotated_target_x, rotated_target_y);
        nav_msgs::msg::Path path_msg;
        path_msg.header.frame_id = "map";
        path_msg.header.stamp = this->now();

        // RRT*는 로봇이 중심에 있다는 전제로 plan 시작
        planner_.setStart({local_robot_x, local_robot_y});
        planner_.setGoal({rotated_target_x, rotated_target_y});
        auto raw_path = planner_.plan();
        //RCLCPP_INFO(this->get_logger(), "Raw path size: %zu", raw_path.size());
        auto smoothed_path = planner_.smoothPath(raw_path, 130);
        
        //RCLCPP_INFO(this->get_logger(), "Path size: %zu", smoothed_path.size());
        int center_x = current_costmap_.metadata.size_x / 2;
        int center_y = current_costmap_.metadata.size_y / 2;
        double resolution = current_costmap_.metadata.resolution;
        
        for (const auto& coordinate : smoothed_path) {
            // base_link 기준 상대 좌표 (m 단위)
            double x_local = (coordinate.x - center_x) * resolution;
            double y_local = (coordinate.y - center_y) * resolution;
        
            // 회전 보상 (base_link → map)
            double x_rot = x_local * cos(robot_yaw_) - y_local * sin(robot_yaw_);
            double y_rot = x_local * sin(robot_yaw_) + y_local * cos(robot_yaw_);
        
            geometry_msgs::msg::PoseStamped point_pose;
            point_pose.pose.position.x = robot_x_ + x_rot;
            point_pose.pose.position.y = robot_y_ + y_rot;
            point_pose.pose.position.z = 0.0;
        
            //RCLCPP_INFO(this->get_logger(), "Path point: x=%.2f, y=%.2f", point_pose.pose.position.x, point_pose.pose.position.y);
            path_msg.poses.push_back(point_pose);
        }
        pub_path_->publish(path_msg);
    }

    void convertGPSToXY(double lat, double lon, double& x, double& y) {
        double lat_rad = lat * M_PI / 180.0;
        double lon_rad = lon * M_PI / 180.0;
        const double R = 6378137.0;
        x = lon_rad * R * cos(lat_rad);
        y = lat_rad * R;
    }

    bool clipToCostmapBoundary(double robot_x, double robot_y, double &target_x, double &target_y)
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
    // bool clipToCostmapBoundary(const nav2_msgs::msg::Costmap& costmap,
    //     double robot_x, double robot_y, double robot_yaw,
    //     double& target_x, double& target_y)
    //     {
    //     const auto& meta = costmap.metadata;
    //     double origin_x = meta.origin.position.x;
    //     double origin_y = meta.origin.position.y;
    //     double resolution = meta.resolution;
    //     int size_x = meta.size_x;
    //     int size_y = meta.size_y;

    //     // 1. robot 기준에서 방향 벡터 구하기
    //     double dx = target_x - robot_x;
    //     double dy = target_y - robot_y;

    //     // 2. robot_yaw 기준 회전: 로컬 좌표계 (base_link 기준)
    //     double local_x =  dx * cos(-robot_yaw) - dy * sin(-robot_yaw);
    //     double local_y =  dx * sin(-robot_yaw) + dy * cos(-robot_yaw);

    //     // 3. costmap 중심 (robot이 중심이라고 가정)
    //     int cx = size_x / 2;
    //     int cy = size_y / 2;

    //     // 4. 목표 지점 → costmap 인덱스
    //     int map_x = static_cast<int>(cx + local_x / resolution);
    //     int map_y = static_cast<int>(cy + local_y / resolution);

    //     // 5. 경계 검사
    //     if (map_x < 0 || map_x >= size_x || map_y < 0 || map_y >= size_y)
    //     {
    //     // costmap 경계 클리핑
    //     map_x = std::min(std::max(map_x, 0), size_x - 1);
    //     map_y = std::min(std::max(map_y, 0), size_y - 1);
    //     }

    //     // 6. 다시 map 좌표계로 변환
    //     double clipped_local_x = (map_x - cx) * resolution;
    //     double clipped_local_y = (map_y - cy) * resolution;

    //     double global_dx = clipped_local_x * cos(robot_yaw) - clipped_local_y * sin(robot_yaw);
    //     double global_dy = clipped_local_x * sin(robot_yaw) + clipped_local_y * cos(robot_yaw);

    //     target_x = robot_x + global_dx;
    //     target_y = robot_y + global_dy;

    //     return true;
    //     }
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<GlobalPathPlannerNode>());
    rclcpp::shutdown();
    return 0;
}
