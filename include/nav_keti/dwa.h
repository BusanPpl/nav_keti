#ifndef DWA_H
#define DWA_H

#include <sensor_msgs/msg/laser_scan.hpp>
#include <nav2_msgs/msg/costmap.hpp>
#include <iostream>
#include <cmath>
#include <vector>
#include <limits>
#include <numeric>
#include <tuple>

#include "cost_values.h"
#include "angles/angles.h"
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/pose2_d.hpp>


class CostmapData {
    public:
        // 기본 생성자
        CostmapData()
          : width_(0), height_(0), resolution_(0.0) {}
    
        // nav2_msgs::msg::Costmap 메시지로부터 채우는 함수
        void fromCostmapMsg(const nav2_msgs::msg::Costmap &msg)
        {
            width_      = msg.metadata.size_x;
            height_     = msg.metadata.size_y;
            resolution_ = msg.metadata.resolution;
            origin_     = msg.metadata.origin;
    
            // data_ 벡터에 복사 (1차원 배열)
            data_.assign(msg.data.begin(), msg.data.end());
        }
    
        // (x, y)가 맵 범위 안에 있는지 확인
        bool inBounds(int x, int y) const
        {
            return (x >= 0 && x < width_ && y >= 0 && y < height_);
        }
    
        uint8_t getCost(int x, int y) const
        {
            if (!inBounds(x, y)) {
                return 255;
            }
            return data_[y * width_ + x];
        }
    
        // (x, y) 셀의 cost 값 쓰기
        void setCost(int x, int y, uint8_t cost)
        {
            if (!inBounds(x, y)) {
                return;
            }
            data_[y * width_ + x] = cost;
        }
    
        // 해상도, 크기, 원점 등에 대한 Getter
        int getWidth() const { return width_; }
        int getHeight() const { return height_; }
        double getResolution() const { return resolution_; }
        geometry_msgs::msg::Pose getOrigin() const { return origin_; }
    
        // (map_x, map_y) -> 월드 좌표
        // std::pair<double,double> toWorldCoords(int map_x, int map_y) const
        // {
        //     double wx = origin_.position.x + (map_x + 0.5) * resolution_;
        //     double wy = origin_.position.y + (map_y + 0.5) * resolution_;
        //     return {wx, wy};
        // }
        
        std::pair<double,double> toWorldCoords(int map_x, int map_y, double robot_x, double robot_y, double robot_yaw) const
        {
            double local_x = origin_.position.x + (map_x + 0.5) * resolution_;
            double local_y = origin_.position.y + (map_y + 0.5) * resolution_;
        
            double wx = robot_x + cos(robot_yaw) * local_x - sin(robot_yaw) * local_y;
            double wy = robot_y + sin(robot_yaw) * local_x + cos(robot_yaw) * local_y;
        
            return {wx, wy};
        }
        
        // (월드 좌표) -> (map_x, map_y) 셀 인덱스
        // std::pair<int,int> toMapCoords(double wx, double wy) const
        // {
        //     int mx = static_cast<int>((wx - origin_.position.x) / resolution_);
        //     int my = static_cast<int>((wy - origin_.position.y) / resolution_);
        //     return {mx, my};
        // }

        std::pair<int,int> toMapCoords(double wx, double wy, double robot_x, double robot_y, double robot_yaw) const
        {
            double dx = wx - robot_x;
            double dy = wy - robot_y;

            double rotated_x =  cos(-robot_yaw) * dx - sin(-robot_yaw) * dy;
            double rotated_y =  sin(-robot_yaw) * dx + cos(-robot_yaw) * dy;

            int mx = static_cast<int>((rotated_x - origin_.position.x) / resolution_);
            int my = static_cast<int>((rotated_y - origin_.position.y) / resolution_);

            return {mx, my};
        }
    
    private:
        int width_;                       // costmap의 가로 폭(셀 단위)
        int height_;                      // costmap의 세로 높이(셀 단위)
        double resolution_;               // 해상도 (m/셀)
        geometry_msgs::msg::Pose origin_; // 맵의 원점 (월드 좌표계에서)
        std::vector<uint8_t> data_;       // 실제 cost 데이터 (1차원)
    };

class LidarPoint {
public:
    double range_, angle_;
    LidarPoint(double range, double angle) : range_(range), angle_(angle) {}
};

class RobotState {
public:
    double x, y, theta;
    RobotState(double x_, double y_, double theta_) : x(x_), y(y_), theta(theta_) {}
};

class Goal {
public:
    double x, y, theta;
    Goal(double x_, double y_) : x(x_), y(y_), theta(0) {}
    Goal(double x_, double y_, double theta_) : x(x_), y(y_), theta(theta_) {}
};

class DWA {
public:
    DWA(double max_v, double v_resolution, double max_omega, double omega_resolution, double dt, const std::vector<double>& weights);    
    void laserCallback(const sensor_msgs::msg::LaserScan::SharedPtr msg);
    void costmapCallback(const nav2_msgs::msg::Costmap::SharedPtr msg);
    void updateGlobalPlan(const std::vector<geometry_msgs::msg::PoseStamped>& plan);
    std::vector<double> computeVelocityCommands(const RobotState& robot_state, const Goal& goal, double dt);

private:
    std::vector<double> weights_;
    CostmapData costmap_data_;
    std::vector<LidarPoint> lidar_data_;     
    std::vector<geometry_msgs::msg::PoseStamped> global_plan_;                // 라이다 정보
    double max_v_, max_omega_;                               // 최대 선속도,최대 각속도
    double v_resolution_, omega_resolution_;                 // 선속도 해상도, 각속도 해상도
    double dt_;                                              // 시간 간격
    double direction_gain, speed_gain, obstacle_gain;
    double total_cost;
    std::vector<double> computeCost(const RobotState& robot_state, const Goal& goal,
                                    double v, double omega, double dt);
    double shortest_angular_distance(double from, double to);
};
#endif 
