#ifndef DWA_H
#define DWA_H

#include <sensor_msgs/msg/laser_scan.hpp>
#include <iostream>
#include <cmath>
#include <vector>
#include <limits>
#include <numeric>
#include <tuple>


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
    double x, y;
    Goal(double x_, double y_) : x(x_), y(y_) {}
};


class DWA {
public:
    /*
    All Double
    Velocity, V_res, Omega, O_res
    Gain->Dir, spd, Obs
    dt
    */
    DWA(double max_v, double v_resolution, double max_omega, double omega_resolution, 
        double direction_gain_, double speed_gain_, double obstacle_gain_, double dt);    
    void laserCallback(const sensor_msgs::msg::LaserScan::SharedPtr msg);
    std::vector<double> computeVelocityCommands(const RobotState& robot_state, const Goal& goal, double dt);

private:
    std::vector<LidarPoint> lidar_data_;                     // 라이다 정보
    double max_v_, max_omega_;                               // 최대 선속도,최대 각속도
    double v_resolution_, omega_resolution_;                 // 선속도 해상도, 각속도 해상도
    double dt_;                                              // 시간 간격
    double direction_gain, speed_gain, obstacle_gain;
    double total_cost;
    std::vector<double> computeCost(const RobotState& robot_state, const Goal& goal,
                                    double v, double omega, double dt);
};
#endif 
