#include "nav_keti/dynamic_window_approach.h"

using namespace std;

DWA::DWA(double max_v, double v_resolution, double max_omega, double omega_resolution, 
        double direction_gain_, double speed_gain_, double obstacle_gain_, double dt): 
            max_v_(max_v),
            max_omega_(max_omega),
            v_resolution_(v_resolution),
            omega_resolution_(omega_resolution),
            direction_gain(direction_gain_),
            speed_gain(speed_gain_),
            obstacle_gain(obstacle_gain_),
            dt_(dt) {}

void DWA::laserCallback(const sensor_msgs::msg::LaserScan::SharedPtr msg) {
    lidar_data_.clear();
    // 라이다 데이터 추출하여 저장
    for (int i = 0; i < msg->ranges.size(); ++i) {
        double range = msg->ranges[i];
        
        if(msg->ranges[i]==numeric_limits<double>::infinity()){
            range=msg->range_max*1.3;
        }
        double angle = msg->angle_min + i * msg->angle_increment;
        lidar_data_.push_back(LidarPoint(range, angle));
        // if(!dt_) dt_=msg->range_max/max_v_;
    }
}

vector<double> DWA::computeVelocityCommands(const RobotState& robot_state, const Goal& goal, double dt) {
    vector<double> vel;
    double best_v = 0.0;
    double best_omega = 0.0;
    double best_cost = 0;
    vector<vector<double>> vw_set;
    vector<double> direction_costs, speed_costs, distance_to_obstacle_costs;
    
    for (double omega = -max_omega_; omega <= max_omega_; omega += omega_resolution_) {
        for (double v = 0; v <= max_v_; v += v_resolution_) {
            if(fabs(v) < v_resolution_/2) continue;
            vector<double> cost = computeCost(robot_state, goal, v, omega, dt_);
            if (cost[2] < fabs(v)*dt_) {
                break;
            }
            else{
                direction_costs.push_back(cost[0]);
                speed_costs.push_back(cost[1]);
                distance_to_obstacle_costs.push_back(cost[2]);
                vw_set.push_back({v,omega});
            }
        }

        for (double v = 0; v >= -max_v_; v -= v_resolution_) {
            if(fabs(v) < v_resolution_/2) continue;
            vector<double> cost = computeCost(robot_state, goal, v, omega, dt_);
            if (cost[2] < fabs(v)*dt_) {

                break;
            }

            else{
                direction_costs.push_back(cost[0]);
                speed_costs.push_back(cost[1]);
                distance_to_obstacle_costs.push_back(cost[2]);
                vw_set.push_back({v,omega});
            }
        }
    }
    
    auto normalize = [](vector<double>& costs) {

        double min_cost = numeric_limits<double>::max();
        double max_cost = numeric_limits<double>::min();

        for (const auto& cost : costs) {
            min_cost = min(min_cost, cost);
            max_cost = max(max_cost, cost);
        }

        double range = max_cost - min_cost;
        range = range > 0 ? range : 1.0; 
        for (auto& cost : costs) {
            cost = (cost - min_cost) / range;
        }
    };
    
    normalize(direction_costs);
    normalize(speed_costs);
    normalize(distance_to_obstacle_costs);

    for (size_t i = 0; i < direction_costs.size(); ++i) {
        if (distance_to_obstacle_costs[i] == 0) continue;
        double total_cost = direction_gain * direction_costs[i] +
                            speed_gain * speed_costs[i] +
                            obstacle_gain * distance_to_obstacle_costs[i];
        
        if (best_cost<total_cost) {
            
            best_cost = total_cost;
            best_v = vw_set[i][0];
            best_omega = vw_set[i][1];
        }
        //cout<<"dir: "<<direction_costs[i]<<" spd: "<<speed_costs[i]<<" obs: "<<distance_to_obstacle_costs[i]<<endl;
    }
    
    vel.push_back(best_v);
    vel.push_back(best_omega);
    return vel;
}

vector<double> DWA::computeCost(const RobotState& robot_state, const Goal& goal, double v, double omega, double dt) {
    vector<double> cost;
    
    double predict_theta = (robot_state.theta + omega*dt);
    double predict_x = robot_state.x + v * cos(predict_theta) * dt;
    double predict_y = robot_state.y + v * sin(predict_theta) * dt;

    double dir_dtheta = atan2(goal.y - robot_state.y, goal.x - robot_state.x);
    if (dir_dtheta < 0) {
        dir_dtheta += 2 * M_PI;
    }

    double direction_cost = fabs(dir_dtheta - predict_theta);
    if (direction_cost > M_PI) {
        direction_cost = fabs(2 * M_PI-direction_cost);
    }
    
    direction_cost=1-direction_cost/M_PI;

    double speed_cost = v + max_v_; 

    double min_distance_to_obstacle = numeric_limits<double>::max();

    for (const auto lidar_point : lidar_data_) {
        
        double obstacle_distance = lidar_point.range_;
        double obstacle_angle = lidar_point.angle_;

        double obstacle_x = robot_state.x + obstacle_distance * std::cos(obstacle_angle + robot_state.theta);
        double obstacle_y = robot_state.y + obstacle_distance * std::sin(obstacle_angle + robot_state.theta);

        double dx = obstacle_x - predict_x;
        double dy = obstacle_y - predict_y;

        double obstacle_dist = hypot(dx, dy);

        if (obstacle_dist < min_distance_to_obstacle) min_distance_to_obstacle = obstacle_dist;
    }
    
    cost.push_back(direction_cost);
    cost.push_back(speed_cost);
    cost.push_back(min_distance_to_obstacle);
    return cost;
}