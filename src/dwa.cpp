#include "nav_keti/dwa.h"

using namespace std;

DWA::DWA(double max_v, double v_resolution, double max_omega, double omega_resolution, double dt, const std::vector<double>& weights): 
            max_v_(max_v),
            max_omega_(max_omega),
            v_resolution_(v_resolution),
            omega_resolution_(omega_resolution),
            dt_(dt),
            weights_(weights) {}

void DWA::laserCallback(const sensor_msgs::msg::LaserScan::SharedPtr msg) {
    lidar_data_.clear();
    if (msg->ranges.empty()) {
        std::cerr << "[DWA] Error: received empty LaserScan data!" << std::endl;
        return;
    }
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
    if (lidar_data_.empty()) {
        std::cerr << "[DWA] Error: LIDAR data is empty after parsing!" << std::endl;
    }
}

void DWA::costmapCallback(const nav2_msgs::msg::Costmap::SharedPtr msg) {
    //cout << "Costmap Callback" << endl;
    costmap_data_.fromCostmapMsg(*msg);
}

void DWA::updateGlobalPlan(const std::vector<geometry_msgs::msg::PoseStamped>& plan) {
    //cout << "Global Plan Callback" << endl;
    global_plan_.clear();
    global_plan_ = plan;
}

double DWA::shortest_angular_distance(double from, double to) {
    double diff = to - from;
    while (diff > M_PI) diff -= 2.0 * M_PI;
    while (diff < -M_PI) diff += 2.0 * M_PI;
    return diff;
}

vector<double> DWA::computeVelocityCommands(const RobotState& robot_state, const Goal& goal, double dt) {
    vector<double> vel;
    double best_v = 0.0;
    double best_omega = 0.0;
    double best_cost = numeric_limits<double>::max();
    vector<vector<double>> vw_set, costs_list;
    if (costmap_data_.getWidth() == 0 || costmap_data_.getHeight() == 0) {
        std::cerr << "[DWA] Costmap not ready yet!" << std::endl;
        return {0.0, 0.0};
    }
    // 인덱스 기반 반복으로 정확한 탐색 영역 보장
    int omega_steps = static_cast<int>(round((2.0 * max_omega_) / omega_resolution_));
    int v_steps = static_cast<int>(round((2.0 * max_v_) / v_resolution_));

    // 첫 번째 루프
    for (int i = 0; i <= omega_steps; ++i) {
        double omega = -max_omega_ + i * omega_resolution_;
        for (int j = 0; j <= v_steps; ++j) {
            double v = -max_v_ + j * v_resolution_;
            if (fabs(v) < v_resolution_ / 2.0) continue;

            vector<double> cost = computeCost(robot_state, goal, v, omega, dt_);
            costs_list.push_back(cost);
            vw_set.push_back({v, omega});
        }
    }

    if (costs_list.empty() || costs_list[0].empty()) {
        std::cerr << "[DWA] No valid velocity samples generated or cost vector is empty!" << std::endl;
        return {0.0, 0.0};
    }
    // 두 번째 루프 (조건 기반 필터링 포함)
    for (int i = 0; i <= omega_steps; ++i) {
        double omega = -max_omega_ + i * omega_resolution_;

        // 양의 속도 방향
        for (int j = 0; j <= v_steps; ++j) {
            double v = j * v_resolution_;
            if (fabs(v) < v_resolution_ / 2.0) continue;

            vector<double> cost = computeCost(robot_state, goal, v, omega, dt_);
            if (cost[2] < fabs(v) * dt_) break;

            costs_list.push_back(cost);
            vw_set.push_back({v, omega});
        }

        // 음의 속도 방향
        for (int j = 0; j <= v_steps; ++j) {
            double v = -j * v_resolution_;
            if (fabs(v) < v_resolution_ / 2.0) continue;

            vector<double> cost = computeCost(robot_state, goal, v, omega, dt_);
            if (cost[2] < fabs(v) * dt_) break;

            costs_list.push_back(cost);
            vw_set.push_back({v, omega});
        }
    }

    auto normalize = [](vector<double>& costs) {
        double min_cost = *min_element(costs.begin(), costs.end());
        double max_cost = *max_element(costs.begin(), costs.end());

        double range = max_cost - min_cost;
        range = (range > 0) ? range : 1.0; // 범위가 0이면 1로 설정하여 나눗셈 방지
        for (auto& cost : costs) {
            cost = (cost - min_cost) / range;
        }
    };
    if (costs_list.empty() || costs_list[0].empty()) {
        std::cerr << "[DWA] No valid velocity samples generated or cost vector is empty!" << std::endl;
        return {0.0, 0.0};
    }
    size_t num_costs = costs_list[0].size();
    vector<vector<double>> separated_costs(num_costs);

    for (const auto& cost : costs_list) {
        if (cost.size() != num_costs) {
            std::cerr << "[DWA] cost.size() != expected num_costs (" 
                      << cost.size() << " vs " << num_costs << ")" << std::endl;
            return {0.0, 0.0};
        }
        for (size_t i = 0; i < num_costs; ++i) {
            separated_costs[i].push_back(cost[i]);
        }
    }

    if (separated_costs[0].size() != vw_set.size()) {
        std::cerr << "[DWA] separated_costs size mismatch with vw_set! "
                  << separated_costs[0].size() << " vs " << vw_set.size() << std::endl;
        return {0.0, 0.0};
    }
    for (auto& cost_vec : separated_costs) {
        normalize(cost_vec);
    }
    int q=0;

    for (size_t i = 0; i < vw_set.size(); i++) {
        double total_cost = 
            weights_[0] * separated_costs[0][i] +
            weights_[1] * separated_costs[1][i] +
            weights_[2] * separated_costs[2][i] +
            weights_[3] * separated_costs[3][i] +
            weights_[4] * separated_costs[4][i] +
            weights_[5] * separated_costs[5][i] +
            weights_[6] * separated_costs[6][i] +
            weights_[7] * separated_costs[7][i] +
            weights_[8] * separated_costs[8][i] +
            weights_[9] * separated_costs[9][i] +
            weights_[10] * separated_costs[10][i] +
            weights_[11] * separated_costs[11][i];
        
        if (total_cost < best_cost) {
            best_cost = total_cost;
            best_v = vw_set[i][0];
            best_omega = vw_set[i][1];
            q = i;
        }
    }
    
    //cout<<separated_costs[0][q]<<" "<<separated_costs[1][q]<<" "<<separated_costs[2][q]<<" "<<separated_costs[3][q]<<" "<<separated_costs[4][q]<<endl;
    vel.push_back(best_v);
    vel.push_back(best_omega);
    return vel;
}

vector<double> DWA::computeCost(const RobotState& robot_state, const Goal& goal, double v, double omega, double dt) {
    //std::cout << "robot_state: " << robot_state.x << ", " << robot_state.y << ", " << robot_state.theta << std::endl;
    vector<double> cost;
    // 예측 위치 계산
    double predict_theta = (robot_state.theta + omega*dt);
    double predict_x = robot_state.x + v * cos(predict_theta) * dt;
    double predict_y = robot_state.y + v * sin(predict_theta) * dt;

    double nearest_path_x = goal.x;
    double nearest_path_y = goal.y;
    double lookahead_distance = max_v_ * dt_ * 1.1;
    double min_distance = numeric_limits<double>::max();
    
    for (const auto& pose : global_plan_) {
        double path_x = pose.pose.position.x;
        double path_y = pose.pose.position.y;

        double dx = path_x - robot_state.x;
        double dy = path_y - robot_state.y;
        double dist = hypot(dx, dy);

        if (dist > lookahead_distance) {
            nearest_path_x = path_x;
            nearest_path_y = path_y;
            break;
        }
    }

    //std::cout << "nearest_path_x: " << nearest_path_x << ", nearest_path_y: " << nearest_path_y << std::endl;
    // 방향 비용 계산
    double dir_dtheta = atan2(goal.y - robot_state.y, goal.x - robot_state.x);
    double direction_cost = fabs(shortest_angular_distance(predict_theta, dir_dtheta)) / M_PI;
    direction_cost=direction_cost / M_PI;

    // 속도 비용 계산
    double speed_cost = (max_v_ - fabs(v)) / max_v_; 
    
    // 장애물과의 거리 계산
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

    // double obstacle_distance_cost = 1.0 / (min_distance_to_obstacle + 1.0);
    double k = 1.5;
    double obstacle_distance_cost = exp(-k * min_distance_to_obstacle);

    // Costmap 비용 계산
    double costmap_cost = 1.0;
    auto [map_x, map_y] = costmap_data_.toMapCoords(predict_x, predict_y, robot_state.x, robot_state.y, robot_state.theta);

    if (costmap_data_.inBounds(map_x, map_y)) {
        uint8_t costmap_value = costmap_data_.getCost(map_x, map_y);
        if (costmap_value == cost::LETHAL_OBSTACLE ||
            costmap_value == cost::INSCRIBED_INFLATED_OBSTACLE ||
            costmap_value == cost::NO_INFORMATION) {
            costmap_cost = 1.0; 
        } else {
            costmap_cost = static_cast<double>(costmap_value) / 255.0;  // 정규화된 costmap 비용
        }
    } else {
        costmap_cost = 1.0;  // Costmap을 벗어난 경우 비용 최대
    }

    //  `PathAlignCritic` 적용 (전역 경로 정렬)
    double global_plan_angle = atan2(nearest_path_y - robot_state.y, nearest_path_x - robot_state.x);
    double path_alignment_cost = fabs(shortest_angular_distance(predict_theta, global_plan_angle)) / M_PI;

    //  `PathDistCritic` 적용 (전역 경로 거리 유지)
    double path_distance_cost = hypot(nearest_path_x - predict_x, nearest_path_y - predict_y);
    path_distance_cost /= lookahead_distance;

    double forward_point_distance = 0.1; // DWB 기본값과 동일
    double angle_to_goal = atan2(goal.y - robot_state.y, goal.x - robot_state.x);

    // 목표 위치를 미세하게 조정
    double adjusted_goal_x = goal.x + forward_point_distance * cos(angle_to_goal);
    double adjusted_goal_y = goal.y + forward_point_distance * sin(angle_to_goal);
    
    double goal_align_cost = fabs(shortest_angular_distance(atan2(adjusted_goal_y - predict_y, adjusted_goal_x - predict_x), predict_theta)) / M_PI;

    // 목표 거리 평가
    double goal_distance_cost = hypot(goal.x - predict_x, goal.y - predict_y);

    double map_grid_cost = 1.0;
    if (costmap_data_.inBounds(map_x, map_y)) {
        double min_manhattan_dist = std::numeric_limits<double>::max();
        unsigned int width = costmap_data_.getWidth();
        unsigned int height = costmap_data_.getHeight();
        unsigned int max_search_dist = 10;  // 탐색 범위 제한

        for (unsigned int offset = 1; offset < min(max(width, height), max_search_dist); ++offset) {
            for (int dx = -offset; dx <= offset; dx++) {
                for (int dy = -offset; dy <= offset; dy++) {
                    int cx = map_x + dx;
                    int cy = map_y + dy;

                    if (costmap_data_.inBounds(cx, cy)) {
                        //최고값 기준
                        if (costmap_data_.getCost(cx, cy) == cost::LETHAL_OBSTACLE) {
                            double manhattan_dist = abs(dx) + abs(dy);
                            min_manhattan_dist = min(min_manhattan_dist, manhattan_dist);
                        }
                    }
                }
            }
        }
        map_grid_cost = min_manhattan_dist / (width + height);
    }

    double rotate_to_goal_cost = fabs(shortest_angular_distance(predict_theta, goal.theta));  // RotateToGoalCritic
    double twirl_cost = fabs(omega);  // TwirlingCritic

    // 진동하는 정도
    static double prev_velocity_sign = 1.0;
    double current_velocity_sign = (v > 0.0) ? 1.0 : -1.0;
    double oscillation_cost = (prev_velocity_sign != current_velocity_sign) ? 1.0 : 0.0;
    prev_velocity_sign = current_velocity_sign;


    cost.push_back(direction_cost); //
    cost.push_back(speed_cost);
    cost.push_back(obstacle_distance_cost); //

    cost.push_back(costmap_cost); 
    cost.push_back(path_alignment_cost); //
    cost.push_back(path_distance_cost); //
    cost.push_back(goal_align_cost); //
    cost.push_back(goal_distance_cost); //
    cost.push_back(map_grid_cost);
    cost.push_back(rotate_to_goal_cost);
    cost.push_back(twirl_cost);
    cost.push_back(oscillation_cost); //

    return cost;
}
