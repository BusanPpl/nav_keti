#include "nav_keti/dynamic_costmap_updater.h"


DynamicCostmapUpdater::DynamicCostmapUpdater(double resolution, double robot_radius, double inscribed_radius, double cost_scaling_factor)
    : resolution_(resolution), robot_radius_(robot_radius), Node("costmap_updater"), inscribed_radius_(inscribed_radius), cost_scaling_factor_(cost_scaling_factor) {
        this->declare_parameter<double>("robot_max_speed", 2.6f);
        updateParameters();
        parameter_event_sub_ = this->add_on_set_parameters_callback(
        std::bind(&DynamicCostmapUpdater::parameterCallback, this, std::placeholders::_1));
    }

void DynamicCostmapUpdater::resetCostmap(nav_msgs::msg::OccupancyGrid &costmap_msg) {
    std::fill(costmap_msg.data.begin(), costmap_msg.data.end(), cost::NO_INFORMATION);
}

void DynamicCostmapUpdater::updateParameters() {
    this->get_parameter("robot_max_speed", robot_max_speed);
}

rcl_interfaces::msg::SetParametersResult DynamicCostmapUpdater::parameterCallback(const std::vector<rclcpp::Parameter>& parameters) {
    rcl_interfaces::msg::SetParametersResult result;
    result.successful = true;

    for (const auto& param : parameters) {
        RCLCPP_INFO(this->get_logger(), "Parameter changed: %s = %s",
                    param.get_name().c_str(), param.value_to_string().c_str());
    }
    updateParameters();
    return result;
}

double DynamicCostmapUpdater::computeCost(double distance, double combined_radius) const {
    unsigned char cost = 0;

    if (distance < 0) {
        cost = cost::FREE_SPACE;
    } else if (distance  < combined_radius) {
        cost = cost::LETHAL_OBSTACLE;
    } else if (distance  <= inscribed_radius_ + combined_radius) {
        cost = cost::INSCRIBED_INFLATED_OBSTACLE;
    } else {
        double factor =
            exp(-1.0 * cost_scaling_factor_ * (distance - inscribed_radius_));
        // double factor =
        //     exp(-1.0 * cost_scaling_factor_ * (distance * resolution_ - inscribed_radius_));
        cost = static_cast<unsigned char>((cost::INSCRIBED_INFLATED_OBSTACLE - 1) * factor);
    }

    return cost;
}

void DynamicCostmapUpdater::ADIC(nav_msgs::msg::OccupancyGrid &costmap_msg,
                                               const Eigen::Vector4f &obstacle_state,
                                               double obstacle_radius, double time_step) {

    if (obstacle_state(0) == 0 && obstacle_state(1) == 0 && 
        obstacle_state(2) == 0 && obstacle_state(3) == 0) { 
        resetCostmap(costmap_msg);
        return;
    }

    double combined_radius = robot_radius_ + obstacle_radius;
    //double combined_radius = 0.5;
    double obstacle_x = obstacle_state(0);
    double obstacle_y = obstacle_state(1);
    double velocity_x = obstacle_state(2);
    double velocity_y = obstacle_state(3);

    // 로봇의 중심 좌표 계산
    double robot_x = costmap_msg.info.origin.position.x + (costmap_msg.info.width * resolution_) / 2.0;
    double robot_y = costmap_msg.info.origin.position.y + (costmap_msg.info.height * resolution_) / 2.0;

    int width = costmap_msg.info.width;
    int height = costmap_msg.info.height;
    double origin_x = costmap_msg.info.origin.position.x;
    double origin_y = costmap_msg.info.origin.position.y;

    // 장애물까지의 거리
    double distance_to_obstacle = std::sqrt((obstacle_x - robot_x) * (obstacle_x - robot_x) +
                                            (obstacle_y - robot_y) * (obstacle_y - robot_y));

    // 이동 방향 벡터 정규화
    Eigen::Matrix<double, 2, 1> movement_vector(velocity_x, velocity_y);
    if (movement_vector.norm() > 0) {
        movement_vector.normalize();
    }

    Eigen::Matrix<double, 2, 1> obstacle_vector(obstacle_x - robot_x, obstacle_y - robot_y);
    if (obstacle_vector.norm() > 0) {
        obstacle_vector.normalize();
    }

    double relative_speed = std::sqrt(velocity_x * velocity_x + velocity_y * velocity_y);

    int cnt=0;
    // 코스트맵을 순회하면서 영향 맵 갱신
    for (int dy = -height / 2; dy < height / 2; ++dy) {
        for (int dx = -width / 2; dx < width / 2; ++dx) {
            double px = robot_x + dx * resolution_;
            double py = robot_y + dy * resolution_;

            double d_cell_to_obstacle = std::sqrt((px - obstacle_x) * (px - obstacle_x) + (py - obstacle_y) * (py - obstacle_y));

            Eigen::Matrix<double, 2, 1> propagation_vector(px - obstacle_x, py - obstacle_y);
            if (propagation_vector.norm() > 0) {
                propagation_vector.normalize();
            }

            // DADIM 거리 조정 적용
            double angle_cosine = movement_vector.dot(propagation_vector);

            //새로 추가한 부분
            double angle_cosine_obstacle = obstacle_vector.dot(propagation_vector);

            double denominator = combined_radius + 1.0 * (angle_cosine - angle_cosine_obstacle) * relative_speed * time_step;

            double adjusted_distance = (d_cell_to_obstacle * combined_radius) / denominator; 

            double cost = computeCost(adjusted_distance, combined_radius);

            int grid_x = static_cast<int>(std::round((px - origin_x) / resolution_));
            int grid_y = static_cast<int>(std::round((py - origin_y) / resolution_));

            if (grid_x >= 0 && grid_x < width && grid_y >= 0 && grid_y < height) {
                int index = grid_y * width + grid_x;

                int new_influence = static_cast<unsigned int>(cost);

                if (new_influence > costmap_msg.data[index]) {
                    costmap_msg.data[index] = new_influence;
                }

                if (costmap_msg.data[index] > cost::LETHAL_OBSTACLE) {
                    costmap_msg.data[index] = cost::LETHAL_OBSTACLE;
                }
                // if (new_influence > 128&&new_influence < 254) {
                //     cnt++;
                // }
            }
        }
    }
// RCLCPP_INFO(this->get_logger(), "cnt : %d", cnt);
}


void DynamicCostmapUpdater::DADIM(nav_msgs::msg::OccupancyGrid &costmap_msg,
                                    const std::vector<Obstacle> &obstacles,
                                    double obstacle_radius, double time_step) {

    // if (obstacle_state(0) == 0 && obstacle_state(1) == 0 && 
    //     obstacle_state(2) == 0 && obstacle_state(3) == 0) { 
    //     resetCostmap(costmap_msg);
    //     return;
    // }
    for (const auto& obstacle : obstacles) {
        if (obstacle.position.isZero() && obstacle.velocity.isZero()) continue;
    double combined_radius = robot_radius_ + obstacle_radius;
    double obstacle_x = obstacle_state(0);
    double obstacle_y = obstacle_state(1);
    double velocity_x = obstacle_state(2);
    double velocity_y = obstacle_state(3);
    //RCLCPP_INFO(this->get_logger(), "obstacle_x : %f, obstacle_y : %f, velocity_x : %f, velocity_y : %f", obstacle_x, obstacle_y, velocity_x, velocity_y);
    // 로봇의 중심 좌표 계산
    double robot_x = costmap_msg.info.origin.position.x + (costmap_msg.info.width * resolution_) / 2.0;
    double robot_y = costmap_msg.info.origin.position.y + (costmap_msg.info.height * resolution_) / 2.0;

    int width = costmap_msg.info.width;
    int height = costmap_msg.info.height;
    double origin_x = costmap_msg.info.origin.position.x;
    double origin_y = costmap_msg.info.origin.position.y;

    // 장애물까지의 거리
    double distance_to_obstacle = std::sqrt((obstacle_x - robot_x) * (obstacle_x - robot_x) +
                                            (obstacle_y - robot_y) * (obstacle_y - robot_y));
    
    // 이동 방향 벡터 정규화
    Eigen::Matrix<double, 2, 1> movement_vector(velocity_x, velocity_y);
    if (movement_vector.norm() > 0) {
        movement_vector.normalize();
    }

    Eigen::Matrix<double, 2, 1> obstacle_vector(obstacle_x - robot_x, obstacle_y - robot_y);
    if (obstacle_vector.norm() > 0) {
        obstacle_vector.normalize();
    }

    double relative_speed = std::sqrt(velocity_x * velocity_x + velocity_y * velocity_y);
    
    double gamma = 0.8;
    int cnt=0;
    // 코스트맵을 순회하면서 영향 맵 갱신
    for (int dy = -height / 2; dy < height / 2; ++dy) {
        for (int dx = -width / 2; dx < width / 2; ++dx) {
            double px = robot_x + dx * resolution_;
            double py = robot_y + dy * resolution_;

            double d_cell_to_obstacle = std::sqrt((px - obstacle_x) * (px - obstacle_x) +
                                                  (py - obstacle_y) * (py - obstacle_y));

            Eigen::Matrix<double, 2, 1> propagation_vector(px - obstacle_x, py - obstacle_y);
            if (propagation_vector.norm() > 0) {
                propagation_vector.normalize();
            }
            // DADIM 거리 조정 적용
            double angle_cosine = movement_vector.dot(propagation_vector);
            double denominator = combined_radius + angle_cosine * relative_speed * time_step;
    
            double adjusted_distance = (d_cell_to_obstacle * combined_radius) / denominator; 

            // 이거 써봤더니 이상한 값으로 만들어버림 잘 튜닝하느니 안쓰는게 나을듯
            double angle_cosine_obstacle = obstacle_vector.dot(propagation_vector);
            double denominator_obs = combined_radius + angle_cosine_obstacle * relative_speed * time_step;
        
            double adjusted_distance_obs = (d_cell_to_obstacle * combined_radius) / denominator_obs; 

            //장애물 반경만큼 cost반영
            double influence = (d_cell_to_obstacle <= combined_radius) ? cost::LETHAL_OBSTACLE 
                                                                        : (adjusted_distance < 0) ? cost::FREE_SPACE
                                                                                                     : cost::LETHAL_OBSTACLE * std::exp(-gamma * adjusted_distance);

            int grid_x = static_cast<int>(std::round((px - origin_x) / resolution_));
            int grid_y = static_cast<int>(std::round((py - origin_y) / resolution_));

            if (grid_x >= 0 && grid_x < width && grid_y >= 0 && grid_y < height) {
                int index = grid_y * width + grid_x;

                int new_influence = static_cast<int>(influence);

                if (new_influence > costmap_msg.data[index]) {
                    costmap_msg.data[index] = new_influence;
                }

                if (costmap_msg.data[index] > cost::LETHAL_OBSTACLE) {
                    costmap_msg.data[index] = cost::LETHAL_OBSTACLE;
                }
                // if (new_influence > 128&&new_influence < 254) {
                //     cnt++;
                // }
            }
        }
    }
    //RCLCPP_INFO(this->get_logger(), "cnt : %d", cnt);
}
}


void DynamicCostmapUpdater::TruncatedVelocityObstacles(nav_msgs::msg::OccupancyGrid &costmap_msg,
                                           const Eigen::Vector4f &obstacle_state,
                                           double obstacle_radius, double truncation_time) {

    if (obstacle_state(0) == 0 && obstacle_state(1) == 0 && obstacle_state(2) == 0 && obstacle_state(3) == 0) { // 데이터가 유효하지 않은 경우
        resetCostmap(costmap_msg);
        return;
    }

    double combined_radius = robot_radius_ + obstacle_radius;

    double obstacle_x = obstacle_state(0);
    double obstacle_y = obstacle_state(1);
    double velocity_x = obstacle_state(2);
    double velocity_y = obstacle_state(3);

    // 로봇의 좌표 (코스트맵의 한가운데)
    double robot_x = costmap_msg.info.origin.position.x + (costmap_msg.info.width * resolution_) / 2.0;
    double robot_y = costmap_msg.info.origin.position.y + (costmap_msg.info.height * resolution_) / 2.0;

    // 장애물의 상대 속도 계산
    double relative_vx = velocity_x;
    double relative_vy = velocity_y;

    int width = costmap_msg.info.width;
    int height = costmap_msg.info.height;
    double origin_x = costmap_msg.info.origin.position.x;
    double origin_y = costmap_msg.info.origin.position.y;

    // 로봇 → 장애물 방향 벡터 계산
    double distance_to_obstacle = std::sqrt((obstacle_x - robot_x) * (obstacle_x - robot_x) +
                                            (obstacle_y - robot_y) * (obstacle_y - robot_y));
    double theta_rco = std::atan2(obstacle_y - robot_y, obstacle_x - robot_x);
    double theta_ray = std::asin(std::clamp(combined_radius / distance_to_obstacle, -1.0, 1.0));
    double theta_left_ray = theta_rco + theta_ray;
    double theta_right_ray = theta_rco - theta_ray;

    Eigen::Vector2d relative_velocity(relative_vx, relative_vy);
    Eigen::Vector2d direction_to_obstacle(obstacle_x - robot_x, obstacle_y - robot_y);
    direction_to_obstacle.normalize();
    double projection = relative_velocity.dot(direction_to_obstacle);
    double new_truncation_distance = std::max(robot_radius_, distance_to_obstacle * (1 - std::clamp(projection / robot_max_speed, 0.0, 1.0)) + robot_radius_);
    for (int dy = -height / 2; dy < height / 2; ++dy) {
        for (int dx = -width / 2; dx < width / 2; ++dx) {
            double px = robot_x + dx * resolution_;
            double py = robot_y + dy * resolution_;

            // VO 영역 내 위치 확인
            double relative_vp_x = px - relative_vx * truncation_time;
            double relative_vp_y = py - relative_vy * truncation_time;
            double angle_to_relative_vp = std::atan2(relative_vp_y - robot_y, relative_vp_x - robot_x);

            // 상대 벡터 거리 계산
            double projected_distance = std::sqrt((relative_vp_x - robot_x) * (relative_vp_x - robot_x) +
                                                  (relative_vp_y - robot_y) * (relative_vp_y - robot_y));
                                                  
            if (projected_distance >= robot_radius_ + (truncation_time * std::sqrt(relative_vx * relative_vx + relative_vy * relative_vy)) &&
                projected_distance <= new_truncation_distance &&
                theta_right_ray <= angle_to_relative_vp && angle_to_relative_vp <= theta_left_ray)
            {
                // 코스트맵 좌표 변환
                int grid_x = static_cast<int>(std::round((px - origin_x) / resolution_));
                int grid_y = static_cast<int>(std::round((py - origin_y) / resolution_));

                if (grid_x >= 0 && grid_x < width && grid_y >= 0 && grid_y < height) {
                    int index = grid_y * width + grid_x;

                    // VO 영역에 lethal_obstacle 설정
                    costmap_msg.data[index] = (costmap_msg.data[index] > cost::LETHAL_OBSTACLE) ? costmap_msg.data[index] : cost::LETHAL_OBSTACLE;
                }
            }
        }
    }
}


void DynamicCostmapUpdater::VelocityObstacle(nav_msgs::msg::OccupancyGrid &costmap_msg,
                                           const Eigen::Vector4f &obstacle_state,
                                           double obstacle_radius) {

    if (obstacle_state(0) == 0 && obstacle_state(1) == 0 && obstacle_state(2) == 0 && obstacle_state(3) == 0) { // 데이터가 유효하지 않은 경우
        resetCostmap(costmap_msg);
        return;
    }

    double combined_radius = robot_radius_ + obstacle_radius;

    double obstacle_x = obstacle_state(0);
    double obstacle_y = obstacle_state(1);
    double velocity_x = obstacle_state(2);
    double velocity_y = obstacle_state(3);

    // 로봇의 좌표 (코스트맵의 한가운데)
    double robot_x = costmap_msg.info.origin.position.x + (costmap_msg.info.width * resolution_) / 2.0;
    double robot_y = costmap_msg.info.origin.position.y + (costmap_msg.info.height * resolution_) / 2.0;

    // 장애물의 상대 속도 계산
    double relative_vx = velocity_x;
    double relative_vy = velocity_y;

    int width = costmap_msg.info.width;
    int height = costmap_msg.info.height;
    double origin_x = costmap_msg.info.origin.position.x;
    double origin_y = costmap_msg.info.origin.position.y;

    // 로봇 → 장애물 방향 벡터 계산
    double distance_to_obstacle = std::sqrt((obstacle_x - robot_x) * (obstacle_x - robot_x) +
                                            (obstacle_y - robot_y) * (obstacle_y - robot_y));
    double theta_rco = std::atan2(obstacle_y - robot_y, obstacle_x - robot_x);
    double theta_ray = std::asin(std::clamp(combined_radius / distance_to_obstacle, -1.0, 1.0));
    double theta_left_ray = theta_rco + theta_ray;
    double theta_right_ray = theta_rco - theta_ray;

    for (int dy = -height / 2; dy < height / 2; ++dy) {
        for (int dx = -width / 2; dx < width / 2; ++dx) {
            double px = robot_x + dx * resolution_;
            double py = robot_y + dy * resolution_;

            // VO 영역 내 위치 확인
            double relative_vp_x = px - relative_vx;
            double relative_vp_y = py - relative_vy;
            double angle_to_relative_vp = std::atan2(relative_vp_y - robot_y, relative_vp_x - robot_x);

            // VO 영역과 잘림 거리 조건 확인
            if (theta_right_ray <= angle_to_relative_vp && angle_to_relative_vp <= theta_left_ray) {

                // 코스트맵 좌표 변환
                int grid_x = static_cast<int>((px - origin_x) / resolution_);
                int grid_y = static_cast<int>((py - origin_y) / resolution_);

                if (grid_x >= 0 && grid_x < width && grid_y >= 0 && grid_y < height) {
                    int index = grid_y * width + grid_x;

                    // VO 영역에 lethal_obstacle 설정
                    costmap_msg.data[index] = (costmap_msg.data[index] > cost::LETHAL_OBSTACLE) ? costmap_msg.data[index] : cost::LETHAL_OBSTACLE;
                }
            }
        }
    }
}



//DAJ에서 속도만 0처리
void DynamicCostmapUpdater::Circle(nav_msgs::msg::OccupancyGrid &costmap_msg,
                                    const Eigen::Vector4f &obstacle_state,
                                    double obstacle_radius) {

if (obstacle_state(0) == 0 && obstacle_state(1) == 0 && obstacle_state(2) == 0 && obstacle_state(3) == 0) { 
    resetCostmap(costmap_msg);
    return;
}

double combined_radius = robot_radius_ + obstacle_radius;
//double combined_radius = 0.5;
double obstacle_x = obstacle_state(0);
double obstacle_y = obstacle_state(1);
double velocity_x = 0.0;
double velocity_y = 0.0;

// 로봇의 중심 좌표 계산
double robot_x = costmap_msg.info.origin.position.x + (costmap_msg.info.width * resolution_) / 2.0;
double robot_y = costmap_msg.info.origin.position.y + (costmap_msg.info.height * resolution_) / 2.0;

int width = costmap_msg.info.width;
int height = costmap_msg.info.height;
double origin_x = costmap_msg.info.origin.position.x;
double origin_y = costmap_msg.info.origin.position.y;

// 장애물까지의 거리
double distance_to_obstacle = std::sqrt((obstacle_x - robot_x) * (obstacle_x - robot_x) +
                                        (obstacle_y - robot_y) * (obstacle_y - robot_y));

// 이동 방향 벡터 정규화
Eigen::Matrix<double, 2, 1> movement_vector(velocity_x, velocity_y);
if (movement_vector.norm() > 0) {
    movement_vector.normalize();
}

Eigen::Matrix<double, 2, 1> obstacle_vector(obstacle_x - robot_x, obstacle_y - robot_y);
if (obstacle_vector.norm() > 0) {
    obstacle_vector.normalize();
}

double relative_speed = std::sqrt(velocity_x * velocity_x + velocity_y * velocity_y);

// 코스트맵을 순회하면서 영향 맵 갱신
for (int dy = -height / 2; dy < height / 2; ++dy) {
    for (int dx = -width / 2; dx < width / 2; ++dx) {
        double px = robot_x + dx * resolution_;
        double py = robot_y + dy * resolution_;

        double d_cell_to_obstacle = std::sqrt((px - obstacle_x) * (px - obstacle_x) + (py - obstacle_y) * (py - obstacle_y));

        Eigen::Matrix<double, 2, 1> propagation_vector(px - obstacle_x, py - obstacle_y);
        if (propagation_vector.norm() > 0) {
            propagation_vector.normalize();
        }

        // DADIM 거리 조정 적용
        double angle_cosine = movement_vector.dot(propagation_vector);

        //새로 추가한 부분
        double angle_cosine_obstacle = obstacle_vector.dot(propagation_vector);

        double denominator = combined_radius + (angle_cosine - angle_cosine_obstacle) * relative_speed;

        double adjusted_distance = (d_cell_to_obstacle * combined_radius) / denominator; 

        double cost = computeCost(adjusted_distance, combined_radius);

        int grid_x = static_cast<int>(std::round((px - origin_x) / resolution_));
        int grid_y = static_cast<int>(std::round((py - origin_y) / resolution_));

        if (grid_x >= 0 && grid_x < width && grid_y >= 0 && grid_y < height) {
            int index = grid_y * width + grid_x;

            int new_influence = static_cast<unsigned int>(cost);

            if (new_influence > costmap_msg.data[index]) {
                costmap_msg.data[index] = new_influence;
            }

            if (costmap_msg.data[index] > cost::LETHAL_OBSTACLE) {
                costmap_msg.data[index] = cost::LETHAL_OBSTACLE;
            }
        }
    }
}
}
