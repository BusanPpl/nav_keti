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

void DynamicCostmapUpdater::getVel(double v, double yaw, double angle) {
    robot_vel = v;
    robot_yaw = yaw;
    robot_angle = angle;
}

double DynamicCostmapUpdater::computeCost(double adjusted_distance, double distance, double combined_radius) const {
    unsigned char cost = 0;
    if (distance  < combined_radius) {
        cost = cost::LETHAL_OBSTACLE;
        return cost;
    } else if (distance  <= inscribed_radius_ + combined_radius) {
        cost = cost::INSCRIBED_INFLATED_OBSTACLE;
        return cost;
    }

    if (adjusted_distance < 0) {
        cost = cost::FREE_SPACE;
    } else {
        double factor =
            exp(-1.0 * cost_scaling_factor_ * (adjusted_distance - inscribed_radius_));
        // double factor =
        //     exp(-1.0 * cost_scaling_factor_ * (distance * resolution_ - inscribed_radius_));
        cost = static_cast<unsigned char>((cost::INSCRIBED_INFLATED_OBSTACLE - 1) * factor);
    }

    return cost;
}

void DynamicCostmapUpdater::ADIC(nav_msgs::msg::OccupancyGrid &costmap_msg, const std::vector<Obstacle> &obstacles, double time_step) {
    double robot_x = costmap_msg.info.origin.position.x + (costmap_msg.info.width * resolution_) / 2.0;
    double robot_y = costmap_msg.info.origin.position.y + (costmap_msg.info.height * resolution_) / 2.0;

    int width = costmap_msg.info.width;
    int height = costmap_msg.info.height;
    double origin_x = costmap_msg.info.origin.position.x;
    double origin_y = costmap_msg.info.origin.position.y;
    int cnt=0;

    double vx = robot_vel * cos(robot_angle);
    double vy = robot_vel * sin(robot_angle);
    // RCLCPP_INFO(this->get_logger(), "robot: robot_x: %.2f, robot_y: %.2f", vx, vy);

    const std::unordered_set<int> tracked_labels = {1, 2, 10};

    for (const auto& obstacle : obstacles) {

        int label = obstacle.label;
        // RCLCPP_INFO(this->get_logger(), "Label : %d", label);

        if (tracked_labels.count(label)) {
            // RCLCPP_INFO(this->get_logger(), "[TRACKED] Label: %d (x=%.2f, y=%.2f)",
            //             label, obstacle.position.x(), obstacle.position.y());
            continue;
        }

        if (obstacle.position.isZero() && obstacle.velocity.isZero()) continue;
        
        double obstacle_x = obstacle.position.x();
        double obstacle_y = obstacle.position.y();
        
        //if(cnt>0) RCLCPP_INFO(this->get_logger(), "obstacle_x : %f, obstacle_y : %f", obstacle_x, obstacle_y);
        double velocity_x = obstacle.velocity.x() - vx;
        double velocity_y = obstacle.velocity.y() - vy;
        //RCLCPP_INFO(this->get_logger(), "Velocity: vx: %.2f, vy: %.2f", velocity_x, velocity_y);
        double combined_radius = robot_radius_ + obstacle.radius;

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

                // 새로 추가한 부분
                double angle_cosine_obstacle = obstacle_vector.dot(propagation_vector);

                double denominator = combined_radius + 1.0 * (angle_cosine - angle_cosine_obstacle) * relative_speed * time_step;

                double adjusted_distance = (d_cell_to_obstacle * combined_radius) / denominator;

                double cost = computeCost(adjusted_distance, d_cell_to_obstacle, combined_radius); //cost를 계산할 뿐

                //if (cnt<2&&cost>250) RCLCPP_INFO(this->get_logger(), "cnt : %d", cnt); 
                int grid_x = static_cast<int>(std::round((px - origin_x) / resolution_));
                int grid_y = static_cast<int>(std::round((py - origin_y) / resolution_));

                if (grid_x >= 0 && grid_x < width && grid_y >= 0 && grid_y < height) {
                    int index = grid_y * width + grid_x;
                    uint8_t raw_cost = costmap_msg.data[index];

                    double final_cost;
                    if (raw_cost == cost::NO_INFORMATION) {
                        final_cost = std::clamp(cost, 0.0, static_cast<double>(cost::LETHAL_OBSTACLE - 1));
                    } else {
                        double current_cost = static_cast<double>(raw_cost);
                        final_cost = std::clamp(std::max(current_cost, cost), 0.0, static_cast<double>(cost::LETHAL_OBSTACLE - 1));
                    }

                    costmap_msg.data[index] = static_cast<int8_t>(std::round(final_cost));
                }
            }
        }
        cnt++;
    }
}

void DynamicCostmapUpdater::DADIM(nav_msgs::msg::OccupancyGrid &costmap_msg, const std::vector<Obstacle> &obstacles, double time_step) {

    double robot_x = costmap_msg.info.origin.position.x + (costmap_msg.info.width * resolution_) / 2.0;
    double robot_y = costmap_msg.info.origin.position.y + (costmap_msg.info.height * resolution_) / 2.0;

    int width = costmap_msg.info.width;
    int height = costmap_msg.info.height;
    double origin_x = costmap_msg.info.origin.position.x;
    double origin_y = costmap_msg.info.origin.position.y;

    for (const auto& obstacle : obstacles) {
    
        if (obstacle.position.isZero() && obstacle.velocity.isZero()) continue;
        
        double obstacle_x = obstacle.position.x();
        double obstacle_y = obstacle.position.y();
        double velocity_x = obstacle.velocity.x();
        double velocity_y = obstacle.velocity.y();
        double combined_radius = robot_radius_ + obstacle.radius;
        // 로봇의 중심 좌표 계산

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
                double d_cell_to_obstacle = std::hypot(px - obstacle_x, py - obstacle_y);

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

                }
            }
        }

    }
}


void DynamicCostmapUpdater::TruncatedVelocityObstacles(nav_msgs::msg::OccupancyGrid &costmap_msg, const std::vector<Obstacle> &obstacles, double truncation_time) {
    double robot_x = costmap_msg.info.origin.position.x + (costmap_msg.info.width * resolution_) / 2.0;
    double robot_y = costmap_msg.info.origin.position.y + (costmap_msg.info.height * resolution_) / 2.0;

    int width = costmap_msg.info.width;
    int height = costmap_msg.info.height;
    double origin_x = costmap_msg.info.origin.position.x;
    double origin_y = costmap_msg.info.origin.position.y;

    for (const auto& obstacle : obstacles) {
        if (obstacle.position.isZero() && obstacle.velocity.isZero()) continue;

        double obstacle_x = obstacle.position.x();
        double obstacle_y = obstacle.position.y();
        double velocity_x = obstacle.velocity.x();
        double velocity_y = obstacle.velocity.y();
        double combined_radius = robot_radius_ + obstacle.radius;

        double distance_to_obstacle = std::sqrt((obstacle_x - robot_x) * (obstacle_x - robot_x) +
        (obstacle_y - robot_y) * (obstacle_y - robot_y));
        double theta_rco = std::atan2(obstacle_y - robot_y, obstacle_x - robot_x);
        double theta_ray = std::asin(std::clamp(combined_radius / distance_to_obstacle, -1.0, 1.0));
        double theta_left_ray = theta_rco + theta_ray;
        double theta_right_ray = theta_rco - theta_ray;

        Eigen::Vector2d relative_velocity(velocity_x, velocity_y);
        Eigen::Vector2d direction_to_obstacle(obstacle_x - robot_x, obstacle_y - robot_y);
        direction_to_obstacle.normalize();
        double projection = relative_velocity.dot(direction_to_obstacle);

        double new_truncation_distance = std::max(robot_radius_, distance_to_obstacle * (1 - std::clamp(projection / robot_max_speed, 0.0, 1.0)) + robot_radius_);

        for (int dy = -height / 2; dy < height / 2; ++dy) {
            for (int dx = -width / 2; dx < width / 2; ++dx) {
                double px = robot_x + dx * resolution_;
                double py = robot_y + dy * resolution_;

                // VO 영역 내 위치 확인
                double relative_vp_x = px - velocity_x * truncation_time;
                double relative_vp_y = py - velocity_y * truncation_time;
                double angle_to_relative_vp = std::atan2(relative_vp_y - robot_y, relative_vp_x - robot_x);

                // 상대 벡터 거리 계산
                double projected_distance = std::sqrt((relative_vp_x - robot_x) * (relative_vp_x - robot_x) +
                (relative_vp_y - robot_y) * (relative_vp_y - robot_y));

                if (projected_distance >= robot_radius_ + (truncation_time * std::sqrt(velocity_x * velocity_x + velocity_y * velocity_y)) &&
                projected_distance <= new_truncation_distance &&
                theta_right_ray <= angle_to_relative_vp && angle_to_relative_vp <= theta_left_ray)
                {
                    int grid_x = static_cast<int>(std::round((px - origin_x) / resolution_));
                    int grid_y = static_cast<int>(std::round((py - origin_y) / resolution_));

                    if (grid_x >= 0 && grid_x < width && grid_y >= 0 && grid_y < height) {
                        int index = grid_y * width + grid_x;
                        costmap_msg.data[index] = static_cast<int8_t>(
                            std::min<int>(static_cast<int>(costmap_msg.data[index]), static_cast<int>(cost::LETHAL_OBSTACLE))
                        );
                    }
                }
            }
        }
    }
}


void DynamicCostmapUpdater::VelocityObstacle(nav_msgs::msg::OccupancyGrid &costmap_msg, const std::vector<Obstacle> &obstacles) {
    double robot_x = costmap_msg.info.origin.position.x + (costmap_msg.info.width * resolution_) / 2.0;
    double robot_y = costmap_msg.info.origin.position.y + (costmap_msg.info.height * resolution_) / 2.0;

    int width = costmap_msg.info.width;
    int height = costmap_msg.info.height;
    double origin_x = costmap_msg.info.origin.position.x;
    double origin_y = costmap_msg.info.origin.position.y;

    for (const auto& obstacle : obstacles) {

    if (obstacle.position.isZero() && obstacle.velocity.isZero()) continue;

    double obstacle_x = obstacle.position.x();
    double obstacle_y = obstacle.position.y();
    double velocity_x = obstacle.velocity.x();
    double velocity_y = obstacle.velocity.y();
    double combined_radius = robot_radius_ + obstacle.radius;

    double distance_to_obstacle = std::sqrt((obstacle_x - robot_x) * (obstacle_x - robot_x) + (obstacle_y - robot_y) * (obstacle_y - robot_y));
    double theta_rco = std::atan2(obstacle_y - robot_y, obstacle_x - robot_x);
    double theta_ray = std::asin(std::clamp(combined_radius / distance_to_obstacle, -1.0, 1.0));
    double theta_left_ray = theta_rco + theta_ray;
    double theta_right_ray = theta_rco - theta_ray;

    for (int dy = -height / 2; dy < height / 2; ++dy) {
        for (int dx = -width / 2; dx < width / 2; ++dx) {
            double px = robot_x + dx * resolution_;
            double py = robot_y + dy * resolution_;

            double relative_vp_x = px - velocity_x;
            double relative_vp_y = py - velocity_y;
            double angle_to_relative_vp = std::atan2(relative_vp_y - robot_y, relative_vp_x - robot_x);

            if (theta_right_ray <= angle_to_relative_vp && angle_to_relative_vp <= theta_left_ray) {
                int grid_x = static_cast<int>((px - origin_x) / resolution_);
                int grid_y = static_cast<int>((py - origin_y) / resolution_);

                if (grid_x >= 0 && grid_x < width && grid_y >= 0 && grid_y < height) {
                    int index = grid_y * width + grid_x;
                    costmap_msg.data[index] = static_cast<int8_t>(
                        std::min<int>(static_cast<int>(costmap_msg.data[index]), static_cast<int>(cost::LETHAL_OBSTACLE))
                    );
                    }
                }
            }
        }
    }
}


//DAJ에서 속도만 0처리
void DynamicCostmapUpdater::Circle(nav_msgs::msg::OccupancyGrid &costmap_msg, const std::vector<Obstacle> &obstacles) {
    double robot_x = costmap_msg.info.origin.position.x + (costmap_msg.info.width * resolution_) / 2.0;
    double robot_y = costmap_msg.info.origin.position.y + (costmap_msg.info.height * resolution_) / 2.0;

    int width = costmap_msg.info.width;
    int height = costmap_msg.info.height;
    double origin_x = costmap_msg.info.origin.position.x;
    double origin_y = costmap_msg.info.origin.position.y;

for (const auto& obstacle : obstacles) {
    if (obstacle.position.isZero()) continue;

    double obstacle_x = obstacle.position.x();
    double obstacle_y = obstacle.position.y();
    double velocity_x = 0.0;
    double velocity_y = 0.0;
    double combined_radius = robot_radius_ + obstacle.radius;

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

    for (int dy = -height / 2; dy < height / 2; ++dy) {
        for (int dx = -width / 2; dx < width / 2; ++dx) {
            double px = robot_x + dx * resolution_;
            double py = robot_y + dy * resolution_;

            double d_cell_to_obstacle = std::sqrt((px - obstacle_x) * (px - obstacle_x) + (py - obstacle_y) * (py - obstacle_y));

            Eigen::Matrix<double, 2, 1> propagation_vector(px - obstacle_x, py - obstacle_y);
            if (propagation_vector.norm() > 0) {
            propagation_vector.normalize();
            }

            double angle_cosine = movement_vector.dot(propagation_vector);
            double angle_cosine_obstacle = obstacle_vector.dot(propagation_vector);

            double denominator = combined_radius + (angle_cosine - angle_cosine_obstacle) * relative_speed;
            double adjusted_distance = (d_cell_to_obstacle * combined_radius) / denominator;

            double cost = computeCost(adjusted_distance, d_cell_to_obstacle, combined_radius);

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
}
