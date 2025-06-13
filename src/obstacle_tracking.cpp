#include "nav_keti/obstacle_tracking.h"

ObstacleTracker::ObstacleTracker() : Node("costmap_obstacle_tracker") {
    this->declare_parameter<double>("process_noise_factor", 1.0);
    this->declare_parameter<double>("measurement_noise_factor", 0.05);
    updateParameters();
    parameter_event_sub_ = this->add_on_set_parameters_callback(
        std::bind(&ObstacleTracker::parameterCallback, this, std::placeholders::_1));
    
    // Costmap 데이터 구독
    subscription_ = this->create_subscription<nav_msgs::msg::OccupancyGrid>(
        "/costmap", 10, std::bind(&ObstacleTracker::costmapCallback, this, std::placeholders::_1));

    initializeKalmanFilter();
    RCLCPP_INFO(this->get_logger(), "Costmap Obstacle Tracker with Kalman Filter started.");
}

void ObstacleTracker::updateParameters() {
    process_noise_factor = this->get_parameter("process_noise_factor").as_double();
    measurement_noise_factor = this->get_parameter("measurement_noise_factor").as_double();
}

rcl_interfaces::msg::SetParametersResult ObstacleTracker::parameterCallback(const std::vector<rclcpp::Parameter>& parameters) {    
    rcl_interfaces::msg::SetParametersResult result;
    result.successful = true;

    for (const auto& param : parameters) {
        RCLCPP_INFO(this->get_logger(), "Parameter changed: %s = %s",
                    param.get_name().c_str(), param.value_to_string().c_str());
    }
    updateParameters();
    return result;
}

Eigen::Vector4f ObstacleTracker::getState() const {
    return state_;
}

double ObstacleTracker::getObstacleRadius() const {
    return obstacle_radius_;
}

void ObstacleTracker::initializeKalmanFilter() {
    // 초기 상태 [x, y, vx, vy]
    state_ = Eigen::Vector4f(0.0, 0.0, 0.0, 0.0);
    
    // 초기 상태 공분산
    state_covariance_ = Eigen::Matrix4f::Identity() * process_noise_factor;

    // 상태 전이 행렬 (vx와 vy가 위치에 영향을 줌)
    transition_matrix_ = Eigen::Matrix4f::Identity();
    transition_matrix_(0, 2) = 0.2;
    transition_matrix_(1, 3) = 0.2;

    // 프로세스 노이즈 공분산
    process_noise_ = Eigen::Matrix4f::Identity() * 0.1;

    // 측정 행렬 (x와 y만 측정)
    measurement_matrix_ = Eigen::Matrix<float, 2, 4>::Zero();
    measurement_matrix_(0, 0) = 1.0;
    measurement_matrix_(1, 1) = 1.0;

    // 측정 노이즈 공분산
    measurement_noise_ = Eigen::Matrix2f::Identity() * measurement_noise_factor;
    
    last_update_time_ = std::chrono::steady_clock::now();
}

bool ObstacleTracker::costmapCallback(const nav_msgs::msg::OccupancyGrid::SharedPtr msg) {
    auto current_time = std::chrono::steady_clock::now();
    
    // RCLCPP_INFO(this->get_logger(), "%f", process_noise_factor);
    // 장애물 위치 추출
    std::vector<Point> points;
    const std::vector<int8_t>& data = msg->data;
    int width = msg->info.width;
    int height = msg->info.height;
    double resolution = msg->info.resolution;
    double origin_x = msg->info.origin.position.x;
    double origin_y = msg->info.origin.position.y;

    // 데이터에서 장애물 찾기 (값이 일정 범위 내에 있는 경우)
    for (int y = 0; y < height; ++y) {
        for (int x = 0; x < width; ++x) {
            int index = y * width + x;
            int cost_value = data[index];

            if (cost_value == static_cast<int8_t>(cost::LETHAL_OBSTACLE)) {
                double world_x = origin_x + x * resolution;
                double world_y = origin_y + y * resolution;
                points.push_back(ObstacleTracker::Point{world_x, world_y});
            }
        }
    }

    // 장애물이 없으면 센트로이드 계산하지 않음
    if (points.empty()) {
        // RCLCPP_WARN(this->get_logger(), "No obstacles detected in costmap.");
        resetState();
        return false;
    }

    // 센트로이드 계산
    Point centroid = calculateCentroid(points);
    calculateRadius(centroid, points);

    // 칼만 필터로 추적 업데이트
    Eigen::Vector2f measurement(centroid.x, centroid.y); // [x, y]
    std::chrono::duration<double> elapsed = current_time - last_update_time_;
    double dt = elapsed.count();  // 초 단위
    last_update_time_ = current_time;
    updateKalmanFilter(measurement, dt);
    return true;
}


void ObstacleTracker::calculateRadius(const Point& centroid, const std::vector<Point>& points) {
    if (points.empty()) {
        // RCLCPP_WARN(this->get_logger(), "No points available for radius calculation.");
    }

    float max_distance = 0.0f;

    for (const auto& point : points) {
        float distance = std::sqrt(std::pow(point.x - centroid.x, 2) + std::pow(point.y - centroid.y, 2));
        if (distance > max_distance) {
            max_distance = distance;
        }
    }

    obstacle_radius_ = max_distance;
}


ObstacleTracker::Point ObstacleTracker::calculateCentroid(const std::vector<Point>& points) {
    Point centroid = {0.0f, 0.0f};
    if (points.empty()) {
        // RCLCPP_WARN(this->get_logger(), "No valid points detected for the obstacle.");
        return centroid;
    }

    // 평균 계산
    for (const auto& point : points) {
        centroid.x += point.x;
        centroid.y += point.y;
    }

    centroid.x /= points.size();
    centroid.y /= points.size();

    return centroid;
}


void ObstacleTracker::updateKalmanFilter(const Eigen::Vector2f& measurement, double dt) {
    transition_matrix_(0, 2) = dt;
    transition_matrix_(1, 3) = dt;

    // 예측 단계
    state_ = transition_matrix_ * state_;
    state_covariance_ = transition_matrix_ * state_covariance_ * transition_matrix_.transpose() + process_noise_;

    // 칼만 게인 계산
    Eigen::Matrix2f S = measurement_matrix_ * state_covariance_ * measurement_matrix_.transpose() + measurement_noise_;
    Eigen::Matrix<float, 4, 2> K = state_covariance_ * measurement_matrix_.transpose() * S.inverse();

    // 업데이트 단계
    Eigen::Vector2f innovation = measurement - measurement_matrix_ * state_;
    state_ = state_ + K * innovation;
    state_covariance_ = (Eigen::Matrix4f::Identity() - K * measurement_matrix_) * state_covariance_;
}


void ObstacleTracker::resetState() {
    // 상태와 공분산 초기화
    state_ = Eigen::Vector4f(0.0, 0.0, 0.0, 0.0);
    state_covariance_ = Eigen::Matrix4f::Identity() * 1.0;
    // RCLCPP_INFO(this->get_logger(), "State reset: No obstacles detected.");
}

void ObstacleTracker::logObstacleData() {
    RCLCPP_INFO(this->get_logger(),
                "Obstacle Position (x: %.2f, y: %.2f), Velocity (vx: %.2f, vy: %.2f)",
                state_(0), state_(1), state_(2), state_(3));
}
