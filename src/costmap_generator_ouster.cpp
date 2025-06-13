#include "nav_keti/costmap_generator.h"

CostmapGenerator::CostmapGenerator() : Node("costmap_generator"){
    this->declare_parameter<int>("width", 100);
    this->declare_parameter<int>("height", 100);
    this->declare_parameter<double>("resolution", 0.1f);
    this->declare_parameter<double>("max_obstacle_height", 2.0f);
    this->declare_parameter<int>("gradient_size", 2);
    this->declare_parameter<int>("gradient_factor", 5);
    this->declare_parameter<double>("sensor_rotation_angle", 0.0f);
    this->declare_parameter<double>("robot_radius", 0.2f);
    this->declare_parameter<double>("process_noise_factor", 1.0);
    this->declare_parameter<double>("measurement_noise_factor", 0.05);
    
    updateParameters();

    parameter_event_sub_ = this->add_on_set_parameters_callback(
        std::bind(&CostmapGenerator::parameterCallback, this, std::placeholders::_1));

    Updater = std::make_unique<DynamicCostmapUpdater>(resolution, robot_radius, 0.3, 0.5);

    point_cloud_sub_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
        "/ouster/points", rclcpp::SensorDataQoS(), std::bind(&CostmapGenerator::pointCloudCallback, this, std::placeholders::_1));

    laser_scan_sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
        "/scan", rclcpp::SensorDataQoS(), std::bind(&CostmapGenerator::laserScanCallback, this, std::placeholders::_1));

    costmap_pub = this->create_publisher<nav_msgs::msg::OccupancyGrid>("/costmap", 10);

    marker_pub_ = this->create_publisher<visualization_msgs::msg::Marker>("/obstacle_marker", 10);

    publish_thread_ = std::thread(&CostmapGenerator::publishCostmapLoop, this);
}


CostmapGenerator::~CostmapGenerator() {
    stop_publishing_ = true;
    if (publish_thread_.joinable()) {
        publish_thread_.join();
    }
}

void CostmapGenerator::updateParameters() {
    this->get_parameter("width", width);
    this->get_parameter("height", height);
    this->get_parameter("resolution", resolution);
    this->get_parameter("max_obstacle_height", max_obstacle_height);
    this->get_parameter("gradient_size", GRADIENT_SIZE);
    this->get_parameter("gradient_factor", GRADIENT_FACTOR);
    this->get_parameter("sensor_rotation_angle", sensor_rotation_angle);
    this->get_parameter("robot_radius", robot_radius);
}

rcl_interfaces::msg::SetParametersResult CostmapGenerator::parameterCallback(const std::vector<rclcpp::Parameter>& parameters) {    
    rcl_interfaces::msg::SetParametersResult result;
    result.successful = true;

    for (const auto& param : parameters) {
        RCLCPP_INFO(this->get_logger(), "Parameter changed: %s = %s",
                    param.get_name().c_str(), param.value_to_string().c_str());
    }
    updateParameters();
    return result;
}

//함수 오버로딩
void CostmapGenerator::pointCloudCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg) {
    updateCostmap(msg);
}

void CostmapGenerator::laserScanCallback(const sensor_msgs::msg::LaserScan::SharedPtr msg) {
    updateCostmap(msg);
}

void CostmapGenerator::updateCostmap(const sensor_msgs::msg::PointCloud2::SharedPtr& cloud) {
    const double origin_x = -width * resolution / 2.0;
    const double origin_y = -height * resolution / 2.0;
    std::vector<int8_t> data(width * height, cost::NO_INFORMATION);

    sensor_msgs::PointCloud2ConstIterator<float> iter_x(*cloud, "x");
    sensor_msgs::PointCloud2ConstIterator<float> iter_y(*cloud, "y");
    sensor_msgs::PointCloud2ConstIterator<float> iter_z(*cloud, "z");

    double cos_angle = std::cos(sensor_rotation_angle);
    double sin_angle = std::sin(sensor_rotation_angle);

    double min_distance = 2.0; 
    double max_distance = 3.0; 

    for (size_t i = 0; i < cloud->height * cloud->width; ++i, ++iter_x, ++iter_y, ++iter_z) {
        if (std::isfinite(*iter_x) && std::isfinite(*iter_y) && std::isfinite(*iter_z)) {
            double distance = std::sqrt((*iter_x) * (*iter_x) + (*iter_y) * (*iter_y));

            // 거리 조건 추가
            if (distance >= min_distance && distance <= max_distance && *iter_z <= max_obstacle_height) {
                double rotated_x = *iter_x * cos_angle - *iter_y * sin_angle;
                double rotated_y = *iter_x * sin_angle + *iter_y * cos_angle;

                int grid_x = static_cast<int>((rotated_x - origin_x) / resolution);
                int grid_y = static_cast<int>((rotated_y - origin_y) / resolution);

                if (grid_x >= 0 && grid_x < width && grid_y >= 0 && grid_y < height) {
                    int obstacle_index = grid_y * width + grid_x;
                    data[obstacle_index] = cost::LETHAL_OBSTACLE;  // 장애물 값 설정
                }
            }
        }
    }

    publishCostmap(data, width, height, resolution);
}


void CostmapGenerator::updateCostmap(const sensor_msgs::msg::LaserScan::SharedPtr& scan) {
    const double origin_x = -width * resolution / 2.0;
    const double origin_y = -height * resolution / 2.0;
    std::vector<int8_t> data(width * height, cost::NO_INFORMATION);

    double cos_angle = std::cos(sensor_rotation_angle);
    double sin_angle = std::sin(sensor_rotation_angle);

    double min_distance = 1.0;  // 최소 거리 (예: 0.5m)
    double max_distance = 2.0; // 최대 거리 (예: 5.0m)

    for (size_t i = 0; i < scan->ranges.size(); ++i) {
        double angle = scan->angle_min + i * scan->angle_increment;
        double range = scan->ranges[i];

        // 거리 조건 추가
        if (std::isfinite(range) && range > scan->range_min && range < scan->range_max &&
            range >= min_distance && range <= max_distance) {  // 범위 조건
            double x = range * std::cos(angle);
            double y = range * std::sin(angle);

            double rotated_x = x * cos_angle - y * sin_angle;
            double rotated_y = x * sin_angle + y * cos_angle;

            int grid_x = static_cast<int>((rotated_x - origin_x) / resolution);
            int grid_y = static_cast<int>((rotated_y - origin_y) / resolution);

            if (grid_x >= 0 && grid_x < width && grid_y >= 0 && grid_y < height) {
                int obstacle_index = grid_y * width + grid_x;
                data[obstacle_index] = cost::LETHAL_OBSTACLE;  // 장애물 값 설정
            }
        }
    }

    publishCostmap(data, width, height, resolution);
}

void CostmapGenerator::applyPadding(std::vector<int8_t>& data, int grid_x, int grid_y) {
    for (int dy = -GRADIENT_SIZE; dy <= GRADIENT_SIZE; ++dy) {
        for (int dx = -GRADIENT_SIZE; dx <= GRADIENT_SIZE; ++dx) {
            int neighbor_x = grid_x + dx;
            int neighbor_y = grid_y + dy;

            if (neighbor_x >= 0 && neighbor_x < width && neighbor_y >= 0 && neighbor_y < height) {
                int neighbor_index = neighbor_y * width + neighbor_x;
                unsigned char gradient_cost = std::max(0, cost::LETHAL_OBSTACLE - GRADIENT_FACTOR * (std::abs(dx) + std::abs(dy)));
                data[neighbor_index] = std::max(data[neighbor_index], static_cast<int8_t>(gradient_cost));
            }
        }
    }
}

void CostmapGenerator::publishObstacleMarker(const Eigen::Vector4f& state) {
    auto marker = visualization_msgs::msg::Marker();
    marker.header.frame_id = "map";
    marker.header.stamp = this->get_clock()->now();
    marker.ns = "obstacle";
    marker.id = 0;
    marker.type = visualization_msgs::msg::Marker::ARROW; // 점을 구 형태로 표시
    marker.action = visualization_msgs::msg::Marker::ADD;

    // 장애물의 현재 좌표 (state[0]: x, state[1]: y)
    geometry_msgs::msg::Point start_point;
    start_point.x = state[0];
    start_point.y = state[1];
    start_point.z = 0.0;

    // 속도 벡터 (state[2]: vx, state[3]: vy) 이용
    geometry_msgs::msg::Point end_point;
    end_point.x = state[0] + state[2]*4;  // vx 방향으로 길이를 설정
    end_point.y = state[1] + state[3]*4;  // vy 방향으로 길이를 설정
    end_point.z = 0.0;

    marker.points.push_back(start_point);  // 화살표 시작점
    marker.points.push_back(end_point);    // 화살표 끝점

    // 화살표 크기 설정
    marker.scale.x = 0.5;   // 화살표 두께
    marker.scale.y = 1.3;    // 화살표 머리 크기
    marker.scale.z = 0.1;

    // 색상 설정 (파란색)
    marker.color.r = 0.0;
    marker.color.g = 0.0;
    marker.color.b = 1.0;
    marker.color.a = 1.0; // 투명도 설정

    marker.lifetime = rclcpp::Duration::from_seconds(0); // 영구 표시

    marker_pub_->publish(marker);
}


void CostmapGenerator::publishCostmapLoop() {
    while (rclcpp::ok() && !stop_publishing_) {
        double interval = publish_interval_.load(); 
        {
            std::lock_guard<std::mutex> lock(costmap_mutex_);
            
            if (!latest_data_.empty()) {
                publishCostmap(latest_data_, width, height, resolution);
            }
        }
        std::this_thread::sleep_for(std::chrono::duration<double>(publish_interval_));
    }
}

void CostmapGenerator::publishCostmap(const std::vector<int8_t>& data, int width, int height, double resolution) {
    auto costmap_msg = std::make_shared<nav_msgs::msg::OccupancyGrid>();
    
    costmap_msg->header.stamp = this->get_clock()->now();
    costmap_msg->header.frame_id = "map";
    costmap_msg->info.resolution = resolution;
    costmap_msg->info.width = width;
    costmap_msg->info.height = height;
    costmap_msg->info.origin.position.x = -width * resolution / 2.0;
    costmap_msg->info.origin.position.y = -height * resolution / 2.0;
    costmap_msg->info.origin.orientation.w = 1.0;
    costmap_msg->data = data;
    
    if (Tracker.costmapCallback(costmap_msg)){
        Eigen::Vector4f state = Tracker.getState();
        float obstacle_radius = Tracker.getObstacleRadius();
        Updater -> DAJ(*costmap_msg, state, obstacle_radius, 0.3);
        publishObstacleMarker(state);
    }
    costmap_pub->publish(*costmap_msg);
}


int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<CostmapGenerator>());
    rclcpp::shutdown();
    return 0;
}
