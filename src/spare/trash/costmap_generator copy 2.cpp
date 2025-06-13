#include "nav_keti/costmap_generator.h"
using json = nlohmann::json;

CostmapGenerator::CostmapGenerator() : Node("costmap_generator"), TopicHandler(this){
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

    auto custom_qos = rclcpp::QoS(rclcpp::KeepLast(1)).transient_local().reliable();

    parameter_event_sub_ = this->add_on_set_parameters_callback(
        std::bind(&CostmapGenerator::parameterCallback, this, std::placeholders::_1));

    Updater = std::make_unique<DynamicCostmapUpdater>(resolution, robot_radius, 0.1, 0.8);

    tracker_states_sub_ = this->create_subscription<std_msgs::msg::Float32MultiArray>(
        "/tracker_states", 10,
    std::bind(&CostmapGenerator::trackerStatesCallback, this, std::placeholders::_1));

    detect_sub_ = this->create_subscription<std_msgs::msg::String>(
        "/detection_result",
        10,
        std::bind(&CostmapGenerator::detectCallback, this, std::placeholders::_1)
    );

    tf_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(*this);
    tf_buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

    //Getter 함수로 구현해봐야 도움이 안됨 왜냐면 작동 시퀀스를 만들기 어려움. 타이머로 조절하는건 조금 위험하지 않을까 생각함
    setPointCloudCallback(std::bind(&CostmapGenerator::pointCloudCallback, this, std::placeholders::_1));
    setLaserScanCallback(std::bind(&CostmapGenerator::laserScanCallback, this, std::placeholders::_1));
    initSubscriptions();
    initPublishers();
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

void CostmapGenerator::detectCallback(const std_msgs::msg::String::SharedPtr msg) {
    RCLCPP_INFO(this->get_logger(), "[COSTMAP] Received detection message: %s", msg->data.c_str());
    try {
        json parsed = json::parse(msg->data);

        for (const auto& obj : parsed) {
            int label = obj["label_3d"];
            std::string class_name = obj["class_name"];
            float score = obj["score_3d"];
            auto bbox = obj["bbox_3d"];

            RCLCPP_INFO(this->get_logger(), "[COSTMAP] Label: %d, Class: %s, Score: %f", label, class_name.c_str(), score);

            std::vector<float> bbox_list = bbox.get<std::vector<float>>();
            for (auto v : bbox_list) {
                RCLCPP_INFO(this->get_logger(), "[COSTMAP] BBox value: %f", v);
            }
        }
    } catch (json::parse_error& e) {
        RCLCPP_ERROR(this->get_logger(), "JSON parse error: %s", e.what());
    } catch (std::exception& e) {
        RCLCPP_ERROR(this->get_logger(), "Error: %s", e.what());
    }
}


void CostmapGenerator::trackerStatesCallback(const std_msgs::msg::Float32MultiArray::SharedPtr msg) {
    for (size_t i = 0; i + 4 < msg->data.size(); i += 5) {
        float x = msg->data[i + 1];
        float y = msg->data[i + 2];
        float vx = msg->data[i + 3];
        float vy = msg->data[i + 4];
        if (x == 0.0f && y == 0.0f && vx == 0.0f && vy == 0.0f) continue;
        double radius = 0.4; // 또는 동적으로 추정
        obstacles.emplace_back(x, y, vx, vy, radius);
    }
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

    for (size_t i = 0; i < cloud->height * cloud->width; ++i, ++iter_x, ++iter_y, ++iter_z) {
        if (std::isfinite(*iter_x) && std::isfinite(*iter_y) && std::isfinite(*iter_z)) {
            if (*iter_z < 0 || *iter_z > max_obstacle_height) {
                continue;
            }

            double rotated_x = *iter_x * cos_angle - *iter_y * sin_angle;
            double rotated_y = *iter_x * sin_angle + *iter_y * cos_angle;

            int grid_x = static_cast<int>((rotated_x - origin_x) / resolution);
            int grid_y = static_cast<int>((rotated_y - origin_y) / resolution);

            if (grid_x >= 0 && grid_x < width && grid_y >= 0 && grid_y < height) {
                int obstacle_index = grid_y * width + grid_x;
                data[obstacle_index] = cost::LETHAL_OBSTACLE;
                applyPadding(data, grid_x, grid_y);
            }
        }
    }

    generateCostmap(data, width, height, resolution);
}


void CostmapGenerator::updateCostmap(const sensor_msgs::msg::LaserScan::SharedPtr& scan) {

    const double origin_x = -width * resolution / 2.0;
    const double origin_y = -height * resolution / 2.0;
    std::vector<int8_t> data(width * height, cost::NO_INFORMATION); 

    double cos_angle = std::cos(sensor_rotation_angle);
    double sin_angle = std::sin(sensor_rotation_angle);

    for (size_t i = 0; i < scan->ranges.size(); ++i) {
        double angle = scan->angle_min + i * scan->angle_increment;
        double range = scan->ranges[i];

        if (std::isfinite(range) && range > scan->range_min && range < scan->range_max) {
            double x = range * std::cos(angle);
            double y = range * std::sin(angle);

            double rotated_x = x * cos_angle - y * sin_angle;
            double rotated_y = x * sin_angle + y * cos_angle;

            int grid_x = static_cast<int>((rotated_x - origin_x) / resolution);
            int grid_y = static_cast<int>((rotated_y - origin_y) / resolution);

            if (grid_x >= 0 && grid_x < width && grid_y >= 0 && grid_y < height) {
                int obstacle_index = grid_y * width + grid_x;
                data[obstacle_index] = cost::LETHAL_OBSTACLE;
                applyPadding(data, grid_x, grid_y);
            }

        }
    }

    generateCostmap(data, width, height, resolution);
}


void CostmapGenerator::applyPadding(std::vector<int8_t>& data, int grid_x, int grid_y) {
    // 감쇠 상수 (클수록 빠르게 감쇠함)
    const double decay_factor = 0.3;  // 조절 가능한 파라미터

    for (int dy = -GRADIENT_SIZE; dy <= GRADIENT_SIZE; ++dy) {
        for (int dx = -GRADIENT_SIZE; dx <= GRADIENT_SIZE; ++dx) {
            int neighbor_x = grid_x + dx;
            int neighbor_y = grid_y + dy;

            if (neighbor_x >= 0 && neighbor_x < width && neighbor_y >= 0 && neighbor_y < height) {
                int neighbor_index = neighbor_y * width + neighbor_x;

                // 장애물로부터의 유클리드 거리 계산
                double distance = std::sqrt(dx * dx + dy * dy);

                // 지수 함수 기반 감쇠 비용 계산
                int gradient_cost = static_cast<int>(
                    cost::LETHAL_OBSTACLE * std::exp(-decay_factor * distance));

                // 최소값 제한 (0보다 작은 값 방지)
                gradient_cost = std::clamp(gradient_cost, 0, static_cast<int>(cost::LETHAL_OBSTACLE));

                // 최대 비용 유지
                data[neighbor_index] = std::max(data[neighbor_index], static_cast<int8_t>(gradient_cost));
            }
        }
    }
}

std::vector<int8_t> CostmapGenerator::normalizeCostmap(const std::vector<int8_t>& data) {
    std::vector<int8_t> normalized_data = data;
    static char cost_translation_table[256] = {0};
    cost_translation_table[0] = 0;
    cost_translation_table[253] = 99;
    cost_translation_table[254] = 100;
    cost_translation_table[255] = -1;

    for (int i = 1; i < 253; i++) {
        cost_translation_table[i] = static_cast<char>(1 + (97 * (i - 1)) / 251);
    }

    for (size_t i = 0; i < normalized_data.size(); ++i) {
        normalized_data[i] = cost_translation_table[static_cast<uint8_t>(data[i])];
    }

    return normalized_data;
}

void CostmapGenerator::generateCostmap(const std::vector<int8_t>& data, int width, int height, double resolution) {
    auto occupancy_grid_msg = std::make_shared<nav_msgs::msg::OccupancyGrid>();

    occupancy_grid_msg->header.stamp = this->get_clock()->now();
    // occupancy_grid_msg->header.frame_id = "map";
    occupancy_grid_msg->header.frame_id = "base_link";
    occupancy_grid_msg->info.resolution = resolution;
    occupancy_grid_msg->info.width = width;
    occupancy_grid_msg->info.height = height;
    occupancy_grid_msg->info.origin.position.x = -width * resolution / 2.0;
    occupancy_grid_msg->info.origin.position.y = -height * resolution / 2.0;
    occupancy_grid_msg->info.origin.orientation.w = 1.0;
    occupancy_grid_msg->data = data;
    
    std::lock_guard<std::mutex> lock(costmap_mutex_);

    if (!obstacles.empty()) {
        Updater->ADIC(*occupancy_grid_msg, obstacles, 0.3);
    }
    obstacles.clear();
    publishCostmap(occupancy_grid_msg);
    publishGridmap(occupancy_grid_msg);
}

void CostmapGenerator::publishCostmap(const std::shared_ptr<nav_msgs::msg::OccupancyGrid>& occupancy_grid_msg) {
    auto costmap_msg = std::make_shared<nav2_msgs::msg::Costmap>();
    costmap_msg->header = occupancy_grid_msg->header;
    costmap_msg->metadata.resolution = occupancy_grid_msg->info.resolution;
    costmap_msg->metadata.size_x = occupancy_grid_msg->info.width;
    costmap_msg->metadata.size_y = occupancy_grid_msg->info.height;
    costmap_msg->metadata.origin = occupancy_grid_msg->info.origin;
    costmap_msg->data.assign(occupancy_grid_msg->data.begin(), occupancy_grid_msg->data.end());
    publishRawCostmap(*costmap_msg);
}

void CostmapGenerator::publishGridmap(const std::shared_ptr<nav_msgs::msg::OccupancyGrid>& occupancy_grid_msg) {
    occupancy_grid_msg->data = normalizeCostmap(occupancy_grid_msg->data);
    publishOccupancyGrid(*occupancy_grid_msg);
}

void CostmapGenerator::publishObstacleMarker(const Eigen::Vector4f& state) {
    auto marker = visualization_msgs::msg::Marker();
    marker.header.frame_id = "base_link";
    // marker.header.frame_id = "map";
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

    publishMarker(marker);
}

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<CostmapGenerator>());
    rclcpp::shutdown();
    return 0;
}
