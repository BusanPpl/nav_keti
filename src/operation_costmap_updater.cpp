#include "nav_keti/operation_costmap_updater.h"


OperationCostmapUpdater::OperationCostmapUpdater(double resolution, double robot_radius, double inscribed_radius, double cost_scaling_factor)
    : Node("operation_updater"), resolution_(resolution) {}


void OperationCostmapUpdater::getVel(double v, double yaw, double angle) {
    robot_vel = v;
    robot_yaw = yaw;
    robot_angle = angle;
}


void OperationCostmapUpdater::update(nav_msgs::msg::OccupancyGrid &costmap_msg, const std::vector<Object> &objects) {
    const std::unordered_set<int> static_labels = {1, 2, 10}; //무시함
    static const std::unordered_set<int> target_labels = {1, 2, 3, 4};

    for (const auto& obj : objects) {
        if (static_labels.count(obj.label)) {
            RCLCPP_INFO(this->get_logger(), "Skipping static label: %d", obj.label);
            continue;
        }

        if (target_labels.count(obj.label)) {
            double padding = getPaddingFactor(obj.label);
            applyCircularPadding(costmap_msg, obj, padding);
            // RCLCPP_INFO(this->get_logger(), "Labeled object [%d] padded at (%.2f, %.2f), r=%.2f",
            //             obj.label, obj.position.x(), obj.position.y(), padding * obj.radius);
        } else {
            applyDefaultPadding(costmap_msg, obj);
            //RCLCPP_INFO(this->get_logger(), "Default padding for label [%d]", obj.label);
        }
    }
}


void OperationCostmapUpdater::applyCircularPadding(nav_msgs::msg::OccupancyGrid &costmap_msg,
                                                   const Object &object,
                                                   double padding_factor) {

    const double radius = object.radius * padding_factor;

    const double origin_x = costmap_msg.info.origin.position.x;
    const double origin_y = costmap_msg.info.origin.position.y;
    const int width = costmap_msg.info.width;
    const int height = costmap_msg.info.height;

    const double cx = object.position.x();
    const double cy = object.position.y();

    int cell_radius = static_cast<int>(std::ceil(radius / resolution_));

    for (int dy = -cell_radius; dy <= cell_radius; ++dy) {
        for (int dx = -cell_radius; dx <= cell_radius; ++dx) {
            double px = cx + dx * resolution_;
            double py = cy + dy * resolution_;
            double dist = std::hypot(px - cx, py - cy);

            if (dist <= radius) {
                int grid_x = static_cast<int>((px - origin_x) / resolution_);
                int grid_y = static_cast<int>((py - origin_y) / resolution_);

                if (grid_x >= 0 && grid_x < width && grid_y >= 0 && grid_y < height) {
                    int index = grid_y * width + grid_x;
                    costmap_msg.data[index] = cost::LETHAL_OBSTACLE;
                }
            }
        }
    }
}


void OperationCostmapUpdater::applyDefaultPadding(nav_msgs::msg::OccupancyGrid &costmap_msg, const Object &object) {
    applyCircularPadding(costmap_msg, object, 1.0); // 기본 패딩 계수
}

double OperationCostmapUpdater::getPaddingFactor(int label) {
    switch (label) {
        case 1: return 3.0;  // Pedestrian
        case 2: return 2.0;  // Barrel
        case 3: return 1.0;  // Robot
        case 4: return 1.0;  // Car
        default: return 1.0; // 기본 값
    }
}

