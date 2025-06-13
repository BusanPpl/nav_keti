#include "rclcpp/rclcpp.hpp"
#include "nav_keti/topic_handler.h"
#include "nav_keti/path_selector.h"

class GlobalPathPlannerNode : public rclcpp::Node, public TopicHandler
{
public:
    GlobalPathPlannerNode()
    : Node("global_path_planner_node"),
      TopicHandler(this),
      planner_("rrt_star"),
      gps_ready_(false),
      map_ready_(false)
    {
        declare_parameter<std::string>("planner_algorithm", "rrt_star");
        get_parameter("planner_algorithm", algorithm_type_);
        planner_.setMode(algorithm_type_);
        setGpsCallback(std::bind(&GlobalPathPlannerNode::onGpsMsg, this, std::placeholders::_1));
        setTargetGpsCallback(std::bind(&GlobalPathPlannerNode::onTargetGpsMsg, this, std::placeholders::_1));
        setCostmapCallback(std::bind(&GlobalPathPlannerNode::onCostmapMsg, this, std::placeholders::_1));
        setGoalPoseCallback(std::bind(&GlobalPathPlannerNode::onGoalposeMsg, this, std::placeholders::_1));
        initSubscriptions();
        initPublishers();
    }

private:
    Planner planner_;
    std::string algorithm_type_;
    double robot_x = 0.0, robot_y = 0.0;
    double target_x = 0.0, target_y = 0.0;
    bool gps_ready_, map_ready_;

    nav2_msgs::msg::Costmap latest_costmap_;  

    void onGpsMsg(sensor_msgs::msg::NavSatFix::SharedPtr msg) {
        convertGPSToXY(-msg->latitude, -msg->longitude, robot_x, robot_y);

        if (!gps_ready_) {
            gps_ready_ = true;
            convertGPSToXY(-msg->latitude, -msg->longitude + 0.0002, target_x, target_y);
        }

        if (map_ready_) {
            computePath();
        }
    }

    void onTargetGpsMsg(sensor_msgs::msg::NavSatFix::SharedPtr msg) {
        convertGPSToXY(-msg->latitude, -msg->longitude, target_x, target_y);
    }

    void onCostmapMsg(nav2_msgs::msg::Costmap::SharedPtr msg) {
        latest_costmap_ = *msg;
        map_ready_ = true;
        computePath();
    }

    void onGoalposeMsg(geometry_msgs::msg::PoseStamped::SharedPtr msg) {
        target_x = msg->pose.position.x;
        target_y = msg->pose.position.y;
    }

    void computePath() {
        const auto& costmap = latest_costmap_;  

        if (costmap.data.empty()) {
            return;
        }

        double yaw = getYaw();

        double local_robot_x = ( - costmap.metadata.origin.position.x) / costmap.metadata.resolution;
        double local_robot_y = ( - costmap.metadata.origin.position.y) / costmap.metadata.resolution;

        double costmap_target_x = (target_x - robot_x - costmap.metadata.origin.position.x) / costmap.metadata.resolution;
        double costmap_target_y = (target_y - robot_y - costmap.metadata.origin.position.y) / costmap.metadata.resolution;

        double dx = costmap_target_x - local_robot_x;
        double dy = costmap_target_y - local_robot_y;

        double rotated_dx = dx * cos(yaw) + dy * sin(yaw);
        double rotated_dy = -dx * sin(yaw) + dy * cos(yaw);

        double rotated_target_x = local_robot_x + rotated_dx;
        double rotated_target_y = local_robot_y + rotated_dy;

        if (!clipToCostmapBoundary(local_robot_x, local_robot_y, rotated_target_x, rotated_target_y))
        {
            // RCLCPP_WARN(this->get_logger(), "Target clipped to costmap boundary.");
        }

        planner_.setCostmapData(costmap.data, costmap.metadata.size_x, costmap.metadata.size_y, costmap.metadata.resolution);
        planner_.setStart({local_robot_x, local_robot_y});
        planner_.setGoal({rotated_target_x, rotated_target_y});

        auto path = planner_.smoothPath(planner_.plan(), 130);
        nav_msgs::msg::Path msg;
        msg.header.frame_id = "map";
        msg.header.stamp = now();

        int center_x = costmap.metadata.size_x / 2;
        int center_y = costmap.metadata.size_y / 2;
        double resolution = costmap.metadata.resolution;

        for (const auto& p : path) {
            double x_local = (p.x - center_x) * resolution;
            double y_local = (p.y - center_y) * resolution;

            double x_rot = x_local * cos(yaw) - y_local * sin(yaw);
            double y_rot = x_local * sin(yaw) + y_local * cos(yaw);

            geometry_msgs::msg::PoseStamped pose;
            pose.pose.position.x = robot_x + x_rot;
            pose.pose.position.y = robot_y + y_rot;
            pose.pose.position.z = 0.0;
            msg.poses.push_back(pose);
        }

        publishPath(msg);
    }

    bool clipToCostmapBoundary(double robot_x, double robot_y, double &target_x, double &target_y)
    {
        int map_min_x = 0;
        int map_min_y = 0;
        int map_max_x = latest_costmap_.metadata.size_x - 1;
        int map_max_y = latest_costmap_.metadata.size_y - 1;

        int x0 = static_cast<int>(std::round(robot_x));
        int y0 = static_cast<int>(std::round(robot_y));
        int x1 = static_cast<int>(std::round(target_x));
        int y1 = static_cast<int>(std::round(target_y));
        // Bresenham's line algorithm
        int dx = abs(x1 - x0);
        int dy = abs(y1 - y0);
        int sx = (x0 < x1) ? 1 : -1;
        int sy = (y0 < y1) ? 1 : -1;
        int err = dx - dy;

        int count = 0;
        while (true)
        {
            if (x0 < map_min_x || x0 >= map_max_x || y0 < map_min_y || y0 >= map_max_y)
            {
                target_x = x0 - sx;
                target_y = y0 - sy;
                return false;
            }

            if (x0 == x1 && y0 == y1)
                break;

            int e2 = 2 * err;
            if (e2 > -dy) {
                err -= dy;
                x0 += sx;
            } else if (e2 < dx) {
                err += dx;
                y0 += sy;
            }

            if (++count > 10000)
            {
                RCLCPP_ERROR(this->get_logger(), "Loop exceeded limit, exiting to avoid infinite loop.");
                return false;
            }
        }

        target_x = x1;
        target_y = y1;
        // RCLCPP_INFO(this->get_logger(), "Target position adjusted to x=%.2f, y=%.2f", target_x, target_y);

        return true;
    }
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<GlobalPathPlannerNode>());
    rclcpp::shutdown();
    return 0;
}
