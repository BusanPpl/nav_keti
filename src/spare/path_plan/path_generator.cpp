#include "nav_keti/path_generator.h"


using namespace std::chrono_literals;
using namespace rclcpp;
using namespace std;

PathGenerator::PathGenerator() : Node("Astar"), map_exist_(false), pose_flag(false)
{   
    RCLCPP_INFO(this->get_logger(), "Astar node");
    this->declare_parameter<string>("target_frame", "map");
    this->declare_parameter<string>("source_frame", "base_link");
    this->declare_parameter<vector<double>>("goal_pose", {1.5, -0.5});
    tf_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(this);
    tf_buffer_=std::make_shared<tf2_ros::Buffer>(this->get_clock());
    tf_listener_=std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
    pub_robot_path_ = this->create_publisher<nav_msgs::msg::Path>("planned_path", 10);
    this_thread::sleep_for(2s);
    requestMap();
}

PathGenerator::~PathGenerator()
{
    RCLCPP_INFO(this->get_logger(), "Astar node terminated.");
}


void PathGenerator::requestMap()
{
    client_ = this->create_client<nav_msgs::srv::GetMap>("/map_server/map");

    while (!client_->wait_for_service(5s)) {
      if (!ok()) {
        RCLCPP_ERROR(this->get_logger(), "Interrupted while waiting for the service. Exiting.");
        return;
      }
      RCLCPP_INFO(this->get_logger(), "Waiting for the map service to be available...");
    }

    auto request = std::make_shared<nav_msgs::srv::GetMap::Request>();
    auto future = client_->async_send_request(request);
    
    auto result = spin_until_future_complete(this->get_node_base_interface(), future);

    if (result == FutureReturnCode::SUCCESS) {
      auto response = future.get();
      RCLCPP_INFO(this->get_logger(), "Map received. Width: %d, Height: %d",
                  response->map.info.width, response->map.info.height);
    } else {
      RCLCPP_ERROR(this->get_logger(), "Failed to call service /map_server/map");
    }

    handleMapResponse(future);

}

void PathGenerator::handleMapResponse(Client<nav_msgs::srv::GetMap>::SharedFuture future)
{
    RCLCPP_INFO(this->get_logger(), "handleMapResponse");
    auto response = future.get();
    if (!response || !response->map.info.width || !response->map.info.height) {
        RCLCPP_ERROR(this->get_logger(), "Failed");
        return;
    }

    map_info_ = response->map.info;

    // A* 맵 설정
    map_generator_.setWorldSize({(int)map_info_.width, (int)map_info_.height});
    map_generator_.setHeuristic(AStar::Heuristic::manhattan);
    map_generator_.setDiagonalMovement(true);


    std::vector<uint8_t> costmap_data(response->map.data.begin(), response->map.data.end());
    map_generator_.setCostmap(costmap_data);


    RCLCPP_INFO(this->get_logger(), "map_info_");
    // 벽 추가, 여기 잘못 세팅하면 거의 무한루프에 가까움
    int x, y;
    for (int i = 0; i < map_info_.width * map_info_.height; ++i) {
        x = i % map_info_.width;
        y = i / map_info_.width;

        if (response->map.data[i] >0) {
            map_generator_.addCollision({x, y}, 2);  // 값이 0이 아니면 장애물로 인식
        }
    }

    map_exist_ = true;
    RCLCPP_INFO(this->get_logger(), "Map received");
    navGoalHandler();
    
}

void PathGenerator::pathCallback(const nav_msgs::msg::Path::SharedPtr msg)
{

    for (size_t i = 0; i < msg->poses.size(); ++i)
    {   
        auto pose = msg->poses[i];
        geometry_msgs::msg::TransformStamped transform_stamped;

        transform_stamped.header.stamp = this->get_clock()->now();

        transform_stamped.header.frame_id = "map";
        transform_stamped.child_frame_id = "path_pose_" + std::to_string(i); 

        transform_stamped.transform.translation.x = pose.pose.position.x;
        transform_stamped.transform.translation.y = pose.pose.position.y;
        transform_stamped.transform.translation.z = pose.pose.position.z;

        transform_stamped.transform.rotation.x = 0.0;
        transform_stamped.transform.rotation.y = 0.0;
        transform_stamped.transform.rotation.z = 0.0;
        transform_stamped.transform.rotation.w = 1.0;
        tf_broadcaster_->sendTransform(transform_stamped);
    }
}

void PathGenerator::robotPose() {
    RCLCPP_INFO(this->get_logger(), "robotPose");
    string target_frame_, source_frame_;
    this->get_parameter("target_frame", target_frame_);
    this->get_parameter("source_frame", source_frame_);
    geometry_msgs::msg::TransformStamped transform;

    try {
        transform = tf_buffer_->lookupTransform(target_frame_, source_frame_, tf2::timeFromSec(0));

        r_x = transform.transform.translation.x;
        r_y = transform.transform.translation.y;
    } 
    catch (const tf2::TransformException& ex) {
        RCLCPP_ERROR(this->get_logger(), "%s", ex.what());
    }
}

void PathGenerator::navGoalHandler()
{
    if (!map_exist_) {
        RCLCPP_WARN(this->get_logger(), "Map is not ready yet.");
        return;
    }
    robotPose();
    vector<double> goalpose;
    this->get_parameter("goal_pose", goalpose);
    float goal_x = goalpose[0];
    float goal_y = goalpose[1];
    AStar::Vec2i target;
    target.x = (goal_x - map_info_.origin.position.x) / map_info_.resolution;
    target.y = (goal_y - map_info_.origin.position.y) / map_info_.resolution;
    AStar::Vec2i source;
    source.x = (r_x - map_info_.origin.position.x) / map_info_.resolution;
    source.y = (r_y - map_info_.origin.position.y) / map_info_.resolution;
    RCLCPP_INFO(this->get_logger(), "findPath");
    auto path = map_generator_.findPath(source, target);
    RCLCPP_INFO(this->get_logger(), "finish findPath");
    nav_msgs::msg::Path path_msg;
    if (path.empty()) {
        RCLCPP_WARN(this->get_logger(), "No path found.");
        return;
    }

    for (auto coordinate = path.end() - 1; coordinate >= path.begin(); --coordinate) {
        geometry_msgs::msg::PoseStamped point_pose;
        point_pose.pose.position.x = (coordinate->x * map_info_.resolution) + map_info_.origin.position.x;
        point_pose.pose.position.y = (coordinate->y * map_info_.resolution) + map_info_.origin.position.y;
        path_msg.poses.push_back(point_pose);
    }

    path_msg.header.frame_id = "map";
    
    pub_robot_path_->publish(path_msg);
    RCLCPP_INFO(this->get_logger(), "Path published.");
    pathCallback(std::make_shared<nav_msgs::msg::Path>(path_msg));
    shutdown();

}




int main(int argc, char **argv)
{
    init(argc, argv);
    auto PG = std::make_shared<PathGenerator>();
    spin(PG);
    shutdown();
    return 0;
}
