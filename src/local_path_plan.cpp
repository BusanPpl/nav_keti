#include "nav_keti/local_path_plan.h"

using namespace std;
using namespace rclcpp;
using namespace std::placeholders;

LocalPathPlanner::LocalPathPlanner()
: Node("local_path"), TopicHandler(this), r(10) {
    this->declare_parameter("max_v", 1.3);
    this->declare_parameter("v_resolution", 0.05);
    this->declare_parameter("max_omega", 1.0);
    this->declare_parameter("omega_resolution", 0.01);
    this->declare_parameter("dt", 2.0);
    this->declare_parameter("boundary", 50.0);
    this->declare_parameter("thresh_d", 0.25);
    this->declare_parameter("thresh_a", 0.05);
    this->declare_parameter("weights", std::vector<double>{
        1.0, 1.0, 1.0, 1.0, 3.0, 0.0, 0.0, 2.0, 0.0, 0.0, 0.0, 0.0
    });

    // Load parameters
    double max_v = this->get_parameter("max_v").as_double();
    double v_resolution = this->get_parameter("v_resolution").as_double();
    double max_omega = this->get_parameter("max_omega").as_double();
    double omega_resolution = this->get_parameter("omega_resolution").as_double();
    std::vector<double> weights = this->get_parameter("weights").as_double_array();

    dt = this->get_parameter("dt").as_double();
    boundary = this->get_parameter("boundary").as_double();
    thresh_dist = this->get_parameter("thresh_d").as_double();
    thresh_ang = this->get_parameter("thresh_a").as_double();

    trans_spd = max_v;
    ang_spd = max_omega;

    robot_x = robot_y = robot_yaw = 0.0;
    vx = w = 0.0;
    f1 = f2 = detect = false;
    last_time = this->get_clock()->now();

    dwa = std::make_unique<DWA>(max_v, v_resolution, max_omega, omega_resolution, dt, weights);

    // 콜백 등록
    setGpsCallback(std::bind(&LocalPathPlanner::onGpsMsg, this, _1));
    setTargetGpsCallback(std::bind(&LocalPathPlanner::onTargetGpsMsg, this, _1));
    setLaserScanCallback(std::bind(&LocalPathPlanner::laserCallback, this, _1));
    setCostmapCallback(std::bind(&LocalPathPlanner::costmapCallback, this, _1));
    setGlobalPathCallback(std::bind(&LocalPathPlanner::globalPathCallback, this, _1));
    setGoalPoseCallback(std::bind(&LocalPathPlanner::onGoalposeMsg, this, _1));

    initSubscriptions();
    initPublishers();

    running_ = false;
    thread_obj = std::thread(&LocalPathPlanner::LocalPathPlannerGoal, this);
}

LocalPathPlanner::~LocalPathPlanner() {
    stopSpeed();
    RCLCPP_INFO(this->get_logger(), "LocalPathPlanner node terminated.");
    if (thread_obj.joinable()) thread_obj.join();
}

void LocalPathPlanner::onGpsMsg(sensor_msgs::msg::NavSatFix::SharedPtr msg) {
    convertGPSToXY(msg->latitude, msg->longitude, robot_x, robot_y);
    robot_x = - robot_x;
    robot_y = - robot_y;
    
    //RCLCPP_INFO(this->get_logger(), "robot_pose x: %.2f y: %.2f", robot_x, robot_y);
    if (!gps_ready) {
        gps_ready = true;
        convertGPSToXY(msg->latitude, msg->longitude + 0.002, target_x, target_y);
        target_x = - target_x;
        target_y = - target_y;
        //RCLCPP_INFO(this->get_logger(), "goal_pose x: %.2f y: %.2f", target_x, target_y);
    }
}

void LocalPathPlanner::onTargetGpsMsg(sensor_msgs::msg::NavSatFix::SharedPtr msg) {
    convertGPSToXY(-msg->latitude, -msg->longitude, target_x, target_y); 
}

std::tuple<double, double> LocalPathPlanner::getDir() {
    robot_yaw = getYaw(); 
    double dx = target_x - robot_x; 
    double dy = target_y - robot_y; 
    double local_x = dx * cos(robot_yaw) + dy * cos(M_PI / 2 - robot_yaw); 
    double local_y = -dx * cos(M_PI / 2 - robot_yaw) + dy * cos(robot_yaw); 
    if (std::isnan(local_x) || std::isnan(local_y)) return {0.0, 0.0}; 
    double local_theta = atan2(local_y, local_x); 
    double dist = sqrt(dx * dx + dy * dy); 
    return {dist, local_theta}; 
}

void LocalPathPlanner::laserCallback(const sensor_msgs::msg::LaserScan::SharedPtr msg) {
    if (!laser_ready) {
        laser_ready = true;
        RCLCPP_INFO(this->get_logger(), "laser_ready");
    }
    //장애물 없을때를 대비해야함
    detect = std::any_of(msg->ranges.begin(), msg->ranges.end(), [this](float r) {
        return std::isfinite(r) && r < boundary;
    });

    detect = true;

    dwa->laserCallback(msg);
    last_time = this->get_clock()->now();
}

void LocalPathPlanner::costmapCallback(const nav2_msgs::msg::Costmap::SharedPtr msg) {
    if (!costmap_ready) {
        costmap_ready = true;
        RCLCPP_INFO(this->get_logger(), "costmap_ready");
    }
    dwa->costmapCallback(msg);
}

void LocalPathPlanner::globalPathCallback(const nav_msgs::msg::Path::SharedPtr msg) {
    if (!global_path_ready) {
        global_path_ready = true;
        RCLCPP_INFO(this->get_logger(), "global_path_ready");
    }
    dwa->updateGlobalPlan(msg->poses);
}

void LocalPathPlanner::onGoalposeMsg(geometry_msgs::msg::PoseStamped::SharedPtr msg) {
    target_x = msg->pose.position.x;
    target_y = msg->pose.position.y;
    tf2::Quaternion q(msg->pose.orientation.x, msg->pose.orientation.y, msg->pose.orientation.z, msg->pose.orientation.w);
    double roll, pitch, yaw;
    tf2::Matrix3x3(q).getRPY(roll, pitch, yaw);
    target_yaw = yaw;
    RCLCPP_INFO(this->get_logger(), "New goal_pose x: %.2f y: %.2f", target_x, target_y);

    if (!running_) {
        if (thread_obj.joinable()) thread_obj.join();
        running_ = true;
        thread_obj = std::thread(&LocalPathPlanner::LocalPathPlannerGoal, this);
    }
}

void LocalPathPlanner::LocalPathPlannerGoal() {
    double dist, local_theta;
    running_ = true;
    while (!laser_ready || !costmap_ready || !global_path_ready) {
        //RCLCPP_WARN(this->get_logger(), "Waiting for topics...");
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }

    std::tie(dist, local_theta) = getDir();
    //목적지로의 주행
    while (dist > thresh_dist) {
        if (!detect) {
            speed_.linear.x = 0;
            speed_.angular.z = checkAngularLimitVelocity(local_theta);
        } else {
            dwa->updateGlobalPlan(globalPath().poses);
            dwa->costmapCallback(std::make_shared<nav2_msgs::msg::Costmap>(costmap()));

            RobotState robot_state(robot_x, robot_y, getYaw());
            Goal goal(target_x, target_y);

            auto vel = dwa->computeVelocityCommands(robot_state, goal, dt);
            if (vel.size() < 2) continue;

            speed_.linear.x = checkLinearLimitVelocity(vel[0]) * 1.5;
            speed_.angular.z = checkAngularLimitVelocity(vel[1]) * 1.5;
        }

        publishCmd(speed_);
        std::tie(dist, local_theta) = getDir();
        RCLCPP_INFO(this->get_logger(), "Velocity vx: %.2f yaw: %.2f, dist: %.2f", speed_.linear.x, speed_.angular.z, dist);
    }

    double dyaw = target_yaw - getYaw();
    //목적지에서의 회전 정렬
    while (true) {
        if (std::fabs(std::sin(dyaw)) < 0.01 && std::fabs(std::cos(dyaw)) > std::cos(thresh_ang)) break;
        speed_.linear.x = 0;
        speed_.angular.z = checkAngularLimitVelocity(dyaw > 0 ? std::fabs(dyaw) : -std::fabs(dyaw));
        publishCmd(speed_);
        dyaw = target_yaw - getYaw();
    }

    stopSpeed();
    running_ = false;
    RCLCPP_INFO(this->get_logger(), "Goal reached");
}

void LocalPathPlanner::stopSpeed() {
    speed_.linear.x = 0;
    speed_.angular.z = 0;
    publishCmd(speed_);
}

double LocalPathPlanner::constrain(double input, double low, double high) {
    return std::max(low, std::min(input, high));
}

double LocalPathPlanner::checkLinearLimitVelocity(double vel) {
    return constrain(vel, -trans_spd, trans_spd);
}

double LocalPathPlanner::checkAngularLimitVelocity(double vel) {
    return constrain(vel, -ang_spd, ang_spd);
}

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<LocalPathPlanner>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
