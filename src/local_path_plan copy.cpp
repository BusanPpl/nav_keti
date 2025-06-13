#include "nav_keti/local_path_plan.h"

using namespace std;
using namespace rclcpp;
using namespace std::placeholders;

LocalPathPlanner::LocalPathPlanner()
: Node("local_path"), TopicHandler(this), r(10)
{
    this->declare_parameter("max_v", 1.3);
    this->declare_parameter("v_resolution", 0.05);
    this->declare_parameter("max_omega", 1.0);
    this->declare_parameter("omega_resolution", 0.01);
    this->declare_parameter("dt", 2.0);
    this->declare_parameter("boundary", 50.0);
    this->declare_parameter("thresh_d", 0.25);
    this->declare_parameter("thresh_a", 0.05);
    this->declare_parameter("weights", std::vector<double>{
        1.0, // direction
        1.0, // speed
        1.0, // obstacle distance
        1.0, // costmap
        3.0, // global plan align
        0.0, // global path distance
        0.0, // goal align
        2.0, // goal distance
        0.0, // mapgrid
        0.0, // rotate to goal
        0.0, // twirling
        0.0  // oscillation
    });

    // Load parameters
    double max_v = this->get_parameter("max_v").as_double();
    double v_resolution = this->get_parameter("v_resolution").as_double();
    double max_omega = this->get_parameter("max_omega").as_double();
    double omega_resolution = this->get_parameter("omega_resolution").as_double();
    std::vector<double> weights = this->get_parameter("weights").as_double_array();

    std::ostringstream oss;
    oss << "weights: ";
    for (double w : weights) {
        oss << w << " ";
    }
    RCLCPP_INFO(this->get_logger(), "%s", oss.str().c_str());

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

    gps_sub_ = this->create_subscription<sensor_msgs::msg::NavSatFix>("gps", 10, std::bind(&LocalPathPlanner::gpsCallback, this, _1));
    target_gps_sub_ = this->create_subscription<sensor_msgs::msg::NavSatFix>("target_gps", 10, std::bind(&LocalPathPlanner::targetGpsCallback, this, _1));
    imu_subscriber_ = this->create_subscription<sensor_msgs::msg::Imu>("imu", 10, std::bind(&LocalPathPlanner::imuCallback, this, _1));
    scan_sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>("scan", 10, std::bind(&LocalPathPlanner::laserCallback, this, _1));
    costmap_sub_ = this->create_subscription<nav2_msgs::msg::Costmap>("costmap_raw", 10, std::bind(&LocalPathPlanner::costmapCallback, this, _1));
    global_path_sub_ = this->create_subscription<nav_msgs::msg::Path>("planned_path", 10, std::bind(&LocalPathPlanner::globalPathCallback, this, _1));
    goal_pose_sub_ = this->create_subscription<geometry_msgs::msg::PoseStamped>("/goal_pose", 10, std::bind(&LocalPathPlanner::goalPoseCallback, this, _1));
    cmd_pub_ = this->create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 10);

    running_ = false;
    thread_obj = std::thread(&LocalPathPlanner::LocalPathPlannerGoal, this);

    initSubscriptions();
    initPublishers();
}

LocalPathPlanner::~LocalPathPlanner() {
    stopSpeed();
    RCLCPP_INFO(this->get_logger(), "LocalPathPlanner node terminated.");
    if (thread_obj.joinable()) {
        thread_obj.join();  
    }
}

void LocalPathPlanner::gpsCallback(const sensor_msgs::msg::NavSatFix::SharedPtr msg) 
{   
    //RCLCPP_INFO(this->get_logger(), "lat: %.12f, long: %.12f", msg->latitude, msg->longitude);
    if(!f1) { 
        f1 = true; 
        convertGPSToXY(-msg->latitude, -msg->longitude + 0.0002, target_x, target_y);
    }
    convertGPSToXY(-msg->latitude, -msg->longitude, robot_x, robot_y); 
}

void LocalPathPlanner::targetGpsCallback(const sensor_msgs::msg::NavSatFix::SharedPtr msg)
{
    // target_latitude_ = msg->latitude;
    // target_longitude_ = msg->longitude;
    convertGPSToXY(msg->latitude, msg->longitude, target_x, target_y); 
}

void LocalPathPlanner::convertGPSToXY(double latitude, double longitude, double &x, double &y)
{
    double lat_rad = latitude * M_PI / 180.0;
    double lon_rad = longitude * M_PI / 180.0;

    // 위도, 경도를 미터 단위로 변환 (지구 반경 6378137m, 평면 근사)
    const double earth_radius = 6378137.0;  // 미터 단위 (WGS-84)

    // 위도 및 경도 차이를 거리(m)로 변환 (위도 차이는 항상 일정)
    x = lon_rad * earth_radius * cos(lat_rad);  // 경도는 위도에 따라 조정
    y = lat_rad * earth_radius;
}

std::tuple<double, double> LocalPathPlanner::getDir() { 
    //RCLCPP_INFO(this->get_logger(), "robot_x: %.2f, robot_y: %.2f", robot_x, robot_y);
    //RCLCPP_INFO(this->get_logger(), "target_x: %.2f, target_y: %.2f", target_x, target_y);
    
    double dx = target_x - robot_x; 
    double dy = target_y - robot_y; 
    double local_x = dx * cos(robot_yaw) + dy * cos(M_PI / 2 - robot_yaw); 
    double local_y = -dx * cos(M_PI / 2 - robot_yaw) + dy * cos(robot_yaw); 
    if (std::isnan(local_x) || std::isnan(local_y)) {
        RCLCPP_ERROR(this->get_logger(), "[getDir] NaN detected! dx=%.3f dy=%.3f robot_yaw=%.3f", dx, dy, robot_yaw);
        return std::make_tuple(0.0, 0.0);  // 안전하게 리턴
    }
    double local_theta = atan2(local_y, local_x); 
    double dist = sqrt(dx * dx + dy * dy); 
    //RCLCPP_INFO(this->get_logger(), "dist: %.2f, theta: %.2f", dist, local_theta);
    return make_tuple(dist, local_theta); 
} 

void LocalPathPlanner::laserCallback(const sensor_msgs::msg::LaserScan::SharedPtr msg) {
    if (!laser_ready) {
        laser_ready=true;
    }
    auto laser_data = msg->ranges;
    detect = true;
    for (const auto& laser : laser_data) {
        if (laser < boundary) {
            // dt = (this->get_clock()->now() - last_time).seconds();
            // RCLCPP_INFO(this->get_logger(), "%.2f",dt);
            detect = true;
            dwa->laserCallback(msg);
            last_time = this->get_clock()->now();
            break;
        }
    }
}

void LocalPathPlanner::imuCallback(const sensor_msgs::msg::Imu::SharedPtr msg) {

    tf2::Quaternion quat(
        msg->orientation.x,
        msg->orientation.y,
        msg->orientation.z,
        msg->orientation.w
    );
    double declination = 0.0; //0.122 = 전주
    double roll, pitch;
    tf2::Matrix3x3(quat).getRPY(roll, pitch, robot_yaw);
    robot_yaw += declination;
    if (robot_yaw < 0) {
        robot_yaw += 2 * M_PI;
    }
}

void LocalPathPlanner::costmapCallback(const nav2_msgs::msg::Costmap::SharedPtr msg) {
    if (!costmap_ready) {
        costmap_ready =true;
    }
    // std::lock_guard<std::mutex> lock(data_mutex);
    latest_costmap_msg_ = msg;
    dwa->costmapCallback(msg);
}

void LocalPathPlanner::globalPathCallback(const nav_msgs::msg::Path::SharedPtr msg) {
    if (!global_path_ready) {
        global_path_ready =true;
    }
    // std::lock_guard<std::mutex> lock(data_mutex);
    dwa->updateGlobalPlan(msg->poses); 
}

void LocalPathPlanner::goalPoseCallback(const geometry_msgs::msg::PoseStamped::SharedPtr msg)
{
    target_x = msg->pose.position.x;
    target_y = msg->pose.position.y;
    tf2::Quaternion q(
        msg->pose.orientation.x,
        msg->pose.orientation.y,
        msg->pose.orientation.z,
        msg->pose.orientation.w
    );
    double roll, pitch, yaw;
    tf2::Matrix3x3(q).getRPY(roll, pitch, yaw);
    target_yaw = yaw;
    RCLCPP_INFO(this->get_logger(), "Received new goal_pose → x: %.2f, y: %.2f", target_x, target_y);

    if (!running_) {
        if (thread_obj.joinable()) {
            thread_obj.join();  
        }
        running_ = true;
        thread_obj = std::thread(&LocalPathPlanner::LocalPathPlannerGoal, this);
    }
}

void LocalPathPlanner::LocalPathPlannerGoal() {
    double dist, local_theta;
    running_ = true;
    while (!f1 || (target_x == robot_x && target_y == robot_y) || !laser_ready || !costmap_ready || !global_path_ready) {
        RCLCPP_WARN(this->get_logger(), "Waiting ");
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }

    std::tie(dist, local_theta) = getDir();
    // std::cout<<"dist: "<<dist<<" local_theta: "<<local_theta<<std::endl;
    while (dist > thresh_dist) {
        if (!detect) {
            if (local_theta > thresh_ang) {
                speed_.linear.x = 0;
                speed_.angular.z = checkAngularLimitVelocity(local_theta);
            } else if (-thresh_ang > local_theta) {
                speed_.linear.x = 0;
                speed_.angular.z = checkAngularLimitVelocity(local_theta);
            } else {
                speed_.linear.x = checkLinearLimitVelocity(dist);
                speed_.angular.z = 0;
            }
        } else {
            RobotState robot_state(robot_x, robot_y, robot_yaw);
            Goal goal(target_x, target_y);

            vector<double> vel = dwa->computeVelocityCommands(robot_state, goal, dt);
            if (vel.size() < 2) {
                RCLCPP_ERROR(this->get_logger(), "Invalid velocity command size: %lu", vel.size());
                continue; // 또는 stopSpeed(); return;
            }
            speed_.linear.x = checkLinearLimitVelocity(vel[0]) * 1.5;
            speed_.angular.z = checkAngularLimitVelocity(vel[1]) * 1.5;
            RCLCPP_INFO(this->get_logger(), "linear: %.2f and angular: %.2f",vel[0], vel[1]);
        }
            //rate.sleep();
            cmd_pub_->publish(speed_);
            tie(dist, local_theta) = getDir();
            //RCLCPP_INFO(this->get_logger(), "dist: %.2f, theta: %.2f", dist, local_theta);
    }

    double dyaw = target_yaw - robot_yaw;

    while (1) {
        tie(dist, local_theta) = getDir();
        if (sin(dyaw) < 0 && cos(dyaw) < cos(thresh_ang)) {
            speed_.linear.x = 0;
            speed_.angular.z = checkAngularLimitVelocity(-abs(dyaw));
        } else if (sin(dyaw) > 0 && cos(dyaw) < cos(thresh_ang)) {
            speed_.linear.x = 0;
            speed_.angular.z = checkAngularLimitVelocity(abs(dyaw));
        } else {
            break;
        }
        cmd_pub_->publish(speed_);
        dyaw = target_yaw - robot_yaw;
    }

    stopSpeed();
    running_ = false;
    RCLCPP_INFO(this->get_logger(), "Goal reached");

}

void LocalPathPlanner::stopSpeed() {
    speed_.linear.x = 0;
    speed_.angular.z = 0;
    cmd_pub_->publish(speed_);
}

double LocalPathPlanner::constrain(double input, double low, double high) {
    if (input < low) input = low;
    else if (input > high) input = high;
    return input;
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
