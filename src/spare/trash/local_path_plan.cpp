#include "nav_keti/local_path_plan.h"

using namespace std; 
using namespace rclcpp; 
using namespace std::placeholders;

LPP::LPP(double max_v, double v_resolution, double max_omega, double omega_resolution, 
        double direction_gain_, double speed_gain_, double obstacle_gain_, double dt_, 
        double boundary_, double thresh_d, double thresh_a) : 
    Node("LPP"), 
    dwa(max_v, v_resolution, max_omega, omega_resolution, direction_gain_, speed_gain_, obstacle_gain_, dt), 
    cmd_pub_(this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 1)), 
    scan_sub_(this->create_subscription<sensor_msgs::msg::LaserScan>("scan", 1, std::bind(&LPP::laserCallback, this, _1))), 
    gps_sub_(this->create_subscription<sensor_msgs::msg::NavSatFix>("gps", 1, std::bind(&LPP::gpsCallback, this, _1))), 
    target_gps_sub_(this->create_subscription<sensor_msgs::msg::NavSatFix>("target_gps", 1, std::bind(&LPP::targetGpsCallback, this, _1))), 
    costmap_sub_(this->create_subscription<nav2_msgs::msg::Costmap>("/costmap_raw", 1, std::bind(&LPP::costmapCallback, this, _1))), 
    imu_subscriber_(this->create_subscription<sensor_msgs::msg::Imu>("imu", 1, std::bind(&LPP::imuCallback, this, _1))), 
    global_path_sub_(this->create_subscription<nav_msgs::msg::Path>("planned_path", 1, std::bind(&LPP::globalPathCallback, this, _1))), 
    tf_buffer_(std::make_shared<tf2_ros::Buffer>(this->get_clock())), 
    tf_listener_(std::make_shared<tf2_ros::TransformListener>(*tf_buffer_)), 
    r(15), 
    last_time(this->get_clock()->now()), 
    boundary(boundary_), dt(dt_), 
    thresh_dist(thresh_d), thresh_ang(thresh_a), 
    trans_spd(max_v), ang_spd(max_omega), 
    f1(false), f2(false), detect(false), 
    robot_x(0), robot_y(0), robot_yaw(0), 
    vx(0), w(0), 
    goal_x(0.0), goal_y(0.0), goal_yaw(0) 
{ 
    RCLCPP_INFO(this->get_logger(), "LPP node started."); 
    thread_obj=thread(&LPP::LPPGoal, this); 
} 

LPP::~LPP() {
    stopSpeed(); 
    RCLCPP_INFO(this->get_logger(), "LPP node terminated."); 
    if (thread_obj.joinable()) { 
        thread_obj.join();  
    } 
}

#include <csignal>
void signalHandler(int) {
    shutdown(); 
}

void LPP::run(){
    std::signal(SIGINT, signalHandler); 
    spin(shared_from_this()); 
} 

void LPP::gpsCallback(const sensor_msgs::msg::NavSatFix::SharedPtr msg) 
{ 
    // std::lock_guard<std::mutex> lock(data_mutex);
    convertGPSToXY(msg->latitude, msg->longitude, robot_x, robot_y); 
    convertGPSToXY(msg->latitude, msg->longitude - 0.01, goal_x, goal_y); 
} 

void LPP::targetGpsCallback(const sensor_msgs::msg::NavSatFix::SharedPtr msg)
{   
    //std::lock_guard<std::mutex> lock(data_mutex);
    // target_latitude_ = msg->latitude;
    // target_longitude_ = msg->longitude;
    convertGPSToXY(msg->latitude, msg->longitude, goal_x, goal_y); 
}

std::tuple<double, double> LPP::getDir() {

    double dx = goal_x - robot_x;
    double dy = goal_y - robot_y;

    double local_x = dx * cos(robot_yaw) + dy * cos(M_PI / 2 - robot_yaw);
    double local_y = -dx * cos(M_PI / 2 - robot_yaw) + dy * cos(robot_yaw);
    double local_theta = atan2(local_y, local_x);
    double dist = sqrt(dx * dx + dy * dy);
    RCLCPP_INFO(this->get_logger(), "dist: %f, theta: %f", dist, local_theta);
    return std::make_tuple(dist, local_theta);
}

void LPP::laserCallback(const sensor_msgs::msg::LaserScan::SharedPtr msg) {
    // std::lock_guard<std::mutex> lock(data_mutex);
    RCLCPP_INFO(this->get_logger(), "laser");
    auto laser_data = msg->ranges;
    detect = true;
    for (const auto& laser : laser_data) {
        if (laser < boundary) {
            dt = (this->get_clock()->now() - last_time).seconds();
            detect = true;
            dwa.laserCallback(msg);
            last_time = this->get_clock()->now();
            break;
        }
    }
}

//IMU는 로봇의 방향을 알기 위해 사용
void LPP::imuCallback(const sensor_msgs::msg::Imu::SharedPtr msg) {
    std::lock_guard<std::mutex> lock(data_mutex);
    if ((this->get_clock()->now() - msg->header.stamp).seconds() > 0.1) {
        RCLCPP_WARN(this->get_logger(), "Ignoring old IMU data");
        return;
    }
    RCLCPP_INFO(this->get_logger(), "imu received");
    // 쿼터니언 데이터 가져오기
    tf2::Quaternion quat(
        msg->orientation.x,
        msg->orientation.y,
        msg->orientation.z,
        msg->orientation.w
    );

    double roll, pitch;
    tf2::Matrix3x3(quat).getRPY(roll, pitch, robot_yaw);
}

void LPP::costmapCallback(const nav2_msgs::msg::Costmap::SharedPtr msg) {
    if(!f2) {
        f2 = true;
        RCLCPP_INFO(this->get_logger(), "costmap received");
    }
    // std::lock_guard<std::mutex> lock(data_mutex);
    latest_costmap_msg_ = msg;
    dwa.costmapCallback(msg);
}

void LPP::globalPathCallback(const nav_msgs::msg::Path::SharedPtr msg) {
    if(!f1) {
        f1 = true;
        RCLCPP_INFO(this->get_logger(), "global path received");
    }
    // std::lock_guard<std::mutex> lock(data_mutex);
    dwa.updateGlobalPlan(msg->poses); 
}

void LPP::LPPGoal() {
    while(rclcpp::ok()) {

    double dist, local_theta;
    std::tie(dist, local_theta) = getDir();
    
    while (dist > thresh_dist && f1 && f2) {
        // std::lock_guard<std::mutex> lock(data_mutex);
        RobotState robot_state(robot_x, robot_y, robot_yaw);
        Goal goal(goal_x, goal_y);
        vector<double> vel = dwa.computeVelocityCommands(robot_state, goal, dt);

        speed_.linear.x = checkLinearLimitVelocity(vel[0]);
        speed_.angular.z = checkAngularLimitVelocity(vel[1]);
        
        r.sleep();
        cmd_pub_->publish(speed_);
        tie(dist, local_theta) = getDir();
        std::this_thread::sleep_for(std::chrono::milliseconds(50));  // CPU 양보

        
    }
    double dyaw = goal_yaw - robot_yaw;
    while (1) {
        // std::lock_guard<std::mutex> lock(data_mutex);
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
        dyaw = goal_yaw - robot_yaw;
    }

    stopSpeed();
    // RCLCPP_INFO(this->get_logger(), "Goal reached");
    // shutdown();
}
}

void LPP::stopSpeed() {
    speed_.linear.x = 0;
    speed_.angular.z = 0;
    cmd_pub_->publish(speed_);
}

double LPP::constrain(double input, double low, double high) {
    if (input < low) input = low;
    else if (input > high) input = high;
    return input;
}

double LPP::checkLinearLimitVelocity(double vel) {
    return constrain(vel, -trans_spd, trans_spd);
}

double LPP::checkAngularLimitVelocity(double vel) {
    return constrain(vel, -ang_spd, ang_spd);
}

void LPP::convertGPSToXY(double latitude, double longitude, double &x, double &y)
{

    double lat_rad = latitude * M_PI / 180.0;
    double lon_rad = longitude * M_PI / 180.0;

    // 위도, 경도를 미터 단위로 변환 (지구 반경 6378137m, 평면 근사)
    const double earth_radius = 6378137.0;  // 미터 단위 (WGS-84)

    // 위도 및 경도 차이를 거리(m)로 변환 (위도 차이는 항상 일정)
    x = lon_rad * earth_radius * cos(lat_rad);  // 경도는 위도에 따라 조정
    y = lat_rad * earth_radius;
}

