#include "nav_keti/Move3.h"

using namespace std;
using namespace rclcpp;
using namespace std::placeholders;

Move::Move(double max_v, double v_resolution, double max_omega, double omega_resolution, 
           double direction_gain_, double speed_gain_, double obstacle_gain_, double dt_, 
           double boundary_, double thresh_d, double thresh_a) 
    : Node("move"), 
    dwa(max_v, v_resolution, max_omega, omega_resolution, direction_gain_, speed_gain_, obstacle_gain_, dt_), 
    boundary(boundary_), dt(dt_), 
    thresh_dist(thresh_d), thresh_ang(thresh_a), 
    trans_spd(max_v), ang_spd(max_omega), 
    f1(false), f2(false), detect(false), 
    x(0), y(0), yaw(0), vx(0), w(0), 
    last_time(this->get_clock()->now()), 
    rate(10), 
    gps_sub_(this->create_subscription<sensor_msgs::msg::NavSatFix>("gps", rclcpp::SensorDataQoS(), std::bind(&Move::gpsCallback, this, _1))), 
    target_gps_sub_(this->create_subscription<sensor_msgs::msg::NavSatFix>("target_gps", 1, std::bind(&Move::targetGpsCallback, this, _1))), 
    imu_subscriber_(this->create_subscription<sensor_msgs::msg::Imu>("imu", rclcpp::SensorDataQoS(), std::bind(&Move::imuCallback, this, _1))), 
    tf_buffer_(std::make_shared<tf2_ros::Buffer>(this->get_clock())), 
    tf_listener_(std::make_shared<tf2_ros::TransformListener>(*tf_buffer_))
    {
        this_thread::sleep_for(2s);
        scan_sub_=this->create_subscription<sensor_msgs::msg::LaserScan>("scan", 1, std::bind(&Move::laserCallback, this, _1));
        cmd_pub_ = this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 1);
        this->declare_parameter<string>("target_frame", "map");
        this->declare_parameter<string>("source_frame", "base_footprint");
        this->declare_parameter<vector<double>>("goal_pose", {4.0, -1.0, 0.0, 0.0, 0.0, 1.0});
        thread_obj=thread(&Move::moveGoal, this);
    }

Move::~Move() {
    stopSpeed();
    RCLCPP_INFO(this->get_logger(), "Move node terminated.");
    if (thread_obj.joinable()) {
        thread_obj.join();  
    }
}

void Move::gpsCallback(const sensor_msgs::msg::NavSatFix::SharedPtr msg) 
{
    if(!f1) {
        f1 = true;
        convertGPSToXY(msg->latitude, msg->longitude + 0.01, goal_x, goal_y); 
    }
    // std::lock_guard<std::mutex> lock(data_mutex);
    convertGPSToXY(msg->latitude, msg->longitude, x, y); 

}

void Move::targetGpsCallback(const sensor_msgs::msg::NavSatFix::SharedPtr msg)
{   
    //std::lock_guard<std::mutex> lock(data_mutex);
    // target_latitude_ = msg->latitude;
    // target_longitude_ = msg->longitude;
    convertGPSToXY(msg->latitude, msg->longitude, goal_x, goal_y); 
}

void Move::convertGPSToXY(double latitude, double longitude, double &x, double &y)
{
    double lat_rad = latitude * M_PI / 180.0;
    double lon_rad = longitude * M_PI / 180.0;

    // 위도, 경도를 미터 단위로 변환 (지구 반경 6378137m, 평면 근사)
    const double earth_radius = 6378137.0;  // 미터 단위 (WGS-84)

    // 위도 및 경도 차이를 거리(m)로 변환 (위도 차이는 항상 일정)
    x = lon_rad * earth_radius * cos(lat_rad);  // 경도는 위도에 따라 조정
    y = lat_rad * earth_radius;
}

std::tuple<double, double> Move::getDir() { 

    double dx = goal_x - x; 
    double dy = goal_y - y; 
    double local_x = dx * cos(yaw) + dy * cos(M_PI / 2 - yaw); 
    double local_y = -dx * cos(M_PI / 2 - yaw) + dy * cos(yaw); 
    double local_theta = atan2(local_y, local_x); 
    double dist = sqrt(dx * dx + dy * dy); 
    //RCLCPP_INFO(this->get_logger(), "dist: %.2f, theta: %.2f", dist, local_theta);
    return make_tuple(dist, local_theta); 
} 

void Move::laserCallback(const sensor_msgs::msg::LaserScan::SharedPtr msg) {
    
    auto laser_data = msg->ranges;
    detect = true;
    for (const auto& laser : laser_data) {
        if (laser < boundary) {
            // dt = (this->get_clock()->now() - last_time).seconds();
            // RCLCPP_INFO(this->get_logger(), "%.2f",dt);
            detect = true;
            dwa.laserCallback(msg);
            last_time = this->get_clock()->now();
            break;
        }
    }
}

void Move::imuCallback(const sensor_msgs::msg::Imu::SharedPtr msg) {

    tf2::Quaternion quat(
        msg->orientation.x,
        msg->orientation.y,
        msg->orientation.z,
        msg->orientation.w
    );

    double roll, pitch;
    tf2::Matrix3x3(quat).getRPY(roll, pitch, yaw);
    if (yaw < 0) {
        yaw += 2 * M_PI;
    }
}

void Move::moveGoal() {
    double dist, local_theta;
    
    while(!f1)
    {
        cout<<" f1: "<<f1<<endl;
        std::tie(dist, local_theta) = getDir();
    }

  std::tie(dist, local_theta) = getDir();
  
    while (dist > thresh_dist&&f1) {
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
            RobotState robot_state(x, y, yaw);
            Goal goal(goal_x, goal_y);
            
            vector<double> vel = dwa.computeVelocityCommands(robot_state, goal, dt);

            speed_.linear.x = checkLinearLimitVelocity(vel[0]);
            speed_.angular.z = checkAngularLimitVelocity(vel[1]);
            //RCLCPP_INFO(this->get_logger(), "linear: %.2f and angular: %.2f",vel[0], vel[1]);
        }
            //rate.sleep();
            //cmd_pub_->publish(speed_);
            tie(dist, local_theta) = getDir();
    }
    double dyaw = goal_yaw - yaw;
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
        dyaw = goal_yaw - yaw;
    }

    stopSpeed();
    RCLCPP_INFO(this->get_logger(), "Goal reached");
    shutdown();
}

void Move::stopSpeed() {
    speed_.linear.x = 0;
    speed_.angular.z = 0;
    cmd_pub_->publish(speed_);
}

double Move::constrain(double input, double low, double high) {
    if (input < low) input = low;
    else if (input > high) input = high;
    return input;
}

double Move::checkLinearLimitVelocity(double vel) {
    return constrain(vel, -trans_spd, trans_spd);
}

double Move::checkAngularLimitVelocity(double vel) {
    return constrain(vel, -ang_spd, ang_spd);
}
