#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/nav_sat_fix.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <GeographicLib/MagneticModel.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <cmath>
#include <ctime>

class YawEstimator : public rclcpp::Node {
public:
    YawEstimator()
    : Node("yaw_gps"), magnetic_model_("wmm2025")
    {
        gps_sub_ = this->create_subscription<sensor_msgs::msg::NavSatFix>(
            "/gps/fix", 10, std::bind(&YawEstimator::gpsCallback, this, std::placeholders::_1));
        imu_sub_ = this->create_subscription<sensor_msgs::msg::Imu>(
            "/imu/data_raw", rclcpp::SensorDataQoS(), std::bind(&YawEstimator::imuCallback, this, std::placeholders::_1));
    }

private:
    void gpsCallback(sensor_msgs::msg::NavSatFix::SharedPtr msg) {
        gps_msg_ = *msg;
        gps_ready_ = true;
        computeYaw();
    }

    void imuCallback(sensor_msgs::msg::Imu::SharedPtr msg) {
        imu_msg_ = *msg;
        imu_ready_ = true;
        computeYaw();
    }

    void computeYaw() {
        if (!gps_ready_ || !imu_ready_) return;

        tf2::Quaternion quat(
            imu_msg_.orientation.x,
            imu_msg_.orientation.y,
            imu_msg_.orientation.z,
            imu_msg_.orientation.w
        );

        double roll, pitch, yaw;
        tf2::Matrix3x3(quat).getRPY(roll, pitch, yaw);

        double Bx, By, Bz;
        double lat = gps_msg_.latitude;
        double lon = gps_msg_.longitude;
        double alt = gps_msg_.altitude;
        int yr = getCurrentYear();

        magnetic_model_(yr, lat, lon, alt, Bx, By, Bz);

        double H, F, D, I;
        GeographicLib::MagneticModel::FieldComponents(Bx, By, Bz, H, F, D, I);

        double decl_rad = D * M_PI / 180.0;
        double corrected_yaw = yaw + decl_rad;

        if (corrected_yaw < 0) corrected_yaw += 2 * M_PI;
        if (corrected_yaw >= 2 * M_PI) corrected_yaw -= 2 * M_PI;

        RCLCPP_INFO(this->get_logger(), "Yaw: %.3f rad (%.1f deg)", corrected_yaw, corrected_yaw * 180.0 / M_PI);
    }

    int getCurrentYear() const {
        std::time_t now = std::time(nullptr);
        std::tm* tm_struct = std::localtime(&now);
        return std::max(2025, tm_struct->tm_year + 1900);
    }

    sensor_msgs::msg::NavSatFix gps_msg_;
    sensor_msgs::msg::Imu imu_msg_;
    bool gps_ready_ = false;
    bool imu_ready_ = false;

    rclcpp::Subscription<sensor_msgs::msg::NavSatFix>::SharedPtr gps_sub_;
    rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub_;
    GeographicLib::MagneticModel magnetic_model_;
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<YawEstimator>());
    rclcpp::shutdown();
    return 0;
}
