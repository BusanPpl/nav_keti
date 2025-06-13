#include "nav_keti/Move3.h"


int main(int argc, char * argv[])
{   
    
    rclcpp::init(argc, argv);
    double max_v = 1.0;             
    double max_omega = 1.4;         
    double v_resolution = 0.04;      
    double omega_resolution = 0.01;  
    double direction_gain_= 0.3;
    double speed_gain_= 0.7;
    double obstacle_gain_ = 0.4;
    double dt = 2.0; 
    double boundary = 50;
    double thresh_d = 0.25;
    double thresh_a = 0.1;
    auto node = std::make_shared<Move>(max_v, v_resolution, max_omega, omega_resolution, direction_gain_, speed_gain_, obstacle_gain_, dt, boundary, thresh_d, thresh_a);
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}