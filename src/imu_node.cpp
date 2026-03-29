
#include "autonomous_robot_localization/imu_class.hpp"

#include <chrono>
using namespace std::chrono_literals;

int main(int argc, char** argv) 
{
    rclcpp::init(argc, argv);

    auto imu_node = std::make_shared<IMU_Node>();
    // imu_node.node_spin();
    rclcpp::spin(imu_node);

    rclcpp::shutdown();
    return 0;
}