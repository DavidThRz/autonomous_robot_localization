
#include "autonomous_robot_localization/ADIS16460_driver.hpp"

#include <chrono>
using namespace std::chrono_literals;

int main(int argc, char** argv) 
{
    rclcpp::init(argc, argv);
    auto node = rclcpp::Node::make_shared("imu_node");

    rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr imu_publisher = 
        node->create_publisher<sensor_msgs::msg::Imu>("imu/data", 10);

    ADIS16460_driver imu_driver(imu_publisher);
    if (!imu_driver.testImu()) 
    {
        std::cerr << "IMU not responding correctly. Exiting." << std::endl;
        return -1;
    }

    auto timer = node->create_wall_timer(
        100ms, 
        std::bind(&ADIS16460_driver::readIMUData, &imu_driver)
    );

    RCLCPP_INFO(node->get_logger(), "Nodo IMU iniciado");

    rclcpp::spin(node);
    rclcpp::shutdown();

    return 0;
}