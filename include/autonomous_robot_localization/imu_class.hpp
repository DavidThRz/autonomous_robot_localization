
#ifndef IMU_CLASS_HPP
#define IMU_CLASS_HPP

#include "autonomous_robot_localization/ADIS16460_driver.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/imu.hpp"

class IMU_Node : public rclcpp::Node
{
public:
    IMU_Node();
    ~IMU_Node();

    void publishIMUData();

private:
    rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr imu_publisher;
    rclcpp::TimerBase::SharedPtr timer;

    ADIS16460_driver imu_driver;
};

#endif