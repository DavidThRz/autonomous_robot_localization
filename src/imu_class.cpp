
#include "autonomous_robot_localization/imu_class.hpp"

#include <chrono>
using namespace std::chrono_literals;

IMU_Node::IMU_Node() : Node("imu_node")
{    
    ADIS16460_driver imu_driver;
    if (!imu_driver.testImu()) 
    {
        std::cerr << "IMU not responding correctly. Exiting." << std::endl;
        return;
    }

    imu_publisher = 
        this->create_publisher<sensor_msgs::msg::Imu>("imu/data", 10);

    timer = this->create_wall_timer(
        100ms, 
        std::bind(&IMU_Node::publishIMUData, this)
    );

    RCLCPP_INFO(this->get_logger(), "Starting IMU node");
}

IMU_Node::~IMU_Node()
{
    RCLCPP_INFO(this->get_logger(), "Finalizing IMU node");
}

void IMU_Node::publishIMUData()
{
    double gyro_x, gyro_y, gyro_z;
    double accl_x, accl_y, accl_z;

    if (!imu_driver.getIMUData(gyro_x, gyro_y, gyro_z, accl_x, accl_y, accl_z)) 
    {
        RCLCPP_ERROR(this->get_logger(), "Error reading IMU data");
        return;
    }

    sensor_msgs::msg::Imu imu_msg;
    imu_msg.angular_velocity.x = gyro_x;
    imu_msg.angular_velocity.y = gyro_y;
    imu_msg.angular_velocity.z = gyro_z;
    imu_msg.linear_acceleration.x = accl_x;
    imu_msg.linear_acceleration.y = accl_y;
    imu_msg.linear_acceleration.z = accl_z;
    imu_publisher->publish(imu_msg);
}
