
#ifndef IMU_CLASS_HPP
#define IMU_CLASS_HPP

#include "autonomous_robot_localization/ADIS16460_driver.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "std_srvs/srv/trigger.hpp"

#define CALIBRATION_DURATION_SEC 15 /* Ensure high precission according to datasheet */
    

class IMU_Node : public rclcpp::Node
{
public:
    IMU_Node();
    ~IMU_Node();

    void publishIMUData();

    void calibrateIMU(const std::shared_ptr<std_srvs::srv::Trigger::Request> request, std::shared_ptr<std_srvs::srv::Trigger::Response> response);

private:
    rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr imu_publisher;
    rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr calibration_service;
    rclcpp::TimerBase::SharedPtr timer;

    ADIS16460_driver imu_driver_;

    bool calibrating_;
    bool calibrate_requested_;
    double sum_gyro_x, sum_gyro_y, sum_gyro_z;
    double sum_accl_x, sum_accl_y, sum_accl_z;
    uint16_t num_samples_;
};

#endif