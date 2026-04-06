
#include "autonomous_robot_localization/imu_class.hpp"

#include <chrono>
using namespace std::chrono_literals;

IMU_Node::IMU_Node() : Node("imu_node")
{    
    if (!imu_driver_.testImu()) 
    {
        RCLCPP_FATAL(this->get_logger(), "IMU not responding correctly at startup. Exiting.");
        throw std::runtime_error("IMU initialization failed");
    }
    RCLCPP_INFO(this->get_logger(), "IMU detected and initialized successfully.");

    imu_publisher = 
        this->create_publisher<sensor_msgs::msg::Imu>("imu/data", 10);

    timer = 
        this->create_wall_timer(
        100ms, 
        std::bind(&IMU_Node::publishIMUData, this)
    );

    calibration_service = 
        this->create_service<std_srvs::srv::Trigger>(
        "calibrate_imu",
        std::bind(&IMU_Node::calibrateIMU, this, std::placeholders::_1, std::placeholders::_2)
    );

    calibrating_ = false;
    calibrate_requested_ = false;
    sum_gyro_x = sum_gyro_y = sum_gyro_z = 0.0;
    sum_accl_x = sum_accl_y = sum_accl_z = 0.0;
    num_samples_ = 0;

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

    if (!imu_driver_.getIMUData(gyro_x, gyro_y, gyro_z, accl_x, accl_y, accl_z)) 
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

    /* Check calibration */
    static auto start_time = std::chrono::steady_clock::now();
    if (!calibrating_ && calibrate_requested_) 
    {
        calibrating_ = true;
        calibrate_requested_ = false;
        start_time = std::chrono::steady_clock::now();
        num_samples_ = 0;
        RCLCPP_INFO(this->get_logger(), "Calibration started, collecting data...");

        imu_driver_.resetBiasOffsets();
        return; /* Start calibrating with offsets reset */
    }

    if (calibrating_)
    {
        num_samples_++;
        sum_gyro_x += gyro_x;
        sum_gyro_y += gyro_y;
        sum_gyro_z += gyro_z;
        sum_accl_x += accl_x;
        sum_accl_y += accl_y;
        sum_accl_z += accl_z;

        if (std::chrono::steady_clock::now() - start_time > CALIBRATION_DURATION_SEC * 1s) 
        {
            sum_gyro_x /= num_samples_;
            sum_gyro_y /= num_samples_;
            sum_gyro_z /= num_samples_;
            sum_accl_x /= num_samples_;
            sum_accl_y /= num_samples_;
            sum_accl_z /= num_samples_;
            sum_accl_z += C_g; /* Remove gravity from Z axis */

            std::cout << " >> Calibration results (bias estimates):" << std::endl;
            std::cout << " >> Gyro bias (º/s): x=" << sum_gyro_x << ", y=" << sum_gyro_y << ", z=" << sum_gyro_z << std::endl;
            std::cout << " >> Accel bias (mg): x=" << sum_accl_x << ", y=" << sum_accl_y << ", z=" << sum_accl_z << std::endl;

            imu_driver_.setBiasOffsets(-sum_gyro_x, -sum_gyro_y, -sum_gyro_z, -sum_accl_x, -sum_accl_y, -sum_accl_z);

            std::cout << " >> Calibration completed" << std::endl;
            calibrating_ = false;
        }
    }
}

void IMU_Node::calibrateIMU(const std::shared_ptr<std_srvs::srv::Trigger::Request> request, std::shared_ptr<std_srvs::srv::Trigger::Response> response)
{
    (void)request;
    std::cout << "Starting IMU calibration..." << std::endl;

    if (calibrating_) 
    {
        response->success = false;
        response->message = "Calibration already in progress";
        return;
    }

    if (calibrate_requested_) 
    {
        response->success = false;
        response->message = "Calibration already requested, waiting for completion";
        return;
    }

    calibrate_requested_ = true;

    sum_gyro_x = 0.0;
    sum_gyro_y = 0.0;
    sum_gyro_z = 0.0;
    sum_accl_x = 0.0;
    sum_accl_y = 0.0;
    sum_accl_z = 0.0;

    num_samples_ = 0;

    response->success = true;
    response->message = "Starting calibration process";
}
