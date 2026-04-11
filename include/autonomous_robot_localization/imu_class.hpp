
#ifndef IMU_CLASS_HPP
#define IMU_CLASS_HPP

#include "autonomous_robot_localization/ADIS16460_driver.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "std_srvs/srv/trigger.hpp"

#include <yaml-cpp/yaml.h>
#include <filesystem>
#include <memory>



enum class ImuState
{
    RUNNING,
    CALIBRATING,
    CALIBRATING_COVARIANCES
};

class IMU_Node : public rclcpp::Node
{
public:
    IMU_Node();
    ~IMU_Node();

    /* Calibration duration in seconds — per ADIS16460 datasheet recommendation */
    static constexpr int kCalibrationDurationSec = 15;

    void publishIMUData();

    void calibrateIMU(const std::shared_ptr<std_srvs::srv::Trigger::Request> request, std::shared_ptr<std_srvs::srv::Trigger::Response> response);

    void calibrateIMUCovariances(const std::shared_ptr<std_srvs::srv::Trigger::Request> request, std::shared_ptr<std_srvs::srv::Trigger::Response> response);

private:

    void computeCalibration(const sensor_msgs::msg::Imu& imu_msg);

    void computeCovariancesCalibration(const sensor_msgs::msg::Imu& imu_msg);

    /* Covariance persistence methods */
    bool loadCovariancesFromFile();
    bool saveCovariancesToFile();

    rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr imu_publisher;
    rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr calibration_service;
    rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr covariance_calibration_service;
    rclcpp::TimerBase::SharedPtr timer;

    std::unique_ptr<ADIS16460_driver> imu_driver_;
    ImuState imu_state_;

    double sum_gyro_x, sum_gyro_y, sum_gyro_z;
    double sum_accl_x, sum_accl_y, sum_accl_z;

    std::vector<double> gyro_x_samples, gyro_y_samples, gyro_z_samples;
    std::vector<double> accl_x_samples, accl_y_samples, accl_z_samples;

    uint16_t num_samples_;
    rclcpp::Time calibration_start_time_;

    double gyro_x_covariance_, gyro_y_covariance_, gyro_z_covariance_;
    double accl_x_covariance_, accl_y_covariance_, accl_z_covariance_;

    double gyro_x_bias_, gyro_y_bias_, gyro_z_bias_;
    double accl_x_bias_, accl_y_bias_, accl_z_bias_;

    std::string frame_id_;
    std::string covariance_file_path_;
};

#endif