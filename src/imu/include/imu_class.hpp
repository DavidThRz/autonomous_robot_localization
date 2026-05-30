
#ifndef IMU_CLASS_HPP
#define IMU_CLASS_HPP

#include "ADIS16460_driver.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "std_srvs/srv/trigger.hpp"

#include <yaml-cpp/yaml.h>
#include <filesystem>
#include <memory>
#include <array>
#include <cmath>



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

private:

    /* Calibration duration in seconds — per ADIS16460 datasheet recommendation */
    static constexpr int kCalibrationDurationSec = 15;

    void publishIMUData();

    void calibrateIMU(const std::shared_ptr<std_srvs::srv::Trigger::Request> request, std::shared_ptr<std_srvs::srv::Trigger::Response> response);

    void calibrateIMUCovariances(const std::shared_ptr<std_srvs::srv::Trigger::Request> request, std::shared_ptr<std_srvs::srv::Trigger::Response> response);

    void computeCalibration(const sensor_msgs::msg::Imu& imu_msg);

    void computeCovariancesCalibration(const sensor_msgs::msg::Imu& imu_msg);

    /* Tilt compensation: rotates acceleration vector to horizontal plane */
    void applyTiltCompensation(double& accl_x, double& accl_y, double& accl_z);

    /* Covariance persistence methods */
    bool loadCovariancesFromFile();
    bool saveCovariancesToFile();

    /* Tilt calibration persistence methods */
    bool loadTiltFromFile();
    bool saveTiltToFile();

    rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr imu_publisher;
    rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr calibration_service;
    rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr covariance_calibration_service;
    rclcpp::TimerBase::SharedPtr timer;

    std::unique_ptr<ADIS16460_driver> imu_driver_;
    ImuState imu_state_;

    double sum_accl_x, sum_accl_y, sum_accl_z;

    /* Welford's online algorithm running statistics */
    double mean_gyro_x_, mean_gyro_y_, mean_gyro_z_;
    double mean_accl_x_, mean_accl_y_, mean_accl_z_;
    double m2_gyro_x_, m2_gyro_y_, m2_gyro_z_;
    double m2_accl_x_, m2_accl_y_, m2_accl_z_;

    uint16_t num_samples_;
    rclcpp::Time calibration_start_time_;

    double gyro_x_covariance_, gyro_y_covariance_, gyro_z_covariance_;
    double accl_x_covariance_, accl_y_covariance_, accl_z_covariance_;

    /* Tilt calibration state */
    double roll_rad_{0.0};
    double pitch_rad_{0.0};
    std::array<double, 9> tilt_rotation_matrix_{};  /* 3x3 row-major: R = Ry(pitch) · Rx(roll) */
    bool tilt_calibrated_{false};

    std::string frame_id_;
    std::string covariance_file_path_;
    std::string tilt_calibration_file_path_;
};

#endif