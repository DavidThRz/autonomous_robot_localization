
#include "autonomous_robot_localization/imu_class.hpp"

#include <chrono>
using namespace std::chrono_literals;

IMU_Node::IMU_Node() : Node("imu_node"), imu_driver_(nullptr)
{    
    this->declare_parameter<std::string>("spi_device", "/dev/spidev0.0");
    this->declare_parameter<int>("publish_rate_freq", 10);
    this->declare_parameter<std::string>("frame_id", "imu_link");

    const std::string spi_device = this->get_parameter("spi_device").as_string();
    imu_driver_ = new ADIS16460_driver(spi_device);
    if (!imu_driver_->testImu()) 
    {
        RCLCPP_FATAL(this->get_logger(), "IMU not responding correctly at startup. Exiting.");
        throw std::runtime_error("IMU initialization failed");
    }
    RCLCPP_INFO(this->get_logger(), "IMU detected and initialized successfully.");
    
    uint16_t publish_rate_freq_ = this->get_parameter("publish_rate_freq").as_int();

    imu_publisher = 
        this->create_publisher<sensor_msgs::msg::Imu>("imu/data", 10);

    timer = 
        this->create_wall_timer(
        std::chrono::milliseconds(1000 / publish_rate_freq_),
        std::bind(&IMU_Node::publishIMUData, this)
    );

    calibration_service = 
        this->create_service<std_srvs::srv::Trigger>(
        "imu_calibration",
        std::bind(&IMU_Node::calibrateIMU, this, std::placeholders::_1, std::placeholders::_2)
    );

    covariance_calibration_service = 
        this->create_service<std_srvs::srv::Trigger>(
        "imu_covariance_calibration",
        std::bind(&IMU_Node::calibrateIMUCovariances, this, std::placeholders::_1, std::placeholders::_2)
    );

    sum_gyro_x = sum_gyro_y = sum_gyro_z = 0.0;
    sum_accl_x = sum_accl_y = sum_accl_z = 0.0;
    num_samples_ = 0;

    gyro_x_covariance_ = gyro_y_covariance_ = gyro_z_covariance_ = 1.0e-6;
    accl_x_covariance_ = accl_y_covariance_ = accl_z_covariance_ = 1.0e-3;

    imu_state_ = ImuState::RUNNING;

    RCLCPP_INFO(this->get_logger(), "Starting IMU node");
}

IMU_Node::~IMU_Node()
{
    RCLCPP_INFO(this->get_logger(), "Finalizing IMU node");
    delete imu_driver_;
}

void IMU_Node::publishIMUData()
{
    double gyro_x, gyro_y, gyro_z;
    double accl_x, accl_y, accl_z;

    if (!imu_driver_->getIMUData(gyro_x, gyro_y, gyro_z, accl_x, accl_y, accl_z)) 
    {
        RCLCPP_ERROR(this->get_logger(), "Error reading IMU data");
        return;
    }

    sensor_msgs::msg::Imu imu_msg;
    imu_msg.header.stamp = this->get_clock()->now();
    imu_msg.header.frame_id = "imu_link";
    imu_msg.angular_velocity.x = gyro_x;
    imu_msg.angular_velocity.y = gyro_y;
    imu_msg.angular_velocity.z = gyro_z;
    imu_msg.linear_acceleration.x = accl_x;
    imu_msg.linear_acceleration.y = accl_y;
    imu_msg.linear_acceleration.z = accl_z;
    imu_msg.angular_velocity_covariance[0] = gyro_x_covariance_;
    imu_msg.angular_velocity_covariance[4] = gyro_y_covariance_;
    imu_msg.angular_velocity_covariance[8] = gyro_z_covariance_;
    imu_msg.linear_acceleration_covariance[0] = accl_x_covariance_;
    imu_msg.linear_acceleration_covariance[4] = accl_y_covariance_;
    imu_msg.linear_acceleration_covariance[8] = accl_z_covariance_;
    imu_publisher->publish(imu_msg);

    if (imu_state_ == ImuState::CALIBRATING)
    {
        this->computeCalibration(imu_msg);
    }
    else if (imu_state_ == ImuState::CALIBRATING_COVARIANCES)
    {
        this->computeCovariancesCalibration(imu_msg);
    }
}

void IMU_Node::calibrateIMU(const std::shared_ptr<std_srvs::srv::Trigger::Request> request, std::shared_ptr<std_srvs::srv::Trigger::Response> response)
{
    (void)request;
    std::cout << "Starting IMU calibration..." << std::endl;

    if (imu_state_ == ImuState::CALIBRATING || imu_state_ == ImuState::CALIBRATING_COVARIANCES) 
    {
        response->success = false;
        response->message = "Calibration already in progress";
        return;
    }

    imu_state_ = ImuState::CALIBRATING;

    sum_gyro_x = 0.0;
    sum_gyro_y = 0.0;
    sum_gyro_z = 0.0;
    sum_accl_x = 0.0;
    sum_accl_y = 0.0;
    sum_accl_z = 0.0;

    num_samples_ = 0;
    calibration_start_time_ = this->get_clock()->now();

    imu_driver_->getBiasOffsets(gyro_x_bias_, gyro_y_bias_, gyro_z_bias_, accl_x_bias_, accl_y_bias_, accl_z_bias_);

    response->success = true;
    response->message = "Starting calibration process";
}

void IMU_Node::calibrateIMUCovariances(const std::shared_ptr<std_srvs::srv::Trigger::Request> request, std::shared_ptr<std_srvs::srv::Trigger::Response> response)
{
    (void)request;
    std::cout << "Starting IMU covariance calibration..." << std::endl;

    if (imu_state_ == ImuState::CALIBRATING || imu_state_ == ImuState::CALIBRATING_COVARIANCES) 
    {
        response->success = false;
        response->message = "Calibration already in progress";
        return;
    }

    imu_state_ = ImuState::CALIBRATING_COVARIANCES;

    gyro_x_samples.clear();
    gyro_y_samples.clear();
    gyro_z_samples.clear();
    accl_x_samples.clear();
    accl_y_samples.clear();
    accl_z_samples.clear();

    num_samples_ = 0;
    calibration_start_time_ = this->get_clock()->now();

    response->success = true;
    response->message = "Starting covariance calibration process";
}

void IMU_Node::computeCalibration(const sensor_msgs::msg::Imu& imu_msg)
{
    num_samples_++;
    sum_gyro_x += imu_msg.angular_velocity.x;
    sum_gyro_y += imu_msg.angular_velocity.y;
    sum_gyro_z += imu_msg.angular_velocity.z;
    sum_accl_x += imu_msg.linear_acceleration.x;
    sum_accl_y += imu_msg.linear_acceleration.y;
    sum_accl_z += imu_msg.linear_acceleration.z;

    if ((this->get_clock()->now() - calibration_start_time_).seconds() < CALIBRATION_DURATION_SEC)
        return;

    sum_gyro_x = sum_gyro_x / num_samples_ - gyro_x_bias_;
    sum_gyro_y = sum_gyro_y / num_samples_ - gyro_y_bias_;
    sum_gyro_z = sum_gyro_z / num_samples_ - gyro_z_bias_;
    sum_accl_x = sum_accl_x / num_samples_ - accl_x_bias_;
    sum_accl_y = sum_accl_y / num_samples_ - accl_y_bias_;
    sum_accl_z = sum_accl_z / num_samples_ - accl_z_bias_;
    sum_accl_z += C_g; /* Remove gravity from Z axis */

    std::cout << " >> Calibration results (bias estimates):" << std::endl;
    std::cout << " >> Gyro bias (º/s): x=" << sum_gyro_x << ", y=" << sum_gyro_y << ", z=" << sum_gyro_z << std::endl;
    std::cout << " >> Accel bias (mg): x=" << sum_accl_x << ", y=" << sum_accl_y << ", z=" << sum_accl_z << std::endl;

    imu_driver_->setBiasOffsets(-sum_gyro_x, -sum_gyro_y, -sum_gyro_z, -sum_accl_x, -sum_accl_y, -sum_accl_z);

    std::cout << " >> Calibration completed" << std::endl;
    imu_state_ = ImuState::RUNNING;

}

void IMU_Node::computeCovariancesCalibration(const sensor_msgs::msg::Imu& imu_msg)
{
    num_samples_++;

    gyro_x_samples.push_back(imu_msg.angular_velocity.x);
    gyro_y_samples.push_back(imu_msg.angular_velocity.y);
    gyro_z_samples.push_back(imu_msg.angular_velocity.z);
    accl_x_samples.push_back(imu_msg.linear_acceleration.x);
    accl_y_samples.push_back(imu_msg.linear_acceleration.y);
    accl_z_samples.push_back(imu_msg.linear_acceleration.z);

    if ((this->get_clock()->now() - calibration_start_time_).seconds() < CALIBRATION_DURATION_SEC)
        return;

    double gyro_x_var = 0.0, gyro_y_var = 0.0, gyro_z_var = 0.0;
    double accl_x_var = 0.0, accl_y_var = 0.0, accl_z_var = 0.0;

    sum_gyro_x = std::accumulate(gyro_x_samples.begin(), gyro_x_samples.end(), 0.0);
    sum_gyro_y = std::accumulate(gyro_y_samples.begin(), gyro_y_samples.end(), 0.0);
    sum_gyro_z = std::accumulate(gyro_z_samples.begin(), gyro_z_samples.end(), 0.0);
    sum_accl_x = std::accumulate(accl_x_samples.begin(), accl_x_samples.end(), 0.0);
    sum_accl_y = std::accumulate(accl_y_samples.begin(), accl_y_samples.end(), 0.0);
    sum_accl_z = std::accumulate(accl_z_samples.begin(), accl_z_samples.end(), 0.0);
    double mean_gyro_x = sum_gyro_x / num_samples_;
    double mean_gyro_y = sum_gyro_y / num_samples_;
    double mean_gyro_z = sum_gyro_z / num_samples_;
    double mean_accl_x = sum_accl_x / num_samples_;
    double mean_accl_y = sum_accl_y / num_samples_;
    double mean_accl_z = sum_accl_z / num_samples_;

    for (size_t i = 0; i < gyro_x_samples.size(); ++i) 
    {
        gyro_x_var += std::pow(gyro_x_samples[i] - mean_gyro_x, 2);
        gyro_y_var += std::pow(gyro_y_samples[i] - mean_gyro_y, 2);
        gyro_z_var += std::pow(gyro_z_samples[i] - mean_gyro_z, 2);
        accl_x_var += std::pow(accl_x_samples[i] - mean_accl_x, 2);
        accl_y_var += std::pow(accl_y_samples[i] - mean_accl_y, 2);
        accl_z_var += std::pow(accl_z_samples[i] - mean_accl_z, 2);
    }

    gyro_x_var /= (num_samples_ - 1);
    gyro_y_var /= (num_samples_ - 1);
    gyro_z_var /= (num_samples_ - 1);
    accl_x_var /= (num_samples_ - 1);
    accl_y_var /= (num_samples_ - 1);
    accl_z_var /= (num_samples_ - 1);

    std::cout << " >> Covariance calibration results (variance estimates):" << std::endl;
    std::cout << " >> Gyro variance (º/s)^2: x=" << gyro_x_var << ", y=" << gyro_y_var << ", z=" << gyro_z_var << std::endl;
    std::cout << " >> Accel variance (mg)^2: x=" << accl_x_var << ", y=" << accl_y_var << ", z=" << accl_z_var << std::endl;

    gyro_x_covariance_ = gyro_x_var;
    gyro_y_covariance_ = gyro_y_var;
    gyro_z_covariance_ = gyro_z_var;
    accl_x_covariance_ = accl_x_var;
    accl_y_covariance_ = accl_y_var;
    accl_z_covariance_ = accl_z_var;

    std::cout << " >> Covariance calibration completed" << std::endl;
    imu_state_ = ImuState::RUNNING;
}