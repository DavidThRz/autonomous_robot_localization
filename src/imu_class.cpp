
#include "autonomous_robot_localization/imu_class.hpp"

#include <chrono>
#include <fstream>
#include <iomanip>
#include <ctime>
using namespace std::chrono_literals;

IMU_Node::IMU_Node() : Node("imu_node")
{    
    this->declare_parameter<std::string>("spi_device", "/dev/spidev0.0");
    this->declare_parameter<int>("publish_rate_freq", 10);
    this->declare_parameter<std::string>("frame_id", "imu_link");
    this->declare_parameter<std::string>("covariance_file", "");

    frame_id_ = this->get_parameter("frame_id").as_string();

    const std::string spi_device = this->get_parameter("spi_device").as_string();
    imu_driver_ = std::make_unique<ADIS16460_driver>(spi_device);
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

    /* Default covariance values (conservative estimates) */
    gyro_x_covariance_ = gyro_y_covariance_ = gyro_z_covariance_ = 1.0e-6;
    accl_x_covariance_ = accl_y_covariance_ = accl_z_covariance_ = 1.0e-3;

    /* Resolve covariance file path */
    covariance_file_path_ = this->get_parameter("covariance_file").as_string();
    if (covariance_file_path_.empty())
    {
        const char* home = std::getenv("HOME");
        if (home)
        {
            covariance_file_path_ = std::string(home) + "/.ros/imu_covariance_calibration.yaml";
        }
        else
        {
            covariance_file_path_ = "/tmp/imu_covariance_calibration.yaml";
            RCLCPP_WARN(this->get_logger(), "HOME not set, using fallback path: %s", covariance_file_path_.c_str());
        }
    }
    RCLCPP_INFO(this->get_logger(), "Covariance file path: %s", covariance_file_path_.c_str());

    /* Attempt to load previously calibrated covariances from file */
    if (loadCovariancesFromFile())
    {
        RCLCPP_INFO(this->get_logger(), 
            "Loaded covariances from file — Gyro: [%.3e, %.3e, %.3e], Accl: [%.3e, %.3e, %.3e]",
            gyro_x_covariance_, gyro_y_covariance_, gyro_z_covariance_,
            accl_x_covariance_, accl_y_covariance_, accl_z_covariance_);
    }
    else
    {
        RCLCPP_WARN(this->get_logger(), 
            "No covariance calibration file found, using default values. "
            "Run the covariance calibration service to generate calibration data.");
    }

    imu_state_ = ImuState::RUNNING;

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

    if (!imu_driver_->getIMUData(gyro_x, gyro_y, gyro_z, accl_x, accl_y, accl_z)) 
    {
        RCLCPP_ERROR(this->get_logger(), "Error reading IMU data");
        return;
    }

    sensor_msgs::msg::Imu imu_msg;
    imu_msg.header.stamp = this->get_clock()->now();
    imu_msg.header.frame_id = frame_id_;
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
    RCLCPP_INFO(this->get_logger(), "Starting IMU calibration...");

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
    RCLCPP_INFO(this->get_logger(), "Starting IMU covariance calibration...");

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

    /* Pre-allocate sample vectors to avoid reallocations during data collection */
    const size_t expected_samples = static_cast<size_t>(
        this->get_parameter("publish_rate_freq").as_int() * kCalibrationDurationSec);
    gyro_x_samples.reserve(expected_samples);
    gyro_y_samples.reserve(expected_samples);
    gyro_z_samples.reserve(expected_samples);
    accl_x_samples.reserve(expected_samples);
    accl_y_samples.reserve(expected_samples);
    accl_z_samples.reserve(expected_samples);

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

    if ((this->get_clock()->now() - calibration_start_time_).seconds() < kCalibrationDurationSec)
        return;

    sum_gyro_x = sum_gyro_x / num_samples_ - gyro_x_bias_;
    sum_gyro_y = sum_gyro_y / num_samples_ - gyro_y_bias_;
    sum_gyro_z = sum_gyro_z / num_samples_ - gyro_z_bias_;
    sum_accl_x = sum_accl_x / num_samples_ - accl_x_bias_;
    sum_accl_y = sum_accl_y / num_samples_ - accl_y_bias_;
    sum_accl_z = sum_accl_z / num_samples_ - accl_z_bias_;
    sum_accl_z += C_g; /* Remove gravity from Z axis */

    RCLCPP_INFO(this->get_logger(), "Calibration results (bias estimates):");
    RCLCPP_INFO(this->get_logger(), "  Gyro bias (rad/s): x=%.6f, y=%.6f, z=%.6f", sum_gyro_x, sum_gyro_y, sum_gyro_z);
    RCLCPP_INFO(this->get_logger(), "  Accel bias (m/s2): x=%.6f, y=%.6f, z=%.6f", sum_accl_x, sum_accl_y, sum_accl_z);

    imu_driver_->setBiasOffsets(-sum_gyro_x, -sum_gyro_y, -sum_gyro_z, -sum_accl_x, -sum_accl_y, -sum_accl_z);

    RCLCPP_INFO(this->get_logger(), "Calibration completed");
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

    if ((this->get_clock()->now() - calibration_start_time_).seconds() < kCalibrationDurationSec)
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

    RCLCPP_INFO(this->get_logger(), "Covariance calibration results (variance estimates):");
    RCLCPP_INFO(this->get_logger(), "  Gyro variance (rad/s)^2: x=%.3e, y=%.3e, z=%.3e", gyro_x_var, gyro_y_var, gyro_z_var);
    RCLCPP_INFO(this->get_logger(), "  Accel variance (m/s2)^2: x=%.3e, y=%.3e, z=%.3e", accl_x_var, accl_y_var, accl_z_var);

    gyro_x_covariance_ = gyro_x_var;
    gyro_y_covariance_ = gyro_y_var;
    gyro_z_covariance_ = gyro_z_var;
    accl_x_covariance_ = accl_x_var;
    accl_y_covariance_ = accl_y_var;
    accl_z_covariance_ = accl_z_var;

    /* Persist covariances to file for reuse across restarts */
    if (saveCovariancesToFile())
    {
        RCLCPP_INFO(this->get_logger(), "Covariance calibration data saved to: %s", covariance_file_path_.c_str());
    }
    else
    {
        RCLCPP_ERROR(this->get_logger(), "Failed to save covariance calibration data to: %s", covariance_file_path_.c_str());
    }

    RCLCPP_INFO(this->get_logger(), "Covariance calibration completed");
    imu_state_ = ImuState::RUNNING;
}

bool IMU_Node::loadCovariancesFromFile()
{
    if (!std::filesystem::exists(covariance_file_path_))
    {
        return false;
    }

    try
    {
        YAML::Node config = YAML::LoadFile(covariance_file_path_);

        if (!config["calibration"])
        {
            RCLCPP_WARN(this->get_logger(), "Covariance file exists but missing 'calibration' key: %s", covariance_file_path_.c_str());
            return false;
        }

        YAML::Node calibration = config["calibration"];

        /* Load angular velocity covariances */
        if (calibration["angular_velocity_covariance"])
        {
            YAML::Node gyro_cov = calibration["angular_velocity_covariance"];
            gyro_x_covariance_ = gyro_cov["x"].as<double>();
            gyro_y_covariance_ = gyro_cov["y"].as<double>();
            gyro_z_covariance_ = gyro_cov["z"].as<double>();
        }
        else
        {
            RCLCPP_WARN(this->get_logger(), "Covariance file missing 'angular_velocity_covariance' section");
            return false;
        }

        /* Load linear acceleration covariances */
        if (calibration["linear_acceleration_covariance"])
        {
            YAML::Node accl_cov = calibration["linear_acceleration_covariance"];
            accl_x_covariance_ = accl_cov["x"].as<double>();
            accl_y_covariance_ = accl_cov["y"].as<double>();
            accl_z_covariance_ = accl_cov["z"].as<double>();
        }
        else
        {
            RCLCPP_WARN(this->get_logger(), "Covariance file missing 'linear_acceleration_covariance' section");
            return false;
        }

        /* Log calibration metadata if available */
        if (calibration["timestamp"])
        {
            RCLCPP_INFO(this->get_logger(), "Covariance calibration was performed at: %s", 
                calibration["timestamp"].as<std::string>().c_str());
        }
        if (calibration["num_samples"])
        {
            RCLCPP_INFO(this->get_logger(), "Calibration used %d samples", 
                calibration["num_samples"].as<int>());
        }

        return true;
    }
    catch (const YAML::ParserException& e)
    {
        RCLCPP_ERROR(this->get_logger(), "Failed to parse covariance file '%s': %s", 
            covariance_file_path_.c_str(), e.what());
        return false;
    }
    catch (const YAML::BadConversion& e)
    {
        RCLCPP_ERROR(this->get_logger(), "Invalid data type in covariance file '%s': %s", 
            covariance_file_path_.c_str(), e.what());
        return false;
    }
    catch (const std::exception& e)
    {
        RCLCPP_ERROR(this->get_logger(), "Unexpected error loading covariance file '%s': %s", 
            covariance_file_path_.c_str(), e.what());
        return false;
    }
}

bool IMU_Node::saveCovariancesToFile()
{
    try
    {
        /* Create parent directories if they don't exist */
        std::filesystem::path file_path(covariance_file_path_);
        if (file_path.has_parent_path())
        {
            std::filesystem::create_directories(file_path.parent_path());
        }

        /* Generate ISO 8601 timestamp */
        auto now = std::chrono::system_clock::now();
        auto time_t_now = std::chrono::system_clock::to_time_t(now);
        std::tm tm_now{};
        gmtime_r(&time_t_now, &tm_now);
        std::ostringstream timestamp_ss;
        timestamp_ss << std::put_time(&tm_now, "%Y-%m-%dT%H:%M:%SZ");

        /* Build YAML document using Emitter for guaranteed valid output */
        YAML::Emitter out;
        out << YAML::Comment("IMU Covariance Calibration Data - Auto-generated by imu_node");
        out << YAML::Comment("DO NOT EDIT MANUALLY unless you know what you are doing");
        out << YAML::BeginMap;
        out << YAML::Key << "calibration" << YAML::Value;
        out << YAML::BeginMap;

        out << YAML::Key << "timestamp" << YAML::Value << timestamp_ss.str();
        out << YAML::Key << "num_samples" << YAML::Value << static_cast<int>(num_samples_);
        out << YAML::Key << "duration_sec" << YAML::Value << static_cast<double>(kCalibrationDurationSec);

        out << YAML::Key << "angular_velocity_covariance" << YAML::Value;
        out << YAML::BeginMap;
        out << YAML::Key << "x" << YAML::Value << gyro_x_covariance_;
        out << YAML::Key << "y" << YAML::Value << gyro_y_covariance_;
        out << YAML::Key << "z" << YAML::Value << gyro_z_covariance_;
        out << YAML::EndMap;

        out << YAML::Key << "linear_acceleration_covariance" << YAML::Value;
        out << YAML::BeginMap;
        out << YAML::Key << "x" << YAML::Value << accl_x_covariance_;
        out << YAML::Key << "y" << YAML::Value << accl_y_covariance_;
        out << YAML::Key << "z" << YAML::Value << accl_z_covariance_;
        out << YAML::EndMap;

        out << YAML::EndMap;
        out << YAML::EndMap;

        /* Write to file atomically: write to temp file first, then rename */
        std::string temp_path = covariance_file_path_ + ".tmp";
        std::ofstream fout(temp_path);
        if (!fout.is_open())
        {
            RCLCPP_ERROR(this->get_logger(), "Cannot open file for writing: %s", temp_path.c_str());
            return false;
        }

        fout << out.c_str() << std::endl;
        fout.close();

        if (fout.fail())
        {
            RCLCPP_ERROR(this->get_logger(), "Error writing to file: %s", temp_path.c_str());
            std::filesystem::remove(temp_path);
            return false;
        }

        /* Atomic rename to prevent corruption if interrupted mid-write */
        std::filesystem::rename(temp_path, covariance_file_path_);

        return true;
    }
    catch (const std::exception& e)
    {
        RCLCPP_ERROR(this->get_logger(), "Failed to save covariance file '%s': %s", 
            covariance_file_path_.c_str(), e.what());
        return false;
    }
}