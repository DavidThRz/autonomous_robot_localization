
#include "rclcpp/rclcpp.hpp"
#include "autonomous_robot_localization/ADIS16460_driver.hpp"

int main(int argc, char** argv) 
{
    rclcpp::init(argc, argv);

    ADIS16460_driver imu_driver;

    bool test_result = imu_driver.testImu();
    if (test_result)
        std::cout << "IMU test passed successfully." << std::endl;
    else
        std::cerr << "IMU test failed." << std::endl;

    while(rclcpp::ok())
    {
        double accel_x, accel_y, accel_z;
        double gyro_x, gyro_y, gyro_z;
        if (!imu_driver.testAcclData(accel_x, accel_y, accel_z) ||
            !imu_driver.testGyroData(gyro_x, gyro_y, gyro_z)) 
        {
            std::cerr << "Failed to read IMU data." << std::endl;
            break;
        }

        rclcpp::sleep_for(std::chrono::milliseconds(100));
    }

    return 0;
}