/**
 *  ekf_main.cpp
 *  Created on: April 2026
 *
 *  Entry point for the EKF sensor-fusion node.
 */

#include "ekf_node.hpp"

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<EkfNode>());
    rclcpp::shutdown();
    return 0;
}
