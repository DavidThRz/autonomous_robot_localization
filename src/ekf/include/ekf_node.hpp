/**
 *  ekf_node.hpp
 *  Created on: April 2026
 *
 *  This file is part of the Autonomous Robot Localization
 *  project.
 *  Extended Kalman Filter that fuses visual odometry
 *  (TwistWithCovarianceStamped) and IMU (sensor_msgs/Imu)
 *  to estimate the robot's 2D planar pose.
 *
 *  State vector: x = [x, y, θ, vx, vy, ωz]
 *  ─────────────────────────────────────────
 *    x, y  : world-frame position      (m)
 *    θ     : heading / yaw             (rad)
 *    vx,vy : body-frame linear velocity (m/s)
 *    ωz    : body-frame angular rate   (rad/s)
 */

#ifndef EKF_NODE_HPP
#define EKF_NODE_HPP

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "geometry_msgs/msg/twist_with_covariance_stamped.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"

#include <Eigen/Dense>
#include <array>

class EkfNode : public rclcpp::Node
{
public:
    EkfNode();

private:
    /* ── Fixed-size Eigen types (stack-allocated, SIMD-friendly) ────────── */
    using Vec6   = Eigen::Matrix<double, 6, 1>;
    using Mat6   = Eigen::Matrix<double, 6, 6>;
    using Vec3   = Eigen::Matrix<double, 3, 1>;
    using Mat3   = Eigen::Matrix<double, 3, 3>;
    using Mat3x6 = Eigen::Matrix<double, 3, 6>;
    using Mat1x6 = Eigen::Matrix<double, 1, 6>;

    /* ── EKF core ────────────────────────────────────────────────────────── */

    /**
     * Prediction step (called at IMU rate, ~100 Hz).
     * Advances state using kinematic model; ax_body / ay_body act as
     * control inputs for the velocity states.
     */
    void predict(double ax_body, double ay_body, double dt);

    /**
     * IMU measurement update — corrects ωz with the gyroscope.
     * Called at IMU rate right after predict().
     */
    void updateGyro(double omega_z_meas, double gyro_z_cov);

    /**
     * Visual-odometry measurement update — corrects [vx, vy, ωz]
     * with the twist produced by the optical-flow pipeline.
     * Called at camera rate (lower frequency).
     */
    void updateVo(double vx, double vy, double omega_z,
                  const std::array<double, 36>& cov_array);

    static double normalizeAngle(double a);

    /* ── ROS2 callbacks ──────────────────────────────────────────────────── */
    void imuCallback(const sensor_msgs::msg::Imu::SharedPtr msg);
    void twistCallback(
        const geometry_msgs::msg::TwistWithCovarianceStamped::SharedPtr msg);
    void publishPose(const rclcpp::Time& stamp);

    /* ── Filter state ────────────────────────────────────────────────────── */
    Vec6 x_{Vec6::Zero()};             // State estimate
    Mat6 P_{Mat6::Identity()};         // State covariance (P₀ = I)
    Mat6 Q_{Mat6::Zero()};             // Process noise covariance

    bool         initialized_{false};
    rclcpp::Time last_imu_stamp_;

    /* ── Tuning parameters (from ROS2 params / YAML) ─────────────────────── */
    double q_pos_;    // Process noise σ² for x, y
    double q_yaw_;    // Process noise σ² for θ
    double q_vel_;    // Process noise σ² for vx, vy
    double q_omega_;  // Process noise σ² for ωz
    double max_dt_;   // Discard IMU messages if dt exceeds this (s)

    /* ── ROS2 interfaces ─────────────────────────────────────────────────── */
    rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub_;
    rclcpp::Subscription<
        geometry_msgs::msg::TwistWithCovarianceStamped>::SharedPtr twist_sub_;
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pose_pub_;
};

#endif  // EKF_NODE_HPP
