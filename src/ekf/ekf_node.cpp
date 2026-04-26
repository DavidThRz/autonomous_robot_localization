/**
 *  ekf_node.cpp
 *  Created on: April 2026
 *
 *  This file is part of the Autonomous Robot Localization
 *  project.
 *
 *  Extended Kalman Filter — 2D planar motion.
 *  State: x = [x, y, θ, vx, vy, ωz]
 *
 *  ── Prediction (at IMU rate ~100 Hz) ──────────────────────
 *  Uses body-frame accelerations (ax, ay) and current velocity
 *  state to advance position and heading.
 *
 *  Process model:
 *    x'  = x  + (vx·cosθ − vy·sinθ) · dt
 *    y'  = y  + (vx·sinθ + vy·cosθ) · dt
 *    θ'  = θ  + ωz · dt
 *    vx' = vx + ax_body · dt          (IMU accel as control)
 *    vy' = vy + ay_body · dt          (IMU accel as control)
 *    ωz' = ωz                         (constant — corrected below)
 *
 *  ── IMU correction (at IMU rate) ──────────────────────────
 *  Corrects ωz with the high-quality gyroscope measurement.
 *  H = [0, 0, 0, 0, 0, 1]
 *
 *  ── VO correction (at camera rate, slower) ────────────────
 *  Corrects body-frame velocities [vx, vy, ωz] with the optical-
 *  flow twist from visual_odometry.cpp.
 *  H = diag(0,0,0,1,1,1) rows for vx, vy, ωz
 */

#include "ekf_node.hpp"
#include <cmath>

/* ── Constructor ────────────────────────────────────────────────────────── */

EkfNode::EkfNode() : Node("ekf_node")
{
    /* ── Declare tuning parameters ─────────────────────────────────────── */
    declare_parameter<double>("process_noise_position", 0.01);
    declare_parameter<double>("process_noise_yaw",      0.01);
    declare_parameter<double>("process_noise_velocity", 0.10);
    declare_parameter<double>("process_noise_omega",    0.05);
    declare_parameter<double>("max_dt",                 0.5);

    q_pos_   = get_parameter("process_noise_position").as_double();
    q_yaw_   = get_parameter("process_noise_yaw").as_double();
    q_vel_   = get_parameter("process_noise_velocity").as_double();
    q_omega_ = get_parameter("process_noise_omega").as_double();
    max_dt_  = get_parameter("max_dt").as_double();

    /* ── Build diagonal process noise matrix Q ─────────────────────────── */
    Q_.diagonal() << q_pos_, q_pos_, q_yaw_, q_vel_, q_vel_, q_omega_;

    /* ── ROS2 interfaces ────────────────────────────────────────────────── */
    rclcpp::QoS qos(10);

    imu_sub_ = create_subscription<sensor_msgs::msg::Imu>(
        "imu/data", qos,
        std::bind(&EkfNode::imuCallback, this, std::placeholders::_1));

    twist_sub_ = create_subscription<
        geometry_msgs::msg::TwistWithCovarianceStamped>(
        "odometry/visual", qos,
        std::bind(&EkfNode::twistCallback, this, std::placeholders::_1));

    pose_pub_ = create_publisher<geometry_msgs::msg::PoseStamped>(
        "ekf/pose", 10);

    RCLCPP_INFO(get_logger(),
        "EKF node started — fusing [imu/data] + [odometry/visual] "
        "→ [ekf/pose]");
    RCLCPP_INFO(get_logger(),
        "Process noise: pos=%.4f yaw=%.4f vel=%.4f omega=%.4f",
        q_pos_, q_yaw_, q_vel_, q_omega_);
}

/* ── ROS2 Callbacks ─────────────────────────────────────────────────────── */

void EkfNode::imuCallback(const sensor_msgs::msg::Imu::SharedPtr msg)
{
    const rclcpp::Time curr_stamp(msg->header.stamp);

    /* First message: initialise timestamp and return */
    if (!initialized_)
    {
        last_imu_stamp_ = curr_stamp;
        initialized_    = true;
        return;
    }

    const double dt = (curr_stamp - last_imu_stamp_).seconds();
    last_imu_stamp_ = curr_stamp;

    if (dt <= 0.0 || dt > max_dt_)
    {
        RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000,
            "IMU dt out of range (%.4f s) — skipping predict step", dt);
        return;
    }

    /* ── 1. Predict: advance state with IMU acceleration control input ─── */
    predict(msg->linear_acceleration.x,
            msg->linear_acceleration.y,
            dt);

    /* ── 2. Correct ωz with gyroscope measurement ───────────────────────── */
    const double gyro_z_cov = msg->angular_velocity_covariance[8];
    if (gyro_z_cov > 0.0)
    {
        updateGyro(msg->angular_velocity.z, gyro_z_cov);
    }

    /* ── 3. Publish fused pose ──────────────────────────────────────────── */
    publishPose(curr_stamp);
}

void EkfNode::twistCallback(
    const geometry_msgs::msg::TwistWithCovarianceStamped::SharedPtr msg)
{
    if (!initialized_)
        return;

    updateVo(msg->twist.twist.linear.x,
             msg->twist.twist.linear.y,
             msg->twist.twist.angular.z,
             msg->twist.covariance);
}

/* ── EKF Core ───────────────────────────────────────────────────────────── */

void EkfNode::predict(double ax_body, double ay_body, double dt)
{
    const double theta = x_[2];
    const double vx    = x_[3];
    const double vy    = x_[4];
    const double c     = std::cos(theta);
    const double s     = std::sin(theta);

    /* ── Non-linear state propagation ──────────────────────────────────── */
    x_[0]  = x_[0] + (vx * c - vy * s) * dt;
    x_[1]  = x_[1] + (vx * s + vy * c) * dt;
    x_[2]  = normalizeAngle(x_[2] + x_[5] * dt);
    x_[3]  = x_[3] + ax_body * dt;
    x_[4]  = x_[4] + ay_body * dt;
    /* x_[5] (ωz) constant in prediction; corrected in updateGyro() */

    /* ── Jacobian of f(x) w.r.t. x  (analytic, evaluated at pre-update x) */
    /*  F = I  +  ∂f/∂x · dt  (already including dt in the off-diagonal)   */
    Mat6 F = Mat6::Identity();

    /* ∂x'/∂θ  = -(vx·sinθ + vy·cosθ)·dt  */
    F(0, 2) = -(vx * s + vy * c) * dt;
    /* ∂x'/∂vx = cosθ·dt,  ∂x'/∂vy = -sinθ·dt */
    F(0, 3) =  c * dt;
    F(0, 4) = -s * dt;

    /* ∂y'/∂θ  =  (vx·cosθ − vy·sinθ)·dt  */
    F(1, 2) =  (vx * c - vy * s) * dt;
    /* ∂y'/∂vx = sinθ·dt,  ∂y'/∂vy = cosθ·dt */
    F(1, 3) =  s * dt;
    F(1, 4) =  c * dt;

    /* ∂θ'/∂ωz = dt */
    F(2, 5) =  dt;

    /* ── Covariance propagation: P = F·P·Fᵀ + Q ─────────────────────── */
    P_ = F * P_ * F.transpose() + Q_;
}

void EkfNode::updateGyro(double omega_z_meas, double gyro_z_cov)
{
    /* Measurement model: z = ωz,  H = [0,0,0,0,0,1]  (1×6) */
    const Mat1x6 H = (Mat1x6() << 0, 0, 0, 0, 0, 1).finished();

    const double innov = omega_z_meas - x_[5];           // y = z - H·x
    const double S     = (H * P_ * H.transpose())(0, 0)  // S = H·P·Hᵀ + R
                         + gyro_z_cov;

    const Vec6 K = P_ * H.transpose() / S;               // K = P·Hᵀ·S⁻¹

    x_ += K * innov;                                      // x = x + K·y
    x_[2] = normalizeAngle(x_[2]);

    P_ = (Mat6::Identity() - K * H) * P_;                // P = (I-K·H)·P
}

void EkfNode::updateVo(double vx, double vy, double omega_z,
                       const std::array<double, 36>& cov_array)
{
    /* Extract diagonal covariances from the 6×6 row-major twist covariance.
     * Order: [vx, vy, vz, wx, wy, wz]  →  diagonal indices: 0, 7, 35      */
    const double r_vx = (cov_array[0]  > 0.0) ? cov_array[0]  : 1.0;
    const double r_vy = (cov_array[7]  > 0.0) ? cov_array[7]  : 1.0;
    const double r_wz = (cov_array[35] > 0.0) ? cov_array[35] : 1.0;

    /* Measurement model:
     *   z = [vx, vy, ωz]ᵀ
     *   H = [0,0,0,1,0,0]   (3×6)
     *       [0,0,0,0,1,0]
     *       [0,0,0,0,0,1]
     */
    const Mat3x6 H = (Mat3x6() <<
        0, 0, 0, 1, 0, 0,
        0, 0, 0, 0, 1, 0,
        0, 0, 0, 0, 0, 1
    ).finished();

    const Vec3 z     = (Vec3() << vx, vy, omega_z).finished();
    const Vec3 innov = z - H * x_;    // y = z - H·x

    Mat3 R = Mat3::Zero();
    R(0, 0) = r_vx;
    R(1, 1) = r_vy;
    R(2, 2) = r_wz;

    const Mat3             S = H * P_ * H.transpose() + R;   // 3×3
    const Eigen::Matrix<double, 6, 3> K = P_ * H.transpose() * S.inverse();

    x_ += K * innov;
    x_[2] = normalizeAngle(x_[2]);

    P_ = (Mat6::Identity() - K * H) * P_;
}

/* ── Utilities ──────────────────────────────────────────────────────────── */

double EkfNode::normalizeAngle(double a)
{
    while (a >  M_PI) a -= 2.0 * M_PI;
    while (a < -M_PI) a += 2.0 * M_PI;
    return a;
}

void EkfNode::publishPose(const rclcpp::Time& stamp)
{
    geometry_msgs::msg::PoseStamped pose_msg;
    pose_msg.header.stamp    = stamp;
    pose_msg.header.frame_id = "odom";

    pose_msg.pose.position.x = x_[0];
    pose_msg.pose.position.y = x_[1];
    pose_msg.pose.position.z = 0.0;

    /* Quaternion for a pure yaw rotation: q = [0, 0, sin(θ/2), cos(θ/2)] */
    const double half_theta          = x_[2] * 0.5;
    pose_msg.pose.orientation.x      = 0.0;
    pose_msg.pose.orientation.y      = 0.0;
    pose_msg.pose.orientation.z      = std::sin(half_theta);
    pose_msg.pose.orientation.w      = std::cos(half_theta);

    pose_pub_->publish(pose_msg);
}
