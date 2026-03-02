/**
 *  visual_odometry.cpp
 *  Created on: November 30, 2025
 *  
 *  This file is part of the Autonomous Robot Localization 
 *  project.
 *  It receives images from the photographer and computes 
 *  the position of the robot using visual odometry.
 */
 
#include "rclcpp/rclcpp.hpp"
#include <opencv2/opencv.hpp>
#include <sensor_msgs/msg/image.hpp>
#include "geometry_msgs/msg/pose_stamped.hpp"
#include <std_msgs/msg/bool.hpp>

struct state_space_t
{
    double linear_x;
    double linear_y;
    double angular_z;
};

class VisualNode : public rclcpp::Node
{
public:
    VisualNode() : Node("visual_estimator_node")
    {
        rclcpp::QoS qos(3);
        qos.reliable();
        qos.durability_volatile();

        img_pub_ =
            create_publisher<sensor_msgs::msg::Image>("stream/image", 1);

        pose_pub_ =
            create_publisher<geometry_msgs::msg::PoseStamped>("odometry/position", 10);
        
        img_sub_ = 
            create_subscription<sensor_msgs::msg::Image>("camera/image", qos, std::bind(&VisualNode::imgCallback, this, std::placeholders::_1));
        
        stream_status_sub_ = 
            create_subscription<std_msgs::msg::Bool>("status/stream", qos, std::bind(&VisualNode::streamStatusCallback, this, std::placeholders::_1));


        stream_msg_.header.frame_id = "camera_frame";
        stream_msg_.encoding = "mono8";
        stream_status_active_ = false;
    }

private:

    void computeOpticalFlowFarneback(const cv::Mat& prev_frame, const cv::Mat& curr_frame, std::vector<cv::Point2f>& prev_pts, std::vector<cv::Point2f>& curr_pts);

    void computeOpticalFlowLK(const cv::Mat& prev_frame, const cv::Mat& curr_frame, std::vector<cv::Point2f>& prev_pts, std::vector<cv::Point2f>& curr_pts);

    void imgCallback(const sensor_msgs::msg::Image::SharedPtr msg);

    void streamStatusCallback(const std_msgs::msg::Bool::SharedPtr msg);

    void removeOutliers(std::vector<cv::Point2f>& prev_pts, std::vector<cv::Point2f>& curr_pts);
    
    cv::Mat new_frame_;
    sensor_msgs::msg::Image stream_msg_;
    bool stream_status_active_;

    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr img_pub_;
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pose_pub_;

    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr img_sub_;
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr stream_status_sub_;

};

void VisualNode::computeOpticalFlowFarneback(const cv::Mat& prev_frame, const cv::Mat& curr_frame, std::vector<cv::Point2f>& prev_pts, std::vector<cv::Point2f>& curr_pts)
{
    //pyr_scale: factor de escala entre pirámides (0 < pyr_scale < 1)
    static const float pyr_scale = 0.5;
    //levels: número de niveles en la pirámide
    static const int levels = 2;
    //winsize: tamaño de la ventana de búsqueda
    static const int winsize = 13;
    //iterations: número de iteraciones en cada nivel de la pirámide
    static const int iterations = 1;
    //poly_n: tamaño del vecindario para la aproximación polinómica
    static const int poly_n = 7;
    //poly_sigma: desviación estándar del filtro Gaussiano
    static const double poly_sigma = 1.2;
    
    if (prev_frame.empty()) 
    {
        std::cerr << "Previous frame is empty" << std::endl;
        return;
    }

    if (curr_frame.empty()) 
    {
        std::cerr << "Current frame is empty" << std::endl;
        return;
    }

    static cv::Mat flow_xy = cv::Mat::zeros(prev_frame.size(), CV_32FC2);
    cv::calcOpticalFlowFarneback(prev_frame, curr_frame, flow_xy,
                                 pyr_scale, levels, winsize, iterations,
                                poly_n, poly_sigma, cv::OPTFLOW_USE_INITIAL_FLOW );


    int step = 16;
    for (int y = 0; y < curr_frame.rows; y += step)
    {
        for (int x = 0; x < curr_frame.cols; x += step)
        {
            const cv::Point2f& fxy = flow_xy.at<cv::Point2f>(y, x);
            cv::Point p1(x, y);
            cv::Point p2(cvRound(x + fxy.x), cvRound(y + fxy.y));

            prev_pts.push_back(p1);
            curr_pts.push_back(p2);            
        }
    }
}

void VisualNode::computeOpticalFlowLK(const cv::Mat& prev_frame, const cv::Mat& curr_frame, std::vector<cv::Point2f>& prev_pts, std::vector<cv::Point2f>& curr_pts)
{
    // maxCorners: número máximo de esquinas a detectar
    static const int maxCorners = 100;
    // qualityLevel: calidad mínima de las esquinas (0 < qualityLevel < 1)
    static const double qualityLevel = 0.1;
    // minDistance: distancia mínima entre esquinas detectadas
    static const double minDistance = 10.0;

    static const cv::Size winSize(21, 21);
    static const int      maxLevel = 3;
    static const cv::TermCriteria termCrit(
        cv::TermCriteria::COUNT | cv::TermCriteria::EPS, 30, 0.01);
    
    if (prev_frame.empty()) 
    {
        std::cerr << "Previous frame is empty" << std::endl;
        return;
    }

    if (curr_frame.empty()) 
    {
        std::cerr << "Current frame is empty" << std::endl;
        return;
    }

    cv::goodFeaturesToTrack(prev_frame, prev_pts, maxCorners, qualityLevel, minDistance);

    if (prev_pts.empty())
        return;

    cv::cornerSubPix(prev_frame, prev_pts, cv::Size(5,5), cv::Size(-1,-1), termCrit);

    std::vector<uchar> status;
    std::vector<float> err;
    cv::calcOpticalFlowPyrLK(prev_frame, curr_frame, prev_pts, curr_pts, status, err, winSize, maxLevel, termCrit);

    size_t write = 0;
    for (size_t i = 0; i < status.size(); ++i)
    {
        if (status[i])
        {
            prev_pts[write] = prev_pts[i];
            curr_pts[write] = curr_pts[i];
            ++write;
        }
    }
    prev_pts.resize(write);
    curr_pts.resize(write);
}

static state_space_t computeVelocity(const std::vector<cv::Point2f>& prev_pts, const std::vector<cv::Point2f>& curr_pts)
{
    if (prev_pts.size() != curr_pts.size() || prev_pts.empty())
        return state_space_t{0.0, 0.0, 0.0};

    cv::Point2f mass_center_prev(0.0f, 0.0f);
    cv::Point2f mass_center_curr(0.0f, 0.0f);
    for (size_t i = 0; i < prev_pts.size(); ++i)
    {
        mass_center_prev += prev_pts[i];
        mass_center_curr += curr_pts[i];
    }
    mass_center_prev *= (1.0f / prev_pts.size());
    mass_center_curr *= (1.0f / curr_pts.size());
    cv::Point2f velocity = mass_center_curr - mass_center_prev;

#define MOV_THRESHOLD 0.01
    velocity.x = std::fabs(velocity.x) > MOV_THRESHOLD ? velocity.x : 0.0f;
    velocity.y = std::fabs(velocity.y) > MOV_THRESHOLD ? velocity.y : 0.0f;

    if (velocity.x == 0.0f && velocity.y == 0.0f)
        return state_space_t{0.0, 0.0, 0.0};

    double yaw_velocity = 0;
    std::vector<cv::Mat> vector_prev, vector_curr;
    for (size_t i = 0; i < prev_pts.size(); ++i) 
    {
        cv::Mat prev = (cv::Mat_<double>(2,1) << prev_pts[i].x - mass_center_prev.x, prev_pts[i].y - mass_center_prev.y);
        cv::Mat curr = (cv::Mat_<double>(2,1) << curr_pts[i].x - mass_center_curr.x, curr_pts[i].y - mass_center_curr.y);
        vector_prev.push_back(prev);
        vector_curr.push_back(curr);
    }
    cv::Mat R = cv::Mat::zeros(2, 2, CV_64F);
    for (size_t i = 0; i < vector_prev.size(); ++i) 
    {
        R += vector_curr[i] * vector_prev[i].t();
    }
    R /= static_cast<double>(vector_prev.size());
    cv::SVD svd(R);
    cv::Mat R_ortho = svd.u * svd.vt;
    yaw_velocity = atan2(R_ortho.at<double>(1,0), R_ortho.at<double>(0,0)) * 180.0 / CV_PI;

    state_space_t vel;
    vel.linear_x = velocity.x;
    vel.linear_y = velocity.y;
    vel.angular_z = yaw_velocity;
    return vel;
}

void VisualNode::imgCallback(const sensor_msgs::msg::Image::SharedPtr msg)
{
    new_frame_ = cv::Mat(msg->height, msg->width, CV_8UC1, const_cast<uint8_t*>(msg->data.data()), msg->step);

    static cv::Mat prev_frame = new_frame_.clone();
    std::vector<cv::Point2f> prev_pts, curr_pts;

    // computeOpticalFlowFarneback(prev_frame, new_frame_, prev_pts, curr_pts);
    computeOpticalFlowLK(prev_frame, new_frame_, prev_pts, curr_pts);
    prev_frame = new_frame_.clone();

    removeOutliers(prev_pts, curr_pts);

    /* Paint frame to stream */
    for (size_t i = 0; i < prev_pts.size(); ++i)
    {
        cv::arrowedLine(new_frame_, prev_pts[i], curr_pts[i], cv::Scalar(0, 0, 255), 2, cv::LINE_AA, 0, 0.3);
        cv::circle(new_frame_, curr_pts[i], 2, cv::Scalar(0, 255, 0), -1);
    }

    state_space_t vel = computeVelocity(prev_pts, curr_pts);
    static state_space_t position = {0.0, 0.0, 0.0};
    static rclcpp::Time prev_time = msg->header.stamp;
    double time_diff = msg->header.stamp.sec - prev_time.seconds() + (msg->header.stamp.nanosec - prev_time.nanoseconds()) * 1e-9;

    position.linear_x += vel.linear_x * time_diff;
    position.linear_y += vel.linear_y * time_diff;
    position.angular_z += vel.angular_z * time_diff;
    prev_time = msg->header.stamp;

    if (true)   // TODO: send only when map stream is active
    {
        geometry_msgs::msg::PoseStamped  pose_msg;
        pose_msg.header.frame_id = "map";
        pose_msg.header.stamp = msg->header.stamp;
        pose_msg.pose.position.x = position.linear_x;
        pose_msg.pose.position.y = position.linear_y;
        pose_msg.pose.orientation.z = position.angular_z;
        pose_pub_->publish(pose_msg);
    }

    if (stream_status_active_)
    {
        stream_msg_.header.stamp = msg->header.stamp;
        stream_msg_.height = new_frame_.rows;
        stream_msg_.width = new_frame_.cols;
        stream_msg_.step = new_frame_.cols;
        stream_msg_.data.assign(new_frame_.datastart, new_frame_.dataend);

        img_pub_->publish(stream_msg_);
    }
}

void VisualNode::streamStatusCallback(const std_msgs::msg::Bool::SharedPtr msg)
{
    stream_status_active_ = msg->data;
}

void VisualNode::removeOutliers(std::vector<cv::Point2f>& prev_pts, std::vector<cv::Point2f>& curr_pts)
{
    if (prev_pts.size() != curr_pts.size() || prev_pts.empty())
        return;

    const size_t n = prev_pts.size();
    std::vector<float> magnitudes(n);
    std::vector<float> angles(n);

    for (size_t i = 0; i < n; ++i)
    {
        const cv::Point2f diff = curr_pts[i] - prev_pts[i];
        magnitudes[i] = std::hypot(diff.x, diff.y);
        angles[i] = std::atan2(diff.y, diff.x) * 180.0 / CV_PI;
    }
    
    auto filterData = [&](std::vector<float>&values, float threshold)
    {
        const size_t m = values.size();

        float mean = 0.0f;
        for (float v : values)
            mean += v;
        mean /= static_cast<float>(m);

        float variance = 0.0f;
        for (float v : values)
            variance += (v - mean) * (v - mean);
        variance /= static_cast<float>(m);
        float std_dev = std::sqrt(variance);

        size_t write_idx = 0;
        for (size_t i = 0; i < values.size(); ++i)
        {
            if (std::abs(values[i] - mean) <= threshold * std_dev)
            {
                prev_pts[write_idx]  = prev_pts[i];
                curr_pts[write_idx]  = curr_pts[i];
                magnitudes[write_idx] = magnitudes[i];
                angles[write_idx]     = angles[i];
                values[write_idx]     = values[i];
                ++write_idx;
            }
        }
        prev_pts.resize(write_idx);
        curr_pts.resize(write_idx);
        magnitudes.resize(write_idx);
        angles.resize(write_idx);
        values.resize(write_idx);
    };

    filterData(magnitudes, 1.5f);
    filterData(angles, 1.0f);
}

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<VisualNode>());
    rclcpp::shutdown();

    return 0;
}
