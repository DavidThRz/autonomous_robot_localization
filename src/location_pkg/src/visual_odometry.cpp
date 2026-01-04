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

class Odometer : public rclcpp::Node
{
public:
    Odometer() : Node("visual_estimator_node")
    {
        img_sub_ = 
            create_subscription<sensor_msgs::msg::Image>("camera/image", rclcpp::SensorDataQoS(), std::bind(&Odometer::imgCallback, this, std::placeholders::_1));

        img_pub_ =
            create_publisher<sensor_msgs::msg::Image>("stream/image", 1);

        stream_msg_.header.frame_id = "camera_frame";
        stream_msg_.encoding = "mono8";
    }

private:

    void imgCallback(const sensor_msgs::msg::Image::SharedPtr msg);
    
    cv::Mat new_frame_;
    sensor_msgs::msg::Image stream_msg_;

    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr img_sub_;
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr img_pub_;
};

void computeOpticalFlowFarneback(const cv::Mat& prev_frame, const cv::Mat& curr_frame, cv::Mat& flow)
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


    flow = curr_frame.clone();

    int step = 16;
    for (int y = 0; y < flow.rows; y += step)
    {
        for (int x = 0; x < flow.cols; x += step)
        {
            const cv::Point2f& fxy = flow_xy.at<cv::Point2f>(y, x);
            cv::Point p1(x, y);
            cv::Point p2(cvRound(x + fxy.x), cvRound(y + fxy.y));

            // Dibujar flecha
            cv::arrowedLine(flow, p1, p2, cv::Scalar(0, 255, 0), 1, cv::LINE_AA, 0, 0.3);
        }
    }
}

void computeOpticalFlowLK(const cv::Mat& prev_frame, const cv::Mat& curr_frame, cv::Mat& flow)
{
    // maxCorners: número máximo de esquinas a detectar
    static const int maxCorners = 100;
    // qualityLevel: calidad mínima de las esquinas (0 < qualityLevel < 1)
    static const double qualityLevel = 0.1;
    // minDistance: distancia mínima entre esquinas detectadas
    static const double minDistance = 10.0;

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

    std::vector<cv::Point2f> prev_pts;
    cv::goodFeaturesToTrack(prev_frame, prev_pts, maxCorners, qualityLevel, minDistance);

    if (prev_pts.empty())
        return;

    std::vector<cv::Point2f> curr_pts;
    std::vector<uchar> status;
    std::vector<float> err;
    cv::calcOpticalFlowPyrLK(prev_frame, curr_frame, prev_pts, curr_pts, status, err);

    flow = curr_frame.clone();

    for (size_t i = 0; i < prev_pts.size(); ++i)
    {
        if (status[i])
        {
            cv::arrowedLine(flow, prev_pts[i], curr_pts[i], cv::Scalar(0, 0, 255), 2, cv::LINE_AA, 0, 0.3);
            cv::circle(flow, curr_pts[i], 2, cv::Scalar(0, 255, 0), -1);
        }
    }
}

void Odometer::imgCallback(const sensor_msgs::msg::Image::SharedPtr msg)
{
    new_frame_ = cv::Mat(msg->height, msg->width, CV_8UC1, const_cast<uint8_t*>(msg->data.data()), msg->step);

    static cv::Mat prev_frame = new_frame_.clone();
    static cv::Mat stream_frame;

    // computeOpticalFlowFarneback(prev_frame, new_frame_, stream_frame);
    computeOpticalFlowLK(prev_frame, new_frame_, stream_frame);
    prev_frame = new_frame_.clone();

    stream_msg_.header.stamp = this->now();
    stream_msg_.height = stream_frame.rows;
    stream_msg_.width = stream_frame.cols;
    stream_msg_.step = stream_frame.cols;
    stream_msg_.data.assign(stream_frame.datastart, stream_frame.dataend);

    img_pub_->publish(stream_msg_);
}

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<Odometer>());
    rclcpp::shutdown();

    return 0;
}
