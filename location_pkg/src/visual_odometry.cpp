
#include "rclcpp/rclcpp.hpp"
#include <opencv2/opencv.hpp>
#include <boost/filesystem.hpp>


bool captureFrame(cv::Mat& frame)
{
    static cv::VideoCapture cap(0, cv::CAP_V4L2 );

    if (!cap.isOpened()) 
    {
        std::cerr << "Camera could not be opened" << std::endl;
        return false;
    }

    cap >> frame;

    if (frame.empty()) 
    {
        std::cerr << "Failed to capture frame" << std::endl;
        return false;
    }

    cvtColor(frame, frame, cv::COLOR_BGR2GRAY);

    //TODO: Undistort image

    return true;
}


int main(int argc, char **argv) 
{
    rclcpp::init(argc, argv);

    auto node = rclcpp::Node::make_shared("visual_node");
    rclcpp::Rate rate(1);

    cv::Mat new_frame;

    while (rclcpp::ok()) 
    {
        captureFrame(new_frame);

        rclcpp::spin_some(node);
        rate.sleep();
    }

    rclcpp::shutdown();
}