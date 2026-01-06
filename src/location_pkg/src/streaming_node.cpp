/**
 *  visual_odometry.cpp
 *  Created on: January 3, 2026
 *  
 *  This file is part of the Autonomous Robot Localization 
 *  project.
 *  It receives images to be sent to a stream and be visualized by the user.
 */
 
#include "rclcpp/rclcpp.hpp"
#include <opencv2/opencv.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <std_msgs/msg/bool.hpp>


#include <httplib.h>
#include <mutex>

using std::placeholders::_1;
using std::placeholders::_2;

class StreamingNode : public rclcpp::Node
{
public:
    
    StreamingNode() : Node("streaming_node"), stop_flag_(false), new_frame_(false)
    {
rclcpp::QoS qos(3);
        qos.reliable();
        qos.durability_volatile();

        image_sub_ = create_subscription<sensor_msgs::msg::Image>("stream/image", qos, std::bind(&StreamingNode::imgCallback, this, _1));

        stream_status_pub_ = create_publisher<std_msgs::msg::Bool>("status/stream", 1);

        server_thread_ = std::thread(&StreamingNode::streamServer, this);

        RCLCPP_INFO(this->get_logger(), "Streaming node started");
    }

    ~StreamingNode()
    {
        stop_flag_ = true;
        if (server_thread_.joinable())
            server_thread_.join();
    }

private:

    void publishStreamStatus(bool connected);

    void imgCallback(const sensor_msgs::msg::Image::SharedPtr msg);

    void streamServer();

    void handleStream(const httplib::Request&, httplib::Response& res);

    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr image_sub_;

    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr stream_status_pub_;

    cv::Mat stream_frame_;
    std::mutex frame_mutex_;

    std::atomic<bool> stop_flag_;
    std::atomic<bool> new_frame_;

    std::thread server_thread_;
};

void StreamingNode::publishStreamStatus(bool connected)
{
    std_msgs::msg::Bool msg;
    msg.data = connected;
    stream_status_pub_->publish(msg);
}

void StreamingNode::imgCallback(const sensor_msgs::msg::Image::SharedPtr msg)
{
    cv::Mat gray_img(msg->height, msg->width, CV_8UC1, const_cast<uint8_t*>(msg->data.data()), msg->step);

    {
        std::lock_guard<std::mutex> lock(frame_mutex_);
        stream_frame_ = gray_img.clone();
        new_frame_ = true;
    }
}

void StreamingNode::streamServer()
{
    httplib::Server svr;

    svr.Get("/stream", std::bind(&StreamingNode::handleStream, this, _1, _2));

    if (!svr.bind_to_port("0.0.0.0", 8080, 0.1))
    {
        RCLCPP_ERROR(this->get_logger(), "Failed to bind to port 8080");
        return;
    }

    RCLCPP_INFO(this->get_logger(), "Streaming at http://0.0.0.0:8080/stream");

    rclcpp::on_shutdown([&svr]()
    {
        svr.stop();
    });

    svr.listen_after_bind();
}

void StreamingNode::handleStream(const httplib::Request&, httplib::Response& res)
{
    res.set_content_provider(
    "multipart/x-mixed-replace; boundary=frame",
    [&](size_t, httplib::DataSink& sink)
    {
        publishStreamStatus(true);
        
        while(!stop_flag_)
        {
            //TODO: evitar dormir el thread
            if (!new_frame_)
            {
                std::this_thread::sleep_for(std::chrono::milliseconds(1));
                continue;
            }

            cv::Mat frame;
            {
                std::lock_guard<std::mutex> lock(frame_mutex_);
                frame = stream_frame_.clone();
                new_frame_ = false;
            }

            std::vector<uchar> buffer;
            cv::imencode(".jpg", frame, buffer);

            std::ostringstream header;
            header << "--frame\r\n"
                    << "Content-Type: image/jpeg\r\n"
                    << "Content-Length: " << buffer.size()
                    << "\r\n\r\n";

            if (!sink.write(header.str().c_str(), header.str().size())) break;

            if (!sink.write(reinterpret_cast<const char*>(buffer.data()), buffer.size())) break;

            if (!sink.write("\r\n", 2)) break;
        }

        publishStreamStatus(false);

        return true;
    });
}

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<StreamingNode>());
    rclcpp::shutdown();
    
    return 0;
}