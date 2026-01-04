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

#include <httplib.h>

cv::Mat stream_frame;
std::atomic<bool> new_frame(false);

void imageCallback(const sensor_msgs::msg::Image::SharedPtr msg)
{
    static cv::Mat gray_img;
    gray_img = cv::Mat(msg->height, msg->width, CV_8UC1, const_cast<uint8_t*>(msg->data.data()), msg->step);

    // cv::cvtColor(gray, stream_frame, cv::COLOR_GRAY2BGR);
    stream_frame = gray_img.clone();

    new_frame = true;
}

void start_stream_server(cv::Mat& stream_frame, std::atomic<bool> &new_frame, std::atomic<bool> &stop_flag)
{
    httplib::Server svr;

    svr.Get("/stream", [&](const httplib::Request &, httplib::Response &res) 
    {
        res.set_content_provider
        (
            "multipart/x-mixed-replace; boundary=frame",
            [&](size_t, httplib::DataSink &sink) 
            {
                while (!stop_flag) 
                {
                    if (!new_frame || stream_frame.empty()) 
                    {
                        std::this_thread::sleep_for(std::chrono::milliseconds(1));
                        continue;
                    }

                    std::vector<uchar> buf;
                    cv::imencode(".jpg", stream_frame, buf);

                    std::ostringstream os;
                    os << "--frame\r\n"
                       << "Content-Type: image/jpeg\r\n\r\n";

                    bool ok;
                    ok = sink.write(os.str().c_str(), os.str().size());
                    if (!ok) break;
                    ok = sink.write(reinterpret_cast<const char *>(buf.data()), buf.size());
                    if (!ok) break;
                    ok = sink.write("\r\n", 2);
                    if (!ok) break;

                    new_frame = false;
                }
                std::cout << "Stream was closed" << std::endl;
                return true;
            });
    });

    rclcpp::on_shutdown([&]() 
    {
        svr.stop();
        RCLCPP_INFO(rclcpp::get_logger("web_stream"), "Server stopped correctly.");
    });

    bool bind_result = svr.bind_to_port("0.0.0.0", 8080, 0.1);
    if (!bind_result) 
    {
        RCLCPP_ERROR(rclcpp::get_logger("web_stream"), "No se pudo iniciar el servidor en el puerto 8080");
        return;
    }

    RCLCPP_INFO(rclcpp::get_logger("web_stream"), "Servidor web en http://0.0.0.0:8080/stream");
    
    while (!stop_flag)
        svr.listen_after_bind();
}

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);

    auto node = rclcpp::Node::make_shared("image_stream_node");

    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr img_sub = 
     node->create_subscription<sensor_msgs::msg::Image>("stream/image", rclcpp::SensorDataQoS(), imageCallback);

    std::atomic<bool> stop_flag(false);
    std::thread server_thread(start_stream_server, std::ref(stream_frame), std::ref(new_frame), std::ref(stop_flag));

    rclcpp::spin(node);

    stop_flag = true;
    if (server_thread.joinable())
        server_thread.join();

    rclcpp::shutdown();
    
    return 0;
}