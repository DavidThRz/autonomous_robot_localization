#include "rclcpp/rclcpp.hpp"
#include <opencv2/opencv.hpp>
#include <httplib.h>
#include <thread>
#include <vector>
#include <sstream>
#include <atomic>
#include <chrono>

// Función que levanta el servidor MJPEG
void start_stream_server(cv::VideoCapture &cap, std::atomic<bool> &send_flag, std::atomic<bool> &stop_flag)
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
                    if (!send_flag) 
                    {
                        std::this_thread::sleep_for(std::chrono::milliseconds(50));
                        continue;
                    }

                    cv::Mat frame;
                    cap >> frame;
                    if (frame.empty())
                        continue;

                    std::vector<uchar> buf;
                    cv::imencode(".jpg", frame, buf);

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

                    std::this_thread::sleep_for(std::chrono::milliseconds(33)); // ~30 FPS
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

    auto node = rclcpp::Node::make_shared("web_stream_node");

    // Abrir la cámara
    cv::VideoCapture cap(0, cv::CAP_V4L2);
    if (!cap.isOpened()) 
    {
        RCLCPP_ERROR(node->get_logger(), "No se pudo abrir la cámara");
        rclcpp::shutdown();
        return 1;
    }

    // Flags de control
    std::atomic<bool> send_flag(true);
    std::atomic<bool> stop_flag(false);

    // Hilo del servidor HTTP
    std::thread server_thread(start_stream_server, std::ref(cap), std::ref(send_flag), std::ref(stop_flag));

    rclcpp::Rate rate(10);
    while (rclcpp::ok()) 
    {
        rclcpp::spin_some(node);
        rate.sleep();
    }

    // Detener servidor
    stop_flag = true;
    if (server_thread.joinable())
        server_thread.join();

    cap.release();
    rclcpp::shutdown();
    return 0;
}
