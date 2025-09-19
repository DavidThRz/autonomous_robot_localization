#include "rclcpp/rclcpp.hpp"
#include <opencv2/opencv.hpp>
#include <httplib.h>
#include <thread>
#include <sstream>
#include <vector>

class WebStreamNode : public rclcpp::Node
{
public:
    WebStreamNode() : Node("web_stream_node")
    {
        // Abrir la cámara
        cap_.open(0, cv::CAP_V4L2);
        if (!cap_.isOpened()) {
            RCLCPP_ERROR(this->get_logger(), "No se pudo abrir la cámara");
            rclcpp::shutdown();
            return;
        }

        // Inicializar el servidor
        server_ = std::make_shared<httplib::Server>();
        stop_server_ = false;

        // Hilo del servidor
        server_thread_ = std::thread([this]() { this->runServer(); });
    }

    ~WebStreamNode() {
        stop_server_ = true;
        if (server_thread_.joinable())
            server_thread_.join();
        cap_.release();
    }

private:
    void runServer() {
        // Endpoint /stream para MJPEG
        server_->Get("/stream", [&](const httplib::Request &, httplib::Response &res) {
            res.set_content_provider(
                "multipart/x-mixed-replace; boundary=frame",
                [this](size_t, httplib::DataSink &sink) {
                    while (!stop_server_) {
                        cv::Mat frame;
                        cap_ >> frame;
                        if (frame.empty())
                            continue;

                        // Codificar frame a JPEG
                        std::vector<uchar> buf;
                        cv::imencode(".jpg", frame, buf);

                        // Escribir encabezado del frame
                        std::ostringstream os;
                        os << "--frame\r\n"
                           << "Content-Type: image/jpeg\r\n\r\n";
                        sink.write(os.str().c_str(), os.str().size());

                        // Escribir los datos JPEG
                        sink.write(reinterpret_cast<const char *>(buf.data()), buf.size());
                        sink.write("\r\n", 2);

                        std::this_thread::sleep_for(std::chrono::milliseconds(33)); // ~30 FPS
                    }
                    return true;
                });
        });

        // Bind no bloqueante con timeout 1s
        if (!server_->bind_to_port("0.0.0.0", 8080, 1)) {
            RCLCPP_ERROR(this->get_logger(), "No se pudo iniciar el servidor en el puerto 8080");
            return;
        }

        RCLCPP_INFO(this->get_logger(), "Servidor web en http://0.0.0.0:8080/stream");

        // Bucle principal del servidor (polling)
        while (!stop_server_) {
            server_->poll();
            std::this_thread::sleep_for(std::chrono::milliseconds(10));
        }
    }

    cv::VideoCapture cap_;
    std::shared_ptr<httplib::Server> server_;
    std::thread server_thread_;
    bool stop_server_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<WebStreamNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
