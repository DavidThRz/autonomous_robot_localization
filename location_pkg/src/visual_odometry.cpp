#include "rclcpp/rclcpp.hpp"
#include <opencv2/opencv.hpp>
#include <httplib.h>
#include <thread>
#include <vector>
#include <sstream>
#include <atomic>
#include <chrono>

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

void start_stream_server(cv::Mat& stream_frame, std::atomic<bool> &send_flag, std::atomic<bool> &stop_flag)
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

                    if (stream_frame.empty())
                        continue;

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

void computeOpticalFlowFarneback(const cv::Mat& prev_frame, const cv::Mat& curr_frame, cv::Mat& flow)
{
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

    //pyr_scale: factor de escala entre pirámides (0 < pyr_scale < 1)
    static const float pyr_scale = 0.5;  //0.5
    //levels: número de niveles en la pirámide
    static const int levels = 2;  //3
    //winsize: tamaño de la ventana de búsqueda
    static const int winsize = 13;  //15
    //iterations: número de iteraciones en cada nivel de la pirámide
    static const int iterations = 1;  //3
    //poly_n: tamaño del vecindario para la aproximación polinómica
    static const int poly_n = 7;  //5
    //poly_sigma: desviación estándar del filtro Gaussiano
    static const double poly_sigma = 1.2;

    cv::calcOpticalFlowFarneback(prev_frame, curr_frame, flow_xy,
                                 pyr_scale, levels, winsize, iterations,
                                poly_n, poly_sigma, cv::OPTFLOW_USE_INITIAL_FLOW );


    flow = prev_frame.clone();

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
    cv::goodFeaturesToTrack(prev_frame, prev_pts, 200, 0.01, 10);   //TODO: ajustar parámetros

    if (prev_pts.empty())
        return;

    std::vector<cv::Point2f> curr_pts;
    std::vector<uchar> status;
    std::vector<float> err;
    cv::calcOpticalFlowPyrLK(prev_frame, curr_frame, prev_pts, curr_pts, status, err);

    flow = prev_frame.clone();
    cv::cvtColor(flow, flow, cv::COLOR_GRAY2BGR);

    for (size_t i = 0; i < prev_pts.size(); ++i)
    {
        if (status[i])
        {
            cv::arrowedLine(flow, prev_pts[i], curr_pts[i], cv::Scalar(0, 0, 255), 2, cv::LINE_AA, 0, 0.3);
            cv::circle(flow, curr_pts[i], 2, cv::Scalar(0, 255, 0), -1);
        }
    }
}

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = rclcpp::Node::make_shared("visual_odometry_node");

    // Params for server
    cv::Mat stream_frame;
    std::atomic<bool> send_flag(true);
    std::atomic<bool> stop_flag(false);
    std::thread server_thread(start_stream_server, std::ref(stream_frame), std::ref(send_flag), std::ref(stop_flag));


    while (rclcpp::ok()) 
    {
        static cv::Mat new_frame;
        captureFrame(new_frame);

        static cv::Mat prev_frame = new_frame.clone();

        // computeOpticalFlowFarneback(prev_frame, new_frame, stream_frame);
        computeOpticalFlowLK(prev_frame, new_frame, stream_frame);

        prev_frame = new_frame.clone();
    }

    rclcpp::shutdown();

    // Detener servidor
    stop_flag = true;
    if (server_thread.joinable())
        server_thread.join();

    return 0;
}
