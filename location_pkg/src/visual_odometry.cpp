#include "rclcpp/rclcpp.hpp"
#include <opencv2/opencv.hpp>
#include <httplib.h>
#include <thread>
#include <vector>
#include <sstream>
#include <atomic>
#include <chrono>

#include "geometry_msgs/msg/twist.hpp"

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

void computeOpticalFlowFarneback(const cv::Mat& prev_frame, const cv::Mat& curr_frame, std::vector<cv::Point2f>& prev_pts, std::vector<cv::Point2f>& curr_pts, cv::Mat& flow)
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


    flow = prev_frame.clone();

    int step = 16;
    for (int y = 0; y < flow.rows; y += step)
    {
        for (int x = 0; x < flow.cols; x += step)
        {
            const cv::Point2f& fxy = flow_xy.at<cv::Point2f>(y, x);
            cv::Point p1(x, y);
            cv::Point p2(cvRound(x + fxy.x), cvRound(y + fxy.y));

            prev_pts.push_back(p1);
            curr_pts.push_back(p2);

            cv::arrowedLine(flow, p1, p2, cv::Scalar(0, 255, 0), 1, cv::LINE_AA, 0, 0.3);
        }
    }
}

void computeOpticalFlowLK(const cv::Mat& prev_frame, const cv::Mat& curr_frame, std::vector<cv::Point2f>& prev_pts, std::vector<cv::Point2f>& curr_pts, cv::Mat& flow)
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

    cv::goodFeaturesToTrack(prev_frame, prev_pts, maxCorners, qualityLevel, minDistance);

    if (prev_pts.empty())
        return;

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
    rclcpp::Node::SharedPtr node = rclcpp::Node::make_shared("visual_odometry_node");
    auto twist_pub = node->create_publisher<geometry_msgs::msg::Twist>("visual_odometry/twist", 10);


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

        std::vector<cv::Point2f> prev_pts, curr_pts;

        // computeOpticalFlowFarneback(prev_frame, new_frame, prev_pts, curr_pts, stream_frame);
        computeOpticalFlowLK(prev_frame, new_frame, prev_pts, curr_pts, stream_frame);

        // Obtain translation
        cv::Point2f mass_center_prev = cv::Point2f(0,0);
        cv::Point2f mass_center_curr = cv::Point2f(0,0);
        for (size_t i = 0; i < prev_pts.size(); ++i)
        {
            mass_center_prev += prev_pts[i];
            mass_center_curr += curr_pts[i];
        }
        if (!prev_pts.empty())
        {
            mass_center_prev *= (1.0f / prev_pts.size());
            mass_center_curr *= (1.0f / curr_pts.size());
        }
        cv::Point2f translation = mass_center_curr - mass_center_prev;

        // Obtain rotation
        double angle = 0.0;
        std::vector<cv::Mat> vector_prev, vector_curr;
        for (size_t i = 0; i < prev_pts.size(); ++i) 
        {
            cv::Mat v_prev = (cv::Mat_<double>(2,1) << prev_pts[i].x - mass_center_prev.x, prev_pts[i].y - mass_center_prev.y);
            cv::Mat v_curr = (cv::Mat_<double>(2,1) << curr_pts[i].x - mass_center_curr.x, curr_pts[i].y - mass_center_curr.y);
            vector_prev.push_back(v_prev);
            vector_curr.push_back(v_curr);
        }

        cv::Mat R = cv::Mat::zeros(2, 2, CV_64F);
        for (size_t i = 0; i < vector_prev.size(); ++i) 
        {
            R += vector_curr[i] * vector_prev[i].t();
        }
        R /= static_cast<double>(vector_prev.size());

        cv::SVD svd(R);
        cv::Mat R_ortho = svd.u * svd.vt;
        angle = atan2(R_ortho.at<double>(1,0), R_ortho.at<double>(0,0)) * 180.0 / CV_PI;

        static double total_angle = 0.0;
        static cv::Point2f total_translation = cv::Point2f(0,0);
        total_angle += angle;
        total_translation += translation;
        
        geometry_msgs::msg::Twist twist_msg;
        twist_msg.linear.x = translation.x;
        twist_msg.linear.y = translation.y;
        twist_msg.linear.z = 0.0;
        twist_msg.angular.x = 0.0;
        twist_msg.angular.y = 0.0;
        twist_msg.angular.z = angle; // ángulo en grados o radianes según tu EKF

        twist_pub->publish(twist_msg);


        prev_frame = new_frame.clone();
    }

    rclcpp::shutdown();

    // Detener servidor
    stop_flag = true;
    if (server_thread.joinable())
        server_thread.join();

    return 0;
}
