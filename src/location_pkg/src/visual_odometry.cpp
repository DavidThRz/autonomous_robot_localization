#include "rclcpp/rclcpp.hpp"
#include <opencv2/opencv.hpp>
#include <httplib.h>
#include <thread>
#include <vector>
#include <sstream>
#include <atomic>
#include <chrono>
#include <iostream>

#include <libcamera/libcamera.h>
#include <libcamera/framebuffer_allocator.h>
#include <libcamera/camera_manager.h>
#include <fstream>
#include <sys/mman.h>

#include <iomanip>
#include <iostream>
#include <memory>

using namespace libcamera;
using namespace std::chrono_literals;

static std::shared_ptr<Camera> camera;

static void requestComplete(Request *request)
{
    if (request->status() == Request::RequestCancelled) 
    {
        std::cerr << "\nRequest was cancelled" << std::endl;
        return;
    }

    // Save buffers
    const std::map<const Stream *, FrameBuffer *> &buffers = request->buffers();
    for (auto bufferPair : buffers)
    {
        FrameBuffer *buffer = bufferPair.second;
        const FrameMetadata &metadata = buffer->metadata();

        if (buffer->planes().empty())
            continue;

        const FrameBuffer::Plane &plane = buffer->planes()[0];
        void *mem = mmap(nullptr, plane.length, PROT_READ | PROT_WRITE, MAP_SHARED, plane.fd.get(), 0);
        if (mem == MAP_FAILED) 
        {
            std::cerr << "mmap failed\n";
            continue;
        }

        int width  = bufferPair.first->configuration().size.width;
        int height = bufferPair.first->configuration().size.height;

        cv::Mat img(height, width, CV_8UC3, mem);
        cv::cvtColor(img, img, cv::COLOR_BGR2RGB);

        std::ostringstream name;
        name << "/home/davith/capturas/" << metadata.timestamp << ".jpg";
        if (!cv::imwrite(name.str(), img)) 
            std::cerr << "Error al guardar " << name.str() << std::endl;

        munmap(mem, plane.length);

        std::cout << "Guardada imagen: " << name.str() << std::endl;
    }

    request->reuse(Request::ReuseBuffers);
    camera->queueRequest(request);
}

int captureFrame(cv::Mat& frame)
{
    // Variables to initialize only once
    static std::unique_ptr<CameraManager> cm = std::make_unique<CameraManager>();
    static std::unique_ptr<CameraConfiguration> config;
    static FrameBufferAllocator *allocator;
    static StreamConfiguration *streamConfig = nullptr;

    static bool initialized = false;
    if (!initialized)
    {
        initialized = true;
        std::cout << "\nInitializing Camera Manager..." << std::endl;

        cm->start();

        // Check if there are any cameras
        auto cameras = cm->cameras();
        if (cameras.empty()) 
        {
            std::cout << "No cameras were identified on the system" << std::endl;
            cm->stop();
            return -2;
        }

        std::cout << "\nAvailable cameras:" << std::endl;
        for (auto const &camera : cm->cameras())
            std::cout << camera->id() << std::endl;

        // Select the first available camera
        std::string camera_id = cm->cameras()[0]->id();
        camera = cm->get(camera_id);
        camera->acquire();
        std::cout << "Camera acquired: " << camera->id() << std::endl;

        // Configure the camera
        config = camera->generateConfiguration( { StreamRole::StillCapture } );  //This may affect performance, try StreamRole::Viewfinder or StreamRole::VideoRecording for better fps
        streamConfig = &config->at(0);
        streamConfig->size.width  = 1920;
        streamConfig->size.height = 1080;
        streamConfig->pixelFormat = libcamera::formats::BGR888;
        std::cout << "\nSelected camera configuration: " << streamConfig->toString() << std::endl;
        config->validate();
        camera->configure(config.get());

        // Allocate buffers
        allocator = new FrameBufferAllocator(camera);
        int ret = allocator->allocate(streamConfig->stream());
        if (ret < 0) 
        {
            std::cerr << "Failed to allocate buffers" << std::endl;
            return -3;
        }
        size_t allocated = allocator->buffers(streamConfig->stream()).size();
        std::cout << "\nAllocated " << allocated << " buffers for the camera" << std::endl;

        std::cout << "\nCamera initialized!\n\n" << std::endl;
    }

    // Configure stream
    static Stream *stream = streamConfig->stream();
    std::unique_ptr<Request> request = camera->createRequest();
    if (!request) 
    {
        std::cerr << "Failed to create request" << std::endl;
        return -4;
    }

    const std::unique_ptr<FrameBuffer> &buffer = allocator->buffers(stream)[0];
    int ret = request->addBuffer(stream, buffer.get());
    if (ret < 0) 
    {
        std::cerr << "Failed to add buffer to request" << std::endl;
        return -5;
    }

    // Connect callback
    camera->requestCompleted.connect(requestComplete);

    // Start camera
    camera->start();
    camera->queueRequest(request.get());

    // Capture frames for some time
    std::this_thread::sleep_for(1500ms);

    // Stop camera
    camera->stop();
    allocator->free(streamConfig->stream());
    delete allocator;
    camera->release();
    camera.reset();
    cm->stop();

    return -1;  //For now
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
        if (captureFrame(new_frame) < 0)
            break;

        // static cv::Mat prev_frame = new_frame.clone();

        // computeOpticalFlowFarneback(prev_frame, new_frame, stream_frame);
        // computeOpticalFlowLK(prev_frame, new_frame, stream_frame);

        // prev_frame = new_frame.clone();
    }

    // Stop ROS
    std::cout << "\n========================\n  Shutting down ROS...\n========================\n" << std::endl;
    rclcpp::shutdown();

    // Detener servidor
    stop_flag = true;
    if (server_thread.joinable())
        server_thread.join();

    return 0;
}
