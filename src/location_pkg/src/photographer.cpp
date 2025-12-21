/**
 *  photographer.cpp
 *  Created on: November 30, 2025
 *  
 *  This file is part of the Autonomous Robot Localization 
 *  project.
 *  It handles image capturing from a CSI camera using the 
 *  libcamera library and sends the images to a receiver
 *  using a pipeline.
 */

#include "rclcpp/rclcpp.hpp"
#include <opencv2/opencv.hpp>
#include <sensor_msgs/msg/image.hpp>

#include <libcamera/libcamera.h>
#include <libcamera/framebuffer_allocator.h>
#include <libcamera/camera_manager.h>
#include <sys/mman.h>

using namespace libcamera;

class Photographer : public rclcpp::Node
{
public:

    Photographer() : Node("photographer_node")
    {
        rclcpp::QoS qos(rclcpp::KeepLast(1));
        qos.best_effort();
        qos.durability_volatile();

        publisher_ =
            create_publisher<sensor_msgs::msg::Image>("camera/image", qos);

        // Camera confuration
        cm_ = std::make_unique<CameraManager>();
        std::unique_ptr<CameraConfiguration> config;
        streamConfig_ = nullptr;

        // Initialize camera
        cm_->start();
        auto cameras = cm_->cameras();
        if (cameras.empty()) 
        {
            RCLCPP_ERROR(this->get_logger(), "No cameras were identified on the system");
            cm_->stop();
            rclcpp::shutdown();
            return;
        }

        // Select first available camera
        std::string camera_id = cm_->cameras()[0]->id();
        camera_ = cm_->get(camera_id);
        camera_->acquire();
        RCLCPP_INFO(this->get_logger(), "Camera acquired: %s", camera_->id().c_str());

        // Configure camera
        config = camera_->generateConfiguration( { StreamRole::StillCapture } );  //This may affect performance, try StreamRole::Viewfinder or StreamRole::VideoRecording for better fps    
        streamConfig_ = &config->at(0);
        streamConfig_->size.width  = 1920;
        streamConfig_->size.height = 1080;
        streamConfig_->pixelFormat = libcamera::formats::BGR888;
        RCLCPP_INFO(this->get_logger(), "Selected camera configuration: %s", streamConfig_->toString().c_str());
        config->validate();
        camera_->configure(config.get());

        // Allocate buffers
        allocator_ = new FrameBufferAllocator(camera_);
        int ret = allocator_->allocate(streamConfig_->stream());
        if (ret < 0) 
        {
            RCLCPP_ERROR(this->get_logger(), "Failed to allocate buffers");
            return;
        }

        // Create request
        static Stream *stream = streamConfig_->stream();
        request_ = camera_->createRequest();
        if (!request_)
        {
            RCLCPP_ERROR(this->get_logger(), "Failed to create request");
            return;
        }

        const std::unique_ptr<FrameBuffer> &buffer = allocator_->buffers(stream)[0];
        ret = request_->addBuffer(stream, buffer.get());
        if (ret < 0)
        {
            RCLCPP_ERROR(this->get_logger(), "Failed to add buffer to request");
            return;
        }

        // Connect callback
        camera_->requestCompleted.connect(this, &Photographer::imageCaptured);

        RCLCPP_INFO(this->get_logger(), "Starting camera");
        camera_->start();
        camera_->queueRequest(request_.get());
    }

    ~Photographer()
    {
        RCLCPP_INFO(this->get_logger(), "Stopping camera");
        camera_->stop();
        allocator_->free(streamConfig_->stream());
        delete allocator_;
        camera_->release();
        camera_.reset();
        cm_->stop();
    }

private:

    void imageCaptured(Request *request);

    void publishImage();

    cv::Mat img_;
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr publisher_;

    std::shared_ptr<Camera> camera_;
    std::unique_ptr<Request> request_;
    FrameBufferAllocator *allocator_;
    StreamConfiguration *streamConfig_;
    std::unique_ptr<CameraManager> cm_;

};


void Photographer::imageCaptured(Request *request)
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

        img_ = cv::Mat(height, width, CV_8UC3, mem);
        cv::cvtColor(img_, img_, cv::COLOR_BGR2RGB);

        publishImage();
    }

    request->reuse(Request::ReuseBuffers);
    camera_->queueRequest(request);
}

void Photographer::publishImage()
{
    static sensor_msgs::msg::Image msg = sensor_msgs::msg::Image();
   
    msg.header.frame_id = "camera_frame";
    msg.header.stamp = this->now();
    
    msg.height = img_.rows;
    msg.width = img_.cols;
    msg.encoding = "rgb8";
    msg.is_bigendian = false;
    msg.step = static_cast<sensor_msgs::msg::Image::_step_type>(img_.step);

    size_t size = img_.step * img_.rows;
    msg.data.resize(size);
    memcpy(msg.data.data(), img_.data, size);

    publisher_->publish(msg);
}

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<Photographer>());
    rclcpp::shutdown();
    return 0;
}