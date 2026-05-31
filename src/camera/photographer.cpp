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
#include <map>

using namespace libcamera;

// TODO: correct images

class Photographer : public rclcpp::Node
{
public:

    Photographer() : Node("photographer_node")
    {
        rclcpp::QoS qos(rclcpp::KeepLast(3));
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
        config = camera_->generateConfiguration( { StreamRole::Viewfinder } );
        streamConfig_ = &config->at(0);
        streamConfig_->size.width  = 1024; // 1920;
        streamConfig_->size.height = 768; // 1080;
        streamConfig_->pixelFormat = libcamera::formats::YUV420;
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
        const std::vector<std::unique_ptr<FrameBuffer>> &buffers = allocator_->buffers(stream);
        for (unsigned int i = 0; i < buffers.size(); ++i) {
            std::unique_ptr<Request> request = camera_->createRequest();
            if (!request)
            {
                RCLCPP_ERROR(this->get_logger(), "Failed to create request");
                return;
            }
            int ret = request->addBuffer(stream, buffers[i].get());
            if (ret < 0)
            {
                RCLCPP_ERROR(this->get_logger(), "Failed to add buffer to request");
                return;
            }
            requests_.push_back(std::move(request));
        }

        // Connect callback
        camera_->requestCompleted.connect(this, &Photographer::imageCaptured);
        
        int64_t frame_time = 1000000 / 30; // 33333 microseconds for 30 FPS
        std::array<int64_t, 2> frame_duration_limits = { frame_time, frame_time };
        ControlList controls(camera_->controls());
        controls.set(controls::FrameDurationLimits, frame_duration_limits);

        RCLCPP_INFO(this->get_logger(), "Starting camera");
        camera_->start(&controls);
        for (auto &request : requests_) {
            camera_->queueRequest(request.get());
        }
    }

    ~Photographer()
    {
        RCLCPP_INFO(this->get_logger(), "Stopping camera");
        camera_->stop();
        for (auto const& [fd, mem] : mapped_buffers_) {
            munmap(mem, mapped_lengths_[fd]);
        }
        mapped_buffers_.clear();
        mapped_lengths_.clear();
        requests_.clear();
        allocator_->free(streamConfig_->stream());
        delete allocator_;
        camera_->release();
        camera_.reset();
        cm_->stop();
    }

private:

    void imageCaptured(Request *request);

    void publishImage(rclcpp::Time img_stamp);

    cv::Mat img_;
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr publisher_;

    std::shared_ptr<Camera> camera_;
    std::vector<std::unique_ptr<Request>> requests_;
    FrameBufferAllocator *allocator_;
    StreamConfiguration *streamConfig_;
    std::unique_ptr<CameraManager> cm_;
    std::map<int, void*> mapped_buffers_;
    std::map<int, size_t> mapped_lengths_;
};


void Photographer::imageCaptured(Request *request)
{
    if (request->status() == Request::RequestCancelled) 
    {
        std::cerr << "\nRequest was cancelled" << std::endl;

        request->reuse(Request::ReuseBuffers);
        camera_->queueRequest(request);
        return;
    }

    const ControlList &metadata = request->metadata();
    rclcpp::Time img_stamp;
    if (auto ts_opt = metadata.get(libcamera::controls::SensorTimestamp))
    {
        int64_t ts_ns = *ts_opt;
        img_stamp = rclcpp::Time(ts_ns);
    }

    // Save buffers
    const auto& buffers = request->buffers();
    if (buffers.empty())
    {
        RCLCPP_WARN(this->get_logger(), "No buffers in request");
        request->reuse(Request::ReuseBuffers);
        camera_->queueRequest(request);
        return;
    }

    auto [buffer_stream, buffer] = *buffers.begin();
    const FrameBuffer::Plane &plane = buffer->planes()[0];
    int fd = plane.fd.get();

    void *mem = nullptr;
    if (mapped_buffers_.find(fd) == mapped_buffers_.end())
    {
        mem = mmap(nullptr, plane.length, PROT_READ | PROT_WRITE, MAP_SHARED, fd, 0);
        if (mem == MAP_FAILED)
        {
            std::cerr << "mmap failed\n";
        }
        else
        {
            mapped_buffers_[fd] = mem;
            mapped_lengths_[fd] = plane.length;
        }
    }
    else
    {
        mem = mapped_buffers_[fd];
    }

    if (mem != nullptr && mem != MAP_FAILED) {
        int width  = buffer_stream->configuration().size.width;
        int height = buffer_stream->configuration().size.height;
        int stride = buffer_stream->configuration().stride;

        // Get illuminance from YUV image
        img_ = cv::Mat(height, width, CV_8UC1, mem, stride);
        
        publishImage(img_stamp);
    }

    request->reuse(Request::ReuseBuffers);
    camera_->queueRequest(request);
}

void Photographer::publishImage(rclcpp::Time img_stamp)
{
    static sensor_msgs::msg::Image msg = sensor_msgs::msg::Image();
   
    msg.header.frame_id = "camera_frame";
    msg.header.stamp = img_stamp;
    
    msg.height = img_.rows;
    msg.width = img_.cols;
    msg.encoding = "mono8";
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