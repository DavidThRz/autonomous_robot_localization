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

#include <libcamera/libcamera.h>
#include <libcamera/framebuffer_allocator.h>
#include <libcamera/camera_manager.h>
#include <sys/mman.h>

using namespace libcamera;

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

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = rclcpp::Node::make_shared("photographer_node");

    // Variables to confugure camera
    std::unique_ptr<CameraManager> cm = std::make_unique<CameraManager>();
    std::unique_ptr<CameraConfiguration> config;
    FrameBufferAllocator *allocator;
    StreamConfiguration *streamConfig = nullptr;

    // Initialize camera
    cm->start();
    auto cameras = cm->cameras();
    if (cameras.empty()) 
    {
        std::cerr << "No cameras were identified on the system" << std::endl;
        cm->stop();
        rclcpp::shutdown();
        return -1;
    }

    // Select first available camera
    std::string camera_id = cm->cameras()[0]->id();
    camera = cm->get(camera_id);
    camera->acquire();
    std::cout << "\nCamera acquired: " << camera->id() << std::endl;

    // Configure camera
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
        return -2;
    }

    // Create request
    static Stream *stream = streamConfig->stream();
    std::unique_ptr<Request> request = camera->createRequest();
    if (!request) 
    {
        std::cerr << "Failed to create request" << std::endl;
        return -3;
    }

    const std::unique_ptr<FrameBuffer> &buffer = allocator->buffers(stream)[0];
    ret = request->addBuffer(stream, buffer.get());
    if (ret < 0) 
    {
        std::cerr << "Failed to add buffer to request" << std::endl;
        return -5;
    }


    // Connect callback
    camera->requestCompleted.connect(requestComplete);

    std::cout << "\nStarting camera\n\n" << std::endl;
    camera->start();
    camera->queueRequest(request.get());

    while (rclcpp::ok())
    {

    }

    // Stop camera
    camera->stop();
    allocator->free(streamConfig->stream());
    delete allocator;
    camera->release();
    camera.reset();
    cm->stop();

    // Stop ROS
    rclcpp::shutdown();

    return 0;
}