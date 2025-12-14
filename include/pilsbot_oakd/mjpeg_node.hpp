#ifndef PILSBOT_OAKD_MJPEG_NODE_HPP_
#define PILSBOT_OAKD_MJPEG_NODE_HPP_

#include <memory>
#include <string>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "cv_bridge/cv_bridge.h"

#include <atomic>
#include <thread>
#include <mutex>
#include <depthai/depthai.hpp>
#ifdef HAVE_GSTREAMER
#include <gst/gst.h>
#include <gst/app/gstappsrc.h>
#include <gst/app/gstappsink.h>
#endif

namespace pilsbot_oakd
{

    class MJPEGNode : public rclcpp::Node
    {
    public:
        explicit MJPEGNode(const rclcpp::NodeOptions &options = rclcpp::NodeOptions());
        ~MJPEGNode();

    private:
        bool setup_gstreamer();
        void start_device();
        void stop_device();
        void worker_loop();
        void image_callback(const sensor_msgs::msg::CompressedImage::SharedPtr msg);

        // DepthAI
        std::shared_ptr<dai::Device> device_;
        std::shared_ptr<dai::DataOutputQueue> video_queue_;

        // Worker thread
        std::thread worker_;
        std::atomic<bool> running_{false};

        // Optional GStreamer decoding (use hardware-accelerated decoder when available)
        bool use_gst_{false};
    #ifdef HAVE_GSTREAMER
        GstElement *gst_pipeline_{nullptr};
        GstElement *gst_appsrc_{nullptr};
        GstElement *gst_appsink_{nullptr};
    #endif

        // ROS pub
        rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr pub_;

        // parameters
        std::string output_topic_;
        std::string output_encoding_;
        std::string nv_dec_;
        int preview_width_;
        int preview_height_;
        int color_fps_;
        int jpeg_quality_;
    };

} // namespace pilsbot_oakd

#endif // PILSBOT_OAKD_MJPEG_NODE_HPP_
