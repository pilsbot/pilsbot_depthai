#ifndef PILSBOT_OAKD_MJPEG_NODE_HPP_
#define PILSBOT_OAKD_MJPEG_NODE_HPP_

#include <memory>
#include <string>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "sensor_msgs/msg/compressed_image.hpp"
#include "cv_bridge/cv_bridge.h"

#include <atomic>
#include <thread>
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
        void build_pipeline(dai::Pipeline &pipeline);
        bool setup_gstreamer();
        void start_device();
        void stop_device();
        void worker_loop();
        void left_callback(std::shared_ptr<dai::ADatatype> data);
        void right_callback(std::shared_ptr<dai::ADatatype> data);
        void depth_callback(std::shared_ptr<dai::ADatatype> data);

        // DepthAI
        std::shared_ptr<dai::Device> device_;
        std::shared_ptr<dai::DataOutputQueue> video_queue_;
        std::shared_ptr<dai::DataOutputQueue> left_queue_;
        std::shared_ptr<dai::DataOutputQueue> right_queue_;
        std::shared_ptr<dai::DataOutputQueue> depth_queue_;

        // Worker thread (color stream)
        std::thread worker_;
        std::atomic<bool> running_{false};

        // Optional GStreamer decoding (for raw decode path)
        bool use_gst_{false};
        int gst_frame_count_{0};
    #ifdef HAVE_GSTREAMER
        GstElement *gst_pipeline_{nullptr};
        GstElement *gst_appsrc_{nullptr};
        GstElement *gst_appsink_{nullptr};
    #endif

        // ROS publishers
        rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr pub_raw_;
        rclcpp::Publisher<sensor_msgs::msg::CompressedImage>::SharedPtr pub_compressed_;
        rclcpp::Publisher<sensor_msgs::msg::CompressedImage>::SharedPtr pub_left_compressed_;
        rclcpp::Publisher<sensor_msgs::msg::CompressedImage>::SharedPtr pub_right_compressed_;
        rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr pub_depth_;

        // Color camera parameters
        std::string color_resolution_;
        int isp_scale_num_;
        int isp_scale_den_;
        int color_fps_;
        int color_jpeg_quality_;

        // Mono camera parameters
        std::string mono_resolution_;
        int mono_fps_;
        int mono_jpeg_quality_;

        // Stereo depth parameters
        bool enable_depth_;
        std::string depth_preset_;
        bool lr_check_;
        bool extended_disparity_;
        bool subpixel_;
        int stereo_confidence_;

        // General parameters
        bool enable_raw_decode_;
        std::string tf_prefix_;
        int queue_size_;
        std::string output_encoding_;
        std::string nv_dec_;

        // Derived dimensions
        int color_width_;
        int color_height_;
        int mono_width_;
        int mono_height_;
    };

} // namespace pilsbot_oakd

#endif // PILSBOT_OAKD_MJPEG_NODE_HPP_
