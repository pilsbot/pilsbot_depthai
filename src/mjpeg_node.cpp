// Simple MJPEG node: subscribes to Image and publishes CompressedImage (jpeg)

#include "pilsbot_oakd/mjpeg_node.hpp"

#include <chrono>
#include <opencv2/imgcodecs.hpp>
#include <depthai/depthai.hpp>
#include <atomic>
#include <thread>
#include <mutex>
#ifdef HAVE_GSTREAMER
#include <gst/gst.h>
#include <gst/app/gstappsrc.h>
#include <gst/app/gstappsink.h>
#endif

namespace pilsbot_oakd
{

    MJPEGNode::MJPEGNode(const rclcpp::NodeOptions &options)
        : Node("mjpeg_node", options)
    {
        // Parameters: control DepthAI preview size, fps and JPEG quality
        preview_width_ = this->declare_parameter<int>("preview_width", 640);
        preview_height_ = this->declare_parameter<int>("preview_height", 480);
        color_fps_ = this->declare_parameter<int>("color_fps", 30);
        jpeg_quality_ = this->declare_parameter<int>("jpeg_quality", 80);

        output_topic_ = this->declare_parameter<std::string>("output_topic", "/camera/color/image_raw");
        output_encoding_ = this->declare_parameter<std::string>("output_encoding", "bgr8");
        // Optionally try to use GStreamer-based decoding (hardware-accelerated on Jetson if plugin available)
        use_gst_ = this->declare_parameter<bool>("use_gstreamer", true);
    #ifndef HAVE_GSTREAMER
        if (use_gst_)
        {
            RCLCPP_WARN(this->get_logger(), "Parameter 'use_gstreamer' requested but binary was built without GStreamer support; using cv::imdecode fallback");
        }
    #endif

        pub_ = this->create_publisher<sensor_msgs::msg::Image>(output_topic_, 10);

        RCLCPP_INFO(this->get_logger(), "Starting DepthAI MJPEG pipeline: %dx%d@%d fps -> %s (jpeg=%d)",
                    preview_width_, preview_height_, color_fps_, output_topic_.c_str(), jpeg_quality_);

        start_device();
    }

    MJPEGNode::~MJPEGNode()
    {
        stop_device();
    }

    void MJPEGNode::start_device()
    {
        if (running_)
        {
            RCLCPP_WARN(this->get_logger(), "Already started MJPEG Node");
            return;
        }

        // Build pipeline
        auto pipeline = std::make_shared<dai::Pipeline>();
        auto colorCam = pipeline->create<dai::node::ColorCamera>();
        colorCam->setPreviewSize(preview_width_, preview_height_);
        colorCam->setInterleaved(false);
        colorCam->setColorOrder(dai::ColorCameraProperties::ColorOrder::BGR);
        colorCam->setFps(color_fps_);

        auto encoder = pipeline->create<dai::node::VideoEncoder>();
        encoder->setProfile(dai::VideoEncoderProperties::Profile::MJPEG);
        encoder->setQuality(jpeg_quality_);

        auto xout = pipeline->create<dai::node::XLinkOut>();
        xout->setStreamName("mjpeg");

        colorCam->video.link(encoder->input);
        encoder->bitstream.link(xout->input);

        try
        {
            device_ = std::make_shared<dai::Device>(*pipeline);
        }
        catch (const std::exception &e)
        {
            RCLCPP_ERROR(this->get_logger(), "Failed to create DepthAI device: %s", e.what());
            return;
        }

        video_queue_ = device_->getOutputQueue("mjpeg", 8, false);

        if (use_gst_)
        {
            if (!setup_gstreamer())
            {
                RCLCPP_ERROR(this->get_logger(), "Could not setup gstreamer.");
                use_gst_ = false;
            }
        }

        running_ = true;
        // Start worker thread to fetch encoded MJPEG and publish decoded images
        worker_ = std::thread(&MJPEGNode::worker_loop, this);
    }

    void MJPEGNode::stop_device()
    {
        if (!running_)
            return;
        running_ = false;
        if (worker_.joinable())
            worker_.join();

        if (device_)
        {
            try
            {
                device_->close();
            }
            catch (...)
            {
            }
            device_.reset();
        }
        video_queue_.reset();

#ifdef HAVE_GSTREAMER
        // Teardown GStreamer pipeline if we created one
        if (gst_pipeline_)
        {
            gst_element_set_state(gst_pipeline_, GST_STATE_NULL);
            gst_object_unref(gst_pipeline_);
            gst_pipeline_ = nullptr;
            gst_appsrc_ = nullptr;
            gst_appsink_ = nullptr;
            use_gst_ = false;
        }
#endif
    }

    bool
    MJPEGNode::setup_gstreamer()
    {
#ifndef HAVE_GSTREAMER
        return false;
#else
        // Initialize GStreamer pipeline for JPEG->BGR decoding. We initialize here so the
        // worker thread can decode using GStreamer if available.
        // If initialization fails we fall back to cv::imdecode.
        // Attempt to create GStreamer pipeline. Implementation below.
        // We don't treat failure as fatal.
        GstStateChangeReturn sret = GST_STATE_CHANGE_FAILURE;
        if (!gst_is_initialized())
        {
            GError *err = nullptr;
            if (!gst_init_check (nullptr, nullptr, &err)) {
                RCLCPP_WARN(this->get_logger(), "Could not initialize GStreamer: %s\n",
                    err ? err->message : "unknown error occurred");
                if (err) {
                    g_error_free (err);
                }
                return false;
            }
        }

        // Create pipeline and elements
        gst_pipeline_ = gst_pipeline_new("mjpeg-decoder-pipeline");
        if (!gst_pipeline_)
        {
            RCLCPP_WARN(this->get_logger(), "GStreamer pipeline create failed");
            return false;
        }

        gst_appsrc_ = gst_element_factory_make("appsrc", "src");
        GstElement *decoder = gst_element_factory_make("nvjpegdec", "decoder");
        if (decoder)
        {
            RCLCPP_INFO(this->get_logger(), "Using nvjpegdec GStreamer decoder");
        }
        else
        {
            RCLCPP_WARN(this->get_logger(), "Could not use nvjpegdec GStreamer decoder!");
            decoder = gst_element_factory_make("jpegdec", "decoder");
            if (decoder)
            {
                RCLCPP_INFO(this->get_logger(), "Using jpegdec GStreamer decoder");
            }
        }
        if (!decoder)
        {
            RCLCPP_WARN(this->get_logger(), "Could also not use jpecdec GStreamer decoder!");
            return false;
        }

        GstElement *videoconvert = gst_element_factory_make("videoconvert", "conv");
        gst_appsink_ = gst_element_factory_make("appsink", "sink");

        if (!gst_appsrc_ || !decoder || !videoconvert || !gst_appsink_)
        {
            RCLCPP_WARN(this->get_logger(), "Incomplete GStreamer element set, disabling GStreamer path");
            if (gst_pipeline_)
            {
                gst_object_unref(gst_pipeline_);
                gst_pipeline_ = nullptr;
            }
            if (gst_appsrc_)
            {
                gst_object_unref(gst_appsrc_);
                gst_appsrc_ = nullptr;
            }
            if (decoder)
            {
                gst_object_unref(decoder);
                decoder = nullptr;
            }
            if (videoconvert)
            {
                gst_object_unref(videoconvert);
                videoconvert = nullptr;
            }
            if (gst_appsink_)
            {
                gst_object_unref(gst_appsink_);
                gst_appsink_ = nullptr;
            }
            return false;
        }

        gst_bin_add_many(GST_BIN(gst_pipeline_), gst_appsrc_, decoder, videoconvert, gst_appsink_, nullptr);
        if (!gst_element_link(gst_appsrc_, decoder) || !gst_element_link(decoder, videoconvert) || !gst_element_link(videoconvert, gst_appsink_))
        {
            RCLCPP_WARN(this->get_logger(), "Failed to link GStreamer elements; disabling GStreamer path");
            gst_object_unref(gst_pipeline_);
            gst_pipeline_ = nullptr;
            // TODO: Free other stuff?
            return false;
        }

        // configure appsrc and appsink caps
        GstCaps *src_caps = gst_caps_from_string("image/jpeg");
        g_object_set(G_OBJECT(gst_appsrc_), "caps", src_caps, "format", GST_FORMAT_TIME, nullptr);
        gst_caps_unref(src_caps);

        GstCaps *sink_caps = gst_caps_from_string("video/x-raw,format=BGR");
        g_object_set(G_OBJECT(gst_appsink_), "caps", sink_caps, "emit-signals", FALSE, "max-buffers", 1, "drop", TRUE, nullptr);
        gst_caps_unref(sink_caps);

        sret = gst_element_set_state(gst_pipeline_, GST_STATE_PLAYING);
        if (sret == GST_STATE_CHANGE_FAILURE)
        {
            RCLCPP_WARN(this->get_logger(), "Failed to set GStreamer pipeline to PLAYING; disabling GStreamer path");
            gst_element_set_state(gst_pipeline_, GST_STATE_NULL);
            gst_object_unref(gst_pipeline_);
            gst_pipeline_ = nullptr;
            // TODO: Free other stuff?
            return false;
        }

        RCLCPP_INFO(this->get_logger(), "GStreamer JPEG decoder initialized (use_gst_=true)");
        return true;
#endif
    }

    void MJPEGNode::worker_loop()
    {
        while (running_ && rclcpp::ok())
        {
            try
            {
                auto data = video_queue_->get<dai::ImgFrame>();
                if (!data)
                    continue;

                // getData returns encoded JPEG bytes
                // TODO: Can we directly work on the data without copying into separate vector?
                const std::vector<uint8_t> buf = data->getData();
                cv::Mat mat;
#ifdef HAVE_GSTREAMER
                bool decoded = false;
                if (use_gst_ && gst_pipeline_ && gst_appsrc_ && gst_appsink_)
                {
                    GstBuffer *gst_buf = gst_buffer_new_allocate(NULL, buf.size(), NULL);
                    if (gst_buf)
                    {
                        GstMapInfo map;
                        if (gst_buffer_map(gst_buf, &map, GST_MAP_WRITE))
                        {
                            memcpy(map.data, buf.data(), buf.size());
                            gst_buffer_unmap(gst_buf, &map);
                        }

                        GstFlowReturn fret = gst_app_src_push_buffer(GST_APP_SRC(gst_appsrc_), gst_buf);
                        if (fret == GST_FLOW_OK)
                        {
                            GstSample *sample = gst_app_sink_try_pull_sample(GST_APP_SINK(gst_appsink_), GST_MSECOND * 500);
                            if (!sample)
                            {
                                // try blocking pull briefly
                                sample = gst_app_sink_pull_sample(GST_APP_SINK(gst_appsink_));
                            }
                            if (sample)
                            {
                                GstBuffer *outbuf = gst_sample_get_buffer(sample);
                                GstMapInfo outmap;
                                if (gst_buffer_map(outbuf, &outmap, GST_MAP_READ))
                                {
                                    GstCaps *caps = gst_sample_get_caps(sample);
                                    int width = 0, height = 0;
                                    if (caps)
                                    {
                                        GstStructure *s = gst_caps_get_structure(caps, 0);
                                        gst_structure_get_int(s, "width", &width);
                                        gst_structure_get_int(s, "height", &height);
                                    }
                                    if (width > 0 && height > 0)
                                    {
                                        // TODO: Can we get rid of that copy?
                                        cv::Mat tmp(height, width, CV_8UC3, (void *)outmap.data);
                                        tmp.copyTo(mat);
                                        decoded = true;
                                    }
                                    gst_buffer_unmap(outbuf, &outmap);
                                }
                                gst_sample_unref(sample);
                            }
                        }
                        else
                        {
                            // the buffer was consumed/freed by GStreamer on push or push failed
                        }
                    }
                }
                if (!decoded)
                {
                    if (use_gst_)
                    {
                        RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 5000, "GStreamer failed to decode a frame; falling back to cv::imdecode");
                        use_gst_ = false;
                        // TODO: Free all stuff
                    }
                }
#endif
                mat = cv::imdecode(buf, cv::IMREAD_COLOR);
                if (mat.empty())
                {
                    RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 5000, "Failed to decode MJPEG frame");
                    continue;
                }

                // Convert to ROS Image
                cv_bridge::CvImage out_cv;
                out_cv.header = std_msgs::msg::Header();
                out_cv.header.stamp = rclcpp::Time(std::chrono::nanoseconds(data->getTimestamp().time_since_epoch()).count());
                out_cv.header.frame_id = "camera";
                out_cv.encoding = output_encoding_;
                out_cv.image = mat;

                auto out_msg = out_cv.toImageMsg();
                pub_->publish(*out_msg);
            }
            catch (const std::exception &e)
            {
                RCLCPP_ERROR(this->get_logger(), "Exception in MJPEG worker: %s", e.what());
            }
        }
    }

    // FIMXE: This function seems to be never used?!
    void MJPEGNode::image_callback(const sensor_msgs::msg::CompressedImage::SharedPtr msg)
    {
        try
        {
            // Basic check: ensure format contains 'jpeg' or 'jpg'
            if (msg->format.find("jpeg") == std::string::npos && msg->format.find("jpg") == std::string::npos)
            {
                RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 5000,
                                     "Received compressed image with unexpected format: '%s'", msg->format.c_str());
            }

            // Decode JPEG buffer into cv::Mat
            std::vector<unsigned char> data(msg->data.begin(), msg->data.end());
            cv::Mat mat = cv::imdecode(data, cv::IMREAD_COLOR);
            if (mat.empty())
            {
                RCLCPP_WARN(this->get_logger(), "Failed to decode JPEG image (empty mat)");
                return;
            }

            // Convert cv::Mat to sensor_msgs::msg::Image via cv_bridge
            cv_bridge::CvImage out_cv;
            out_cv.header = msg->header;
            out_cv.encoding = output_encoding_;
            out_cv.image = mat;

            sensor_msgs::msg::Image::SharedPtr out_msg = out_cv.toImageMsg();
            pub_->publish(*out_msg);
        }
        catch (const cv_bridge::Exception &e)
        {
            RCLCPP_ERROR(this->get_logger(), "cv_bridge exception: %s", e.what());
        }
        catch (const std::exception &e)
        {
            RCLCPP_ERROR(this->get_logger(), "Exception in MJPEG callback: %s", e.what());
        }
    }

} // namespace pilsbot_oakd

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::NodeOptions opts;
    auto node = std::make_shared<pilsbot_oakd::MJPEGNode>(opts);
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
