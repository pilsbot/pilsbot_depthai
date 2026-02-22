#include "pilsbot_oakd/mjpeg_node.hpp"

#include <chrono>
#include <cstring>
#include <functional>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/core.hpp>
#include <depthai/depthai.hpp>

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
        // Color camera parameters
        color_resolution_ = this->declare_parameter<std::string>("color_resolution", "1080p");
        isp_scale_num_ = this->declare_parameter<int>("isp_scale_num", 2);
        isp_scale_den_ = this->declare_parameter<int>("isp_scale_den", 3);
        color_fps_ = this->declare_parameter<int>("color_fps", 30);
        color_jpeg_quality_ = this->declare_parameter<int>("color_jpeg_quality", 80);

        // Mono camera parameters
        mono_resolution_ = this->declare_parameter<std::string>("mono_resolution", "400p");
        mono_fps_ = this->declare_parameter<int>("mono_fps", 15);
        mono_jpeg_quality_ = this->declare_parameter<int>("mono_jpeg_quality", 50);

        // Stereo depth parameters
        enable_depth_ = this->declare_parameter<bool>("enable_depth", false);
        depth_preset_ = this->declare_parameter<std::string>("depth_preset", "HIGH_DENSITY");
        lr_check_ = this->declare_parameter<bool>("lr_check", true);
        extended_disparity_ = this->declare_parameter<bool>("extended_disparity", false);
        subpixel_ = this->declare_parameter<bool>("subpixel", false);
        stereo_confidence_ = this->declare_parameter<int>("stereo_confidence", 200);

        // General parameters
        enable_raw_decode_ = this->declare_parameter<bool>("enable_raw_decode", false);
        tf_prefix_ = this->declare_parameter<std::string>("tf_prefix", "oak");
        queue_size_ = this->declare_parameter<int>("queue_size", 4);
        output_encoding_ = this->declare_parameter<std::string>("output_encoding", "rgb8");
        use_gst_ = this->declare_parameter<bool>("use_gstreamer", true);
        nv_dec_ = this->declare_parameter<std::string>("nvdecoder", "");

    #ifndef HAVE_GSTREAMER
        if (use_gst_)
        {
            RCLCPP_WARN(this->get_logger(),
                "use_gstreamer requested but built without GStreamer support; using cv::imdecode fallback");
        }
    #endif

        // Derive color dimensions from sensor resolution + ISP scale
        int sensor_w = 1920, sensor_h = 1080;
        if (color_resolution_ == "4K") { sensor_w = 3840; sensor_h = 2160; }
        color_width_ = sensor_w * isp_scale_num_ / isp_scale_den_;
        color_height_ = sensor_h * isp_scale_num_ / isp_scale_den_;

        // Derive mono dimensions from resolution string
        if (mono_resolution_ == "480p")      { mono_width_ = 640;  mono_height_ = 480; }
        else if (mono_resolution_ == "720p") { mono_width_ = 1280; mono_height_ = 720; }
        else if (mono_resolution_ == "800p") { mono_width_ = 1280; mono_height_ = 800; }
        else                                 { mono_width_ = 640;  mono_height_ = 400; } // 400p default

        // Publishers (always created)
        pub_compressed_ = this->create_publisher<sensor_msgs::msg::CompressedImage>(
            "/camera/color/image_raw/compressed", rclcpp::QoS(10));
        pub_left_compressed_ = this->create_publisher<sensor_msgs::msg::CompressedImage>(
            "/camera/left/image_rect/compressed", rclcpp::QoS(10));
        pub_right_compressed_ = this->create_publisher<sensor_msgs::msg::CompressedImage>(
            "/camera/right/image_rect/compressed", rclcpp::QoS(10));

        // Conditionally created publishers
        if (enable_raw_decode_)
        {
            pub_raw_ = this->create_publisher<sensor_msgs::msg::Image>(
                "/camera/color/image_raw", rclcpp::SensorDataQoS());
        }
        if (enable_depth_)
        {
            pub_depth_ = this->create_publisher<sensor_msgs::msg::Image>(
                "/camera/stereo/depth", rclcpp::QoS(10));
        }

        RCLCPP_INFO(this->get_logger(),
            "Starting pipeline: color %dx%d@%dfps (q=%d), mono %dx%d@%dfps (q=%d), depth=%s, raw_decode=%s",
            color_width_, color_height_, color_fps_, color_jpeg_quality_,
            mono_width_, mono_height_, mono_fps_, mono_jpeg_quality_,
            enable_depth_ ? "on" : "off", enable_raw_decode_ ? "on" : "off");

        start_device();
    }

    MJPEGNode::~MJPEGNode()
    {
        stop_device();
    }

    void MJPEGNode::build_pipeline(dai::Pipeline &pipeline)
    {
        // --- Color camera with ISP scaling ---
        auto colorCam = pipeline.create<dai::node::ColorCamera>();
        colorCam->setResolution(dai::ColorCameraProperties::SensorResolution::THE_1080_P);
        colorCam->setIspScale(isp_scale_num_, isp_scale_den_);
        colorCam->setVideoSize(color_width_, color_height_);
        colorCam->setInterleaved(false);
        colorCam->setColorOrder(dai::ColorCameraProperties::ColorOrder::RGB);
        colorCam->setFps(color_fps_);

        auto colorEnc = pipeline.create<dai::node::VideoEncoder>();
        colorEnc->setProfile(dai::VideoEncoderProperties::Profile::MJPEG);
        colorEnc->setQuality(color_jpeg_quality_);

        auto colorOut = pipeline.create<dai::node::XLinkOut>();
        colorOut->setStreamName("color_mjpeg");
        colorOut->input.setQueueSize(2);
        colorOut->input.setBlocking(false);

        colorCam->video.link(colorEnc->input);
        colorEnc->bitstream.link(colorOut->input);

        // --- Mono cameras ---
        dai::MonoCameraProperties::SensorResolution mono_res;
        if (mono_resolution_ == "480p")      mono_res = dai::MonoCameraProperties::SensorResolution::THE_480_P;
        else if (mono_resolution_ == "720p") mono_res = dai::MonoCameraProperties::SensorResolution::THE_720_P;
        else if (mono_resolution_ == "800p") mono_res = dai::MonoCameraProperties::SensorResolution::THE_800_P;
        else                                 mono_res = dai::MonoCameraProperties::SensorResolution::THE_400_P;

        auto monoLeft = pipeline.create<dai::node::MonoCamera>();
        monoLeft->setResolution(mono_res);
        monoLeft->setBoardSocket(dai::CameraBoardSocket::LEFT);
        monoLeft->setFps(static_cast<float>(mono_fps_));

        auto monoRight = pipeline.create<dai::node::MonoCamera>();
        monoRight->setResolution(mono_res);
        monoRight->setBoardSocket(dai::CameraBoardSocket::RIGHT);
        monoRight->setFps(static_cast<float>(mono_fps_));

        // --- Stereo depth (always created for rectification) ---
        auto stereo = pipeline.create<dai::node::StereoDepth>();
        if (depth_preset_ == "HIGH_ACCURACY")
            stereo->setDefaultProfilePreset(dai::node::StereoDepth::PresetMode::HIGH_ACCURACY);
        else
            stereo->setDefaultProfilePreset(dai::node::StereoDepth::PresetMode::HIGH_DENSITY);
        stereo->setLeftRightCheck(lr_check_);
        stereo->setExtendedDisparity(extended_disparity_);
        stereo->setSubpixel(subpixel_);
        stereo->initialConfig.setConfidenceThreshold(stereo_confidence_);
        stereo->setRectifyEdgeFillColor(0);

        monoLeft->out.link(stereo->left);
        monoRight->out.link(stereo->right);

        // --- Rectified left encoder + output ---
        auto leftEnc = pipeline.create<dai::node::VideoEncoder>();
        leftEnc->setProfile(dai::VideoEncoderProperties::Profile::MJPEG);
        leftEnc->setQuality(mono_jpeg_quality_);

        auto leftOut = pipeline.create<dai::node::XLinkOut>();
        leftOut->setStreamName("left_mjpeg");
        leftOut->input.setQueueSize(2);
        leftOut->input.setBlocking(false);

        stereo->rectifiedLeft.link(leftEnc->input);
        leftEnc->bitstream.link(leftOut->input);

        // --- Rectified right encoder + output ---
        auto rightEnc = pipeline.create<dai::node::VideoEncoder>();
        rightEnc->setProfile(dai::VideoEncoderProperties::Profile::MJPEG);
        rightEnc->setQuality(mono_jpeg_quality_);

        auto rightOut = pipeline.create<dai::node::XLinkOut>();
        rightOut->setStreamName("right_mjpeg");
        rightOut->input.setQueueSize(2);
        rightOut->input.setBlocking(false);

        stereo->rectifiedRight.link(rightEnc->input);
        rightEnc->bitstream.link(rightOut->input);

        // --- Depth output (optional) ---
        if (enable_depth_)
        {
            auto depthOut = pipeline.create<dai::node::XLinkOut>();
            depthOut->setStreamName("depth");
            depthOut->input.setQueueSize(2);
            depthOut->input.setBlocking(false);

            stereo->depth.link(depthOut->input);
        }
    }

    void MJPEGNode::start_device()
    {
        if (running_)
        {
            RCLCPP_WARN(this->get_logger(), "Already started");
            return;
        }

        dai::Pipeline pipeline;
        build_pipeline(pipeline);

        try
        {
            device_ = std::make_shared<dai::Device>(pipeline);
        }
        catch (const std::exception &e)
        {
            RCLCPP_ERROR(this->get_logger(), "Failed to create DepthAI device: %s", e.what());
            return;
        }

        video_queue_ = device_->getOutputQueue("color_mjpeg", queue_size_, false);
        left_queue_ = device_->getOutputQueue("left_mjpeg", queue_size_, false);
        right_queue_ = device_->getOutputQueue("right_mjpeg", queue_size_, false);

        left_queue_->addCallback(std::bind(&MJPEGNode::left_callback, this, std::placeholders::_1));
        right_queue_->addCallback(std::bind(&MJPEGNode::right_callback, this, std::placeholders::_1));

        if (enable_depth_)
        {
            depth_queue_ = device_->getOutputQueue("depth", queue_size_, false);
            depth_queue_->addCallback(std::bind(&MJPEGNode::depth_callback, this, std::placeholders::_1));
        }

        if (enable_raw_decode_ && use_gst_)
        {
            if (!setup_gstreamer())
            {
                RCLCPP_WARN(this->get_logger(), "GStreamer setup failed; will use cv::imdecode fallback");
                use_gst_ = false;
            }
        }
        else
        {
            use_gst_ = false;
        }

        running_ = true;
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
        left_queue_.reset();
        right_queue_.reset();
        depth_queue_.reset();

    #ifdef HAVE_GSTREAMER
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

    // --- Mono/depth callbacks (run on DepthAI internal thread) ---

    void MJPEGNode::left_callback(std::shared_ptr<dai::ADatatype> data)
    {
        auto frame = std::dynamic_pointer_cast<dai::ImgFrame>(data);
        if (!frame) return;

        sensor_msgs::msg::CompressedImage msg;
        msg.header.stamp = this->now();
        msg.header.frame_id = tf_prefix_ + "_left_camera_optical_frame";
        msg.format = "jpeg";
        msg.data = frame->getData();
        pub_left_compressed_->publish(msg);
    }

    void MJPEGNode::right_callback(std::shared_ptr<dai::ADatatype> data)
    {
        auto frame = std::dynamic_pointer_cast<dai::ImgFrame>(data);
        if (!frame) return;

        sensor_msgs::msg::CompressedImage msg;
        msg.header.stamp = this->now();
        msg.header.frame_id = tf_prefix_ + "_right_camera_optical_frame";
        msg.format = "jpeg";
        msg.data = frame->getData();
        pub_right_compressed_->publish(msg);
    }

    void MJPEGNode::depth_callback(std::shared_ptr<dai::ADatatype> data)
    {
        auto frame = std::dynamic_pointer_cast<dai::ImgFrame>(data);
        if (!frame) return;

        // Keep raw data alive while cv::Mat wraps it
        std::vector<uint8_t> raw = frame->getData();
        cv::Mat depth_mat(frame->getHeight(), frame->getWidth(), CV_16UC1, raw.data());

        cv_bridge::CvImage cv_img;
        cv_img.header.stamp = this->now();
        cv_img.header.frame_id = tf_prefix_ + "_left_camera_optical_frame";
        cv_img.encoding = sensor_msgs::image_encodings::TYPE_16UC1;
        cv_img.image = depth_mat;

        pub_depth_->publish(*cv_img.toImageMsg());
    }

    // --- GStreamer setup (only used when enable_raw_decode + use_gstreamer) ---

    bool MJPEGNode::setup_gstreamer()
    {
    #ifndef HAVE_GSTREAMER
        return false;
    #else
        GstStateChangeReturn sret = GST_STATE_CHANGE_FAILURE;
        if (!gst_is_initialized())
        {
            GError *err = nullptr;
            if (!gst_init_check(nullptr, nullptr, &err))
            {
                RCLCPP_WARN(this->get_logger(), "Could not initialize GStreamer: %s",
                    err ? err->message : "unknown error");
                if (err) g_error_free(err);
                return false;
            }
        }

        gst_pipeline_ = gst_pipeline_new("mjpeg-decoder-pipeline");
        if (!gst_pipeline_)
        {
            RCLCPP_WARN(this->get_logger(), "GStreamer pipeline create failed");
            return false;
        }

        gst_appsrc_ = gst_element_factory_make("appsrc", "src");

        GstElement *decoder = nullptr;
        GstElement *jpegparse = nullptr;
        bool want_jpegparse = false;
        if (nv_dec_ == "nvjpegdec")
        {
            decoder = gst_element_factory_make("nvjpegdec", "decoder");
        }
        else if (nv_dec_ == "nvv4l2decoder")
        {
            want_jpegparse = true;
            decoder = gst_element_factory_make("nvv4l2decoder", "decoder");
            if (decoder)
                g_object_set(G_OBJECT(decoder), "mjpeg", TRUE, nullptr);
        }
        else
        {
            RCLCPP_INFO(this->get_logger(), "No NV decoder requested, using software jpegdec");
            decoder = gst_element_factory_make("jpegdec", "decoder");
        }

        if (!decoder)
        {
            RCLCPP_WARN(this->get_logger(), "Requested NV decoder not available; falling back to software jpegdec");
            decoder = gst_element_factory_make("jpegdec", "decoder");
        }

        if (want_jpegparse)
            jpegparse = gst_element_factory_make("jpegparse", "jpegparse");

        // Try hardware nvvidconv, fall back to software videoconvert
        GstElement *nvvidconv = gst_element_factory_make("nvvidconv", "nvconv");
        bool using_nvvidconv = (nvvidconv != nullptr);
        if (using_nvvidconv)
            RCLCPP_INFO(this->get_logger(), "Using nvvidconv for HW NVMM->system memory conversion");

        GstElement *videoconvert = nullptr;
        if (!using_nvvidconv)
            videoconvert = gst_element_factory_make("videoconvert", "conv");

        GstElement *queue = gst_element_factory_make("queue", "queue");
        if (queue)
        {
            g_object_set(G_OBJECT(queue),
                "max-size-buffers", 2,
                "max-size-time", (guint64)0,
                "max-size-bytes", 0,
                nullptr);
        }

        // NVMM capsfilter for HW decoder path
        GstElement *nvmm_capsfilter = nullptr;
        if (using_nvvidconv && (nv_dec_ == "nvv4l2decoder" || nv_dec_ == "nvjpegdec"))
        {
            nvmm_capsfilter = gst_element_factory_make("capsfilter", "nvmm_caps");
            if (nvmm_capsfilter)
            {
                GstCaps *nvmm_caps = gst_caps_from_string("video/x-raw(memory:NVMM)");
                g_object_set(G_OBJECT(nvmm_capsfilter), "caps", nvmm_caps, nullptr);
                gst_caps_unref(nvmm_caps);
            }
        }

        gst_appsink_ = gst_element_factory_make("appsink", "sink");

        bool have_converter = using_nvvidconv ? (nvvidconv != nullptr) : (videoconvert != nullptr);
        if (!gst_appsrc_ || !decoder || !have_converter || !gst_appsink_)
        {
            RCLCPP_WARN(this->get_logger(), "Incomplete GStreamer element set, disabling GStreamer path");
            if (gst_pipeline_)   { gst_object_unref(gst_pipeline_);   gst_pipeline_ = nullptr; }
            if (gst_appsrc_)     { gst_object_unref(gst_appsrc_);     gst_appsrc_ = nullptr; }
            if (jpegparse)         gst_object_unref(jpegparse);
            if (decoder)           gst_object_unref(decoder);
            if (nvvidconv)         gst_object_unref(nvvidconv);
            if (videoconvert)      gst_object_unref(videoconvert);
            if (nvmm_capsfilter)   gst_object_unref(nvmm_capsfilter);
            if (queue)             gst_object_unref(queue);
            if (gst_appsink_)    { gst_object_unref(gst_appsink_);    gst_appsink_ = nullptr; }
            return false;
        }

        // Configure appsrc caps with actual color dimensions
        char src_caps_str[256];
        snprintf(src_caps_str, sizeof(src_caps_str),
            "image/jpeg, width=%d, height=%d, pixel-aspect-ratio=1/1, framerate=%d/1",
            color_width_, color_height_, color_fps_);
        GstCaps *src_caps = gst_caps_from_string(src_caps_str);
        g_object_set(G_OBJECT(gst_appsrc_),
            "caps", src_caps,
            "format", GST_FORMAT_TIME,
            "stream-type", 0,
            "is-live", TRUE,
            "do-timestamp", TRUE,
            nullptr);
        gst_caps_unref(src_caps);

        // Configure appsink caps
        const char *output_format = using_nvvidconv ? "RGBA" : "BGR";
        char sink_caps_str[256];
        snprintf(sink_caps_str, sizeof(sink_caps_str),
            "video/x-raw,format=%s,width=%d,height=%d,pixel-aspect-ratio=1/1,framerate=%d/1",
            output_format, color_width_, color_height_, color_fps_);
        GstCaps *sink_caps = gst_caps_from_string(sink_caps_str);
        g_object_set(G_OBJECT(gst_appsink_),
            "caps", sink_caps,
            "emit-signals", FALSE,
            "max-buffers", 1,
            "drop", TRUE,
            "sync", FALSE,
            nullptr);
        gst_caps_unref(sink_caps);

        // Link pipeline elements
        bool link_ok = false;
        if (jpegparse)
        {
            if (nvmm_capsfilter && nvvidconv)
            {
                if (queue)
                {
                    gst_bin_add_many(GST_BIN(gst_pipeline_), gst_appsrc_, jpegparse, decoder, queue, nvmm_capsfilter, nvvidconv, gst_appsink_, nullptr);
                    link_ok = gst_element_link(gst_appsrc_, jpegparse) &&
                              gst_element_link(jpegparse, decoder) &&
                              gst_element_link(decoder, queue) &&
                              gst_element_link(queue, nvmm_capsfilter) &&
                              gst_element_link(nvmm_capsfilter, nvvidconv) &&
                              gst_element_link(nvvidconv, gst_appsink_);
                }
                else
                {
                    gst_bin_add_many(GST_BIN(gst_pipeline_), gst_appsrc_, jpegparse, decoder, nvmm_capsfilter, nvvidconv, gst_appsink_, nullptr);
                    link_ok = gst_element_link(gst_appsrc_, jpegparse) &&
                              gst_element_link(jpegparse, decoder) &&
                              gst_element_link(decoder, nvmm_capsfilter) &&
                              gst_element_link(nvmm_capsfilter, nvvidconv) &&
                              gst_element_link(nvvidconv, gst_appsink_);
                }
            }
            else
            {
                if (queue)
                {
                    gst_bin_add_many(GST_BIN(gst_pipeline_), gst_appsrc_, jpegparse, decoder, queue, videoconvert, gst_appsink_, nullptr);
                    link_ok = gst_element_link(gst_appsrc_, jpegparse) &&
                              gst_element_link(jpegparse, decoder) &&
                              gst_element_link(decoder, queue) &&
                              gst_element_link(queue, videoconvert) &&
                              gst_element_link(videoconvert, gst_appsink_);
                }
                else
                {
                    gst_bin_add_many(GST_BIN(gst_pipeline_), gst_appsrc_, jpegparse, decoder, videoconvert, gst_appsink_, nullptr);
                    link_ok = gst_element_link(gst_appsrc_, jpegparse) &&
                              gst_element_link(jpegparse, decoder) &&
                              gst_element_link(decoder, videoconvert) &&
                              gst_element_link(videoconvert, gst_appsink_);
                }
            }
        }
        else
        {
            if (nvmm_capsfilter && nvvidconv)
            {
                if (queue)
                {
                    gst_bin_add_many(GST_BIN(gst_pipeline_), gst_appsrc_, decoder, queue, nvmm_capsfilter, nvvidconv, gst_appsink_, nullptr);
                    link_ok = gst_element_link(gst_appsrc_, decoder) &&
                              gst_element_link(decoder, queue) &&
                              gst_element_link(queue, nvmm_capsfilter) &&
                              gst_element_link(nvmm_capsfilter, nvvidconv) &&
                              gst_element_link(nvvidconv, gst_appsink_);
                }
                else
                {
                    gst_bin_add_many(GST_BIN(gst_pipeline_), gst_appsrc_, decoder, nvmm_capsfilter, nvvidconv, gst_appsink_, nullptr);
                    link_ok = gst_element_link(gst_appsrc_, decoder) &&
                              gst_element_link(decoder, nvmm_capsfilter) &&
                              gst_element_link(nvmm_capsfilter, nvvidconv) &&
                              gst_element_link(nvvidconv, gst_appsink_);
                }
            }
            else
            {
                if (queue)
                {
                    gst_bin_add_many(GST_BIN(gst_pipeline_), gst_appsrc_, decoder, queue, videoconvert, gst_appsink_, nullptr);
                    link_ok = gst_element_link(gst_appsrc_, decoder) &&
                              gst_element_link(decoder, queue) &&
                              gst_element_link(queue, videoconvert) &&
                              gst_element_link(videoconvert, gst_appsink_);
                }
                else
                {
                    gst_bin_add_many(GST_BIN(gst_pipeline_), gst_appsrc_, decoder, videoconvert, gst_appsink_, nullptr);
                    link_ok = gst_element_link(gst_appsrc_, decoder) &&
                              gst_element_link(decoder, videoconvert) &&
                              gst_element_link(videoconvert, gst_appsink_);
                }
            }
        }

        if (!link_ok)
        {
            RCLCPP_WARN(this->get_logger(), "Failed to link GStreamer elements; disabling GStreamer path");
            gst_element_set_state(gst_pipeline_, GST_STATE_NULL);
            gst_object_unref(gst_pipeline_);
            gst_pipeline_ = nullptr;
            return false;
        }

        sret = gst_element_set_state(gst_pipeline_, GST_STATE_PLAYING);
        if (sret == GST_STATE_CHANGE_FAILURE)
        {
            RCLCPP_WARN(this->get_logger(), "Failed to set GStreamer pipeline to PLAYING; disabling GStreamer path");
            gst_element_set_state(gst_pipeline_, GST_STATE_NULL);
            gst_object_unref(gst_pipeline_);
            gst_pipeline_ = nullptr;
            return false;
        }

        RCLCPP_INFO(this->get_logger(), "GStreamer JPEG decoder initialized (nvvidconv=%s)",
            using_nvvidconv ? "yes" : "no");
        return true;
    #endif
    }

    // --- Color stream worker thread ---

    void MJPEGNode::worker_loop()
    {
        const std::string frame_id = tf_prefix_ + "_rgb_camera_optical_frame";

        while (running_ && rclcpp::ok())
        {
            try
            {
                auto data = video_queue_->get<dai::ImgFrame>();
                if (!data)
                    continue;

                const std::vector<uint8_t> buf = data->getData();

                // Always publish compressed JPEG
                {
                    sensor_msgs::msg::CompressedImage comp_msg;
                    comp_msg.header.stamp = this->now();
                    comp_msg.header.frame_id = frame_id;
                    comp_msg.format = "jpeg";
                    comp_msg.data = buf;
                    pub_compressed_->publish(comp_msg);
                }

                // Skip decode when raw output is not requested
                if (!enable_raw_decode_)
                    continue;

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

                            GstFlowReturn fret = gst_app_src_push_buffer(GST_APP_SRC(gst_appsrc_), gst_buf);
                            if (fret == GST_FLOW_OK)
                            {
                                GstClockTime timeout_ms = (gst_frame_count_ < 10) ? 500 : 100;
                                GstSample *sample = gst_app_sink_try_pull_sample(
                                    GST_APP_SINK(gst_appsink_), GST_MSECOND * timeout_ms);
                                if (sample)
                                {
                                    GstBuffer *outbuf = gst_sample_get_buffer(sample);
                                    if (outbuf)
                                    {
                                        GstMapInfo outmap;
                                        if (gst_buffer_map(outbuf, &outmap, GST_MAP_READ))
                                        {
                                            GstCaps *caps = gst_sample_get_caps(sample);
                                            int width = 0, height = 0;
                                            const gchar *fmt = nullptr;
                                            if (caps)
                                            {
                                                GstStructure *s = gst_caps_get_structure(caps, 0);
                                                fmt = gst_structure_get_string(s, "format");
                                                gst_structure_get_int(s, "width", &width);
                                                gst_structure_get_int(s, "height", &height);
                                            }

                                            if (width > 0 && height > 0 && outmap.data)
                                            {
                                                if (fmt && strcmp(fmt, "RGBA") == 0)
                                                {
                                                    cv::Mat rgba(height, width, CV_8UC4, (void *)outmap.data);
                                                    cv::cvtColor(rgba, mat, cv::COLOR_RGBA2RGB);
                                                }
                                                else if (fmt && strcmp(fmt, "I420") == 0)
                                                {
                                                    cv::Mat yuv(height + height / 2, width, CV_8UC1, (void *)outmap.data);
                                                    mat.create(height, width, CV_8UC3);
                                                    cv::cvtColor(yuv, mat, cv::COLOR_YUV2RGB_I420);
                                                }
                                                else if (fmt && strcmp(fmt, "BGR") == 0)
                                                {
                                                    cv::Mat tmp(height, width, CV_8UC3, (void *)outmap.data);
                                                    cv::cvtColor(tmp, mat, cv::COLOR_BGR2RGB);
                                                }
                                                else
                                                {
                                                    RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 5000,
                                                        "Unknown GStreamer output format '%s', assuming BGR",
                                                        fmt ? fmt : "null");
                                                    cv::Mat tmp(height, width, CV_8UC3, (void *)outmap.data);
                                                    tmp.copyTo(mat);
                                                }
                                                decoded = true;
                                                gst_frame_count_++;
                                            }
                                            gst_buffer_unmap(outbuf, &outmap);
                                        }
                                    }
                                    gst_sample_unref(sample);
                                }
                            }
                        }
                        else
                        {
                            gst_buffer_unref(gst_buf);
                        }
                    }
                }
    #endif
                if (mat.empty())
                {
                    mat = cv::imdecode(buf, cv::IMREAD_COLOR);
                    if (mat.empty())
                    {
                        RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 5000,
                            "Failed to decode MJPEG frame");
                        continue;
                    }
                }

                cv_bridge::CvImage out_cv;
                out_cv.header.stamp = this->now();
                out_cv.header.frame_id = frame_id;
                out_cv.encoding = output_encoding_;
                out_cv.image = mat;
                pub_raw_->publish(*out_cv.toImageMsg());
            }
            catch (const std::exception &e)
            {
                RCLCPP_ERROR(this->get_logger(), "Exception in worker: %s", e.what());
            }
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
