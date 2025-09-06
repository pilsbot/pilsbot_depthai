#include <mutex>
#include <thread>
#include <atomic>
extern "C" {
#include <libavcodec/avcodec.h>
#include <libavformat/avformat.h>
#include <libavutil/imgutils.h>
#include <libavutil/hwcontext.h>
#include <libavutil/opt.h>
#include <libswscale/swscale.h>
}

#include <opencv2/opencv.hpp>
#include <rclcpp/rclcpp.hpp>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/msg/image.hpp>
#include <depthai/depthai.hpp>

std::tuple<dai::Pipeline, int, int> createPipeline(int previewWidth, int previewHeight, int colorFramerate, std::string video_codec) {
    dai::Pipeline pipeline;
    auto colorCam = pipeline.create<dai::node::ColorCamera>();
    colorCam->setResolution(dai::ColorCameraProperties::SensorResolution::THE_1080_P);
    colorCam->setFps(colorFramerate);
    colorCam->setPreviewSize(previewWidth, previewHeight);
    colorCam->setInterleaved(false);
    colorCam->setColorOrder(dai::ColorCameraProperties::ColorOrder::BGR);
    
    auto encoder = pipeline.create<dai::node::VideoEncoder>();
    if (video_codec == "h264")
        encoder->setDefaultProfilePreset(colorCam->getFps(), dai::VideoEncoderProperties::Profile::H264_MAIN);
    else
        encoder->setDefaultProfilePreset(colorCam->getFps(), dai::VideoEncoderProperties::Profile::H265_MAIN);
    encoder->setKeyframeFrequency(colorFramerate*2);
    // encoder->setBitrateKbps(1500);

    auto xoutVid = pipeline.create<dai::node::XLinkOut>();
    xoutVid->setStreamName("video");
    xoutVid->input.queueSize(1);
    encoder->bitstream.link(xoutVid->input);
    colorCam->video.link(encoder->input);

    return std::make_tuple(pipeline, 1920, 1080);
}

AVBufferRef* hw_device_ctx = nullptr;

AVPixelFormat get_hw_format(AVCodecContext* ctx, const AVPixelFormat* pix_fmts) {
    for (const AVPixelFormat* p = pix_fmts; *p != -1; p++) {
        if (*p == AV_PIX_FMT_CUDA || *p == AV_PIX_FMT_VAAPI || *p == AV_PIX_FMT_QSV) {
            return *p;
        }
    }
    RCLCPP_ERROR(rclcpp::get_logger("h265_decode_node"), "Failed to get HW surface format.");
    return AV_PIX_FMT_NONE;
}

void decodeH265ToImage(AVCodecContext* codecCtx, AVFrame* frame, AVFrame* cpuFrame, AVPacket* pkt, AVFrame* colorFrame, uint64_t pts, const std::vector<uint8_t>& buffer,
                       const rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr& publisher) {

    if (buffer.empty()) {
        // Don't send empty packets
        return;
    }

    // Basic buffer validation: check for NAL start code (0x00 00 01 or 0x00 00 00 01)
    bool has_nal_start = false;
    for (size_t i = 0; i + 3 < buffer.size(); ++i) {
        if ((buffer[i] == 0x00 && buffer[i+1] == 0x00 && buffer[i+2] == 0x01) ||
            (i + 4 < buffer.size() && buffer[i] == 0x00 && buffer[i+1] == 0x00 && buffer[i+2] == 0x00 && buffer[i+3] == 0x01)) {
            has_nal_start = true;
            break;
        }
    }
    if (!has_nal_start) {
        RCLCPP_WARN(rclcpp::get_logger("h265_decode_node"), "Buffer does not contain a valid NAL start code. Skipping decode.");
        return;
    }

    // Debug: print first 8 bytes and NAL unit types
    std::ostringstream oss;
    oss << "Buffer size: " << buffer.size() << ", first bytes: ";
    for (size_t i = 0; i < std::min<size_t>(8, buffer.size()); ++i) {
        oss << std::hex << std::setw(2) << std::setfill('0') << (int)buffer[i] << " ";
    }
    for (size_t i = 0; i + 5 < buffer.size(); ++i) {
        if ((buffer[i] == 0x00 && buffer[i+1] == 0x00 && buffer[i+2] == 0x01) ||
            (buffer[i] == 0x00 && buffer[i+1] == 0x00 && buffer[i+2] == 0x00 && buffer[i+3] == 0x01)) {
            size_t nal_start = (buffer[i+2] == 0x01) ? i+3 : i+4;
            uint8_t nal_unit_header = buffer[nal_start];
            uint8_t nal_type = (nal_unit_header >> 1) & 0x3F;
            oss << "[NAL type: " << (int)nal_type << "] ";
        }
    }
    RCLCPP_DEBUG(rclcpp::get_logger("h265_decode_node"), "%s", oss.str().c_str());

    // Allocate and copy buffer for AVPacket to avoid lifetime/corruption issues
    // if (pkt->data) {
    //     av_packet_unref(pkt);
    // }
    if (av_new_packet(pkt, buffer.size() + AV_INPUT_BUFFER_PADDING_SIZE) < 0) {
        RCLCPP_ERROR(rclcpp::get_logger("h265_decode_node"), "Failed to allocate AVPacket");
        return;
    }
    // Could we optimize it away to avoid this copy?
    memcpy(pkt->data, buffer.data(), buffer.size());
    pkt->pts = pts;
    pkt->dts = pts;

    int ret = avcodec_send_packet(codecCtx, pkt);
    if (ret < 0) {
        av_packet_unref(pkt);
        RCLCPP_ERROR(rclcpp::get_logger("h265_decode_node"), "Error sending packet for decoding");
        return;
    }

    while (ret >= 0) {
        ret = avcodec_receive_frame(codecCtx, frame);
        if (ret == AVERROR(EAGAIN) || ret == AVERROR_EOF) {
            return;
        } else if (ret < 0) {
            av_packet_unref(pkt);
            RCLCPP_ERROR(rclcpp::get_logger("h265_decode_node"), "Error during decoding");
            return;
        }

        AVFrame* sw_frame;
        #if LIBAVUTIL_VERSION_INT >= AV_VERSION_INT(56, 31, 100)
        if (frame->format == AV_PIX_FMT_CUDA || frame->format == AV_PIX_FMT_VAAPI || frame->format == AV_PIX_FMT_QSV) {
            // sw_frame = av_frame_alloc();
            if (av_hwframe_transfer_data(cpuFrame, frame, 0) < 0) {
                RCLCPP_ERROR(rclcpp::get_logger("h265_decode_node"), "Error transferring HW frame to system memory");
                av_frame_free(&sw_frame);
                continue;
            }
            sw_frame = cpuFrame;
        }
        else {
            sw_frame = frame;
        }
        #endif

        int width = sw_frame->width;
        int height = sw_frame->height;
        RCLCPP_DEBUG(rclcpp::get_logger("h265_decode_node"), "Decoded frame format: %d" 
                   "x%d, pixel format: %s", width, height, av_get_pix_fmt_name((AVPixelFormat)sw_frame->format));
        SwsContext* swsContext = sws_getContext(
            width, height, (AVPixelFormat)sw_frame->format,
            width, height, (AVPixelFormat)colorFrame->format,
            SWS_FAST_BILINEAR | SWS_ACCURATE_RND, nullptr, nullptr, nullptr);
        if (!swsContext) {
            RCLCPP_ERROR(rclcpp::get_logger("h265_decode_node"), "cannot allocate sws context!");
            return;
        }

        // bool libav = true;

        // cv::Mat rawYUV = cv::Mat(height * 3 / 2, width, CV_8UC1);
        // if (libav) {
            sensor_msgs::msg::Image::SharedPtr image(new sensor_msgs::msg::Image());
            image->height = frame->height;
            image->width = frame->width;
            image->step = image->width * 3;  // 3 bytes per pixel
            image->encoding = sensor_msgs::image_encodings::BGR8;
            image->data.resize(image->step * image->height);

            av_image_fill_arrays(
                colorFrame->data, colorFrame->linesize, &(image->data[0]),
                (AVPixelFormat)colorFrame->format, sw_frame->width, sw_frame->height, 1);
            sws_scale(
                swsContext, sw_frame->data, sw_frame->linesize, 0,            // src
                codecCtx->height, colorFrame->data, colorFrame->linesize);
            publisher->publish(*image);
        // } else {
        //     // fallback: copy data
        //     rawYUV = cv::Mat(height * 3 / 2, width, CV_8UC1);
        //     uint8_t* mat_data = rawYUV.data;
        //     int y_data_size = height * sw_frame->linesize[0];
        //     memcpy(mat_data, sw_frame->data[0], y_data_size);
        //     mat_data += y_data_size;
        //     int u_data_size = (height / 2) * sw_frame->linesize[1];
        //     memcpy(mat_data, sw_frame->data[1], u_data_size);
        //     mat_data += u_data_size;
        //     int v_data_size = (height / 2) * sw_frame->linesize[2];
        //     memcpy(mat_data, sw_frame->data[2], v_data_size);

        //     cv::Mat rawRGB(height, width, CV_8UC3);
        //     if (sw_frame->format == AV_PIX_FMT_YUV420P) {
        //         cv::cvtColor(rawYUV, rawRGB, cv::COLOR_YUV2RGB_YV12);
        //     } else {
        //         // Fallback for other YUV formats
        //         cv::cvtColor(rawYUV, rawRGB, cv::COLOR_YUV2BGR_NV12);
        //     }
        //     std_msgs::msg::Header header;
        //     header.stamp = rclcpp::Clock().now();
        //     header.frame_id = "camera_frame";
        //     cv_bridge::CvImage cvImage(header, sensor_msgs::image_encodings::BGR8, rawRGB);
        //     auto imgMsg = cvImage.toImageMsg();
        //     publisher->publish(*imgMsg);
        //  }

        // #if LIBAVUTIL_VERSION_INT >= AV_VERSION_INT(56, 31, 100)
        // if (sw_frame != frame) {
        //     av_frame_free(&sw_frame);
        // }
        // #endif
    }
    av_packet_unref(pkt);
}

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<rclcpp::Node>("h265_decode_node");

    int previewWidth = node->declare_parameter<int>("preview_width", 640);
    int previewHeight = node->declare_parameter<int>("preview_height", 480);
    int colorFramerate = node->declare_parameter<int>("color_fps", 30);
    std::string hw_device_type = node->declare_parameter<std::string>("hw_device_type", "none");
    std::string video_codec = node->declare_parameter<std::string>("video_codec", "h264");

    if (hw_device_type != "none") {
        AVHWDeviceType device_type = av_hwdevice_find_type_by_name(hw_device_type.c_str());
        if (device_type == AV_HWDEVICE_TYPE_NONE) {
            RCLCPP_ERROR(node->get_logger(), "Hardware device type %s is not supported.", hw_device_type.c_str());
            return -1;
        }

        if (av_hwdevice_ctx_create(&hw_device_ctx, device_type, nullptr, nullptr, 0) < 0) {
            RCLCPP_ERROR(node->get_logger(), "Failed to create specified HW device.");
            return -1;
        }
    }

    AVCodec* codec = nullptr;
    if (video_codec != "h264" && video_codec != "h265") {
        RCLCPP_ERROR(node->get_logger(), "Unsupported video codec: %s. Use 'h264' or 'h265'.", video_codec.c_str());
        return -1;
    }
    else if (video_codec == "h264") {
        RCLCPP_INFO(node->get_logger(), "Using H.264 video codec.");
        codec = avcodec_find_decoder(AV_CODEC_ID_H264);

    }
    else {
        RCLCPP_INFO(node->get_logger(), "Using H.265 video codec.");
        codec = avcodec_find_decoder(AV_CODEC_ID_H265);
    }

    AVCodecContext* codecCtx = avcodec_alloc_context3(codec);
    AVFrame * colorFrame{NULL};
    colorFrame = av_frame_alloc();
    colorFrame->format = AV_PIX_FMT_BGR24;

    dai::Pipeline pipeline;
    int colorWidth, colorHeight;
    std::tie(pipeline, colorWidth, colorHeight) = createPipeline(previewWidth, previewHeight, colorFramerate, video_codec);
    dai::Device device(pipeline);

    static std::mutex buffer_mutex;
    auto videoQueue = device.getOutputQueue("h265_video", 30, false);
    auto publisher = node->create_publisher<sensor_msgs::msg::Image>("color/video/image", 10);

    if (!codec || !codecCtx) {
        RCLCPP_ERROR(node->get_logger(), "Failed to allocate codec or codec context.");
        return -1;
    }

    if (hw_device_type != "none") {
        codecCtx->get_format = get_hw_format;
        codecCtx->hw_device_ctx = av_buffer_ref(hw_device_ctx);
    }

    if (avcodec_open2(codecCtx, codec, nullptr) < 0) {
        RCLCPP_ERROR(node->get_logger(), "Could not open codec");
        return -1;
    }
    AVFrame* frame = av_frame_alloc();
    AVPacket* pkt = av_packet_alloc();
    AVFrame* cpuFrame = (hw_device_ctx == nullptr) ? nullptr : av_frame_alloc();

    std::atomic<bool> running{true};
    std::thread worker([&]() {
        while (running && rclcpp::ok()) {
            std::lock_guard<std::mutex> lock(buffer_mutex);
            auto videoData = videoQueue->get<dai::ImgFrame>(); // blocks until frame available
            if (!videoData) continue;
            // std::vector<uint8_t> buffer(videoData->getData().begin(), videoData->getData().end());
            // Get PTS from dai::ImgFrame timestamp (in nanoseconds)
            auto ts = videoData->getTimestamp();
            uint64_t pts = std::chrono::duration_cast<std::chrono::nanoseconds>(ts.time_since_epoch()).count();
            decodeH265ToImage(codecCtx, frame, cpuFrame, pkt, colorFrame, pts, videoData->getData(), publisher);
        }
    });

    rclcpp::spin(node);
    running = false;
    if (worker.joinable()) worker.join();

    avcodec_free_context(&codecCtx);
    av_buffer_unref(&hw_device_ctx);
    av_frame_free(&frame);
    av_frame_free(&cpuFrame);
    av_frame_free(&colorFrame);
    av_packet_free(&pkt);
    rclcpp::shutdown();
    return 0;
}
