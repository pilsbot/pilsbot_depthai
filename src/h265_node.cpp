extern "C" {
#include <libavcodec/avcodec.h>
#include <libavformat/avformat.h>
#include <libavutil/imgutils.h>
}

#include <opencv2/opencv.hpp>
#include <rclcpp/rclcpp.hpp>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/msg/image.hpp>

#include <depthai/depthai.hpp>

std::tuple<dai::Pipeline, int, int> createPipeline(int previewWidth, int previewHeight, int colorFramerate) {
    dai::Pipeline pipeline;

    auto colorCam = pipeline.create<dai::node::ColorCamera>();
    colorCam->setResolution(dai::ColorCameraProperties::SensorResolution::THE_1080_P);
    colorCam->setFps(colorFramerate);
    colorCam->setPreviewSize(previewWidth, previewHeight);
    colorCam->setInterleaved(false);
    colorCam->setColorOrder(dai::ColorCameraProperties::ColorOrder::RGB);

    auto h265Enc = pipeline.create<dai::node::VideoEncoder>();
    h265Enc->setDefaultProfilePreset(colorCam->getFps(), dai::VideoEncoderProperties::Profile::H265_MAIN);

    auto xoutVid = pipeline.create<dai::node::XLinkOut>();
    xoutVid->setStreamName("h265_video");
    h265Enc->bitstream.link(xoutVid->input);
    colorCam->video.link(h265Enc->input);

    return std::make_tuple(pipeline, 1920, 1080);
}

void decodeH265ToImage(AVCodecContext* codecCtx, AVFrame* frame, AVPacket* pkt, const std::vector<uint8_t>& buffer,
                       const rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr& publisher) {
    pkt->data = const_cast<uint8_t*>(buffer.data());
    pkt->size = buffer.size();

    int ret = avcodec_send_packet(codecCtx, pkt);
    if (ret < 0) {
        RCLCPP_ERROR(rclcpp::get_logger("h265_decode_node"), "Error sending packet for decoding");
        return;
    }

    while (ret >= 0) {
        ret = avcodec_receive_frame(codecCtx, frame);
        if (ret == AVERROR(EAGAIN) || ret == AVERROR_EOF) {
            return;
        } else if (ret < 0) {
            RCLCPP_ERROR(rclcpp::get_logger("h265_decode_node"), "Error during decoding");
            return;
        }

        // Convert decoded frame to OpenCV Mat
        int width = frame->width;
        int height = frame->height;

        // Create a single matrix for YUV420P (I420) data
        cv::Mat rawYUV(height * 3 / 2, width, CV_8UC1);

        // Copy data from AVFrame to cv::Mat
        uint8_t* mat_data = rawYUV.data;
        // Copy Y plane
        int y_data_size = height * frame->linesize[0];
        memcpy(mat_data, frame->data[0], y_data_size);
        
        // Copy U plane
        mat_data += y_data_size;
        int u_data_size = (height / 2) * frame->linesize[1];
        memcpy(mat_data, frame->data[1], u_data_size);

        // Copy V plane
        mat_data += u_data_size;
        int v_data_size = (height / 2) * frame->linesize[2];
        memcpy(mat_data, frame->data[2], v_data_size);

        // Convert YUV to BGR using the correct color conversion code
        cv::Mat rawRGB(height, width, CV_8UC3);

        cv::cvtColor(rawYUV, rawRGB, cv::COLOR_YUV2RGB_YV12);

        // cv::Mat rawRGB(height, width, CV_8UC3);
         
        // cv::cvtColor(rawYUV, rawRGB, cv::COLOR_YUV420sp2RGB); // Convert YUV to BGR

        // Convert OpenCV Mat to ROS2 message
        std_msgs::msg::Header header;
        header.stamp = rclcpp::Clock().now();
        header.frame_id = "camera_frame";

        cv_bridge::CvImage cvImage(header, sensor_msgs::image_encodings::BGR8, rawRGB);
        auto imgMsg = cvImage.toImageMsg();

        publisher->publish(*imgMsg);
    }
}

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<rclcpp::Node>("h265_decode_node");

    int previewWidth = node->declare_parameter<int>("preview_width", 640);
    int previewHeight = node->declare_parameter<int>("preview_height", 480);
    int colorFramerate = node->declare_parameter<int>("color_fps", 30);

    dai::Pipeline pipeline;
    int colorWidth, colorHeight;
    std::tie(pipeline, colorWidth, colorHeight) = createPipeline(previewWidth, previewHeight, colorFramerate);

    dai::Device device(pipeline);
    auto videoQueue = device.getOutputQueue("h265_video", 30, false);

    auto publisher = node->create_publisher<sensor_msgs::msg::Image>("color/image", 10);

    AVCodec* codec = avcodec_find_decoder(AV_CODEC_ID_HEVC);
    AVCodecContext* codecCtx = avcodec_alloc_context3(codec);
    AVFrame* frame = av_frame_alloc();
    AVPacket* pkt = av_packet_alloc();

    if (avcodec_open2(codecCtx, codec, nullptr) < 0) {
        RCLCPP_ERROR(rclcpp::get_logger("h265_decode_node"), "Could not open codec");
        return -1;
    }

    auto timer_callback =
        [videoQueue, publisher, codecCtx, frame, pkt]() -> void {
        auto videoData = videoQueue->get<dai::ImgFrame>();
        std::vector<uint8_t> buffer(videoData->getData().begin(), videoData->getData().end());

        decodeH265ToImage(codecCtx, frame, pkt, buffer, publisher);
    };

    auto timer = node->create_wall_timer(std::chrono::milliseconds(1000 / colorFramerate), timer_callback);

    rclcpp::spin(node);

    avcodec_free_context(&codecCtx);
    av_frame_free(&frame);
    av_packet_free(&pkt);

    rclcpp::shutdown();
    return 0;
}
