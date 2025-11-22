// Simple MJPEG node: subscribes to Image and publishes CompressedImage (jpeg)

#include "pilsbot_oakd/mjpeg_node.hpp"

#include <chrono>
#include <opencv2/imgcodecs.hpp>
#include <depthai/depthai.hpp>
#include <atomic>
#include <thread>
#include <mutex>

namespace pilsbot_oakd
{

MJPEGNode::MJPEGNode(const rclcpp::NodeOptions & options)
: Node("mjpeg_node", options)
{
	// Parameters: control DepthAI preview size, fps and JPEG quality
	preview_width_ = this->declare_parameter<int>("preview_width", 640);
	preview_height_ = this->declare_parameter<int>("preview_height", 480);
	color_fps_ = this->declare_parameter<int>("color_fps", 30);
	jpeg_quality_ = this->declare_parameter<int>("jpeg_quality", 80);

	output_topic_ = this->declare_parameter<std::string>("output_topic", "/camera/color/image_raw");
	output_encoding_ = this->declare_parameter<std::string>("output_encoding", "bgr8");

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
	std::lock_guard<std::mutex> lk(mutex_);
	if (running_) return;

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

	try {
		device_ = std::make_shared<dai::Device>(*pipeline);
	} catch (const std::exception & e) {
		RCLCPP_ERROR(this->get_logger(), "Failed to create DepthAI device: %s", e.what());
		return;
	}

	video_queue_ = device_->getOutputQueue("mjpeg", 8, false);

	running_ = true;
	// Start worker thread to fetch encoded MJPEG and publish decoded images
	worker_ = std::thread(&MJPEGNode::worker_loop, this);
}

void MJPEGNode::stop_device()
{
	std::lock_guard<std::mutex> lk(mutex_);
	if (!running_) return;
	running_ = false;
	if (worker_.joinable()) worker_.join();
	if (device_) {
		try { device_->close(); } catch(...) {}
		device_.reset();
	}
}

void MJPEGNode::worker_loop()
{
	while (running_ && rclcpp::ok()) {
		try {
			auto data = video_queue_->get<dai::ImgFrame>();
			if (!data) continue;

			// getData returns encoded JPEG bytes
			const std::vector<uint8_t> buf = data->getData();
			cv::Mat mat = cv::imdecode(buf, cv::IMREAD_COLOR);
			if (mat.empty()) {
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
		} catch (const std::exception & e) {
			RCLCPP_ERROR(this->get_logger(), "Exception in MJPEG worker: %s", e.what());
		}
	}
}

void MJPEGNode::image_callback(const sensor_msgs::msg::CompressedImage::SharedPtr msg)
{
	try {
		// Basic check: ensure format contains 'jpeg' or 'jpg'
		if (msg->format.find("jpeg") == std::string::npos && msg->format.find("jpg") == std::string::npos) {
			RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 5000,
													 "Received compressed image with unexpected format: '%s'", msg->format.c_str());
		}

		// Decode JPEG buffer into cv::Mat
		std::vector<unsigned char> data(msg->data.begin(), msg->data.end());
		cv::Mat mat = cv::imdecode(data, cv::IMREAD_COLOR);
		if (mat.empty()) {
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
	} catch (const cv_bridge::Exception & e) {
		RCLCPP_ERROR(this->get_logger(), "cv_bridge exception: %s", e.what());
	} catch (const std::exception & e) {
		RCLCPP_ERROR(this->get_logger(), "Exception in MJPEG callback: %s", e.what());
	}
}

} // namespace pilsbot_oakd

int main(int argc, char ** argv)
{
	rclcpp::init(argc, argv);
	rclcpp::NodeOptions opts;
	auto node = std::make_shared<pilsbot_oakd::MJPEGNode>(opts);
	rclcpp::spin(node);
	rclcpp::shutdown();
	return 0;
}
