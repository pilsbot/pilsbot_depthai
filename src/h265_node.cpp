#include <chrono>
#include <cstdio>
#include <functional>
#include <iostream>
#include <tuple>

#include "rclcpp/rclcpp.hpp"

#include <camera_info_manager/camera_info_manager.hpp>
#include <sensor_msgs/msg/image.hpp>

#include <depthai_bridge/BridgePublisher.hpp>
#include <depthai_bridge/ImageConverter.hpp>

#include <depthai/depthai.hpp>

std::tuple<dai::Pipeline, int, int> createPipeline(
  bool lrcheck,
  bool extended,
  bool subpixel,
  int confidence,
  int LRchecktresh,
  bool useVideo,
  bool usePreview,
  int previewWidth,
  int previewHeight,
  std::string mResolution,
  std::string cResolution,
  int mFramerate,
  int cFramerate)
{
  dai::Pipeline pipeline;

  auto colorCam = pipeline.create<dai::node::ColorCamera>();
  // auto monoLeft = pipeline.create<dai::node::MonoCamera>();
  // auto monoRight = pipeline.create<dai::node::MonoCamera>();
  dai::ColorCameraProperties::SensorResolution colorResolution;
  if (cResolution == "480p" || cResolution == "720p" || cResolution == "1080p") {
    colorResolution = dai::ColorCameraProperties::SensorResolution::THE_1080_P;
  } else if (cResolution == "4K") {
    colorResolution = dai::ColorCameraProperties::SensorResolution::THE_4_K;
  }

  colorCam->setResolution(colorResolution);
  if (cResolution == "480p") {
    colorCam->setVideoSize(848, 480);
    colorCam->setIspScale(4, 5);
  } else if (cResolution == "720p") {
    colorCam->setVideoSize(1280, 720);
    colorCam->setIspScale(2, 3);
  } else if (cResolution == "1080p") {
    colorCam->setVideoSize(1920, 1080);
  } else {
    colorCam->setVideoSize(3840, 2160);
  }

  colorCam->setPreviewSize(previewWidth, previewHeight);
  colorCam->setInterleaved(false);
  colorCam->setColorOrder(dai::ColorCameraProperties::ColorOrder::BGR);
  colorCam->setFps(cFramerate);

  auto h265Enc = pipeline.create<dai::node::VideoEncoder>();
  auto xoutImg = pipeline.create<dai::node::XLinkOut>();
  xoutImg->setStreamName("video");
  xoutImg->input.setQueueSize(1);

  h265Enc->setDefaultProfilePreset(
    colorCam->getFps(), dai::VideoEncoderProperties::Profile::H265_MAIN);

  colorCam->video.link(h265Enc->input);
  h265Enc->bitstream.link(xoutImg->input);

  return std::make_tuple(pipeline, 0, 0);
}

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<rclcpp::Node>("h265_decode_node");

  std::string tfPrefix, monoResolution, colorResolution;
  bool lrcheck, extended, subpixel;
  bool useVideo, usePreview, useDepth;
  int confidence, LRchecktresh, previewWidth, previewHeight, monoFramerate, colorFramerate;
  float dotProjectormA, floodLightmA;

  node->declare_parameter("tf_prefix", "oak");
  node->declare_parameter("lrcheck", true);
  node->declare_parameter("extended", false);
  node->declare_parameter("subpixel", true);
  node->declare_parameter("confidence", 200);
  node->declare_parameter("LRchecktresh", 5);
  node->declare_parameter("monoResolution", "480p");
  node->declare_parameter("monoFramerate", 30);
  node->declare_parameter("colorResolution", "1080p");
  node->declare_parameter("colorFramerate", 30);
  node->declare_parameter("useVideo", true);
  node->declare_parameter("usePreview", false);
  node->declare_parameter("useDepth", true);
  node->declare_parameter("previewWidth", 300);
  node->declare_parameter("previewHeight", 300);
  node->declare_parameter("dotProjectormA", 0.0f);
  node->declare_parameter("floodLightmA", 0.0f);

  node->get_parameter("tf_prefix", tfPrefix);
  node->get_parameter("lrcheck", lrcheck);
  node->get_parameter("extended", extended);
  node->get_parameter("subpixel", subpixel);
  node->get_parameter("confidence", confidence);
  node->get_parameter("LRchecktresh", LRchecktresh);
  node->get_parameter("monoResolution", monoResolution);
  node->get_parameter("monoFramerate", monoFramerate);
  node->get_parameter("colorResolution", colorResolution);
  node->get_parameter("colorFramerate", colorFramerate);
  node->get_parameter("useVideo", useVideo);
  node->get_parameter("usePreview", usePreview);
  node->get_parameter("useDepth", useDepth);
  node->get_parameter("previewWidth", previewWidth);
  node->get_parameter("previewHeight", previewHeight);
  node->get_parameter("dotProjectormA", dotProjectormA);
  node->get_parameter("floodLightmA", floodLightmA);

  int colorWidth, colorHeight;
  if (colorResolution == "480p") {
    colorWidth = 848;
    colorHeight = 480;
  } else if (colorResolution == "720p") {
    colorWidth = 1280;
    colorHeight = 720;
  } else if (colorResolution == "1080p") {
    colorWidth = 1920;
    colorHeight = 1080;
  } else if (colorResolution == "4K") {
    colorWidth = 3840;
    colorHeight = 2160;
  } else {
    RCLCPP_ERROR(
      rclcpp::get_logger(
        "rclcpp"), "Invalid parameter. -> colorResolution: %s", colorResolution.c_str());
    throw std::runtime_error("Invalid color camera resolution.");
  }

  dai::Pipeline pipeline;
  int monoWidth, monoHeight;
  std::tie(pipeline, monoWidth, monoHeight) = createPipeline(
    lrcheck, extended, subpixel, confidence, LRchecktresh, useVideo, usePreview, previewWidth,
    previewHeight, monoResolution, colorResolution, monoFramerate, colorFramerate);
  dai::Device device(pipeline);
  
  auto videoQueue = device.getOutputQueue("video", 30, false);

  auto calibrationHandler = device.readCalibration();

  auto boardName = calibrationHandler.getEepromData().boardName;
  if (monoHeight > 480 && boardName == "OAK-D-LITE") {
    monoWidth = 640;
    monoHeight = 480;
  }

  std::unique_ptr<dai::rosBridge::BridgePublisher<sensor_msgs::msg::Image,
    dai::ImgFrame>> depthPublish, rgbPreviewPublish, rgbPublish;

  dai::rosBridge::ImageConverter rgbConverter(tfPrefix + "_rgb_camera_optical_frame", true);

  auto videoCameraInfo = rgbConverter.calibrationToCameraInfo(
    calibrationHandler,
    dai::CameraBoardSocket::RGB,
    colorWidth, colorHeight);

  rgbPublish = std::make_unique<dai::rosBridge::BridgePublisher<sensor_msgs::msg::Image,
      dai::ImgFrame>>(
    videoQueue,
    node,
    std::string("color/video/image"),
    std::bind(
      &dai::rosBridge::ImageConverter::toRosMsg,
      &rgbConverter,                  // since the converter has the same frame name
                                      // and image type is also same we can reuse it
      std::placeholders::_1,
      std::placeholders::_2),
    colorFramerate,
    videoCameraInfo,
    "color/video");
  
  rgbPublish->addPublisherCallback();

  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
