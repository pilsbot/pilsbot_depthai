cmake_minimum_required(VERSION 3.5)
project(pilsbot_depthai)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)

# Try to find GStreamer (optional). If found we will compile with GStreamer support
find_package(PkgConfig REQUIRED)
pkg_check_modules(GST REQUIRED gstreamer-1.0 gstreamer-app-1.0 QUIET)
if(GST_FOUND)
  message(STATUS "Found GStreamer: ${GST_VERSION}")
  add_compile_definitions(HAVE_GSTREAMER=1)
endif()

find_package(backward_ros REQUIRED)
find_package(camera_info_manager REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(depthai CONFIG REQUIRED)
find_package(depthai_ros_msgs REQUIRED)
find_package(depthai_bridge REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(stereo_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(vision_msgs REQUIRED)

set(dependencies
  camera_info_manager
  cv_bridge
  depthai_ros_msgs
  depthai_bridge
  rclcpp
  sensor_msgs
  stereo_msgs
  std_msgs
  vision_msgs
)

include_directories(
  include
  ${ament_INCLUDE_DIRS}
)

SET(LINKER_LIBS
  depthai::core
  opencv_imgproc
  opencv_highgui
  avcodec 
  avformat 
  avutil 
  avdevice 
  avfilter 
  swscale
)
if(GST_FOUND)
  list(APPEND LINKER_LIBS ${GST_LIBRARIES})
endif()

macro(dai_add_node_ros2 node_name node_src)
  add_executable("${node_name}" "${node_src}")

  target_link_libraries("${node_name}" ${LINKER_LIBS})
  if(GST_FOUND)
    target_include_directories("${node_name}" PRIVATE ${GST_INCLUDE_DIRS})
  endif()

  ament_target_dependencies("${node_name}"
                            ${dependencies})

endmacro()

dai_add_node_ros2(rgb_stereo_node src/rgb_stereo_node.cpp)
# dai_add_node_ros2(h265_node src/h265_node.cpp)
dai_add_node_ros2(mjpeg_node src/mjpeg_node.cpp)


if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # uncomment the line when a copyright and license is not present in all source files
  #set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # uncomment the line when this package is not in a git repo
  #set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

install(TARGETS
  rgb_stereo_node
  # h265_node
  mjpeg_node
        DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY launch/ DESTINATION share/${PROJECT_NAME}/launch
        FILES_MATCHING PATTERN "*.py")

ament_package()
