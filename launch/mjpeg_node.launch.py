from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    # Launch arguments (can be overridden at runtime)
    preview_width_arg = DeclareLaunchArgument('preview_width', default_value='640')
    preview_height_arg = DeclareLaunchArgument('preview_height', default_value='480')
    color_fps_arg = DeclareLaunchArgument('color_fps', default_value='30')
    jpeg_quality_arg = DeclareLaunchArgument('jpeg_quality', default_value='80')
    output_topic_arg = DeclareLaunchArgument('output_topic', default_value='/camera/color/image_raw')
    output_encoding_arg = DeclareLaunchArgument('output_encoding', default_value='bgr8')
    use_gstreamer_arg = DeclareLaunchArgument('use_gstreamer', default_value='true')
    nvdecoder_arg = DeclareLaunchArgument('nvdecoder', default_value='')

    preview_width = LaunchConfiguration('preview_width')
    preview_height = LaunchConfiguration('preview_height')
    color_fps = LaunchConfiguration('color_fps')
    jpeg_quality = LaunchConfiguration('jpeg_quality')
    output_topic = LaunchConfiguration('output_topic')
    output_encoding = LaunchConfiguration('output_encoding')
    use_gstreamer = LaunchConfiguration('use_gstreamer')
    nvdecoder = LaunchConfiguration('nvdecoder')

    mjpeg_node = Node(
        package='pilsbot_depthai',
        executable='mjpeg_node',
        name='mjpeg_node',
        output='screen',
        # prefix=['gdb -ex run --args'],
        parameters=[{
            'preview_width': preview_width,
            'preview_height': preview_height,
            'color_fps': color_fps,
            'jpeg_quality': jpeg_quality,
            'output_topic': output_topic,
            'output_encoding': output_encoding,
            'use_gstreamer': use_gstreamer,
            'nvdecoder': nvdecoder,
        }]
    )

    return LaunchDescription([
        preview_width_arg,
        preview_height_arg,
        color_fps_arg,
        jpeg_quality_arg,
        output_topic_arg,
        output_encoding_arg,
        use_gstreamer_arg,
        nvdecoder_arg,
        mjpeg_node
    ])
