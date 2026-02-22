from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    # Color camera arguments
    color_resolution_arg = DeclareLaunchArgument('color_resolution', default_value='1080p')
    isp_scale_num_arg = DeclareLaunchArgument('isp_scale_num', default_value='2')
    isp_scale_den_arg = DeclareLaunchArgument('isp_scale_den', default_value='3')
    color_fps_arg = DeclareLaunchArgument('color_fps', default_value='30')
    color_jpeg_quality_arg = DeclareLaunchArgument('color_jpeg_quality', default_value='80')

    # Mono camera arguments
    mono_resolution_arg = DeclareLaunchArgument('mono_resolution', default_value='400p')
    mono_fps_arg = DeclareLaunchArgument('mono_fps', default_value='15')
    mono_jpeg_quality_arg = DeclareLaunchArgument('mono_jpeg_quality', default_value='50')

    # Stereo depth arguments
    enable_depth_arg = DeclareLaunchArgument('enable_depth', default_value='false')
    depth_preset_arg = DeclareLaunchArgument('depth_preset', default_value='HIGH_DENSITY')
    lr_check_arg = DeclareLaunchArgument('lr_check', default_value='true')
    extended_disparity_arg = DeclareLaunchArgument('extended_disparity', default_value='false')
    subpixel_arg = DeclareLaunchArgument('subpixel', default_value='false')
    stereo_confidence_arg = DeclareLaunchArgument('stereo_confidence', default_value='200')

    # General arguments
    enable_raw_decode_arg = DeclareLaunchArgument('enable_raw_decode', default_value='false')
    tf_prefix_arg = DeclareLaunchArgument('tf_prefix', default_value='oak')
    queue_size_arg = DeclareLaunchArgument('queue_size', default_value='4')
    output_encoding_arg = DeclareLaunchArgument('output_encoding', default_value='rgb8')
    use_gstreamer_arg = DeclareLaunchArgument('use_gstreamer', default_value='true')
    nvdecoder_arg = DeclareLaunchArgument('nvdecoder', default_value='')

    mjpeg_node = Node(
        package='pilsbot_depthai',
        executable='mjpeg_node',
        name='mjpeg_node',
        output='screen',
        parameters=[{
            'color_resolution': LaunchConfiguration('color_resolution'),
            'isp_scale_num': LaunchConfiguration('isp_scale_num'),
            'isp_scale_den': LaunchConfiguration('isp_scale_den'),
            'color_fps': LaunchConfiguration('color_fps'),
            'color_jpeg_quality': LaunchConfiguration('color_jpeg_quality'),
            'mono_resolution': LaunchConfiguration('mono_resolution'),
            'mono_fps': LaunchConfiguration('mono_fps'),
            'mono_jpeg_quality': LaunchConfiguration('mono_jpeg_quality'),
            'enable_depth': LaunchConfiguration('enable_depth'),
            'depth_preset': LaunchConfiguration('depth_preset'),
            'lr_check': LaunchConfiguration('lr_check'),
            'extended_disparity': LaunchConfiguration('extended_disparity'),
            'subpixel': LaunchConfiguration('subpixel'),
            'stereo_confidence': LaunchConfiguration('stereo_confidence'),
            'enable_raw_decode': LaunchConfiguration('enable_raw_decode'),
            'tf_prefix': LaunchConfiguration('tf_prefix'),
            'queue_size': LaunchConfiguration('queue_size'),
            'output_encoding': LaunchConfiguration('output_encoding'),
            'use_gstreamer': LaunchConfiguration('use_gstreamer'),
            'nvdecoder': LaunchConfiguration('nvdecoder'),
        }]
    )

    return LaunchDescription([
        color_resolution_arg,
        isp_scale_num_arg,
        isp_scale_den_arg,
        color_fps_arg,
        color_jpeg_quality_arg,
        mono_resolution_arg,
        mono_fps_arg,
        mono_jpeg_quality_arg,
        enable_depth_arg,
        depth_preset_arg,
        lr_check_arg,
        extended_disparity_arg,
        subpixel_arg,
        stereo_confidence_arg,
        enable_raw_decode_arg,
        tf_prefix_arg,
        queue_size_arg,
        output_encoding_arg,
        use_gstreamer_arg,
        nvdecoder_arg,
        mjpeg_node,
    ])
