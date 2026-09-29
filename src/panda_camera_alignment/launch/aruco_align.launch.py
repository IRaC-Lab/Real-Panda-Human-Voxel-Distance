from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    launch_args = [
        DeclareLaunchArgument('image_topic', default_value='/camera0/camera/color/image_raw'),
        DeclareLaunchArgument(
            'camera_info_topic', default_value='/camera0/camera/color/camera_info'),
        DeclareLaunchArgument('marker_id', default_value='30'),
        DeclareLaunchArgument('marker_length', default_value='0.05'),
        DeclareLaunchArgument('robot_base_frame', default_value='panda_link0'),
        DeclareLaunchArgument('camera_frame', default_value=''),
        DeclareLaunchArgument(
            'camera_mount_frame', default_value='camera_link',
            description="Root of realsense2_camera's own static TF subtree; "
                        "publish here instead of the optical frame to avoid a "
                        "two-parent conflict with the driver's own TF."),
        DeclareLaunchArgument('rotation_deg', default_value='90.0'),
        DeclareLaunchArgument('translation', default_value='[0.0, -0.25, -0.02]'),
        DeclareLaunchArgument('num_samples', default_value='15'),
        DeclareLaunchArgument('show_image', default_value='True'),
        DeclareLaunchArgument(
            'mode', default_value='static',
            description="'static' (lock once) or 'dynamic' (continuous live TF)"),
        DeclareLaunchArgument('smoothing_alpha', default_value='0.3'),
        DeclareLaunchArgument('publish_rate_hz', default_value='15.0'),
        DeclareLaunchArgument('occlusion_warn_period_sec', default_value='5.0'),
    ]

    node = Node(
        package='panda_camera_alignment',
        executable='aruco_align',
        name='aruco_camera_align',
        output='screen',
        parameters=[{
            'image_topic': LaunchConfiguration('image_topic'),
            'camera_info_topic': LaunchConfiguration('camera_info_topic'),
            'marker_id': LaunchConfiguration('marker_id'),
            'marker_length': LaunchConfiguration('marker_length'),
            'robot_base_frame': LaunchConfiguration('robot_base_frame'),
            'camera_frame': LaunchConfiguration('camera_frame'),
            'camera_mount_frame': LaunchConfiguration('camera_mount_frame'),
            'rotation_deg': LaunchConfiguration('rotation_deg'),
            'translation': LaunchConfiguration('translation'),
            'num_samples': LaunchConfiguration('num_samples'),
            'show_image': LaunchConfiguration('show_image'),
            'mode': LaunchConfiguration('mode'),
            'smoothing_alpha': LaunchConfiguration('smoothing_alpha'),
            'publish_rate_hz': LaunchConfiguration('publish_rate_hz'),
            'occlusion_warn_period_sec': LaunchConfiguration('occlusion_warn_period_sec'),
        }],
    )

    return LaunchDescription(launch_args + [node])
