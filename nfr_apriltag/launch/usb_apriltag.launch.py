from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode
from launch_ros.substitutions import FindPackageShare
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, PythonExpression
from launch import LaunchDescription
def generate_launch_description():
    tag_size = LaunchConfiguration('tag_size')
    resolution_width = LaunchConfiguration('resolution_width')
    resolution_height = LaunchConfiguration('resolution_height')
    camera_path = LaunchConfiguration('camera_path')
    frame_id = LaunchConfiguration('frame_id')
    camera_info_url = LaunchConfiguration('camera_info_url')
    field_path = LaunchConfiguration('field_path')
    tag_size_argument = DeclareLaunchArgument('tag_size', default_value='0.18')
    resolution_width_argument = DeclareLaunchArgument('resolution_width', default_value='1920')
    resolution_height_argument = DeclareLaunchArgument('resolution_height', default_value='1080')
    camera_path_argument = DeclareLaunchArgument('camera_path', default_value='/dev/video0')
    frame_id_argument = DeclareLaunchArgument('frame_id', default_value='camera')
    camera_info_url_argument = DeclareLaunchArgument('camera_info_url', default_value='')
    field_path_argument = DeclareLaunchArgument('field_path', default_value=PathJoinSubstitution((FindPackageShare('nfr_charged_up'),
        'config', 'field.json')))
    rectify_node = ComposableNode(
        package='image_proc',
        plugin='image_proc::RectifyNode',
        name='usb_rectify',
        namespace='',
        parameters=[{
            'output_width': resolution_width,
            'output_height': resolution_height
        }]
    )
    usb_node = ComposableNode(
        package='v4l2_camera',
        plugin='v4l2_camera::V4L2Camera',
        name='usb_camera',
        namespace='',
        parameters=[{
            'video_device': camera_path,
            'image_size': PythonExpression(['[', resolution_width, ',', resolution_height, ']']),
            'camera_frame_id': frame_id,
            'camera_name': frame_id,
            'camera_info_url': camera_info_url
        }]
    )
    apriltag_node = ComposableNode(
        package='apriltag_ros',
        plugin='AprilTagNode',
        name='usb_apriltag',
        namespace='',
        parameters=[{
            'size': tag_size,
            'family': '36h11'
        }],
        remappings=[
            ('camera/image_rect', 'image_rect'),
            ('camera/camera_info', 'camera_info')
        ]
    )
    nfr_apriltag_node = ComposableNode(
        package='nfr_apriltag',
        plugin='nfr::AprilTagLocalizationNode',
        name='realsense_localization',
        namespace='',
        parameters=[{
            'field_path': field_path,
        }]
    )
    usb_container = ComposableNodeContainer(
        package='rclcpp_components',
        name='usb_container',
        executable='component_container_mt',
        namespace='',
        composable_node_descriptions=[
            nfr_apriltag_node,
            rectify_node,
            apriltag_node,
            usb_node
        ]
    )
    return LaunchDescription([
        tag_size_argument,
        resolution_width_argument,
        resolution_height_argument,
        camera_path_argument,
        frame_id_argument,
        camera_info_url_argument,
        field_path_argument,
        usb_container
    ])