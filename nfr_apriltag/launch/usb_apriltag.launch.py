from launch_ros.actions import ComposableNodeContainer, Node
from launch_ros.descriptions import ComposableNode
from launch_ros.substitutions import FindPackageShare
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, PythonExpression
from launch.conditions import IfCondition
from launch import LaunchDescription
def generate_launch_description():
    tag_size = LaunchConfiguration('tag_size')
    resolution_width = LaunchConfiguration('resolution_width')
    resolution_height = LaunchConfiguration('resolution_height')
    field_path = LaunchConfiguration('field_path')
    camera_name = LaunchConfiguration('camera_name')
    camera_port = LaunchConfiguration('camera_port')
    fps = LaunchConfiguration('fps')
    launch_camera_server = LaunchConfiguration('launch_camera_server')
    camera_path = LaunchConfiguration('camera_path')
    frame_id = LaunchConfiguration('frame_id')
    camera_info_url = LaunchConfiguration('camera_info_url')
    pixel_format = LaunchConfiguration('pixel_format')
    camera_frame = LaunchConfiguration('camera_frame')
    tag_size_argument = DeclareLaunchArgument('tag_size', default_value='0.165')
    resolution_width_argument = DeclareLaunchArgument('resolution_width', default_value='1280')
    resolution_height_argument = DeclareLaunchArgument('resolution_height', default_value='720')
    field_path_argument = DeclareLaunchArgument('field_path', default_value=PathJoinSubstitution((FindPackageShare('nfr_charged_up'),
        'config', 'field.json')))
    camera_name_argument = DeclareLaunchArgument('camera_name', default_value='default')
    camera_port_argument = DeclareLaunchArgument('camera_port', default_value='1181')
    fps_argument = DeclareLaunchArgument('fps', default_value='8')
    launch_camera_server_argument = DeclareLaunchArgument('launch_camera_server', default_value='False')
    camera_path_argument = DeclareLaunchArgument('camera_path', default_value='/dev/video0')
    frame_id_argument = DeclareLaunchArgument('frame_id', default_value='camera')
    camera_info_url_argument = DeclareLaunchArgument('camera_info_url', default_value='')
    pixel_format_argument = DeclareLaunchArgument('pixel_format', default_value='YUYV')
    camera_frame_argument = DeclareLaunchArgument('camera_frame', default_value='usb_link')
    usb_node = ComposableNode(
        package='v4l2_camera',
        plugin='v4l2_camera::V4L2Camera',
        name='v4l2_camera',
        namespace='',
        parameters=[{
            'video_device': camera_path,
            'image_size': PythonExpression(['[', resolution_width, ',', resolution_height, ']']),
            'camera_frame_id': frame_id,
            'camera_name': frame_id,
            'camera_info_url': camera_info_url,
            'pixel_format': pixel_format
        }],
        remappings=[
            ('image_raw', 'image')
        ]
    )
    nfr_apriltag_node = ComposableNode(
        package='nfr_apriltag',
        plugin='nfr::NFRAprilTagNode',
        name='realsense_apriltag',
        namespace='',
        parameters=[{
            'size': tag_size,
            'family': '36h11'
        }]
    )
    nfr_apriltag_localization_node = ComposableNode(
        package='nfr_apriltag_localization',
        plugin='nfr::NFRAprilTagLocalizationNode',
        name='realsense_apriltag_localization',
        namespace='',
        parameters=[{
            'field_path': field_path,
            'size': tag_size,
            'use_multi_tag_pnp': True,
            'camera_frame': camera_frame
        }]
    )
    nfr_target_finder_node = ComposableNode(
        package='nfr_target_finder',
        plugin='nfr::NFRTargetFinderNode',
        name='realsense_depth_finder_node',
        namespace='',
        parameters=[{
            'camera_frame': camera_frame,
            'use_depth_subscription': False
        }]
    )
    realsense_container = ComposableNodeContainer(
        package='rclcpp_components',
        name='realsense_container',
        executable='component_container_mt',
        namespace='',
        composable_node_descriptions=[
            nfr_apriltag_node,
            usb_node,
            nfr_apriltag_localization_node,
            nfr_target_finder_node
        ]
    )
    camera_node = Node(
        package='nfr_camera',
        name='nfr_camera_node',
        executable='nfr_camera_node',
        namespace='',
        parameters=[{
            'camera_name': camera_name,
            'resolution_width': resolution_width,
            'resolution_height': resolution_height,
            'camera_port': camera_port,
            'fps': fps
        }],
        condition=IfCondition(launch_camera_server),
        remappings=[
            ("image", "image_rect")
        ]
    )
    return LaunchDescription([
        tag_size_argument,
        resolution_width_argument,
        resolution_height_argument,
        field_path_argument,
        camera_name_argument,
        camera_port_argument,
        fps_argument,
        launch_camera_server_argument,
        camera_path_argument,
        frame_id_argument,
        camera_info_url_argument,
        pixel_format_argument,
        camera_frame_argument,
        realsense_container,
        camera_node
    ])
