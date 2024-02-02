from launch_ros.actions import ComposableNodeContainer, Node
from launch_ros.descriptions import ComposableNode
from launch_ros.substitutions import FindPackageShare
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch.conditions import IfCondition
from launch import LaunchDescription
def generate_launch_description():
    tag_size = LaunchConfiguration('tag_size')
    resolution_width = LaunchConfiguration('resolution_width')
    resolution_height = LaunchConfiguration('resolution_height')
    field_path = LaunchConfiguration('field_path')
    enable_depth = LaunchConfiguration('enable_depth')
    camera_name = LaunchConfiguration('camera_name')
    camera_port = LaunchConfiguration('camera_port')
    fps = LaunchConfiguration('fps')
    launch_camera_server = LaunchConfiguration('launch_camera_server')
    tag_size_argument = DeclareLaunchArgument('tag_size', default_value='0.24')
    resolution_width_argument = DeclareLaunchArgument('resolution_width', default_value='1920')
    resolution_height_argument = DeclareLaunchArgument('resolution_height', default_value='1080')
    field_path_argument = DeclareLaunchArgument('field_path', default_value=PathJoinSubstitution((FindPackageShare('nfr_charged_up'),
        'config', 'field.json')))
    enable_depth_argument = DeclareLaunchArgument('enable_depth', default_value='True')
    camera_name_argument = DeclareLaunchArgument('camera_name', default_value='default')
    camera_port_argument = DeclareLaunchArgument('camera_port', default_value='1181')
    fps_argument = DeclareLaunchArgument('fps', default_value='30')
    launch_camera_server_argument = DeclareLaunchArgument('launch_camera_server', default_value='False')
    realsense_node = ComposableNode(
        package='realsense2_camera',
        plugin='realsense2_camera::RealSenseNodeFactory',
        name='realsense_camera',
        namespace='',
        parameters=[{
            'color_height': resolution_height,
            'color_width': resolution_width,
            'enable_infra1': False,
            'enable_infra2': False,
            'enable_depth': enable_depth,
            'depth_module.global_time_enabled': False
        }],
        remappings=[
            ('color/image_raw', 'image'),
            ('color/camera_info', 'camera_info')
        ]
    )
    rectify_node = ComposableNode(
        package='image_proc',
        plugin='image_proc::RectifyNode',
        name='realsense_rectify',
        namespace='',
        parameters=[{
            'output_width': resolution_width,
            'output_height': resolution_height
        }]
    )
    nfr_apriltag_node = ComposableNode(
        package='nfr_apriltag',
        plugin='nfr::NFRAprilTagNode',
        name='usb_localization',
        namespace='',
        parameters=[{
            'field_path': field_path,
            'size': tag_size,
            'family': '36h11'
        }]
    )
    nfr_depth_finder_node = ComposableNode(
        package='nfr_depth_finder',
        plugin='nfr::NFRDepthFinderNode',
        name='realsense_depth_finder',
        namespace=''
    )
    realsense_container = ComposableNodeContainer(
        package='rclcpp_components',
        name='realsense_container',
        executable='component_container_mt',
        namespace='',
        composable_node_descriptions=[
            nfr_apriltag_node,
            realsense_node,
            nfr_depth_finder_node,
            rectify_node
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
        condition=IfCondition(launch_camera_server)
    )
    return LaunchDescription([
        tag_size_argument,
        resolution_width_argument,
        resolution_height_argument,
        field_path_argument,
        enable_depth_argument,
        camera_name_argument,
        camera_port_argument,
        fps_argument,
        launch_camera_server_argument,
        realsense_container,
        camera_node
    ])