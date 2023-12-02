from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode
from launch_ros.substitutions import FindPackageShare
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch import LaunchDescription
def generate_launch_description():
    tag_size = LaunchConfiguration('tag_size')
    resolution_width = LaunchConfiguration('resolution_width')
    resolution_height = LaunchConfiguration('resolution_height')
    field_path = LaunchConfiguration('field_path')
    use_cuda = LaunchConfiguration('use_cuda')
    tag_size_argument = DeclareLaunchArgument('tag_size', default_value='0.24')
    resolution_width_argument = DeclareLaunchArgument('resolution_width', default_value='1920')
    resolution_height_argument = DeclareLaunchArgument('resolution_height', default_value='1080')
    field_path_argument = DeclareLaunchArgument('field_path', default_value=PathJoinSubstitution((FindPackageShare('nfr_charged_up'), 'config', 'field.json')))
    use_cuda_argument = DeclareLaunchArgument('use_cuda', default_value='True')
    rectify_node = ComposableNode(
        package='isaac_ros_image_proc',
        plugin='nvidia::isaac_ros::image_proc::RectifyNode',
        name='realsense_rectify',
        namespace='',
        parameters=[{
            'output_width': resolution_width,
            'output_height': resolution_height
        }]
    )
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
            'enable_depth': False
        }],
        remappings=[
            ('color/image_raw', 'image'),
            ('color/camera_info', 'camera_info')
        ]
    )
    apriltag_node = ComposableNode(
        package='isaac_ros_apriltag',
        plugin='nvidia::isaac_ros::apriltag::AprilTagNode',
        name='realsense_apriltag',
        namespace='',
        parameters=[{
            'size': tag_size
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
            'use_cuda': use_cuda
        }]
    )
    realsense_container = ComposableNodeContainer(
        package='rclcpp_components',
        name='realsense_container',
        executable='component_container_mt',
        namespace='',
        composable_node_descriptions=[
            nfr_apriltag_node,
            rectify_node,
            apriltag_node,
            realsense_node
        ]
    )
    return LaunchDescription([
        tag_size_argument,
        resolution_width_argument,
        resolution_height_argument,
        field_path_argument,
        use_cuda_argument,
        realsense_container
    ])