from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource, AnyLaunchDescriptionSource
from launch_ros.actions import Node, ComposableNodeContainer
from launch_ros.descriptions import ComposableNode
from ament_index_python.packages import get_package_share_directory
import os
def generate_launch_description():
    with open(os.path.join(get_package_share_directory('nfr_charged_up'), 'config', 'squishy.urdf')) as f:
        robot_desc = f.read()
    navigation_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [
                os.path.join(get_package_share_directory('nav2_bringup'), 'launch'),
                '/navigation_launch.py'
            ]
        ),
        launch_arguments=[('use_sim_time', 'True')]
    )
    rosbridge_launch = IncludeLaunchDescription(
        AnyLaunchDescriptionSource(
            [
                os.path.join(get_package_share_directory('rosbridge_server'), 'launch'),
                '/rosbridge_websocket_launch.xml'
            ]
        ),
        launch_arguments=[('port', '5810')]
    )
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        parameters=[{
            'robot_description': robot_desc,
            'use_sim_time': True
        }]
    )
    rectify_node = ComposableNode(
        package='image_proc',
        plugin='image_proc::RectifyNode',
        name='realsense_rectify',
        namespace='realsense',
        parameters=[{
            'output_width': 1920,
            'output_height': 1080,
            'use_sim_time': True
        }]
    )
    realsense_node = ComposableNode(
        package='realsense2_camera',
        plugin='realsense2_camera::RealSenseNodeFactory',
        name='realsense_camera',
        namespace='realsense',
        parameters=[{
            'color_height': 1080,
            'color_width': 1920,
            'enable_infra1': False,
            'enable_infra2': False,
            'enable_depth': False,
            'use_sim_time': True
        }],
        remappings=[
            ('color/image_raw', 'image'),
            ('color/camera_info', 'camera_info')
        ]
    )
    apriltag_node = ComposableNode(
        package='apriltag_ros',
        plugin='AprilTagNode',
        name='realsense_apriltag',
        namespace='realsense',
        parameters=[{
            'family': '16h5',
            'size': 0.18,
            'use_sim_time': True
        }]
    )
    realsense_container = ComposableNodeContainer(
        package='rclcpp_components',
        name='realsense_container',
        namespace='realsense',
        executable='component_container_mt',
        composable_node_descriptions=[
            rectify_node,
            apriltag_node,
            realsense_node,
        ]
    )
    robot_localization = Node(
        package='robot_localization',
        executable='ekf_node',
        name='ekf_node',
        parameters=[{
            'odom0': 'odom',
            'odom0_config': [
                True, True, False, False, False, True,
                True, True, False, False, False, True,
                False, False, False
            ],
            'pose0': 'realsense_pose',
            'pose0_config': [
                True, True, True, True, True, True,
                False, False, False, False, False, False,
                False, False, False
            ],
            'map_frame': 'map',
            'odom_frame': 'odom',
            'world_frame': 'map',
            'base_link_frame': 'base_link',
            'use_sim_time': True
        }]
    )
    nfr_apriltag_node = Node(
        package='nfr_apriltag',
        executable='nfr_apriltag_node',
        name='nfr_apriltag_node',
        parameters=[{
            'detections_topic': 'realsense/detections',
            'use_sim_time': True
        }]
    )
    nfr_odometry_node = Node(
        package='nfr_odometry',
        executable='nfr_odometry_node',
        name='nfr_odometry_node'
    )
    nfr_navigation_node = Node(
        package='nfr_navigation',
        executable='nfr_navigation_node',
        name='nfr_navigation_node'
    )
    nfr_tf_bridge_node = Node(
        package='nfr_tf_bridge',
        executable='nfr_tf_bridge_node',
        name='nfr_tf_bridge_node'
    )
    return LaunchDescription([
        navigation_launch,
        robot_state_publisher,
        realsense_container,
        robot_localization,
        rosbridge_launch,
        nfr_apriltag_node,
        nfr_odometry_node,
        nfr_navigation_node,
        nfr_tf_bridge_node
    ])