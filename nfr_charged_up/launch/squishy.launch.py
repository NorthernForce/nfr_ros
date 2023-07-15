from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os
def generate_launch_description():
    with open(os.path.join(get_package_share_directory('nfr_charged_up'), 'config', 'squishy.urdf')) as f:
        robot_desc = f.read()
    return LaunchDescription([
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                [
                    os.path.join(get_package_share_directory('nfr_nav'), 'launch'),
                    '/nav2.launch.py'
                ]
            )
        ),
        Node(
            package='nfr_bridge',
            executable='nfr_bridge_node',
            name='nfr_bridge_node',
            parameters=[{
                'team': 172,
                'client_name': 'xavier',
                'ros_table': 'ros',
                'topics_xml': os.path.join(get_package_share_directory('nfr_charged_up'), 'config', 'squishy_bridge.xml')
            }]
        ),
        Node(
            package='nfr_gps',
            executable='nfr_gps_node',
            name='nfr_gps_node',
            parameters=[{
                'tag_poses': os.path.join(get_package_share_directory('nfr_charged_up'), 'config', 'field.json'),
                'detector_topics': ['realsense/detections'],
                'camera_names': ['realsense']
            }]
        ),
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            parameters=[{
                'robot_description': robot_desc
            }]
        ),
        Node(
            package='realsense2_camera',
            executable='realsense2_camera_node',
            name='realsense',
            namespace='realsense',
            remappings=[('color/image_raw', 'image'), ('color/camera_info', 'camera_info')]
        ),
        Node(
            package='image_proc',
            executable='image_proc',
            name='rectify_node',
            namespace='realsense'
        ),
        Node(
            package='apriltag_ros',
            executable='apriltag_node',
            name='apriltag_node',
            namespace='realsense',
            parameters=[{
                'family': '16h5',
                'size': 0.18
            }]
        ),
        Node(
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
                'pose0': 'apriltag_estimations',
                'pose0_config': [
                    True, True, True, True, True, True,
                    False, False, False, False, False, False,
                    False, False, False
                ],
                'map_frame': 'map',
                'odom_frame': 'odom',
                'world_frame': 'map',
                'base_link_frame': 'base_link'
            }]
        )
    ])