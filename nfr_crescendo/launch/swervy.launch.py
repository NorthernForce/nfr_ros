from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, GroupAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node, PushRosNamespace
from ament_index_python.packages import get_package_share_directory
import os
def generate_launch_description():
    with open(os.path.join(get_package_share_directory('nfr_crescendo'), 'config', 'swervy.urdf')) as f:
        robot_desc = f.read()
    apriltag_launch = GroupAction(
        actions=[
            PushRosNamespace('apriltag_camera'),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    [
                        os.path.join(get_package_share_directory('nfr_apriltag'), 'launch'),
                        '/realsense_apriltag.launch.py'
                    ]
                ),
                launch_arguments=[
                    ('field_path', os.path.join(get_package_share_directory('nfr_crescendo'), 'config', 'field.json')),
                    ('pixel_format', 'yuyv'),
                    ('resolution_width', '1280'),
                    ('resolution_height', '720'),
                    ('fps', '15'),
                    ('launch_camera_server', 'True'),
                    ('enable_depth', 'True')
                ]
            )
        ]
    )
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        parameters=[{
            'robot_description': robot_desc
        }]
    )
    navigation_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [
                os.path.join(get_package_share_directory('nfr_navigation'), 'launch'),
                '/navigation.launch.py'
            ]
        ),
        launch_arguments=[
            ('params_file', os.path.join(get_package_share_directory('nfr_crescendo'), 'config', 'swervy_nav2.yaml')),
            ('map_file', os.path.join(get_package_share_directory('nfr_crescendo'), 'config', 'map.yaml'))
        ]
    )
    bridge_node = Node(
        package='nfr_bridge',
        executable='nfr_bridge_node',
        name='nfr_bridge_node',
        parameters=[{
            'target_cameras': ['apriltag_camera']
        }],
        remappings=[
            ('apriltag_camera/targets', 'apriltag_camera/targets_filtered')
        ]
    )
    local_localization_node = Node(
        package='fuse_optimizers',
        executable='fixed_lag_smoother_node',
        name='local_localization_node',
        parameters=[
            os.path.join(get_package_share_directory('nfr_crescendo'), 'config', 'fuse.yaml')
        ]
    )
    global_localization_node = Node(
        package='fuse_optimizers',
        executable='fixed_lag_smoother_node',
        name='global_localization_node',
        parameters=[
            os.path.join(get_package_share_directory('nfr_crescendo'), 'config', 'fuse.yaml')
        ]
    )
    return LaunchDescription([
        robot_state_publisher,
        apriltag_launch,
        navigation_launch,
        bridge_node,
        local_localization_node,
        global_localization_node
    ])