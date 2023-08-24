from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, GroupAction
from launch.launch_description_sources import PythonLaunchDescriptionSource, AnyLaunchDescriptionSource
from launch_ros.actions import Node, PushRosNamespace
from ament_index_python.packages import get_package_share_directory
import os
def generate_launch_description():
    with open(os.path.join(get_package_share_directory('nfr_charged_up'), 'config', 'squishy.urdf')) as f:
        robot_desc = f.read()
    # navigation_launch = IncludeLaunchDescription(
    #     PythonLaunchDescriptionSource(
    #         [
    #             os.path.join(get_package_share_directory('nav2_bringup'), 'launch'),
    #             '/navigation_launch.py'
    #         ]
    #     ),
    #     launch_arguments=[
    #         ('use_sim_time', 'True')
    #     ]
    # )
    map_yaml_file = os.path.join(get_package_share_directory('nfr_charged_up'), 'config', 'map.yaml')
    map_node = Node(
        package='nav2_map_server',
        executable='map_server',
        name='map_server',
        parameters=[{
            'yaml_filename': map_yaml_file
        }]
    )
    lifecycle_map_node = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_map_manager',
        parameters=[{
            'autostart': True,
            'node_names': ['map_server']
        }]
    )
    rosbridge_launch = IncludeLaunchDescription(
        AnyLaunchDescriptionSource(
            [
                os.path.join(get_package_share_directory('rosbridge_server'), 'launch'),
                '/rosbridge_websocket_launch.xml'
            ]
        ),
        launch_arguments=[('port', '5809')]
    )
    realsense_launch = GroupAction(
        actions=[
            PushRosNamespace('realsense'),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    [
                        os.path.join(get_package_share_directory('nfr_apriltag'), 'launch'),
                        '/realsense_apriltag.launch.py'
                    ]
                )
            )
        ]
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
            'pose0': '/realsense/pose_estimations',
            'pose0_config': [
                True, True, False, False, False, True,
                False, False, False, False, False, False,
                False, False, False
            ],
            'world_frame': 'map',
        }]
    )
    nfr_odometry_node = Node(
        package='nfr_odometry',
        executable='nfr_odometry_node',
        name='nfr_odometry_node'
    )
    # nfr_navigation_node = Node(
    #     package='nfr_navigation',
    #     executable='nfr_navigation_node',
    #     name='nfr_navigation_node'
    # )
    nfr_tf_bridge_node = Node(
        package='nfr_tf_bridge',
        executable='nfr_tf_bridge_node',
        name='nfr_tf_bridge_node'
    )
    return LaunchDescription([
        robot_state_publisher,
        rosbridge_launch,
        realsense_launch,
        nfr_tf_bridge_node,
        nfr_odometry_node,
        robot_localization,
        map_node,
        lifecycle_map_node
    ])