from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, GroupAction
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode
from launch.launch_description_sources import PythonLaunchDescriptionSource, AnyLaunchDescriptionSource
from launch_ros.actions import Node, PushRosNamespace
from ament_index_python.packages import get_package_share_directory
import os
def generate_launch_description():
    with open(os.path.join(get_package_share_directory('nfr_charged_up'), 'config', 'swervy.urdf')) as f:
        robot_desc = f.read()
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
                        '/cuda_realsense_apriltag.launch.py'
                    ]
                ),
                launch_arguments=[
                    ('params_file', os.path.join(get_package_share_directory('nfr_charged_up'), 'config', 'swervy_nav2.yaml'))
                ]
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
    navigation_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [
                os.path.join(get_package_share_directory('nfr_navigation'), 'launch'),
                '/navigation.launch.py'
            ]
        )
    )
    local_localization = Node(
        package='robot_localization',
        executable='ekf_node',
        name='local_ekf_node',
        parameters=[{
            'odom0': '/odom',
            'odom0_config': [
                True, True, False, False, False, True,
                True, True, False, False, False, True,
                False, False, False
            ],
            'world_frame': 'odom',
            'use_sim_time': True
        }],
        remappings=[
            ('set_pose', 'local_set_pose')
        ]
    )
    global_localization = Node(
        package='robot_localization',
        executable='ekf_node',
        name='global_ekf_node',
        parameters=[{
            'odom0': '/odom',
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
            'use_sim_time': True
        }],
        remappings=[
            ('set_pose', 'global_set_pose')
        ]
    )
    return LaunchDescription([
        robot_state_publisher,
        rosbridge_launch,
        realsense_launch,
        navigation_launch,
        local_localization,
        global_localization
    ])