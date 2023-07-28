from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, GroupAction
from launch.launch_description_sources import PythonLaunchDescriptionSource, AnyLaunchDescriptionSource
from launch_ros.actions import Node, PushRosNamespace
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
    # realsense_launch = GroupAction(
    #     PushRosNamespace('realsense'),
    #     IncludeLaunchDescription(
    #         PythonLaunchDescriptionSource(
    #             [
    #                 os.path.join(get_package_share_directory('nfr_apriltag'), 'launch'),
    #                 '/realsense_apriltag.launch.py'
    #             ]
    #         )
    #     )
    # )
    cuda_realsense_launch = GroupAction(
        actions=[
            PushRosNamespace('realsense'),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    [
                        os.path.join(get_package_share_directory('nfr_apriltag'), 'launch'),
                        '/cuda_realsense_apriltag.launch.py'
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
        robot_localization,
        rosbridge_launch,
        nfr_odometry_node,
        nfr_navigation_node,
        nfr_tf_bridge_node,
        cuda_realsense_launch
    ])