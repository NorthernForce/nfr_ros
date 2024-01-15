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
            PushRosNamespace('usb_cam1'),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    [
                        os.path.join(get_package_share_directory('nfr_apriltag'), 'launch'),
                        '/usb_apriltag.launch.py'
                    ]
                ),
                launch_arguments=[
                    ('field_path', os.path.join(get_package_share_directory('nfr_crescendo'), 'config', 'field.json')),
                    ('camera_info_url', 'package://nfr_crescendo/config/usb1.yaml'),
                    ('pixel_format', 'yuyv'),
                    ('resolution_width', '640'),
                    ('resolution_height', '480'),
                    ('camera_path', '/dev/video0')
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
    local_localization = Node(
        package='robot_localization',
        executable='ekf_node',
        name='local_ekf_node',
        parameters=[{
            'odom0': '/odom',
            'odom0_config': [
                False, False, False, False, False, False,
                True, True, False, False, False, False,
                False, False, False
            ],
            'odom0_differential': True,
            'imu0': '/imu',
            'imu0_config': [
                False, False, False, False, False, True,
                False, False, False, False, False, False,
                False, False, False
            ],
            'world_frame': 'odom'
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
                False, False, False, False, False, False,
                True, True, False, False, False, False,
                False, False, False
            ],
            'odom0_differential': True,
            'imu0': '/imu',
            'imu0_config': [
                False, False, False, False, False, True,
                False, False, False, False, False, False,
                False, False, False
            ],
            'pose0': '/usb_cam1/pose_estimations',
            'pose0_config': [
                True, True, False, False, False, False,
                False, False, False, False, False, False,
                False, False, False
            ],
            'world_frame': 'map'
        }],
        remappings=[
            ('set_pose', 'global_set_pose')
        ]
    )
    bridge_node = Node(
        package='nfr_bridge',
        executable='nfr_bridge_node',
        name='nfr_bridge_node',
        parameters=[{
            'target_cameras': ['usb_cam1', 'usb_cam2']
        }]
    )
    camera_node = GroupAction(
        actions = [
            PushRosNamespace('usb_cam2'),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource([
                    os.path.join(get_package_share_directory('nfr_camera'), 'launch'),
                    '/usb_cam.launch.py'
                ]),
                launch_arguments=[
                    ('camera_path', '/dev/video2'),
                    ('camera_name', 'usb_cam2'),
                    ('camera_port', '1182'),
                    ('resolution_width', '640'),
                    ('resolution_height', '480'),
                    ('camera_info_url', 'package://nfr_crescendo/config/usb2.yaml'),
                    ('pixel_format', 'yuyv')
                ]
            )
        ]
    )
    note_detection_node = Node(
        package='nfr_note_detector',
        executable='nfr_note_detector_node',
        name='nfr_note_detector_node',
        namespace='usb_cam2',
        parameters=[{
            'camera_path': 'image',
            'camera_name': 'USBCam',
            'camera_port': 1183
        }]
    )
    return LaunchDescription([
        robot_state_publisher,
        apriltag_launch,
        navigation_launch,
        local_localization,
        global_localization,
        bridge_node,
        camera_node,
        note_detection_node
    ])