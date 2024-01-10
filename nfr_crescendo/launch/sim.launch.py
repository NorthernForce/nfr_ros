from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, GroupAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node, PushRosNamespace
from ament_index_python.packages import get_package_share_directory
import os
def generate_launch_description():
    with open(os.path.join(get_package_share_directory('nfr_charged_up'), 'config', 'swervy.urdf')) as f:
        robot_desc = f.read()
    usb_cam_launch = GroupAction(
        actions=[
            PushRosNamespace('usb_cam'),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    [
                        os.path.join(get_package_share_directory('nfr_apriltag'), 'launch'),
                        '/usb_apriltag.launch.py'
                    ]
                ),
                launch_arguments=[('field_path', os.path.join(get_package_share_directory('nfr_crescendo'), 'config', 'field.json')),
                                  ('camera_info_url', 'package://nfr_crescendo/config/ost.yaml')]
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
            ('params_file', os.path.join(get_package_share_directory('nfr_charged_up'), 'config', 'swervy_nav2.yaml'))
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
            'pose0': '/usb_cam/pose_estimations',
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
            'cameras': ['usb_cam']
        }]
    )
    camera_node = Node(
        package='nfr_camera',
        executable='nfr_camera_node',
        name='nfr_camera_node',
        namespace='usb_cam',
        parameters=[{
            'camera_path': 'image',
            'camera_name': 'USBCam'
        }]
    )
    note_detection_node = Node(
        package='nfr_note_detector',
        executable='nfr_note_detector_node',
        name='nfr_note_detector_node',
        namespace='usb_cam',
        parameters=[{
            'camera_path': 'image',
            'camera_name': 'USBCam'
        }]
    )
    return LaunchDescription([
        robot_state_publisher,
        usb_cam_launch,
        navigation_launch,
        local_localization,
        global_localization,
        bridge_node,
        camera_node,
        note_detection_node
    ])