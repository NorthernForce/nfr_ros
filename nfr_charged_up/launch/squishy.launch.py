from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os
def generate_launch_description():
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
        )
    ])