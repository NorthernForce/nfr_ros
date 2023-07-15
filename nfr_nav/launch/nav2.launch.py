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
                    os.path.join(get_package_share_directory('nav2_bringup'), 'launch'),
                    '/navigation_launch.py'
                ]
            )
        )
        # Node(
        #     package='nfr_nav',
        #     executable='nfr_nav_node',
        #     name='nfr_nav_node'
        # )
    ])