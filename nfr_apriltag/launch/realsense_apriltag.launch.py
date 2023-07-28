from launch_ros.actions import ComposableNodeContainer, Node
from launch_ros.descriptions import ComposableNode
from launch import LaunchDescription
def generate_launch_description():
    rectify_node = ComposableNode(
        package='image_proc',
        plugin='image_proc::RectifyNode',
        name='realsense_rectify',
        namespace='',
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
        namespace='',
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
        namespace='',
        parameters=[{
            'family': '16h5',
            'size': 0.18,
            'use_sim_time': True
        }]
    )
    realsense_container = ComposableNodeContainer(
        package='rclcpp_components',
        name='realsense_container',
        namespace='',
        executable='component_container_mt',
        composable_node_descriptions=[
            rectify_node,
            apriltag_node,
            realsense_node,
        ]
    )
    nfr_apriltag_node = Node(
        package='nfr_apriltag',
        executable='nfr_apriltag_node',
        name='nfr_apriltag_node',
        namespace='',
        parameters=[{
            'detections_topic': 'detections',
            'use_cuda': False,
            'use_sim_time': True
        }]
    )
    return LaunchDescription([
        realsense_container,
        nfr_apriltag_node
    ])