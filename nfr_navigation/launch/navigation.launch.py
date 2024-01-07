from launch_ros.actions import ComposableNodeContainer, SetParameter, Node
from launch_ros.substitutions import FindPackageShare
from launch_ros.descriptions import ComposableNode, ParameterFile
from nav2_common.launch import RewrittenYaml
from launch.actions import DeclareLaunchArgument, GroupAction
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch import LaunchDescription
def generate_launch_description():
    autostart = LaunchConfiguration('autostart')
    params_file = LaunchConfiguration('params_file')
    use_sim_time = LaunchConfiguration('use_sim_time')
    map_file = LaunchConfiguration('map_file')
    param_substitutions = {'autostart': autostart}
    remappings = [
        ('/tf', 'tf'),
        ('/tf_static', 'tf_static')
    ]
    autostart_argument = DeclareLaunchArgument('autostart', default_value='True')
    params_file_argument = DeclareLaunchArgument('params_file',
        default_value=PathJoinSubstitution((FindPackageShare('nav2_bringup'), 'params', 'nav2_params.yaml')))
    use_sim_time_argument = DeclareLaunchArgument('use_sim_time', default_value='False')
    map_file_argument = DeclareLaunchArgument('map_file',
        default_value=PathJoinSubstitution((FindPackageShare('nfr_charged_up'), 'config', 'map.yaml')))
    configured_params = ParameterFile(
        RewrittenYaml(
            source_file=params_file,
            param_rewrites=param_substitutions,
            convert_types=True
        ),
        allow_substs=True
    )
    controller_node = ComposableNode(
        package='nav2_controller',
        plugin='nav2_controller::ControllerServer',
        name='controller_server',
        parameters=[configured_params],
        remappings=remappings
    )
    smoother_node = ComposableNode(
        package='nav2_smoother',
        plugin='nav2_smoother::SmootherServer',
        name='smoother_server',
        parameters=[configured_params],
        remappings=remappings
    )
    planner_node = ComposableNode(
        package='nav2_planner',
        plugin='nav2_planner::PlannerServer',
        name='planner_server',
        parameters=[configured_params],
        remappings=remappings
    )
    behavior_node = ComposableNode(
        package='nav2_behaviors',
        plugin='behavior_server::BehaviorServer',
        name='behavior_server',
        parameters=[configured_params],
        remappings=remappings
    )
    bt_navigator_node = ComposableNode(
        package='nav2_bt_navigator',
        plugin='nav2_bt_navigator::BtNavigator',
        name='bt_navigator',
        parameters=[configured_params],
        remappings=remappings
    )
    waypoint_follower_node = ComposableNode(
        package='nav2_waypoint_follower',
        plugin='nav2_waypoint_follower::WaypointFollower',
        name='waypoint_follower',
        parameters=[configured_params],
        remappings=remappings
    )
    velocity_smoother_node = ComposableNode(
        package='nav2_velocity_smoother',
        plugin='nav2_velocity_smoother::VelocitySmoother',
        name='velocity_smoother',
        parameters=[configured_params],
        remappings=remappings
    )
    lifecycle_manager_node = ComposableNode(
        package='nav2_lifecycle_manager',
        plugin='nav2_lifecycle_manager::LifecycleManager',
        name='lifecycle_manager_navigation',
        parameters=[{
            'autostart': autostart,
            'node_names': [
                'controller_server',
                'smoother_server',
                'planner_server',
                'behavior_server',
                'bt_navigator',
                'waypoint_follower',
                'velocity_smoother'
            ]
        }]
    )
    map_node = ComposableNode(
        package='nav2_map_server',
        plugin='nav2_map_server::MapServer',
        name='map_server',
        parameters=[{
            'yaml_filename': map_file
        }]
    )
    lifecycle_map_node = ComposableNode(
        package='nav2_lifecycle_manager',
        plugin='nav2_lifecycle_manager::LifecycleManager',
        name='lifecycle_manager_map',
        parameters=[{
            'autostart': autostart,
            'node_names': ['map_server']
        }]
    )
    return LaunchDescription([
        autostart_argument,
        params_file_argument,
        use_sim_time_argument,
        map_file_argument,
        GroupAction(
            [
                SetParameter('use_sim_time', use_sim_time),
                ComposableNodeContainer(
                    package='rclcpp_components',
                    name='navigation_container',
                    executable='component_container_mt',
                    namespace='',
                    composable_node_descriptions=[
                        controller_node,
                        smoother_node,
                        planner_node,
                        behavior_node,
                        bt_navigator_node,
                        waypoint_follower_node,
                        velocity_smoother_node,
                        lifecycle_manager_node
                    ]
                )
            ]
        ),
        GroupAction(
            [
                SetParameter('use_sim_time', use_sim_time),
                ComposableNodeContainer(
                    package='rclcpp_components',
                    name='map_container',
                    executable='component_container_mt',
                    namespace='',
                    composable_node_descriptions=[
                        map_node,
                        lifecycle_map_node
                    ]
                )
            ]
        )
    ])