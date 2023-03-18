from ament_index_python.packages import get_package_share_path
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration
from nav2_common.launch import RewrittenYaml


def generate_launch_description():
    gazebo_package_path = get_package_share_path("hpr_gazebo")
    sim_slam_package_path = get_package_share_path("hpr_sim_slam")
    rviz_config_path = sim_slam_package_path / "rviz/hpr_slam.rviz"

    namespace_arg = DeclareLaunchArgument(
        name="namespace", 
        default_value='',
        description='Top-level namespace'
    )
    sim_time_arg = DeclareLaunchArgument(
        name="use_sim_time",
        default_value="true",
        choices=["true", "false"],
        description="Flag to enable use simulation time",
    )
    rviz_arg = DeclareLaunchArgument(
        name="rvizconfig",
        default_value=str(rviz_config_path),
        description="Absolute path to rviz config file",
    )
    map_file_arg = DeclareLaunchArgument(
        name="map_file",
        default_value=str(sim_slam_package_path / "map/demo_world_map.yaml"),
        description='Full path to map yaml file to load'
    )
    autostart_arg = DeclareLaunchArgument(
        name="autostart", 
        default_value="true",
        description='Automatically startup the nav2 stack'
    )
    nav2_params_file_arg = DeclareLaunchArgument(
        name='nav2_params_file',
        default_value=str(sim_slam_package_path / "config/nav2_params.yaml"),
        description='Full path to the ROS2 parameters file to use'
    )

    lifecycle_nodes = ['map_server', 'amcl']
    remappings = [('/tf', 'tf'),
                  ('/tf_static', 'tf_static')]

    param_substitutions = {
        'use_sim_time': LaunchConfiguration("use_sim_time"),
        'yaml_filename': LaunchConfiguration("map_file")}

    configured_params = RewrittenYaml(
        source_file=LaunchConfiguration("nav2_params_file"),
        root_key=LaunchConfiguration("namespace"),
        param_rewrites=param_substitutions,
        convert_types=True)

    hpr_gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            str(gazebo_package_path / "launch/sim_homeplater.launch.py")
        )
    )

    robot_localization_node = Node(
        package="robot_localization",
        executable="ekf_node",
        name="ekf_filter_node",
        output="screen",
        parameters=[
            str(sim_slam_package_path / "config/ekf.yaml"),
            {"use_sim_time": LaunchConfiguration("use_sim_time")},
        ],
    )
    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="screen",
        arguments=["-d", LaunchConfiguration("rvizconfig")],
    )
    map_server_node = Node(
        package='nav2_map_server',
        executable='map_server',
        name='map_server',
        output='screen',
        parameters=[configured_params],
        remappings=remappings
    )
    amcl_node = Node(
        package='nav2_amcl',
        executable='amcl',
        name='amcl',
        output='screen',
        parameters=[configured_params],
        remappings=remappings
    )
    lifecycle_manager_node = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager_localization',
        output='screen',
        parameters=[{'use_sim_time': LaunchConfiguration("use_sim_time")},
                    {'autostart': LaunchConfiguration("autostart")},
                    {'node_names': lifecycle_nodes}]
    )

    return LaunchDescription(
        [
            namespace_arg,
            sim_time_arg,
            rviz_arg,
            map_file_arg,
            autostart_arg,
            nav2_params_file_arg,
            hpr_gazebo,
            robot_localization_node,
            rviz_node,
            map_server_node,
            amcl_node,
            lifecycle_manager_node,
        ]
    )
