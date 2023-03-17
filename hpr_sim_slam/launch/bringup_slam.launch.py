from ament_index_python.packages import get_package_share_path
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    gazebo_package_path = get_package_share_path("hpr_gazebo")
    sim_slam_package_path = get_package_share_path("hpr_sim_slam")
    slam_toolbox_package_path = get_package_share_path("slam_toolbox")
    rviz_config_path = sim_slam_package_path / "rviz/hpr_slam.rviz"

    sim_time_arg = DeclareLaunchArgument(
        name="use_sim_time",
        default_value="true",
        choices=["true", "false"],
        description="Flag to enable use simulation time",
    )
    slam_params_file_arg = DeclareLaunchArgument(
        name='slam_params_file',
        default_value=str(slam_toolbox_package_path / "config/mapper_params_online_async.yaml"),
        description='Full path to the ROS2 parameters file to use for the slam_toolbox node')
    rviz_arg = DeclareLaunchArgument(
        name="rvizconfig",
        default_value=str(rviz_config_path),
        description="Absolute path to rviz config file",
    )

    hpr_gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            str(gazebo_package_path / "launch/sim_homeplater.launch.py")
        )
    )
    launch_online_async_slam = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            str(slam_toolbox_package_path / "launch/online_async_launch.py")
        ),
        launch_arguments={
            'use_sim_time': LaunchConfiguration('use_sim_time'),
            'slam_params_file': LaunchConfiguration('slam_params_file'),
        }.items()
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

    return LaunchDescription(
        [
            sim_time_arg,
            slam_params_file_arg,
            rviz_arg,
            hpr_gazebo,
            robot_localization_node,
            launch_online_async_slam,
            rviz_node,
        ]
    )
