from ament_index_python.packages import get_package_share_path
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    gazebo_package_path = get_package_share_path("hpr_gazebo")
    slam_package_path = get_package_share_path("hpr_sim_slam")
    rviz_config_path = slam_package_path / "rviz/hpr_slam.rviz"

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

    hpr_gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            str(gazebo_package_path / "launch/sim_homeplater.launch.py")
        )
    )
    launch_online_async_slam = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            str(get_package_share_path("slam_toolbox") / "launch/online_async_launch.py")
        )
    )
    
    robot_localization_node = Node(
        package="robot_localization",
        executable="ekf_node",
        name="ekf_filter_node",
        output="screen",
        parameters=[
            str(slam_package_path / "config/ekf.yaml"),
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
            rviz_arg,
            hpr_gazebo,
            robot_localization_node,
            launch_online_async_slam,
            rviz_node,
        ]
    )
