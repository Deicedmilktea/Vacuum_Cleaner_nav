from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import Command, LaunchConfiguration, PythonExpression
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    # Get the launch directory
    nav_dir = get_package_share_directory("vacuum_nav")

    # Create our own temporary YAML files that include substitutions
    param_dir = os.path.join(nav_dir, "config")
    if not os.path.exists(param_dir):
        os.makedirs(param_dir)

    # Parameters
    namespace = LaunchConfiguration("namespace")
    use_sim_time = LaunchConfiguration("use_sim_time")
    map_yaml_file = LaunchConfiguration("map")
    use_composition = LaunchConfiguration("use_composition")
    autostart = LaunchConfiguration("autostart")
    params_file = LaunchConfiguration("params_file")

    # Launch configurations
    declare_params_file_cmd = DeclareLaunchArgument(
        "params_file",
        default_value=os.path.join(nav_dir, "config", "nav2_params.yaml"),
        description="Full path to the ROS2 parameters file to use",
    )

    # Launch configurations
    declare_namespace_cmd = DeclareLaunchArgument(
        "namespace", default_value="", description="Top-level namespace"
    )

    declare_use_sim_time_cmd = DeclareLaunchArgument(
        "use_sim_time",
        default_value="false",
        description="Use simulation clock if true",
    )

    declare_map_yaml_cmd = DeclareLaunchArgument(
        "map",
        default_value="",
        description="Full path to map yaml file to load (leave empty for SLAM)",
    )

    declare_use_composition_cmd = DeclareLaunchArgument(
        "use_composition",
        default_value="True",
        description="Use composed bringup if True",
    )

    declare_autostart_cmd = DeclareLaunchArgument(
        "autostart",
        default_value="true",
        description="Automatically startup the nav2 stack",
    )

    # Include the Nav2 launch file with our configurations
    nav2_bringup_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory("nav2_bringup"),
                "launch",
                "bringup_launch.py",
            )
        ),
        launch_arguments={
            "namespace": namespace,
            "use_sim_time": use_sim_time,
            "map": map_yaml_file,
            "use_composition": use_composition,
            "autostart": autostart,
            "params_file": params_file,
        }.items(),
    )

    # RViz
    rviz_config_dir = os.path.join(nav_dir, "rviz", "nav2_default_view.rviz")

    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        arguments=["-d", rviz_config_dir],
        parameters=[{"use_sim_time": use_sim_time}],
        output="screen",
    )

    return LaunchDescription(
        [
            declare_params_file_cmd,
            declare_namespace_cmd,
            declare_use_sim_time_cmd,
            declare_map_yaml_cmd,
            declare_use_composition_cmd,
            declare_autostart_cmd,
            nav2_bringup_launch,
            rviz_node,
        ]
    )
