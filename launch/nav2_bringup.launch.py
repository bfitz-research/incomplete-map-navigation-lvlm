from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

import os


def generate_launch_description():
    # Launch arguments
    use_sim_time = LaunchConfiguration("use_sim_time")
    map_yaml = LaunchConfiguration("map")
    params_file = LaunchConfiguration("params_file")

    # Path to nav2_bringup's bringup_launch.py
    nav2_bringup_dir = get_package_share_directory("nav2_bringup")
    nav2_bringup_launch = os.path.join(
        nav2_bringup_dir, "launch", "bringup_launch.py"
    )

    # === Static TFs ===
    # base_link -> front_laser
    #static_base_to_front_laser = Node(
      #  package="tf2_ros",
      #  executable="static_transform_publisher",
      #  name="static_base_to_front_laser",
      #  arguments=["0.393", "0.0", "0.233", "0", "0", "0", "base_link", "front_laser"],
      #  output="screen",
    #)

    # === Nav2 bringup (map_server + AMCL + planners + controller + BT navigator) ===
    nav2_bringup = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(nav2_bringup_launch),
        launch_arguments={
            "use_sim_time": use_sim_time,
            "map": map_yaml,
            "params_file": params_file,
        }.items(),
    )

    return LaunchDescription(
        [
            # Declare launch arguments
            DeclareLaunchArgument(
                "use_sim_time",
                default_value="true",
                description="Use simulation time from Isaac Sim",
            ),
            DeclareLaunchArgument(
                "map",
                default_value="../maps/map.yaml",
                description="Path to occupancy map YAML file",
            ),
            DeclareLaunchArgument(
                "params_file",
                default_value="../config/nav2_params.yaml",
                description="Path to Nav2 params YAML file",
            ),
            # Static TF for the laser only
            #static_base_to_front_laser,
            # Include full Nav2 bringup (map_server, AMCL, Nav2 stack)
            nav2_bringup,
        ]
    )
