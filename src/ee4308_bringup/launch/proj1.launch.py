from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import (
    LaunchConfiguration,
    PathJoinSubstitution,
)

def generate_launch_description():
    ld = LaunchDescription()

    # ================ 1. PACKAGE SHARE DIRECTORIES ==============
    pkg_ee4308_bringup = FindPackageShare("ee4308_bringup")

    # ================ 2. LAUNCH ARGUMENTS ==============
    arg_map = DeclareLaunchArgument(
        "map",
        default_value="proj1_sim",
        description="Name of the yaml map file to load. Must be in ee4308_bringup/maps, and without the file extension.",
    )
    ld.add_action(arg_map)

    arg_nav2_param_file = DeclareLaunchArgument(
        "nav2_param_file",
        default_value="proj1",
        description="Name of the yaml map file to load the nav2 parameters. Must be in ee4308_bringup/params, and without the file extension.",
    )
    ld.add_action(arg_nav2_param_file)

    # nav2 turtle
    launch_nav2_turtle = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([pkg_ee4308_bringup, "launch", "nav2_turtle.launch.py"])
        ),
        launch_arguments={
            "use_sim_time": "False",
            "map": LaunchConfiguration("map"),
            "nav2_param_file": LaunchConfiguration("nav2_param_file"),
        }.items(),
    )
    ld.add_action(launch_nav2_turtle)

    
    return ld
