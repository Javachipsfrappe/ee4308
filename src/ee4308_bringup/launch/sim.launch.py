from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch_ros.substitutions import FindPackageShare
from launch.actions import SetEnvironmentVariable, AppendEnvironmentVariable
from launch.substitutions import (
    LaunchConfiguration,
    PathJoinSubstitution,
    EqualsSubstitution,
    NotEqualsSubstitution,
)
from launch.conditions import IfCondition

def generate_launch_description():
    ld = LaunchDescription()

    # ================ 1. PACKAGE SHARE DIRECTORIES ==============
    pkg_ros_gz_sim = FindPackageShare("ros_gz_sim")
    pkg_ee4308_bringup = FindPackageShare("ee4308_bringup")

    # ================ 2. LAUNCH ARGUMENTS ==============
    # Launch Arg: project mode
    arg_project = DeclareLaunchArgument(
        "project",
        default_value="1",
        description="'1' to spawn with turtlebot. '2' to spawn with turtlebot and drone.",
    )  # Worlds must have a more relaxed contact coefficient (~100) for faster solving.
    ld.add_action(arg_project)

    # Launch Arg: world
    arg_world = DeclareLaunchArgument(
        "world",
        default_value="turtlebot3_house",
        description="Name of the Gazebo world file to load (without .world extension). Must be in ee4308_bringup/worlds",
    )  # Worlds must have a more relaxed contact coefficient (~100) for faster solving.
    ld.add_action(arg_world)

    # Launch Arg: headless
    arg_headless = DeclareLaunchArgument(
        "headless",
        default_value="False",
        description="If set to True, open the Gazebo simulator and its GUI. If set to False, the GUI is gone and the simulation runs in the background.",
    )
    ld.add_action(arg_headless)

    # Launch Arg: libgl
    arg_libgl = DeclareLaunchArgument(
        "libgl",
        default_value="False",
        description="If set to True, LibGL is used (this is primarily for VirtualBox users). If set to False, the faster ogre2 renderer is used (cannot be used in VirtualBox, only for Dual boot).",
    )
    ld.add_action(arg_libgl)

    # ===================== GAZEBO ========================
    # Changes the renderer for Gz if using VBox.
    env_lib_gl_true = SetEnvironmentVariable(
        "LIBGL_ALWAYS_SOFTWARE",
        "1",
        condition=IfCondition(EqualsSubstitution(LaunchConfiguration("libgl"), "True")),
    )
    ld.add_action(env_lib_gl_true)  # for single/dual boot

    env_lib_gl_false = SetEnvironmentVariable(
        "LIBGL_ALWAYS_SOFTWARE",
        "0",
        condition=IfCondition(
            NotEqualsSubstitution(LaunchConfiguration("libgl"), "True")
        ),
    )
    ld.add_action(env_lib_gl_false)  # for single/dual boot

    # For Gz to find the world models from this package
    env_gz_sim_resource_path = AppendEnvironmentVariable(
        "GZ_SIM_RESOURCE_PATH", PathJoinSubstitution([pkg_ee4308_bringup, "models"])
    )
    ld.add_action(env_gz_sim_resource_path)

    # Launch Gz depending on headless or with GUI
    launch_gz_sim_launch_file = PathJoinSubstitution(
        [pkg_ros_gz_sim, "launch", "gz_sim.launch.py"]
    )
    launch_gz_sim_world_file = PathJoinSubstitution(
        [pkg_ee4308_bringup, "worlds", [LaunchConfiguration("world"), ".world"]]
    )

    launch_gz_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([launch_gz_sim_launch_file]),
        condition=IfCondition(
            NotEqualsSubstitution(LaunchConfiguration("headless"), "True")
        ),
        launch_arguments={
            "gz_args": [
                launch_gz_sim_world_file,
                " -r -v1",
            ],
        }.items(),
    )
    ld.add_action(launch_gz_sim)

    launch_gz_sim_headless = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([launch_gz_sim_launch_file]),
        condition=IfCondition(
            EqualsSubstitution(LaunchConfiguration("headless"), "True")
        ),
        launch_arguments={
            "gz_args": [
                launch_gz_sim_world_file,
                " -r -v1 -s",
            ],
        }.items(),
    )
    ld.add_action(launch_gz_sim_headless)


    return ld
