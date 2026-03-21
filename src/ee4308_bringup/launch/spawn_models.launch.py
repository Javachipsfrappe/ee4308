from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import (
    LaunchConfiguration,
    PathJoinSubstitution,
    EqualsSubstitution,
    FileContent,
)
from launch.conditions import IfCondition


def generate_launch_description():
    ld = LaunchDescription()

    # ================ 1. PACKAGE SHARE DIRECTORIES ==============
    pkg_ee4308_bringup = FindPackageShare("ee4308_bringup")

    # ================ 2. LAUNCH ARGUMENTS ==============
    # Launch Arg: turtle_x
    arg_turtle_x = DeclareLaunchArgument(
        "turtle_x", default_value="-2.0", description="x position of the turtle."
    )
    ld.add_action(arg_turtle_x)

    # Launch Arg: turtle_y
    arg_turtle_y = DeclareLaunchArgument(
        "turtle_y", default_value="-0.5", description="y position of the turtle."
    )
    ld.add_action(arg_turtle_y)

    # Launch Arg: turtle_z
    arg_turtle_z = DeclareLaunchArgument(
        "turtle_z", default_value="0.01", description="z position of the turtle."
    )
    ld.add_action(arg_turtle_z)

    # Launch Arg: drone_x
    arg_drone_x = DeclareLaunchArgument(
        "drone_x", default_value="-2.0", description="x position of the drone for project 1."
    )
    ld.add_action(arg_drone_x)

    # Launch Arg: drone_y
    arg_drone_y = DeclareLaunchArgument(
        "drone_y", default_value="-2.0", description="y position of the drone."
    )
    ld.add_action(arg_drone_y)

    # Launch Arg: drone_z
    arg_drone_z = DeclareLaunchArgument(
        "drone_z", default_value="0.05", description="z position of the drone."
    )
    ld.add_action(arg_drone_z)

    # Launch Arg: project
    arg_project = DeclareLaunchArgument(
        "project", default_value="1", description="'1' for project 1, '2' for project 2."
    )
    ld.add_action(arg_project)

    # Get the turtle URDF file.
    path_turtle_urdf = PathJoinSubstitution([pkg_ee4308_bringup, "models", "turtle", "turtle.urdf"])

    # Get the drone URDF file. # TODO: Use python substitution to avoid running this in mode 1.
    path_drone_urdf = PathJoinSubstitution([pkg_ee4308_bringup, "models", "drone", "drone.urdf"])

    # ======================== Project 1 ==============================
    condition_proj1 = IfCondition(EqualsSubstitution(LaunchConfiguration("project"), "1"))

    # robot state publisher for project 1 (no namespace and prefix)
    node_turtle_state_publisher_1 = Node(
        condition= condition_proj1,
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="robot_state_publisher",
        output="screen",
        parameters=[
            {
                "robot_description": FileContent(path_turtle_urdf),
                "use_sim_time": True,
                "frame_prefix": "/",
            },
        ],
    )
    ld.add_action(node_turtle_state_publisher_1)

    # Robot gz bridge for project 1
    path_bridge_1 = PathJoinSubstitution(
        [pkg_ee4308_bringup, "params", "ros_gz_bridge_proj1.yaml"]
    )
    node_bridge_1 = Node(
        condition=condition_proj1,
        package="ros_gz_bridge",
        executable="parameter_bridge",
        output="screen",
        parameters=[
            {"use_sim_time": True, "config_file": path_bridge_1},
        ],
    )
    ld.add_action(node_bridge_1)

    # Spawn turtle
    path_turtle_sdf_1 = PathJoinSubstitution(
        [pkg_ee4308_bringup, "models", "turtle", "turtle_proj1.sdf"]
    )
    node_spawn_turtle_1 = Node(
        condition=condition_proj1,
        package="ros_gz_sim",
        executable="create",
        arguments=[
            "-name",
            "turtle",
            "-file",
            path_turtle_sdf_1,
            "-x",
            LaunchConfiguration("turtle_x"),
            "-y",
            LaunchConfiguration("turtle_y"),
            "-z",            
            LaunchConfiguration("turtle_z"),
        ],
        output="screen",
    )
    ld.add_action(node_spawn_turtle_1)
    
    # ========================= Project 2 ==============================
    condition_proj2 = IfCondition(EqualsSubstitution(LaunchConfiguration("project"), "2"))

    # turtle state publisher for project 2 ("turtle" namespace and prefix)
    node_turtle_state_publisher_2 = Node(
        condition=condition_proj2,
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="robot_state_publisher",
        output="screen",
        parameters=[
            {
                "robot_description": FileContent(path_turtle_urdf),  # ParameterValue(Command(['xacro ', path_turtle_model]), value_type=str), # Parameter Value required to wrap around xacro (if file accidentally contains colons).
                "use_sim_time": True,
                "frame_prefix": "turtle/",
            },
        ],
        namespace="turtle",
    )
    ld.add_action(node_turtle_state_publisher_2)

    # drone state publisher for project 2 ("turtle" namespace and prefix)
    node_drone_state_publisher_2 = Node(
        condition=condition_proj2,
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="robot_state_publisher",
        output="screen",
        parameters=[
            {
                "robot_description": FileContent(path_drone_urdf),  # ParameterValue(Command(['xacro ', path_turtle_model]), value_type=str), # Parameter Value required to wrap around xacro (if file accidentally contains colons).
                "use_sim_time": True,
                "frame_prefix": "drone/",
            },
        ],
        namespace="drone",
    )
    ld.add_action(node_drone_state_publisher_2)

    # Robot gz bridge for project 1
    path_bridge_2 = PathJoinSubstitution(
        [pkg_ee4308_bringup, "params", "ros_gz_bridge_proj2.yaml"]
    )
    node_bridge_2 = Node(
        condition=condition_proj2,
        package="ros_gz_bridge",
        executable="parameter_bridge",
        output="screen",
        parameters=[
            {"use_sim_time": True, "config_file": path_bridge_2},
        ],
    )
    ld.add_action(node_bridge_2)

    # Spawn turtle in "/turtle" namespace
    path_turtle_sdf_2 = PathJoinSubstitution(
        [pkg_ee4308_bringup, "models", "turtle", "turtle_proj2.sdf"]
    )
    node_spawn_turtle_2 = Node(
        condition=condition_proj2,
        namespace="turtle",
        package="ros_gz_sim",
        executable="create",
        arguments=[
            "-name",
            "turtle",
            "-file",
            path_turtle_sdf_2,
            "-x",
            LaunchConfiguration("turtle_x"),
            "-y",
            LaunchConfiguration("turtle_y"),
            "-z",            
            LaunchConfiguration("turtle_z"),
        ],
        output="screen",
    )
    ld.add_action(node_spawn_turtle_2)

    # static transform for turtle
    node_turtle_static_tf_2 = Node(
        namespace="turtle",
        package='tf2_ros',
        name='turtle_static_tf',
        executable='static_transform_publisher',
        arguments=["--x", "0", "--y", "0", "--z", "0",
           "--yaw", "0", "--pitch", "0", "--roll", "0",
           "--frame-id", "map", "--child-frame-id", "turtle/odom"],
        
        output='screen'
    )
    ld.add_action(node_turtle_static_tf_2)

    # Spawn drone in "/drone" namespace
    path_drone_sdf_2 = PathJoinSubstitution(
        [pkg_ee4308_bringup, "models", "drone", "drone_proj2.sdf"]
    )
    node_spawn_drone_2 = Node(
        condition=condition_proj2,
        namespace="drone",
        package="ros_gz_sim",
        executable="create",
        arguments=[
            "-name",
            "drone",
            "-file",
            path_drone_sdf_2,
            "-x",
            LaunchConfiguration("drone_x"),
            "-y",
            LaunchConfiguration("drone_y"),
            "-z",
            LaunchConfiguration("drone_z"),
        ],
        output="screen",
    )
    ld.add_action(node_spawn_drone_2)

    # static transform from map to drone/odom
    node_drone_static_tf_2 = Node(
        namespace="drone",
        package='tf2_ros',
        name='drone_static_tf',
        executable='static_transform_publisher',
        arguments=["--x", "0", "--y", "0", "--z", "0",
           "--yaw", "0", "--pitch", "0", "--roll", "0",
           "--frame-id", "map", "--child-frame-id", "drone/odom"],
        output='screen'
    )
    ld.add_action(node_drone_static_tf_2)

    # odom to drone/base_footprint is provided by the multicopter_velocity_controller, and has non-zero z transformations (i.e. is the base link) which is against rep120.
    # static transform for drone/base_footprint to drone/base_link
    node_drone_static_tf_base_link_2 = Node(
        namespace="drone",
        package='tf2_ros',
        name='drone_static_tf2',
        executable='static_transform_publisher',
        arguments=["--x", "0", "--y", "0", "--z", "0",
           "--yaw", "0", "--pitch", "0", "--roll", "0",
           "--frame-id", "drone/base_footprint", "--child-frame-id", "drone/base_link"],
        output='screen'
    )
    ld.add_action(node_drone_static_tf_base_link_2)

    return ld
