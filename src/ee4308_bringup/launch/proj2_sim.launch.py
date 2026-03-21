from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch_ros.substitutions import FindPackageShare
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode
from launch.substitutions import (
    LaunchConfiguration,
    PathJoinSubstitution,
)
from launch_ros.actions import Node

def generate_launch_description():
    ld = LaunchDescription()

    # ================ 1. PACKAGE SHARE DIRECTORIES ==============
    pkg_ee4308_bringup = FindPackageShare("ee4308_bringup")

    # ================ 2. LAUNCH ARGUMENTS ==============
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
        "drone_x", default_value="-2.0", description="x position of the drone."
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

    arg_map = DeclareLaunchArgument(
        "map",
        default_value="proj2_sim",
        description="Name of the yaml map file to load. Must be in ee4308_bringup/maps, and without the file extension.",
    )
    ld.add_action(arg_map)

    # param yaml file
    arg_param_file = DeclareLaunchArgument(
        "param_file",
        default_value="proj2",
        description="Name of the yaml map file to load the nav2 parameters. Must be in ee4308_bringup/params, and without the file extension.",
    )
    ld.add_action(arg_param_file)
    path_param_file = PathJoinSubstitution(
        [pkg_ee4308_bringup, "params", [LaunchConfiguration("param_file"), ".yaml"]]
    )

    # ================ 3. LAUNCH FILES ==============
    # sim
    launch_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([pkg_ee4308_bringup, "launch", "sim.launch.py"])
        ),
        launch_arguments={
            "world": LaunchConfiguration("world"),
            "headless": LaunchConfiguration("headless"),
            "libgl": LaunchConfiguration("libgl"),
        }.items(),
    )
    ld.add_action(launch_sim)

    # spawn models
    launch_spawn_models = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([pkg_ee4308_bringup, "launch", "spawn_models.launch.py"])
        ),
        launch_arguments={
            "project": "2",
            "turtle_x": LaunchConfiguration("turtle_x"),
            "turtle_y": LaunchConfiguration("turtle_y"),
            "turtle_z": LaunchConfiguration("turtle_z"),
            "drone_x": LaunchConfiguration("drone_x"),
            "drone_y": LaunchConfiguration("drone_y"),
            "drone_z": LaunchConfiguration("drone_z"),
        }.items(),
    )
    ld.add_action(launch_spawn_models)

    # ================ 4. NODES & COMPOSABLE NODES ==============
    # rviz
    node_rviz = Node(
        package='rviz2',
        executable='rviz2',
        arguments=['-d', PathJoinSubstitution([pkg_ee4308_bringup, 'rviz', 'proj2.rviz'])],
        output='screen'
    )
    ld.add_action(node_rviz)

    # drone estimator composable node
    comp_node_drone_estimator = ComposableNode(
        namespace="drone",
        package='ee4308_drone',
        plugin='ee4308::drone::Estimator',
        name='estimator',
        parameters=[path_param_file, 
            {   # overrides yaml file
                'initial_x': LaunchConfiguration('drone_x'), 
                'initial_y': LaunchConfiguration('drone_y'), 
                'initial_z': LaunchConfiguration('drone_z'),
            }
        ],
        extra_arguments=[{'use_intra_process_comms': True},],
    )

    # drone planner composable node
    comp_node_drone_controller = ComposableNode(
        namespace="drone",
        package='ee4308_drone',
        plugin='ee4308::drone::Controller',
        name='controller',
        parameters=[path_param_file,],
        extra_arguments=[{'use_intra_process_comms': True}],
    )

    # drone planner composable node
    comp_node_drone_behavior = ComposableNode(
        namespace="drone",
        package='ee4308_drone',
        plugin='ee4308::drone::Behavior',
        name='behavior',
        parameters=[path_param_file, 
            {   # overrides yaml file
                'initial_x': LaunchConfiguration('drone_x'), 
                'initial_y': LaunchConfiguration('drone_y'), 
                'initial_z': LaunchConfiguration('drone_z'),
            }
        ],
        extra_arguments=[{'use_intra_process_comms': True}],
    )

    # drone planner composable node
    comp_node_drone_planner = ComposableNode(
        namespace="drone",
        package='ee4308_drone',
        plugin='ee4308::drone::Planner',
        name='planner',
        parameters=[path_param_file,],
        extra_arguments=[{'use_intra_process_comms': True}],
    )

    # drone composable container
    composable_container_drone = ComposableNodeContainer(
        name='drone',
        namespace='drone',
        package='rclcpp_components',
        executable='component_container',
        composable_node_descriptions=[
            comp_node_drone_behavior,
            comp_node_drone_planner,
            comp_node_drone_controller,
            comp_node_drone_estimator,
        ],
        output='screen',
    )
    ld.add_action(composable_container_drone)

    # turtle2 map_loader composable node
    path_map_file = PathJoinSubstitution([pkg_ee4308_bringup, "maps", LaunchConfiguration("map")])
    comp_node_turtle2_map_loader = ComposableNode(
        namespace="turtle",
        package='ee4308_turtle2',
        plugin='ee4308::turtle2::MapLoader',
        name='map_loader',
        parameters=[
            path_param_file,
            {"filepath": path_map_file,},  # override the filepath param (i.e. path to map)
        ],
        extra_arguments=[{'use_intra_process_comms': True}],
    )

    # turtle2 behavior composable node
    comp_node_turtle2_behavior = ComposableNode(
        namespace="turtle",
        package='ee4308_turtle2',
        plugin='ee4308::turtle2::Behavior',
        name='behavior',
        parameters=[
            path_param_file,
        ],
        extra_arguments=[{'use_intra_process_comms': True}],
    )

    # turtle2 controller composable node
    comp_node_turtle2_controller = ComposableNode(
        namespace="turtle",
        package='ee4308_turtle2',
        plugin='ee4308::turtle2::Controller',
        name='controller',
        parameters=[
            path_param_file,
        ],
        extra_arguments=[{'use_intra_process_comms': True}],
    )

    # turtle2 planner composable node
    comp_node_turtle2_planner = ComposableNode(
        namespace="turtle",
        package='ee4308_turtle2',
        plugin='ee4308::turtle2::Planner',
        name='planner',
        parameters=[
            path_param_file,
        ],
        extra_arguments=[{'use_intra_process_comms': True}],
    )
    
    # turtle2 composable container
    composable_container_turtle2 = ComposableNodeContainer(
        name='turtle2',
        namespace='turtle',
        package='rclcpp_components',
        executable='component_container',
        composable_node_descriptions=[
            comp_node_turtle2_behavior,
            comp_node_turtle2_planner,
            comp_node_turtle2_controller,
            comp_node_turtle2_map_loader,
        ],
        output='screen',
    )
    ld.add_action(composable_container_turtle2)
    return ld
