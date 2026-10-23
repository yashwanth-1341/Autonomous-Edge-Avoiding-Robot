import os
from os import pathsep
from pathlib import Path
from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, RegisterEventHandler, SetEnvironmentVariable
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessExit
from launch.substitutions import Command, LaunchConfiguration, PathJoinSubstitution, PythonExpression

from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue

def generate_launch_description():
    bot_description = get_package_share_directory("bot_description")

    model_arg = DeclareLaunchArgument(
        name="model", default_value=os.path.join(
                bot_description, "urdf", "bot.urdf.xacro"
            ),
        description="Absolute path to robot urdf file"
    )

    world_name_arg = DeclareLaunchArgument(name="world_name", default_value="empty")

    # The Gazebo GUI cannot start without a display, so default to headless when none is available.
    has_display = bool(os.environ.get("DISPLAY") or os.environ.get("WAYLAND_DISPLAY"))
    gui_arg = DeclareLaunchArgument(
        name="gui", default_value=str(has_display).lower(),
        description="Start the Gazebo GUI (false runs the server headless)"
    )

    world_path = PathJoinSubstitution([
            bot_description,
            "worlds",
            PythonExpression(expression=["'", LaunchConfiguration("world_name"), "'", " + '.world'"])
        ]
    )

    model_path = str(Path(bot_description).parent.resolve())
    model_path += pathsep + os.path.join(get_package_share_directory("bot_description"), 'models')

    gazebo_resource_path = SetEnvironmentVariable(
        "GZ_SIM_RESOURCE_PATH",
        model_path
    )

    robot_description = ParameterValue(Command([
            "xacro ",
            LaunchConfiguration("model"),
            " is_sim:=True",
        ]),
        value_type=str
    )

    robot_state_publisher_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        parameters=[{"robot_description": robot_description,
                     "use_sim_time": True}]
    )

    # Run `gz sim` directly rather than via ros_gz_sim's gz_sim.launch.py: that one wraps it in
    # `sh -c`, so Ctrl-C only reaches the shell and leaves Gazebo server/GUI running as orphans.
    gz_env = {
        "GZ_SIM_SYSTEM_PLUGIN_PATH": pathsep.join(filter(None, [
            os.environ.get("GZ_SIM_SYSTEM_PLUGIN_PATH"),
            os.environ.get("LD_LIBRARY_PATH"),  # gz_ros2_control-system lives here
        ])),
    }
    gazebo_server = ExecuteProcess(
        cmd=["gz", "sim", world_path, "-v", "4", "-r", "-s"],
        name="gazebo", output="screen", additional_env=gz_env,
    )

    gz_spawn_entity = Node(
        package="ros_gz_sim",
        executable="create",
        output="screen",
        arguments=[
            "-topic", "robot_description",
            "-name", "bot",
            "-x", "0.0",  
            "-y", "0.0",  
            "-z", "2.0",  
            "-R", "0.0", 
            "-P", "0.0",
            "-Y", "0.0",
        ],
    )

    # The GUI runs as its own process (opened once the robot is spawned), with a config
    # that starts the camera close to the robot.
    gazebo_gui = RegisterEventHandler(OnProcessExit(
        target_action=gz_spawn_entity,
        on_exit=[ExecuteProcess(
            cmd=["gz", "sim", "-g", "-v", "4",
                 "--gui-config", os.path.join(bot_description, "config", "gui.config")],
            name="gazebo_gui", output="screen",
            condition=IfCondition(LaunchConfiguration("gui")),
        )],
    ))

    gz_ros2_bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        arguments=[
            "/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock",
            "/scan@sensor_msgs/msg/LaserScan[gz.msgs.LaserScan",
        ],
    )

    return LaunchDescription([
        model_arg,
        world_name_arg,
        gui_arg,
        gazebo_resource_path,
        robot_state_publisher_node,
        gazebo_server,
        gz_spawn_entity,
        gazebo_gui,
        gz_ros2_bridge,
    ])