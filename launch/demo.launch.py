"""Isolated TurtleBot4 maze, AMCL/Nav2 and Vietnamese voice nodes."""

import os
from ament_index_python.packages import get_package_share_directory as share
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    ExecuteProcess,
    SetEnvironmentVariable,
    TimerAction,
)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration as C
from launch_ros.actions import Node


def generate_launch_description():
    p = share("bai07_turtlebot4")
    sim = share("turtlebot4_ignition_bringup")
    nav = share("turtlebot4_navigation")

    def inc(root, file, args):
        return IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(root, "launch", file)),
            launch_arguments=args.items(),
        )

    common = {
        "use_sim_time": True,
        "regions_file": os.path.join(p, "config/regions.yaml"),
        "output_dir": C("output_dir"),
    }
    return LaunchDescription(
        [
            DeclareLaunchArgument("world", default_value="maze"),
            DeclareLaunchArgument(
                "output_dir", default_value=os.path.expanduser("~/bai07_runs/current")
            ),
            DeclareLaunchArgument("asr_python", default_value="/usr/bin/python3"),
            DeclareLaunchArgument("speech", default_value="true"),
            DeclareLaunchArgument("panel", default_value="true"),
            DeclareLaunchArgument("show_rviz", default_value="true"),
            DeclareLaunchArgument("gazebo_gui", default_value="true"),
            SetEnvironmentVariable(
                "IGN_GAZEBO_RESOURCE_PATH",
                ":".join(
                    [
                        os.path.join(sim, "worlds"),
                        "/opt/ros/humble/share",
                        os.environ.get("IGN_GAZEBO_RESOURCE_PATH", ""),
                    ]
                ),
            ),
            SetEnvironmentVariable(
                "IGN_GAZEBO_SYSTEM_PLUGIN_PATH",
                "/opt/ros/humble/lib:" + os.environ.get("LD_LIBRARY_PATH", ""),
            ),
            SetEnvironmentVariable(
                "IGN_GUI_PLUGIN_PATH",
                ":".join(
                    [
                        os.path.join(share("turtlebot4_ignition_gui_plugins"), "lib"),
                        os.path.join(share("irobot_create_ignition_plugins"), "lib"),
                    ]
                ),
            ),
            ExecuteProcess(
                cmd=[
                    "ign",
                    "gazebo",
                    "-s",
                    os.path.join(sim, "worlds/maze.sdf"),
                    "-r",
                    "-v",
                    "2",
                ],
                output="screen",
            ),
            TimerAction(
                period=5.0,
                actions=[
                    ExecuteProcess(
                        cmd=[
                            "ign",
                            "gazebo",
                            "-g",
                            "-v",
                            "2",
                            "--gui-config",
                            os.path.join(p, "config/gazebo_gui.config"),
                        ],
                        condition=IfCondition(C("gazebo_gui")),
                        output="screen",
                    )
                ],
            ),
            Node(
                package="ros_gz_bridge",
                executable="parameter_bridge",
                name="bai07_clock",
                arguments=["/clock@rosgraph_msgs/msg/Clock[ignition.msgs.Clock"],
            ),
            inc(
                sim,
                "turtlebot4_spawn.launch.py",
                {
                    "world": "maze",
                    "model": "standard",
                    "rviz": "false",
                    "x": "0",
                    "y": "0",
                    "yaw": "0",
                },
            ),
            TimerAction(
                period=10.0,
                actions=[
                    inc(
                        nav,
                        "localization.launch.py",
                        {
                            "map": os.path.join(p, "config/map_navigation.yaml"),
                            "use_sim_time": "true",
                            "params": os.path.join(p, "config/localization.yaml"),
                        },
                    )
                ],
            ),
            TimerAction(
                period=20.0,
                actions=[
                    inc(
                        nav,
                        "nav2.launch.py",
                        {
                            "use_sim_time": "true",
                            "params_file": os.path.join(p, "config/nav2.yaml"),
                        },
                    ),
                    Node(
                        package="bai07_turtlebot4",
                        executable="prepare_robot",
                        parameters=[{"use_sim_time": True}],
                        output="screen",
                    ),
                ],
            ),
            TimerAction(
                period=28.0,
                actions=[
                    Node(
                        package="bai07_turtlebot4",
                        executable="evidence_recorder",
                        parameters=[common],
                        output="screen",
                    ),
                    Node(
                        package="bai07_turtlebot4",
                        executable="semantic_map_node",
                        parameters=[common],
                        output="screen",
                    ),
                    Node(
                        package="bai07_turtlebot4",
                        executable="voice_navigation_node",
                        parameters=[common],
                        output="screen",
                    ),
                    Node(
                        package="bai07_turtlebot4",
                        executable="speech_recognition_node",
                        prefix=[C("asr_python")],
                        parameters=[
                            {
                                "output_dir": C("output_dir"),
                                "config_file": os.path.join(p, "config/speech.yaml"),
                            }
                        ],
                        condition=IfCondition(C("speech")),
                        output="screen",
                    ),
                    Node(
                        package="bai07_turtlebot4",
                        executable="demo_panel",
                        parameters=[
                            {"use_sim_time": False, "output_dir": C("output_dir")}
                        ],
                        condition=IfCondition(C("panel")),
                        output="screen",
                    ),
                    Node(
                        package="rviz2",
                        executable="rviz2",
                        arguments=["-d", os.path.join(p, "rviz/navigation.rviz")],
                        parameters=[{"use_sim_time": True}],
                        condition=IfCondition(C("show_rviz")),
                        output="screen",
                    ),
                ],
            ),
        ]
    )
