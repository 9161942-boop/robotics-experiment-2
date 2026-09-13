import os
from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, EmitEvent, RegisterEventHandler
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def _workspace_log_directory(package_share):
    explicit = os.environ.get("MECHARM_FIXED_PICK_LOG_DIR", "").strip()
    if explicit:
        return Path(explicit).expanduser()
    for parent in package_share.parents:
        if (parent / "src").is_dir() and (parent / "install").is_dir():
            return parent / "logs" / "fixed_pick"
    return Path.home() / ".ros" / "mecharm_fixed_pick" / "logs" / "fixed_pick"


def generate_launch_description():
    sim_share = Path(get_package_share_directory("mecharm_fixed_pick_sim"))
    description_share = Path(get_package_share_directory("mycobot_description"))

    model_default = (
        description_share
        / "urdf"
        / "mecharm_270_m5"
        / "mecharm_270_m5_adaptive_gripper.urdf"
    )
    config_default = sim_share / "config" / "fixed_pick.yaml"
    rviz_default = sim_share / "config" / "fixed_pick.rviz"
    log_directory_default = _workspace_log_directory(sim_share)

    model = LaunchConfiguration("model")
    config_file = LaunchConfiguration("config_file")
    log_directory = LaunchConfiguration("log_directory")
    repeat_count = LaunchConfiguration("repeat_count")
    launch_rviz = LaunchConfiguration("launch_rviz")
    exit_on_complete = LaunchConfiguration("exit_on_complete")
    step_duration_sec = LaunchConfiguration("step_duration_sec")
    settle_duration_sec = LaunchConfiguration("settle_duration_sec")
    fault_mode = LaunchConfiguration("fault_mode")
    command_mode = LaunchConfiguration("command_mode")

    robot_description = ParameterValue(Command(["xacro ", model]), value_type=str)

    task_node = Node(
        package="mecharm_fixed_pick_sim",
        executable="fixed_pick_task",
        name="fixed_pick_task",
        output="screen",
            parameters=[
                {"config_file": config_file},
                {"log_directory": log_directory},
                {"repeat_count": ParameterValue(repeat_count, value_type=int)},
            {"exit_on_complete": ParameterValue(exit_on_complete, value_type=bool)},
            {"step_duration_sec": ParameterValue(step_duration_sec, value_type=float)},
            {"settle_duration_sec": ParameterValue(settle_duration_sec, value_type=float)},
            {"fault_mode": fault_mode},
            {"command_mode": command_mode},
        ],
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument("model", default_value=str(model_default)),
            DeclareLaunchArgument("config_file", default_value=str(config_default)),
            DeclareLaunchArgument("log_directory", default_value=str(log_directory_default)),
            DeclareLaunchArgument("repeat_count", default_value="5"),
            DeclareLaunchArgument("launch_rviz", default_value="true"),
            DeclareLaunchArgument("exit_on_complete", default_value="false"),
            DeclareLaunchArgument("step_duration_sec", default_value="0.0"),
            DeclareLaunchArgument("settle_duration_sec", default_value="-1.0"),
            DeclareLaunchArgument("fault_mode", default_value="none"),
            DeclareLaunchArgument("command_mode", default_value="joint_state"),
            Node(
                package="robot_state_publisher",
                executable="robot_state_publisher",
                name="robot_state_publisher",
                output="screen",
                parameters=[{"robot_description": robot_description}],
            ),
            task_node,
            RegisterEventHandler(
                OnProcessExit(
                    target_action=task_node,
                    on_exit=[
                        EmitEvent(
                            event=Shutdown(reason="fixed_pick_task completed")
                        )
                    ],
                ),
                condition=IfCondition(exit_on_complete),
            ),
            Node(
                package="rviz2",
                executable="rviz2",
                name="rviz2",
                output="screen",
                arguments=["-d", str(rviz_default)],
                condition=IfCondition(launch_rviz),
            ),
        ]
    )
