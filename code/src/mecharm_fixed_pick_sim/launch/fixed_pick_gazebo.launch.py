from pathlib import Path
import os
import shutil
import tempfile

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    EmitEvent,
    IncludeLaunchDescription,
    OpaqueFunction,
    RegisterEventHandler,
    SetEnvironmentVariable,
    SetLaunchConfiguration,
    TimerAction,
)
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch.launch_description_sources import PythonLaunchDescriptionSource
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


def _prepare_runtime_controllers(context, *args, **kwargs):
    source = Path(LaunchConfiguration("controllers_file").perform(context)).expanduser()
    destination = Path(tempfile.gettempdir()) / "mecharm_fixed_pick_sim_controllers.yaml"
    if source.resolve() != destination.resolve():
        shutil.copyfile(source, destination)
    return [SetLaunchConfiguration("runtime_controllers_file", str(destination))]


def generate_launch_description():
    sim_share = Path(get_package_share_directory("mecharm_fixed_pick_sim"))
    gazebo_share = Path(get_package_share_directory("gazebo_ros"))
    description_share = Path(get_package_share_directory("mycobot_description"))

    gazebo_model_paths = [
        str(description_share.parent),
        "/usr/share/gazebo-11/models",
    ]
    existing_gazebo_model_path = os.environ.get("GAZEBO_MODEL_PATH")
    if existing_gazebo_model_path:
        gazebo_model_paths.append(existing_gazebo_model_path)

    model_default = sim_share / "urdf" / "mecharm_270_gazebo.urdf.xacro"
    config_default = sim_share / "config" / "fixed_pick.yaml"
    controllers_default = sim_share / "config" / "controllers.yaml"
    world_default = sim_share / "worlds" / "fixed_pick.world"
    log_directory_default = _workspace_log_directory(sim_share)

    model = LaunchConfiguration("model")
    config_file = LaunchConfiguration("config_file")
    controllers_file = LaunchConfiguration("controllers_file")
    runtime_controllers_file = LaunchConfiguration("runtime_controllers_file")
    log_directory = LaunchConfiguration("log_directory")
    repeat_count = LaunchConfiguration("repeat_count")
    launch_gazebo = LaunchConfiguration("launch_gazebo")
    gui = LaunchConfiguration("gui")
    exit_on_complete = LaunchConfiguration("exit_on_complete")
    step_duration_sec = LaunchConfiguration("step_duration_sec")
    settle_duration_sec = LaunchConfiguration("settle_duration_sec")
    fault_mode = LaunchConfiguration("fault_mode")
    task_start_delay_sec = LaunchConfiguration("task_start_delay_sec")

    robot_description = ParameterValue(
        Command(["xacro ", model, " controllers_file:=", runtime_controllers_file]),
        value_type=str,
    )

    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(str(gazebo_share / "launch" / "gazebo.launch.py")),
        launch_arguments={"world": str(world_default), "verbose": "true", "gui": gui}.items(),
        condition=IfCondition(launch_gazebo),
    )

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
            {"command_mode": "trajectory"},
            {"physical_grasp_enabled": ParameterValue(launch_gazebo, value_type=bool)},
        ],
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument("model", default_value=str(model_default)),
            DeclareLaunchArgument("config_file", default_value=str(config_default)),
            DeclareLaunchArgument("controllers_file", default_value=str(controllers_default)),
            DeclareLaunchArgument("log_directory", default_value=str(log_directory_default)),
            DeclareLaunchArgument("repeat_count", default_value="5"),
            DeclareLaunchArgument("launch_gazebo", default_value="true"),
            DeclareLaunchArgument("gui", default_value="true"),
            DeclareLaunchArgument("exit_on_complete", default_value="false"),
            DeclareLaunchArgument("step_duration_sec", default_value="0.0"),
            DeclareLaunchArgument("settle_duration_sec", default_value="-1.0"),
            DeclareLaunchArgument("fault_mode", default_value="none"),
            # High-resolution official DAE visuals can make first Gazebo startup
            # slower than the simplified model; leave enough time for services.
            DeclareLaunchArgument("task_start_delay_sec", default_value="20.0"),
            SetEnvironmentVariable(
                "GAZEBO_MODEL_PATH", os.pathsep.join(gazebo_model_paths)
            ),
            OpaqueFunction(function=_prepare_runtime_controllers),
            gazebo,
            Node(
                package="robot_state_publisher",
                executable="robot_state_publisher",
                name="robot_state_publisher",
                output="screen",
                parameters=[{"robot_description": robot_description}],
            ),
            Node(
                package="gazebo_ros",
                executable="spawn_entity.py",
                arguments=["-topic", "robot_description", "-entity", "mecharm_270"],
                output="screen",
                condition=IfCondition(launch_gazebo),
            ),
            Node(
                package="controller_manager",
                executable="spawner",
                arguments=["joint_state_broadcaster", "--controller-manager", "/controller_manager"],
                output="screen",
                condition=IfCondition(launch_gazebo),
            ),
            Node(
                package="controller_manager",
                executable="spawner",
                arguments=["arm_controller", "--controller-manager", "/controller_manager"],
                output="screen",
                condition=IfCondition(launch_gazebo),
            ),
            Node(
                package="controller_manager",
                executable="spawner",
                arguments=["gripper_trajectory_controller", "--controller-manager", "/controller_manager"],
                output="screen",
                condition=IfCondition(launch_gazebo),
            ),
            TimerAction(period=task_start_delay_sec, actions=[task_node]),
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
        ]
    )
