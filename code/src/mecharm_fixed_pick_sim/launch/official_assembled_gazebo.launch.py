from pathlib import Path
import os
import shutil
import tempfile

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, EmitEvent, IncludeLaunchDescription, OpaqueFunction, RegisterEventHandler, SetEnvironmentVariable, SetLaunchConfiguration, TimerAction
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, LaunchConfiguration, PythonExpression
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def _prepare_controllers(context, *args, **kwargs):
    contact_mode = LaunchConfiguration("contact_grasp").perform(context).lower() == "true"
    source = Path(LaunchConfiguration("controllers_file").perform(context)).expanduser()
    task_config = Path(LaunchConfiguration("config_file").perform(context)).expanduser()
    world = Path(LaunchConfiguration("world").perform(context)).expanduser()
    if contact_mode:
        source = Path(LaunchConfiguration("contact_controllers_file").perform(context)).expanduser()
        world = Path(LaunchConfiguration("contact_world").perform(context)).expanduser()
    destination = Path(tempfile.gettempdir()) / "mecharm_fixed_pick_sim_full_controllers.yaml"
    if source.resolve() != destination.resolve():
        shutil.copyfile(source, destination)
    return [
        SetLaunchConfiguration("runtime_controllers_file", str(destination)),
        SetLaunchConfiguration("runtime_config_file", str(task_config)),
        SetLaunchConfiguration("runtime_world", str(world)),
    ]


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
    gazebo_share = Path(get_package_share_directory("gazebo_ros"))
    description_share = Path(get_package_share_directory("mycobot_description"))
    model_paths = [str(description_share.parent), "/usr/share/gazebo-11/models"]
    if os.environ.get("GAZEBO_MODEL_PATH"):
        model_paths.append(os.environ["GAZEBO_MODEL_PATH"])

    model = LaunchConfiguration("model")
    runtime = LaunchConfiguration("runtime_controllers_file")
    repeat_count = LaunchConfiguration("repeat_count")
    exit_on_complete = LaunchConfiguration("exit_on_complete")
    task_start_delay_sec = LaunchConfiguration("task_start_delay_sec")
    log_directory = LaunchConfiguration("log_directory")
    config_file = LaunchConfiguration("runtime_config_file")
    contact_grasp = LaunchConfiguration("contact_grasp")
    description = ParameterValue(
        Command([
            "xacro ", model,
            " controllers_file:=", runtime,
            " contact_grasp:=", contact_grasp,
        ]),
        value_type=str,
    )
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(str(gazebo_share / "launch" / "gazebo.launch.py")),
        launch_arguments={"world": LaunchConfiguration("runtime_world"), "verbose": "true", "gui": LaunchConfiguration("gui")}.items(),
    )

    def spawner(name):
        return Node(package="controller_manager", executable="spawner", arguments=[name, "--controller-manager", "/controller_manager"], output="screen")

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
            {"command_mode": "trajectory"},
            {"arm_command_topic": "full_arm_controller/joint_trajectory"},
            {"gripper_command_topic": "full_gripper_controller/joint_trajectory"},
            {"physical_grasp_enabled": ParameterValue(
                PythonExpression(["'", contact_grasp, "' != 'true'"]),
                value_type=bool,
            )},
            {"object_tracking_enabled": True},
            {"visual_attachment_enabled": ParameterValue(
                PythonExpression(["'", contact_grasp, "' == 'true'"]),
                value_type=bool,
            )},
            {"explicit_gripper_control": ParameterValue(
                PythonExpression(["'", contact_grasp, "' == 'true'"]),
                value_type=bool,
            )},
        ],
    )

    return LaunchDescription([
        DeclareLaunchArgument("model", default_value=str(sim_share / "urdf" / "mecharm_270_official_assembled.urdf.xacro")),
        DeclareLaunchArgument("controllers_file", default_value=str(sim_share / "config" / "full_model_controllers.yaml")),
        DeclareLaunchArgument("contact_controllers_file", default_value=str(sim_share / "config" / "contact_model_controllers.yaml")),
        DeclareLaunchArgument("config_file", default_value=str(sim_share / "config" / "official_fixed_pick.yaml")),
        DeclareLaunchArgument("contact_config_file", default_value=str(sim_share / "config" / "fixed_point_experiment.yaml")),
        DeclareLaunchArgument("world", default_value=str(sim_share / "worlds" / "fixed_pick.world")),
        DeclareLaunchArgument("contact_world", default_value=str(sim_share / "worlds" / "contact_fixed_pick.world")),
        DeclareLaunchArgument("gui", default_value="true"),
        DeclareLaunchArgument("repeat_count", default_value="1"),
        DeclareLaunchArgument("exit_on_complete", default_value="false"),
        DeclareLaunchArgument("task_start_delay_sec", default_value="20.0"),
        DeclareLaunchArgument("run_task", default_value="false"),
        DeclareLaunchArgument(
            "contact_grasp",
            default_value="false",
            description="Use two-finger collision/friction grasping instead of vacuum attachment",
        ),
        DeclareLaunchArgument("log_directory", default_value=str(_workspace_log_directory(sim_share))),
        SetEnvironmentVariable("GAZEBO_MODEL_PATH", os.pathsep.join(model_paths)),
        OpaqueFunction(function=_prepare_controllers),
        gazebo,
        Node(package="robot_state_publisher", executable="robot_state_publisher", name="robot_state_publisher", output="screen", parameters=[{"robot_description": description}]),
        Node(package="gazebo_ros", executable="spawn_entity.py", arguments=["-topic", "robot_description", "-entity", "mecharm_270_full"], output="screen"),
        spawner("joint_state_broadcaster"),
        spawner("full_arm_controller"),
        spawner("full_gripper_controller"),
        Node(
            package="controller_manager",
            executable="spawner",
            arguments=["contact_pad_effort_controller", "--controller-manager", "/controller_manager"],
            output="screen",
            condition=IfCondition(contact_grasp),
        ),
        TimerAction(period=task_start_delay_sec, actions=[task_node], condition=IfCondition(LaunchConfiguration("run_task"))),
        RegisterEventHandler(
            OnProcessExit(
                target_action=task_node,
                on_exit=[EmitEvent(event=Shutdown(reason="full fixed_pick_task completed"))],
            ),
            condition=IfCondition(exit_on_complete),
        ),
    ])
