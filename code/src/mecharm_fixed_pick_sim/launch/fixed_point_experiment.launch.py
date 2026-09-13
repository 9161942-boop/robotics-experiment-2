import os
from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def _workspace_log_directory(package_share):
    """Choose a portable log directory for this installed workspace."""
    explicit = os.environ.get("MECHARM_FIXED_PICK_LOG_DIR", "").strip()
    if explicit:
        return Path(explicit).expanduser()
    for parent in package_share.parents:
        if (parent / "src").is_dir() and (parent / "install").is_dir():
            return parent / "logs" / "fixed_pick"
    return Path.home() / ".ros" / "mecharm_fixed_pick" / "logs" / "fixed_pick"


def generate_launch_description():
    share = Path(get_package_share_directory("mecharm_fixed_pick_sim"))
    official = share / "launch" / "official_assembled_gazebo.launch.py"
    config = share / "config" / "fixed_point_experiment.yaml"
    return LaunchDescription([
        DeclareLaunchArgument(
            "contact_grasp",
            default_value="false",
            description="Use two-finger contact grasping on the validated experiment trajectory",
        ),
        DeclareLaunchArgument("gui", default_value="true"),
        DeclareLaunchArgument("run_task", default_value="false"),
        DeclareLaunchArgument("repeat_count", default_value="5"),
        DeclareLaunchArgument("exit_on_complete", default_value="false"),
        DeclareLaunchArgument("task_start_delay_sec", default_value="20.0"),
        DeclareLaunchArgument(
            "log_directory",
            default_value=str(_workspace_log_directory(share)),
            description="Directory for fixed-pick CSV validation logs",
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(str(official)),
            launch_arguments={
                "config_file": str(config),
                "contact_grasp": LaunchConfiguration("contact_grasp"),
                "gui": LaunchConfiguration("gui"),
                "run_task": LaunchConfiguration("run_task"),
                "repeat_count": LaunchConfiguration("repeat_count"),
                "exit_on_complete": LaunchConfiguration("exit_on_complete"),
                "task_start_delay_sec": LaunchConfiguration("task_start_delay_sec"),
                "log_directory": LaunchConfiguration("log_directory"),
            }.items(),
        ),
    ])
