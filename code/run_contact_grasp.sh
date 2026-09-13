#!/usr/bin/env bash
# ROS 2 Humble's setup scripts read optional variables before defining them.
# Keep errexit/pipefail active, but enable nounset only after both setup files
# have been sourced.
set +u
set -eo pipefail

workspace_dir="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
cd "$workspace_dir"

if pgrep -x gzserver >/dev/null 2>&1 || pgrep -x gzclient >/dev/null 2>&1; then
  echo "已有 Gazebo 进程，请先关闭旧仿真后再运行。" >&2
  exit 2
fi

# Start from the system ROS installation so an unrelated overlay (for example
# the original development workspace) cannot override this submission.
unset AMENT_PREFIX_PATH COLCON_PREFIX_PATH CMAKE_PREFIX_PATH PYTHONPATH \
  ROS_PACKAGE_PATH GAZEBO_MODEL_PATH GAZEBO_PLUGIN_PATH
source /opt/ros/humble/setup.bash
# local_setup.bash adds this workspace without replaying any build-time parent
# overlay that may have been present when a previous setup.bash was generated.
source "$workspace_dir/install/local_setup.bash"
set -u
log_dir="$workspace_dir/logs/fixed_pick"
mkdir -p "$log_dir"
export GAZEBO_MASTER_URI="${GAZEBO_MASTER_URI:-http://127.0.0.1:11360}"

exec ros2 launch mecharm_fixed_pick_sim fixed_point_experiment.launch.py \
  contact_grasp:=true \
  gui:=true \
  run_task:=true \
  repeat_count:=5 \
  exit_on_complete:=true \
  task_start_delay_sec:=20.0 \
  log_directory:="$log_dir"
