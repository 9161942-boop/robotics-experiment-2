# mechArm Fixed Pick Simulation

This ROS 2 package provides the simulation-side workflow for the fixed-point
pick-and-place lab.

## What It Starts

- mechArm 270 M5 adaptive-gripper URDF from `mycobot_description`
- `robot_state_publisher`
- `fixed_pick_task`, which publishes animated `/joint_states`
- RViz scene markers for the table, pick point A, place point B, and target
- Optional Gazebo Classic scene with the table, physical target object, and place area
- Gazebo physical grasp assistance, object-state validation, and reset between cycles

## Build

```bash
cd /home/hcl/桌面/mecharm_fixed_pick_submission_ws
colcon build --packages-select mycobot_description mecharm_fixed_pick_sim
source install/local_setup.bash
```

## Run RViz Simulation

```bash
ros2 launch mecharm_fixed_pick_sim fixed_pick_rviz.launch.py
```

Headless verification:

```bash
ros2 launch mecharm_fixed_pick_sim fixed_pick_rviz.launch.py \
  launch_rviz:=false exit_on_complete:=true step_duration_sec:=0.05 settle_duration_sec:=0.0
```

## Run Gazebo Scene

```bash
ros2 launch mecharm_fixed_pick_sim fixed_pick_gazebo.launch.py gui:=false
```

The Gazebo launch loads the lab scene, starts `gazebo_ros2_control`, and sends
the same fixed-pick sequence to `arm_controller` and
`gripper_trajectory_controller`. The arm's visual layer uses the official
mechArm 270 M5 `base.dae` and `link1.dae`-`link6.dae` meshes from
`mycobot_description`; the existing simplified boxes/cylinder remain as the
inertial collision geometry so the validated physics and grasp behavior are
unchanged. The simplified gripper visual is retained for the current
single-joint control model.

The launch also adds the installed `mycobot_description/share` directory to
`GAZEBO_MODEL_PATH`, which is required for Gazebo Classic to resolve the
`model://mycobot_description/...` paths generated from the ROS package URIs.

The legacy `fixed_pick_gazebo.launch.py` path can also load the system
`gazebo_ros_vacuum_gripper` plugin. The submission command below uses
`official_assembled_gazebo.launch.py contact_grasp:=true` instead, so it starts
the two-finger contact-pad controllers and the validated contact-mode task
logic without enabling vacuum.

At launch time the controller YAML is copied to
`/tmp/mecharm_fixed_pick_sim_controllers.yaml` because Gazebo Classic on this
machine fails when `gazebo_ros2_control` reads a parameter file from the Chinese
workspace path.

Fast Gazebo launch smoke test without opening Gazebo:

```bash
ros2 launch mecharm_fixed_pick_sim fixed_pick_gazebo.launch.py \
  launch_gazebo:=false exit_on_complete:=true task_start_delay_sec:=0.0 \
  repeat_count:=1 step_duration_sec:=0.05 settle_duration_sec:=0.0
```

Five-cycle physical acceptance test:

```bash
ros2 launch mecharm_fixed_pick_sim fixed_pick_gazebo.launch.py \
  gui:=false exit_on_complete:=true repeat_count:=5 \
  task_start_delay_sec:=20.0 step_duration_sec:=0.9 settle_duration_sec:=0.3
```

The acceptance rule is at least four successful placements out of five. A
placement is successful when the Gazebo target object finishes inside the
configured XY and height tolerances around point B.

## Run the Full Official Model

The separate full-model launch uses the official mechArm 270 M5 arm meshes and
the complete adaptive gripper (`gripper_left1/2/3` and `gripper_right1/2/3`).
It keeps lightweight box/cylinder collision geometry and exposes the six arm
joints plus the master gripper joint through dedicated controllers. It also
runs the same A-to-B fixed-pick task using the full model; the original launch
remains available as the lightweight, already-validated baseline.

```bash
ros2 launch mecharm_fixed_pick_sim full_model_gazebo.launch.py \
  gui:=true repeat_count:=1 exit_on_complete:=false
```

The task starts automatically after Gazebo and its controllers are ready. Use
`exit_on_complete:=true` when you want the launch to close after the run.

For the strict official assembly plus the matching A/B trajectory, use
`official_assembled_gazebo.launch.py` with `run_task:=true`. Its task poses are
calibrated for the official joint origins; the regular `full_model_gazebo.py`
entry remains the already-validated display/task variant.

The full-model visual origins are registered to the fixed-pick controller frame
so the official meshes stay assembled while reusing the validated A/B poses.
In contact mode the master gripper joint drives the mimic links and the two
contact pads are commanded through `contact_pad_effort_controller`. The task
keeps the validated visual object attachment and A/B placement checks used by
this submission version; no vacuum command is sent by the one-click script.

Keep the window open while inspecting the model. In another terminal, after
loading the same ROS environment, the arm can be commanded with a trajectory:

```bash
ros2 topic pub --once /full_arm_controller/joint_trajectory \
  trajectory_msgs/msg/JointTrajectory \
  "{joint_names: [joint1_to_base, joint2_to_joint1, joint3_to_joint2, joint4_to_joint3, joint5_to_joint4, joint6_to_joint5], points: [{positions: [0.0, 0.4, -0.8, 0.0, 0.6, 0.0], time_from_start: {sec: 3}}]}"
```

The adaptive gripper is controlled through its master joint; the other five
gripper joints follow through URDF mimic relationships:

```bash
ros2 topic pub --once /full_gripper_controller/joint_trajectory \
  trajectory_msgs/msg/JointTrajectory \
  "{joint_names: [gripper_controller], points: [{positions: [-0.5], time_from_start: {sec: 2}}]}"
```

Use `0.0` for a more open pose and negative values toward the closed pose.

## Topics

- `/joint_states`: simulated arm and gripper joint positions
- `/mecharm_task/state`: state-machine progress
- `/mecharm_task/result`: JSON summary with success count
- `/mecharm_task/scene`: RViz marker scene
- `/mecharm_grasp/grasping`: Gazebo physical-grasp confirmation
- `/gazebo/model_states`: real Gazebo object positions used for validation

Services used by the Gazebo task:

- `/mecharm_grasp/switch`: enable or release the physical grasp
- `/gazebo/set_entity_state`: reset the target object to point A

## Parameters

Edit `config/fixed_pick.yaml` for:

- fixed pick point A and place point B
- safe height
- home, pick, lift, place joint poses
- repeat count and log directory
- joint limits and gripper open/close values
- placement tolerances and per-step settle times

Logs are written to `/home/hcl/桌面/mecharm_fixed_pick_submission_ws/logs/fixed_pick`
when launched from this submission workspace. The launch files also accept
`log_directory:=...` and the `MECHARM_FIXED_PICK_LOG_DIR` environment variable.

Unreachable-target safety check:

```bash
ros2 launch mecharm_fixed_pick_sim fixed_pick_rviz.launch.py \
  launch_rviz:=false exit_on_complete:=true fault_mode:=unreachable
```
# MechArm 270 手动仿真控制

先启动 Gazebo（接触双指模式）：

```bash
cd /home/hcl/桌面/mecharm_fixed_pick_submission_ws
source /opt/ros/humble/setup.bash
source install/local_setup.bash
ros2 launch mecharm_fixed_pick_sim official_assembled_gazebo.launch.py gui:=true run_task:=false contact_grasp:=true
```

保持该终端运行，再开第二个终端启动鼠标控制面板：

```bash
cd /home/hcl/桌面/mecharm_fixed_pick_submission_ws
source /opt/ros/humble/setup.bash
source install/local_setup.bash
ros2 run mecharm_fixed_pick_sim manual_control_panel
```

面板中拖动 6 个关节滑块、夹爪滑块和接触垫夹持力滑块，然后点击“发送当前姿态”。动作时间决定控制器从当前姿态移动到目标姿态所需的时间。可以按“复位物块到 A 点”重新开始实验。Gazebo 窗口负责观察，面板负责用鼠标逐步操控完整流程。

推荐手动流程：先把机械臂调到 A 点上方，再下探，夹爪滑块调到负值并把接触垫力调为正值完成夹持，随后逐步调节关节搬运到 B 点，最后将夹爪滑块调回正值并把接触垫力调回负值释放。

如果碰撞夹持不稳定，建议使用真空模式：启动 Gazebo 时设置 `contact_grasp:=false`。到达 `pick` 点并闭合后点击面板中的“真空吸附 ON”，抬升和搬运结束后点击“真空释放 OFF”。

播放已保存动画时，面板会按点位名称自动切换真空：名称为 `pick`、`grasp` 或 `grasp_object` 时自动吸附；名称为 `release` 或 `release_object` 时自动释放。因此可直接使用 `initial → above_a → crab → pick → lift → initial → release_ready → release → initial` 序列。自动真空功能仅在 `contact_grasp:=false` 的真空模式下有效。

### 制作可重复动画

在面板的“点位与动画序列”区域，调好姿态后输入名称并点击“保存当前点位”。点位会保存六个关节角、夹爪位置、接触垫力和动作时间。选中点位后点击“加入序列”，按顺序加入 `home`、`above_pick`、`down_pick`、`grasp_object`、`lift_object`、`move_to_place`、`release_object` 等点位，最后点击“播放序列”。

点位和序列会自动写入启动面板时所在目录的 `manual_animation.json`；覆盖前会保留 `manual_animation.json.bak`。下次启动面板会自动载入，可继续编辑或重放。

当前接触模式的目标物块和 A 点支撑已永久移到 `(x=0.30, y=0.18, z=0.031)`，比机械臂底座更远。由于原自动轨迹是旧 A 点示教数据，请使用 `manual_control_panel` 重新记录 `above_pick`、`down_pick`、`grasp_object` 等点位后再播放动画。
