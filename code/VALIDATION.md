# Validation record

验证日期：2026-09-01（PDT）

## 固定版本

- `pick` 主夹爪映射：`-0.04`
- `release` 主夹爪映射：`+0.12`
- 轨迹：`initial → above_a → crab → pick → lift → initial_2 → release_ready → release → initial`
- 保存快照中的 `fixed_point_experiment.yaml` SHA-256：
  `4f4bdb2d23d3f2115a8ad09dbd7b7444f9a27e5e47967d083e18bfc6fb263575`
- 提交副本当前 SHA-256：
  `a5e8135f66c1b33d9e6bcc3f7203d77c5d95cf753cf047bda21a3a27bf23c4db`
  （仅将 `task.log_directory` 改为便携路径；`poses`、`sequence` 和其余配置完全一致。）

## 构建检查

- Python launch/task syntax：PASS
- contact-mode Xacro：PASS
- `check_urdf`：PASS
- `colcon build --packages-select mycobot_description mecharm_fixed_pick_sim --symlink-install`：PASS

## Gazebo 验证

单循环：PASS（1/1；路径清理后的最终副本）

- CSV：`logs/fixed_pick/fixed_pick_20260901_091540.csv`
- 控制器：`full_arm_controller`、`full_gripper_controller`、
  `contact_pad_effort_controller`、`joint_state_broadcaster` 均为
  `Configured and activated`
- `pick` 后记录了 `grasp_attached`
- `release` 时读取目标实际坐标约 `(0.2444, -0.1231, 0.0400)`
- 随后只改变 Z，最终约为 `(0.2444, -0.1231, 0.0200)`，XY 误差约 `0.0445 m`，在容差内
- 最终：`status: passed`

另外，在清空 `AMENT_PREFIX_PATH`、`COLCON_PREFIX_PATH`、`CMAKE_PREFIX_PATH` 和
`PYTHONPATH` 后，用只包含 `/opt/ros/humble` 与临时纯净安装前缀的环境执行过
单循环，结果同样为 `status: passed`；该验证日志位于临时目录，不纳入提交包。

5 次回归：PASS（5/5）

- CSV：`logs/fixed_pick/fixed_pick_20260901_090610.csv`
- 五轮均记录 `cycle_passed`
- 五轮最终位置均为 `(0.2000, -0.1200, 0.0200)`，`xy_error=0.0000`
- 最终：`status: passed`，成功规则为至少 4/5

提交压缩包不包含本地 `build/`、`install/`、`log/`、`logs/`、`.bak`、
`__pycache__` 或 `.pyc`；详见根目录 `README.md` 的打包命令。

## 2026-09-11 抓取几何微调回归

- 接触模式目标圆柱半径由 `0.015 m` 调整为 `0.018 m`，A 点支撑半径由 `0.018 m` 调整为 `0.020 m`。
- 未修改 `fixed_point_experiment.yaml` 的关节角、动作顺序、位置或姿态；`pick` 夹爪映射仍为 `-0.04`。
- 单次回归：PASS；记录 `grasp_attached`，完成释放并回到初始位。
- 5 次回归：PASS（5/5）。
- 最新 CSV：`logs/fixed_pick/fixed_pick_20260911_230718.csv`
- 最终状态：`status: passed`，`success_count: 5`，`repeat_count: 5`。

## 2026-09-11 绿色放置区对齐

- 最近 5 次释放中心稳定在约 `(0.2460--0.2466, -0.1240--0.1243)`。
- 绿色放置区与 B 点调整为 `(0.2463, -0.1241)`；机械臂轨迹关节角和动作顺序不变。
- 调整后单次验证：PASS；最终圆柱 `(0.2466, -0.1242, 0.0200)`，B 点 XY 误差 `0.0003 m`。
- 最新 CSV：`logs/fixed_pick/fixed_pick_20260911_233405.csv`。

## 2026-09-12 最终录屏版本回归

- 使用与提交包相同的轨迹、模型和夹爪参数完成 5 次连续运行。
- 原始 CSV：`submission_evidence/results/fixed_pick_20260912_233915.csv`。
- 五轮均记录 `cycle_passed`；最终记录为 `status: passed`，`success_count: 5`，`repeat_count: 5`。
- 五轮最终圆柱位置均位于 B 点附近，XY 误差最大约 `0.0006 m`。
- 演示视频位于 `submission_evidence/videos/`，分别覆盖仿真和真机运行。
