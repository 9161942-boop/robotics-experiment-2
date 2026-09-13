# mechArm 270 fixed-point contact-grasp submission

这是当前验收通过版本的独立 ROS 2 工作区。提交时使用文末的打包命令，
压缩包包含运行所需的 `mecharm_fixed_pick_sim` 源码、必要的
`mycobot_description` 模型资源、启动脚本、运行证据和说明；本地编译产生的
`build/`、`install/`、`log/` 与 `.bak` 文件不会进入压缩包。

## 版本状态

- pick 主夹爪值：`-0.04`（在上一版基础上再收窄一点）
- release 主夹爪值：`+0.12`
- 接触模式目标圆柱半径：`0.018 m`；A 点支撑半径：`0.020 m`（仅调整抓取几何，不改轨迹）
- 绿色放置区/B 点：`(0.2463, -0.1241)`，与最近实测释放中心对齐，圆柱完整落在区域内。
- 固定轨迹：
  `initial → above_a → crab → pick → lift → initial_2 → release_ready → release → initial`
- `fixed_point_experiment.yaml` 中的关节角、顺序和姿态保持不变。
- 接触垫使用 Gazebo effort 控制器，主夹爪联动使用 position 控制器。

## 编译

```bash
cd /home/hcl/桌面/mecharm_fixed_pick_submission_ws
source /opt/ros/humble/setup.bash
colcon build --symlink-install
source install/local_setup.bash
```

## 一键运行

编译完成后执行：

```bash
cd /home/hcl/桌面/mecharm_fixed_pick_submission_ws
./run_contact_grasp.sh
```

也可以直接运行：

```bash
cd /home/hcl/桌面/mecharm_fixed_pick_submission_ws
source /opt/ros/humble/setup.bash
source install/local_setup.bash
GAZEBO_MASTER_URI=http://127.0.0.1:11360 \
ros2 launch mecharm_fixed_pick_sim fixed_point_experiment.launch.py \
  contact_grasp:=true gui:=true run_task:=true \
  repeat_count:=5 exit_on_complete:=true task_start_delay_sec:=20.0 \
  log_directory:=/home/hcl/桌面/mecharm_fixed_pick_submission_ws/logs/fixed_pick
```

不要同时运行两个 Gazebo 会话。一键脚本默认连续运行 5 次，符合实验二
“至少成功 4 次”的验收要求；如需单次调试，可手动将命令中的
`repeat_count:=5` 改为 `repeat_count:=1`。

## 运行结果记录

任务节点会把 CSV 写入当前工作区的 `logs/fixed_pick/`。验收时应确认：

1. 四个控制器均显示 `Configured and activated`；
2. `pick` 后物体离开 A 点；
3. `release` 时读取圆柱实际 X/Y，先在其正上方约 2 cm，再只改变 Z 垂直落下固定；
4. 最终结果显示 `status: passed`。

日志默认写入本工作区的 `logs/fixed_pick/`；也可以通过
`MECHARM_FIXED_PICK_LOG_DIR` 或 `log_directory:=...` 指定其他目录。

## 提交证据

`submission_evidence/` 保存了最终验收材料：

- `results/fixed_pick_20260912_233915.csv`：录屏前使用的原始 CSV，包含第 1--5 轮完整记录，末行状态为 `passed`；
- `results/fixed_pick_20260912_233505.csv`：另一份独立的 5 轮回归原始 CSV，末行状态为 `passed`；
- `videos/simulation_demo.webm`：仿真演示；
- `videos/real_robot_demo.mp4`：真机演示；
- `docs/ERROR_LOG.md`：调试期间的异常与处理记录；
- `SHA256SUMS.txt`：证据文件校验值。
- `report/experiment2_lab_report.pdf`：英文 LaTeX 实验报告；同目录保留 `experiment2_lab_report.tex` 和报告插图。

每份 CSV 中的 `cycle_passed` 行都可用于核对五轮结果；两份文件均为实际运行记录，不是重复复制的日志。

## 报告与证据边界

报告中的仿真 5/5 结论来自两份 CSV 和 `VALIDATION.md`。实机 MP4 证明真实机械臂运行过程；如果验收现场要求填写真机 5 次成功数或安全检查表，应同时出示对应记录，不能仅凭视频文件存在推断这些数字。

## 提交内容

目标机器需要安装 ROS 2 Humble、Gazebo Classic 11 以及 `package.xml` 中列出的
系统依赖。生成可提交的干净压缩包（不会删除本地任何文件）：

```bash
cd /home/hcl/桌面
tar --exclude='mecharm_fixed_pick_submission_ws/build' \
    --exclude='mecharm_fixed_pick_submission_ws/install' \
    --exclude='mecharm_fixed_pick_submission_ws/log' \
    --exclude='mecharm_fixed_pick_submission_ws/logs' \
    --exclude='*.bak*' --exclude='__pycache__' --exclude='*.pyc' \
    -czf mecharm_fixed_pick_submission_ws.tar.gz \
    mecharm_fixed_pick_submission_ws
```

压缩包内容可用下面的命令检查：

```bash
tar -tzf /home/hcl/桌面/mecharm_fixed_pick_submission_ws.tar.gz \
  | rg '(^|/)(build|install|log|logs)(/|$)|\.bak|__pycache__|\.pyc' \
  && echo '发现不应提交的构建/备份文件' || echo '压缩包干净'
```
