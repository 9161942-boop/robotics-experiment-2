import threading
import tkinter as tk
from tkinter import messagebox
import json
import shutil
import time
from pathlib import Path

import rclpy
from rclpy.executors import ExternalShutdownException
from gazebo_msgs.srv import SetEntityState
from rclpy.node import Node
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from std_srvs.srv import SetBool


ARM_JOINTS = [
    "joint1_to_base", "joint2_to_joint1", "joint3_to_joint2",
    "joint4_to_joint3", "joint5_to_joint4", "joint6_to_joint5",
]
GRIPPER_JOINTS = [
    "gripper_controller", "gripper_base_to_gripper_left2",
    "gripper_left3_to_gripper_left1", "gripper_base_to_gripper_right3",
    "gripper_base_to_gripper_right2", "gripper_right3_to_gripper_right1",
]


class ManualControlNode(Node):
    def __init__(self):
        super().__init__("mecharm_manual_control_panel")
        self.arm_pub = self.create_publisher(JointTrajectory, "full_arm_controller/joint_trajectory", 10)
        self.gripper_pub = self.create_publisher(JointTrajectory, "full_gripper_controller/joint_trajectory", 10)
        self.reset_client = self.create_client(SetEntityState, "/gazebo/set_entity_state")
        self.vacuum_client = self.create_client(SetBool, "/mecharm_grasp/switch")

    def controllers_ready(self):
        return (
            self.arm_pub.get_subscription_count() > 0
            and self.gripper_pub.get_subscription_count() > 0
        )

    def send(self, joints, gripper, pad_position, duration):
        if not self.controllers_ready():
            raise RuntimeError(
                "控制器未连接。请先用 contact_grasp:=true run_task:=false 启动 Gazebo，"
                "并等待 full_arm_controller 与 full_gripper_controller 激活。"
            )
        arm = JointTrajectory(joint_names=ARM_JOINTS)
        arm.points = [JointTrajectoryPoint(positions=list(joints), time_from_start=self._duration(duration))]
        self.arm_pub.publish(arm)
        grip = JointTrajectory(joint_names=GRIPPER_JOINTS + ["left_contact_pad_joint", "right_contact_pad_joint"])
        master = float(gripper)
        grip.points = [JointTrajectoryPoint(
            positions=[master, master, -master, -master, -master, master, pad_position, pad_position],
            time_from_start=self._duration(duration))]
        self.gripper_pub.publish(grip)

    @staticmethod
    def _duration(seconds):
        from builtin_interfaces.msg import Duration
        d = Duration()
        d.sec = int(seconds)
        d.nanosec = int((seconds - d.sec) * 1e9)
        return d

    def reset_object(self):
        if not self.reset_client.service_is_ready():
            return False
        req = SetEntityState.Request()
        req.state.name = "target_object"
        req.state.pose.position.x = 0.30
        req.state.pose.position.y = 0.18
        req.state.pose.position.z = 0.031
        req.state.pose.orientation.w = 1.0
        req.state.reference_frame = "world"
        self.reset_client.call_async(req)
        return True

    def set_vacuum(self, enabled):
        if not self.vacuum_client.service_is_ready():
            return False
        request = SetBool.Request(); request.data = bool(enabled)
        self.vacuum_client.call_async(request)
        return True


class Panel:
    def __init__(self, node):
        self.node = node
        self.data_path = Path.cwd() / "manual_animation.json"
        self.waypoints = {}
        self.sequence = []
        self.root = tk.Tk()
        self.root.title("MechArm 270 手动仿真控制面板")
        self.vars = []
        limits = [(-2.79, 2.79), (-1.31, 2.09), (-3.05, 1.13), (-2.71, 2.71), (-2.01, 2.01), (-4.50, 3.14)]
        for name, (lo, hi) in zip(ARM_JOINTS, limits):
            row = tk.Frame(self.root); row.pack(fill="x")
            tk.Label(row, text=name, width=28, anchor="w").pack(side="left")
            var = tk.DoubleVar(value=0.0); self.vars.append(var)
            tk.Scale(row, variable=var, from_=lo, to=hi, resolution=0.01, orient="horizontal", length=420, showvalue=False).pack(side="left")
            tk.Entry(row, textvariable=var, width=8).pack(side="left", padx=4)
        row = tk.Frame(self.root); row.pack(fill="x")
        tk.Label(row, text="gripper_controller", width=28, anchor="w").pack(side="left")
        self.gripper = tk.DoubleVar(value=0.12)
        tk.Scale(row, variable=self.gripper, from_=-0.74, to=0.15, resolution=0.01, orient="horizontal", length=420, showvalue=False).pack(side="left")
        tk.Entry(row, textvariable=self.gripper, width=8).pack(side="left", padx=4)
        row = tk.Frame(self.root); row.pack(fill="x")
        tk.Label(row, text="双指夹取量 (mm)", width=28, anchor="w").pack(side="left")
        self.effort = tk.DoubleVar(value=0.0)
        tk.Scale(row, variable=self.effort, from_=0.0, to=30.0, resolution=0.5, orient="horizontal", length=420, showvalue=False).pack(side="left")
        tk.Entry(row, textvariable=self.effort, width=8).pack(side="left", padx=4)
        row = tk.Frame(self.root); row.pack(pady=8)
        tk.Label(row, text="动作时间(s)").pack(side="left")
        self.duration = tk.DoubleVar(value=2.0)
        tk.Entry(row, textvariable=self.duration, width=6).pack(side="left", padx=4)
        tk.Button(row, text="发送当前姿态", command=self.send).pack(side="left", padx=4)
        tk.Button(row, text="复位物块到 A 点", command=self.reset).pack(side="left", padx=4)
        tk.Button(row, text="真空吸附 ON", command=lambda: self.vacuum(True)).pack(side="left", padx=4)
        tk.Button(row, text="真空释放 OFF", command=lambda: self.vacuum(False)).pack(side="left", padx=4)
        tk.Button(row, text="退出", command=self.root.destroy).pack(side="left", padx=4)
        self.status = tk.Label(self.root, text="拖动滑块后点击‘发送当前姿态’", anchor="w")
        self.status.pack(fill="x", padx=8, pady=4)
        self._build_waypoint_editor()

    def _build_waypoint_editor(self):
        box = tk.LabelFrame(self.root, text="点位与动画序列")
        box.pack(fill="both", expand=True, padx=8, pady=6)
        left = tk.Frame(box); left.pack(side="left", fill="both", expand=True)
        tk.Label(left, text="已保存点位").pack(anchor="w")
        self.point_list = tk.Listbox(left, height=8)
        self.point_list.pack(fill="both", expand=True)
        row = tk.Frame(left); row.pack(fill="x", pady=3)
        self.point_name = tk.StringVar(value="above_pick")
        tk.Entry(row, textvariable=self.point_name).pack(side="left", fill="x", expand=True)
        tk.Button(row, text="保存当前点位", command=self.save_point).pack(side="left", padx=2)
        tk.Button(row, text="执行选中点位", command=self.execute_point).pack(side="left", padx=2)
        right = tk.Frame(box); right.pack(side="left", fill="both", expand=True, padx=(10, 0))
        tk.Label(right, text="动画序列（按顺序播放）").pack(anchor="w")
        self.sequence_list = tk.Listbox(right, height=8)
        self.sequence_list.pack(fill="both", expand=True)
        row = tk.Frame(right); row.pack(fill="x", pady=3)
        tk.Button(row, text="加入序列", command=self.add_to_sequence).pack(side="left", padx=2)
        tk.Button(row, text="播放序列", command=self.play_sequence).pack(side="left", padx=2)
        tk.Button(row, text="清空序列", command=self.clear_sequence).pack(side="left", padx=2)
        self.load_data()

    def send(self):
        try:
            duration = max(0.1, float(self.duration.get()))
            self.node.send([v.get() for v in self.vars], self.gripper.get(), min(0.060, self.effort.get()/1000.0), duration)
            self.status.config(text=f"已发送：双指夹取量 {self.effort.get():.1f} mm")
        except Exception as exc:
            messagebox.showerror("发送失败", str(exc))

    def current_point(self):
        return {"joints": [round(v.get(), 6) for v in self.vars],
                "gripper": round(self.gripper.get(), 6),
                "pad_size_mm": round(self.effort.get(), 6),
                "duration": max(0.1, float(self.duration.get()))}

    def save_point(self):
        name = self.point_name.get().strip()
        if not name:
            messagebox.showwarning("点位名称", "请输入点位名称")
            return
        self.waypoints[name] = self.current_point()
        self.refresh_lists(); self.save_data()
        self.status.config(text=f"已保存点位：{name}")

    def execute_point(self):
        sel = self.point_list.curselection()
        if not sel: return
        name = self.point_list.get(sel[0]); p = self.waypoints[name]
        for var, value in zip(self.vars, p["joints"]): var.set(value)
        self.gripper.set(p["gripper"]); self.effort.set(p.get("pad_size_mm", 0.0)); self.duration.set(p["duration"])
        self.send()

    def add_to_sequence(self):
        sel = self.point_list.curselection()
        if not sel: return
        self.sequence.append(self.point_list.get(sel[0])); self.refresh_lists(); self.save_data()

    def clear_sequence(self):
        self.sequence.clear(); self.refresh_lists(); self.save_data()

    def play_sequence(self):
        if not self.sequence:
            messagebox.showinfo("动画序列", "请先选择点位并加入序列")
            return
        self.status.config(text="正在播放动画序列…")
        threading.Thread(target=self._play_worker, daemon=True).start()

    def _play_worker(self):
        for name in self.sequence:
            p = self.waypoints.get(name)
            if not p: continue
            self.node.send(p["joints"], p["gripper"], min(0.060, float(p.get("pad_size_mm", 0.0))/1000.0), p["duration"])
            time.sleep(p["duration"] + 0.25)
            # Convention for recorded animation point names:
            # reaching 'pick' engages vacuum; reaching 'release' disengages it.
            normalized = name.strip().lower().replace("-", "_")
            if normalized in {"pick", "grasp", "grasp_object"}:
                self.node.set_vacuum(True)
                time.sleep(0.4)
            elif normalized in {"release", "release_object"}:
                self.node.set_vacuum(False)
                time.sleep(0.4)
        self.root.after(0, lambda: self.status.config(text="动画序列播放完成"))

    def refresh_lists(self):
        self.point_list.delete(0, tk.END)
        for name in self.waypoints: self.point_list.insert(tk.END, name)
        self.sequence_list.delete(0, tk.END)
        for name in self.sequence: self.sequence_list.insert(tk.END, name)

    def save_data(self):
        if self.data_path.exists():
            shutil.copy2(self.data_path, self.data_path.with_suffix(self.data_path.suffix + ".bak"))
        self.data_path.write_text(json.dumps({"waypoints": self.waypoints, "sequence": self.sequence}, indent=2), encoding="utf-8")

    def load_data(self):
        if self.data_path.exists():
            try:
                data = json.loads(self.data_path.read_text(encoding="utf-8"))
                self.waypoints = data.get("waypoints", {}); self.sequence = data.get("sequence", [])
            except (OSError, ValueError):
                pass
        self.refresh_lists()

    def reset(self):
        if self.node.reset_object(): self.status.config(text="已请求将目标物块复位到 A 点")
        else: self.status.config(text="Gazebo 服务尚未就绪，请稍后重试")

    def vacuum(self, enabled):
        if not self.node.vacuum_client.service_is_ready():
            self.status.config(text="真空服务尚未就绪；请使用 contact_grasp:=false 启动 Gazebo")
            return
        req = SetBool.Request(); req.data = bool(enabled)
        self.node.vacuum_client.call_async(req)
        self.status.config(text="已发送真空吸附" if enabled else "已发送真空释放")

    def run(self):
        self.root.mainloop()


def main():
    rclpy.init()
    node = ManualControlNode()
    def spin_node():
        try:
            rclpy.spin(node)
        except ExternalShutdownException:
            pass

    thread = threading.Thread(target=spin_node, daemon=True)
    thread.start()
    try:
        Panel(node).run()
    finally:
        if rclpy.ok():
            rclpy.shutdown()
        thread.join(timeout=1.0)
        node.destroy_node()


if __name__ == "__main__":
    main()
