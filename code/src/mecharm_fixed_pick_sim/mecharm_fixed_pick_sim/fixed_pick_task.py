import csv
import json
import math
from pathlib import Path
import time

from ament_index_python.packages import get_package_share_directory
from gazebo_msgs.msg import ModelStates, LinkStates
from gazebo_msgs.srv import SetEntityState
import rclpy
from rclpy.duration import Duration
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Bool, Float64MultiArray, String
from std_srvs.srv import SetBool
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from visualization_msgs.msg import Marker, MarkerArray
import yaml


def _lerp(start, end, ratio):
    return [a + (b - a) * ratio for a, b in zip(start, end)]


def _to_float_list(values, expected_len, name):
    if not isinstance(values, list) or len(values) != expected_len:
        raise ValueError(f"{name} must contain {expected_len} numeric values")
    return [float(value) for value in values]


class FixedPickTask(Node):
    def __init__(self):
        super().__init__("fixed_pick_task")

        default_config = str(
            Path(get_package_share_directory("mecharm_fixed_pick_sim"))
            / "config"
            / "fixed_pick.yaml"
        )
        self.declare_parameter("config_file", default_config)
        self.declare_parameter("repeat_count", 0)
        self.declare_parameter("auto_start", True)
        self.declare_parameter("exit_on_complete", False)
        self.declare_parameter("fault_mode", "none")
        self.declare_parameter("step_duration_sec", 0.0)
        self.declare_parameter("settle_duration_sec", -1.0)
        self.declare_parameter("loop_rate_hz", 0.0)
        self.declare_parameter("command_mode", "joint_state")
        self.declare_parameter("arm_command_topic", "arm_controller/joint_trajectory")
        self.declare_parameter("gripper_command_topic", "gripper_trajectory_controller/joint_trajectory")
        self.declare_parameter("log_directory", "")
        self.declare_parameter("physical_grasp_enabled", False)
        self.declare_parameter("explicit_gripper_control", False)
        self.declare_parameter("contact_pad_effort_topic", "contact_pad_effort_controller/commands")
        self.declare_parameter("object_tracking_enabled", False)
        self.declare_parameter("visual_attachment_enabled", False)
        # Release the visually attached object just above B so the handoff
        # reads as a placement instead of a sudden snap into the marker.
        self.declare_parameter("visual_release_height_m", 0.02)
        self.declare_parameter("grasp_switch_service", "/mecharm_grasp/switch")
        self.declare_parameter("grasp_status_topic", "/mecharm_grasp/grasping")
        self.declare_parameter("model_states_topic", "/gazebo/model_states")
        self.declare_parameter("set_entity_state_service", "/gazebo/set_entity_state")
        self.declare_parameter("service_timeout_sec", 12.0)
        self.declare_parameter("grasp_timeout_sec", 3.0)

        config_file = self.get_parameter("config_file").value
        self.config = self._load_config(config_file)
        self.joint_names = list(self.config["robot"]["joint_names"])
        self.gripper_joint = self.config["robot"]["gripper_joint"]
        self.all_joint_names = self.joint_names + [self.gripper_joint]

        configured_repeat = int(self.config["task"]["repeat_count"])
        repeat_override = int(self.get_parameter("repeat_count").value)
        self.repeat_count = repeat_override if repeat_override > 0 else configured_repeat
        step_override = float(self.get_parameter("step_duration_sec").value)
        settle_override = float(self.get_parameter("settle_duration_sec").value)
        rate_override = float(self.get_parameter("loop_rate_hz").value)
        self.step_duration = (
            step_override
            if step_override > 0.0
            else float(self.config["task"]["step_duration_sec"])
        )
        self.settle_duration = (
            settle_override
            if settle_override >= 0.0
            else float(self.config["task"]["settle_duration_sec"])
        )
        self.loop_rate = (
            rate_override if rate_override > 0.0 else float(self.config["task"]["loop_rate_hz"])
        )
        self.exit_on_complete = bool(self.get_parameter("exit_on_complete").value)
        self.auto_start = bool(self.get_parameter("auto_start").value)
        self.fault_mode = str(self.get_parameter("fault_mode").value)
        self.command_mode = str(self.get_parameter("command_mode").value)
        self.physical_grasp_enabled = bool(
            self.get_parameter("physical_grasp_enabled").value
        )
        self.explicit_gripper_control = bool(
            self.get_parameter("explicit_gripper_control").value
        )
        self.object_tracking_enabled = bool(
            self.get_parameter("object_tracking_enabled").value
        )
        self.visual_attachment_enabled = bool(
            self.get_parameter("visual_attachment_enabled").value
        )
        self.visual_release_height = max(
            0.0, float(self.get_parameter("visual_release_height_m").value)
        )
        self.service_timeout = float(self.get_parameter("service_timeout_sec").value)
        self.grasp_timeout = float(self.get_parameter("grasp_timeout_sec").value)
        if self.command_mode not in ("joint_state", "trajectory"):
            raise ValueError("command_mode must be 'joint_state' or 'trajectory'")

        self.joint_pub = None
        self.arm_command_pub = None
        self.gripper_command_pub = None
        self.contact_pad_effort_pub = None
        if self.command_mode == "joint_state":
            self.joint_pub = self.create_publisher(JointState, "joint_states", 10)
        else:
            self.arm_command_pub = self.create_publisher(
                JointTrajectory, str(self.get_parameter("arm_command_topic").value), 10
            )
            self.gripper_command_pub = self.create_publisher(
                JointTrajectory, str(self.get_parameter("gripper_command_topic").value), 10
            )
            if self.explicit_gripper_control:
                self.contact_pad_effort_pub = self.create_publisher(
                    Float64MultiArray,
                    str(self.get_parameter("contact_pad_effort_topic").value),
                    10,
                )
        self.state_pub = self.create_publisher(String, "mecharm_task/state", 10)
        self.result_pub = self.create_publisher(String, "mecharm_task/result", 10)
        self.marker_pub = self.create_publisher(MarkerArray, "mecharm_task/scene", 10)

        self.grasp_client = None
        self.reset_client = None
        self._object_pose = None
        self._gripper_pose = None
        self._visual_attached = False
        self._visual_attach_offset = None
        self._pending_visual_release = False
        self._visual_release_drop_active = False
        self._visual_release_drop_started = 0.0
        self._visual_release_drop_start_z = None
        self._visual_release_drop_target = None
        self._visual_release_xy = None
        self._visual_release_orientation = None
        self._last_visual_update_time = 0.0
        self._grasping = False
        self._service_future = None
        self._service_kind = ""
        self._service_deadline = 0.0
        self._waiting_for_grasp = False
        self._grasp_deadline = 0.0
        self._needs_object_reset = self.object_tracking_enabled
        self._reset_wait_started = time.monotonic()
        if self.physical_grasp_enabled:
            self.grasp_client = self.create_client(
                SetBool, str(self.get_parameter("grasp_switch_service").value)
            )
        if self.object_tracking_enabled:
            self.reset_client = self.create_client(
                SetEntityState,
                str(self.get_parameter("set_entity_state_service").value),
            )
            self.create_subscription(
                Bool,
                str(self.get_parameter("grasp_status_topic").value),
                self._on_grasp_status,
                10,
            )
            self.create_subscription(
                ModelStates,
                str(self.get_parameter("model_states_topic").value),
                self._on_model_states,
                10,
            )
            self.create_subscription(LinkStates, "/gazebo/link_states", self._on_link_states, 10)

        self._pose_map = self._build_pose_map()
        self._sequence = self._build_sequence()
        self._startup_error = ""
        try:
            self._validate_task()
        except ValueError as exc:
            self._startup_error = str(exc)

        home = self._pose_map["home"]
        self._current_positions = list(home)
        self._step_start_positions = list(home)
        self._target_positions = list(home)
        self._active_settle_duration = self.settle_duration
        self._active_step_duration = self.step_duration
        self._repeat_index = 0
        self._step_index = -1
        self._step_start_time = None
        self._step_settle_until = None
        self._phase = "idle"
        self._last_step_name = "idle"
        self._success_count = 0
        self._failure_reason = ""
        self.should_exit = False

        self._log_writer = None
        self._log_file = None
        self._open_log()

        self._publish_state("ready")
        self._publish_joint_state()
        self._publish_scene()

        period = 1.0 / max(self.loop_rate, 1.0)
        self.timer = self.create_timer(period, self._tick)
        self.scene_timer = self.create_timer(0.5, self._publish_scene)

        if self._startup_error:
            self._failure_reason = self._startup_error
            self.get_logger().error(self._startup_error)
            self._finish_task()
        elif self.auto_start:
            if self.object_tracking_enabled:
                self._publish_state("waiting_for_gazebo_services")
            else:
                self._start_next_step()

    def _load_config(self, config_file):
        path = Path(config_file).expanduser()
        with path.open("r", encoding="utf-8") as stream:
            return yaml.safe_load(stream)

    def _build_pose_map(self):
        gripper_open = float(self.config["robot"]["gripper_open"])
        gripper_closed = float(self.config["robot"]["gripper_closed"])
        pose_map = {}
        for pose_name, pose in self.config["poses"].items():
            joints = _to_float_list(pose.get("joints"), len(self.joint_names), pose_name)
            gripper = float(pose.get("gripper", gripper_open))
            pose_map[pose_name] = joints + [gripper]
        pose_map["__gripper_open__"] = [gripper_open]
        pose_map["__gripper_closed__"] = [gripper_closed]
        return pose_map

    def _build_sequence(self):
        sequence = []
        for item in self.config["sequence"]:
            pose_name = item["pose"]
            if pose_name not in self._pose_map:
                raise ValueError(f"unknown pose in sequence: {pose_name}")
            positions = list(self._pose_map[pose_name])
            gripper_mode = item.get("gripper")
            if gripper_mode == "open":
                positions[-1] = self._pose_map["__gripper_open__"][0]
            elif gripper_mode == "closed":
                positions[-1] = self._pose_map["__gripper_closed__"][0]
            elif gripper_mode is not None:
                raise ValueError(f"unsupported gripper mode: {gripper_mode}")

            sequence.append(
                {
                    "name": str(item["name"]),
                    "phase": str(item.get("phase", "moving")),
                    "positions": positions,
                    "settle_duration": float(
                        item.get("settle_duration_sec", self.settle_duration)
                    ),
                    "duration": float(item.get("duration_sec", self.step_duration)),
                }
            )
        return sequence

    def _validate_task(self):
        if self.fault_mode == "unreachable":
            raise ValueError("fault_mode=unreachable requested; refusing to start task")

        joint_limits = self.config["safety"]["joint_limits"]
        for pose_name, positions in self._pose_map.items():
            if pose_name.startswith("__"):
                continue
            for joint_name, position in zip(self.all_joint_names, positions):
                if joint_name not in joint_limits:
                    raise ValueError(f"missing joint limit for {joint_name}")
                lower, upper = [float(value) for value in joint_limits[joint_name]]
                if position < lower or position > upper:
                    raise ValueError(
                        f"pose {pose_name} exceeds {joint_name}: "
                        f"{position:.3f} not in [{lower:.3f}, {upper:.3f}]"
                    )

        if self.fault_mode == "joint_limit":
            raise ValueError("fault_mode=joint_limit requested; safety stop triggered")

    def _open_log(self):
        log_directory = str(self.get_parameter("log_directory").value).strip()
        if not log_directory:
            log_directory = self.config["task"]["log_directory"]
        log_dir = Path(log_directory).expanduser()
        log_dir.mkdir(parents=True, exist_ok=True)
        stamp = time.strftime("%Y%m%d_%H%M%S")
        log_path = log_dir / f"fixed_pick_{stamp}.csv"
        self._log_file = log_path.open("w", newline="", encoding="utf-8")
        self._log_writer = csv.writer(self._log_file)
        self._log_writer.writerow(
            [
                "wall_time",
                "repeat",
                "step",
                "phase",
                "status",
                "joint_positions_rad",
                "message",
            ]
        )
        self.get_logger().info(f"logging task run to {log_path}")

    def _write_log(self, status, message=""):
        if self._log_writer is None:
            return
        self._log_writer.writerow(
            [
                time.strftime("%Y-%m-%d %H:%M:%S"),
                self._repeat_index + 1,
                self._last_step_name,
                self._phase,
                status,
                json.dumps([round(v, 5) for v in self._current_positions]),
                message,
            ]
        )
        self._log_file.flush()

    def _on_grasp_status(self, msg):
        self._grasping = bool(msg.data)

    def _on_model_states(self, msg):
        object_name = str(self.config["scene"]["object"]["model_name"])
        try:
            index = msg.name.index(object_name)
        except ValueError:
            return
        self._object_pose = msg.pose[index]

    def _on_link_states(self, msg):
        try:
            index = msg.name.index("mecharm_270_full::gripper_base")
        except ValueError:
            return
        self._gripper_pose = msg.pose[index]

    def _request_visual_pose(self, pose, kind):
        if self._service_future is not None or self.reset_client is None:
            return
        if not self.reset_client.service_is_ready():
            return
        request = SetEntityState.Request()
        request.state.name = str(self.config["scene"]["object"]["model_name"])
        request.state.pose = pose
        request.state.reference_frame = "world"
        self._service_future = self.reset_client.call_async(request)
        self._service_kind = kind
        self._service_deadline = time.monotonic() + self.service_timeout

    def _update_visual_attachment(self):
        if not self._visual_attached or self._gripper_pose is None:
            return
        now = time.monotonic()
        # Gazebo set_entity_state is a service call, not a streaming topic;
        # cap writes to a stable rate to avoid visible teleport jitter.
        if now - self._last_visual_update_time < 0.05:
            return
        self._last_visual_update_time = now
        q = self._gripper_pose.orientation
        rx, ry, rz = self._rotate_vector(q, self._visual_attach_offset)
        pose = type(self._gripper_pose)()
        pose.position.x = self._gripper_pose.position.x + rx
        pose.position.y = self._gripper_pose.position.y + ry
        pose.position.z = self._gripper_pose.position.z + rz
        pose.orientation = self._gripper_pose.orientation
        self._request_visual_pose(pose, "visual_attach")

    @staticmethod
    def _rotate_vector(q, vector):
        """Rotate a vector by quaternion q using q*v*q^-1."""
        x, y, z = vector
        tx = 2.0 * (q.y * z - q.z * y)
        ty = 2.0 * (q.z * x - q.x * z)
        tz = 2.0 * (q.x * y - q.y * x)
        return (
            x + q.w * tx + (q.y * tz - q.z * ty),
            y + q.w * ty + (q.z * tx - q.x * tz),
            z + q.w * tz + (q.x * ty - q.y * tx),
        )

    def _start_visual_attachment(self):
        if not self.visual_attachment_enabled or self._visual_attached:
            return
        if self._object_pose is None or self._gripper_pose is None:
            return
        world_offset = (
            self._object_pose.position.x - self._gripper_pose.position.x,
            self._object_pose.position.y - self._gripper_pose.position.y,
            self._object_pose.position.z - self._gripper_pose.position.z,
        )
        # Store the offset in gripper_base local coordinates.  The previous
        # implementation stored a world-frame offset and then rotated it
        # again, which displaced the object outside the fingers.
        q = self._gripper_pose.orientation
        inverse_q = type(q)(x=-q.x, y=-q.y, z=-q.z, w=q.w)
        self._visual_attach_offset = self._rotate_vector(inverse_q, world_offset)
        self._visual_attached = True
        self._write_log("grasp_attached", "visual attachment to gripper_base")

    def _release_visual_attachment(self):
        if not self._visual_attached:
            return
        self._visual_attached = False
        place = self.config["scene"]["place_point_b"]["pose_xyz"]
        release_x = (
            float(self._object_pose.position.x)
            if self._object_pose is not None
            else float(place[0])
        )
        release_y = (
            float(self._object_pose.position.y)
            if self._object_pose is not None
            else float(place[1])
        )
        self._visual_release_xy = (release_x, release_y)
        pose = type(self._object_pose)()
        pose.position.x = release_x
        pose.position.y = release_y
        pose.position.z = float(place[2]) + self.visual_release_height
        if self._object_pose is not None:
            pose.orientation = self._object_pose.orientation
            self._visual_release_orientation = self._object_pose.orientation
        else:
            pose.orientation.w = 1.0
        # First release above B, then settle straight down to B.  This keeps
        # the handoff visually readable without enabling the unstable gravity
        # path used by the earlier experiment.
        request_pose = pose
        if self._service_future is not None:
            self._pending_visual_release = True
        else:
            self._request_visual_pose(request_pose, "visual_release")
        self._write_log(
            "released",
            f"visual attachment released at x={release_x:.4f}, y={release_y:.4f}, "
            f"above B at z={pose.position.z:.4f} m",
        )

    def _begin_visual_release_drop(self):
        place = self.config["scene"]["place_point_b"]["pose_xyz"]
        target = type(self._object_pose)()
        release_xy = self._visual_release_xy or (float(place[0]), float(place[1]))
        target.position.x = release_xy[0]
        target.position.y = release_xy[1]
        target.position.z = float(place[2])
        if self._visual_release_orientation is not None:
            target.orientation = self._visual_release_orientation
        else:
            target.orientation.w = 1.0
        self._visual_release_drop_target = target
        self._visual_release_drop_start_z = float(place[2]) + self.visual_release_height
        self._visual_release_drop_started = time.monotonic()
        self._visual_release_drop_active = self.visual_release_height > 0.0
        if not self._visual_release_drop_active:
            self._write_log("release_settled", "object fixed at place point B")

    def _update_visual_release_drop(self):
        if not self._visual_release_drop_active or self._visual_release_drop_target is None:
            return
        now = time.monotonic()
        if now - self._last_visual_update_time < 0.05:
            return
        self._last_visual_update_time = now
        duration = 0.35
        ratio = min(max((now - self._visual_release_drop_started) / duration, 0.0), 1.0)
        smooth = 0.5 - 0.5 * math.cos(math.pi * ratio)
        target = self._visual_release_drop_target
        pose = type(target)()
        pose.position.x = target.position.x
        pose.position.y = target.position.y
        pose.position.z = self._visual_release_drop_start_z + (
            target.position.z - self._visual_release_drop_start_z
        ) * smooth
        pose.orientation = target.orientation
        self._request_visual_pose(pose, "visual_release_drop")
        if ratio >= 1.0:
            self._visual_release_drop_active = False
            self._write_log(
                "release_settled",
                f"object fixed directly below release position at z={target.position.z:.4f} m",
            )

    def _request_object_reset(self):
        if not self.reset_client.service_is_ready():
            if time.monotonic() - self._reset_wait_started > self.service_timeout:
                self._abort_task("Gazebo set_entity_state service did not become ready")
            return

        pick = self.config["scene"]["pick_point_a"]["pose_xyz"]
        request = SetEntityState.Request()
        request.state.name = str(self.config["scene"]["object"]["model_name"])
        request.state.pose.position.x = float(pick[0])
        request.state.pose.position.y = float(pick[1])
        request.state.pose.position.z = float(pick[2])
        request.state.pose.orientation.w = 1.0
        request.state.reference_frame = "world"
        self._service_future = self.reset_client.call_async(request)
        self._service_kind = "reset"
        self._service_deadline = time.monotonic() + self.service_timeout
        self._publish_state("resetting_object")

    def _request_grasp_switch(self, enabled):
        if not self.grasp_client.service_is_ready():
            self._abort_task("Gazebo grasp switch service is unavailable")
            return
        request = SetBool.Request()
        request.data = bool(enabled)
        self._service_future = self.grasp_client.call_async(request)
        self._service_kind = "grasp_on" if enabled else "grasp_off"
        self._service_deadline = time.monotonic() + self.service_timeout
        self._publish_state(self._service_kind)

    def _poll_service_future(self):
        if self._service_future is None:
            return False
        if not self._service_future.done():
            if time.monotonic() > self._service_deadline:
                self._abort_task(f"service timeout while handling {self._service_kind}")
            return True

        future = self._service_future
        kind = self._service_kind
        self._service_future = None
        self._service_kind = ""
        try:
            response = future.result()
        except Exception as exc:
            self._abort_task(f"{kind} service failed: {exc}")
            return True
        if response is None or not response.success:
            message = (
                "no response"
                if response is None
                else getattr(response, "message", "request failed")
            )
            self._abort_task(f"{kind} service rejected request: {message}")
            return True

        if kind == "reset":
            self._needs_object_reset = False
            self._last_step_name = "reset_object"
            self._phase = "idle"
            self._write_log("complete", "object reset to pick point A")
            self._step_settle_until = self.get_clock().now() + Duration(seconds=0.5)
        elif kind == "visual_attach" and self._pending_visual_release:
            self._pending_visual_release = False
            place = self.config["scene"]["place_point_b"]["pose_xyz"]
            release_xy = self._visual_release_xy or (float(place[0]), float(place[1]))
            pose = type(self._object_pose)()
            pose.position.x = release_xy[0]
            pose.position.y = release_xy[1]
            pose.position.z = float(place[2]) + self.visual_release_height
            if self._visual_release_orientation is not None:
                pose.orientation = self._visual_release_orientation
            else:
                pose.orientation.w = 1.0
            self._request_visual_pose(pose, "visual_release")
        elif kind == "visual_release":
            self._begin_visual_release_drop()
        elif kind == "grasp_on":
            self._waiting_for_grasp = True
            self._grasp_deadline = time.monotonic() + self.grasp_timeout
        elif kind == "grasp_off":
            self._step_settle_until = self.get_clock().now() + Duration(
                seconds=self._active_settle_duration
            )
        return True

    def _object_is_placed(self):
        if self._object_pose is None:
            return False, "target object pose was not received from Gazebo"
        place = self.config["scene"]["place_point_b"]["pose_xyz"]
        object_config = self.config["scene"]["object"]
        dx = self._object_pose.position.x - float(place[0])
        dy = self._object_pose.position.y - float(place[1])
        distance_xy = math.hypot(dx, dy)
        max_xy = float(object_config["placement_tolerance_xy_m"])
        max_z = float(object_config["placement_max_z_m"])
        message = (
            f"object xyz=({self._object_pose.position.x:.4f}, "
            f"{self._object_pose.position.y:.4f}, {self._object_pose.position.z:.4f}), "
            f"xy_error={distance_xy:.4f}"
        )
        return distance_xy <= max_xy and self._object_pose.position.z <= max_z, message

    def _abort_task(self, reason):
        if self.should_exit:
            return
        self._failure_reason = str(reason)
        self.get_logger().error(self._failure_reason)
        self._write_log("failed", self._failure_reason)
        self._repeat_index = self.repeat_count
        self._finish_task()

    def _tick(self):
        if not self.auto_start or self.should_exit:
            return

        if self.object_tracking_enabled and self._poll_service_future():
            return
        self._update_visual_attachment()
        self._update_visual_release_drop()
        if self.physical_grasp_enabled:
            if self._waiting_for_grasp:
                if self._grasping:
                    self._waiting_for_grasp = False
                    self._write_log("grasp_confirmed")
                    self._publish_state("grasp_confirmed")
                    self._step_settle_until = self.get_clock().now() + Duration(
                        seconds=self._active_settle_duration
                    )
                elif time.monotonic() > self._grasp_deadline:
                    self._abort_task("gripper closed but Gazebo did not confirm a grasp")
                return
        if self.object_tracking_enabled and self._needs_object_reset:
            self._request_object_reset()
            return

        now = self.get_clock().now()
        if self._step_settle_until is not None:
            if now.nanoseconds < self._step_settle_until.nanoseconds:
                self._publish_joint_state()
                return
            self._step_settle_until = None
            self._start_next_step()
            return

        if self._step_start_time is None:
            self._start_next_step()
            return

        elapsed = (now - self._step_start_time).nanoseconds / 1e9
        ratio = min(max(elapsed / self._active_step_duration, 0.0), 1.0)
        ratio = 0.5 - 0.5 * math.cos(math.pi * ratio)
        self._current_positions = _lerp(
            self._step_start_positions, self._target_positions, ratio
        )
        self._publish_joint_state()

        if elapsed >= self._active_step_duration:
            self._current_positions = list(self._target_positions)
            self._publish_joint_state()
            self._write_log("complete")
            self._publish_state(f"complete:{self._last_step_name}")
            self._step_start_time = None
            step_name = self._last_step_name.lower()
            if self.visual_attachment_enabled and step_name == "pick":
                self._start_visual_attachment()
            if self.visual_attachment_enabled and step_name == "release":
                self._release_visual_attachment()
            if self.physical_grasp_enabled and (
                step_name == "close_gripper"
                or step_name in {"pick", "grasp", "grasp_object"}
            ):
                self._request_grasp_switch(True)
                return
            if self.physical_grasp_enabled and (
                step_name == "open_gripper"
                or step_name in {"release", "release_object"}
            ):
                self._request_grasp_switch(False)
                return
            self._step_settle_until = now + Duration(
                seconds=self._active_settle_duration
            )

    def _start_next_step(self):
        if self._repeat_index >= self.repeat_count:
            self._finish_task()
            return

        self._step_index += 1
        if self._step_index >= len(self._sequence):
            cycle_success = True
            cycle_message = "trajectory sequence completed"
            if self.object_tracking_enabled:
                cycle_success, cycle_message = self._object_is_placed()
            if cycle_success:
                self._success_count += 1
                self._write_log("cycle_passed", cycle_message)
                self._publish_state(f"cycle_complete:{self._repeat_index + 1}")
            else:
                self._failure_reason = cycle_message
                self._write_log("cycle_failed", cycle_message)
                self._publish_state(f"cycle_failed:{self._repeat_index + 1}")
            self._repeat_index += 1
            self._step_index = -1
            if self._repeat_index >= self.repeat_count:
                self._finish_task()
                return
            if self.object_tracking_enabled:
                self._needs_object_reset = True
                self._reset_wait_started = time.monotonic()
                return

        step = self._sequence[self._step_index]
        self._step_start_positions = list(self._current_positions)
        self._target_positions = list(step["positions"])
        self._phase = step["phase"]
        self._last_step_name = step["name"]
        self._active_settle_duration = float(step["settle_duration"])
        self._active_step_duration = max(0.05, float(step["duration"]))
        self._step_start_time = self.get_clock().now()
        self._write_log("start")
        self._publish_state(f"start:{self._last_step_name}")
        self.get_logger().info(
            f"cycle {self._repeat_index + 1}/{self.repeat_count}: {self._last_step_name}"
        )
        self._publish_trajectory_command(self._target_positions)

    def _finish_task(self):
        required_successes = min(4, self.repeat_count)
        status = "passed" if self._success_count >= required_successes else "failed"
        result = {
            "status": status,
            "success_count": self._success_count,
            "repeat_count": self.repeat_count,
            "success_rule": (
                f"at least {required_successes} successful runs out of {self.repeat_count}"
            ),
            "failure_reason": self._failure_reason,
        }
        self._publish_state(f"task_{status}")
        self.result_pub.publish(String(data=json.dumps(result, ensure_ascii=False)))
        self._write_log(status, json.dumps(result, ensure_ascii=False))
        self.get_logger().info(json.dumps(result, ensure_ascii=False))
        self.auto_start = False
        if self._log_file is not None:
            self._log_file.close()
            self._log_file = None
        if self.exit_on_complete:
            self.should_exit = True
            raise SystemExit(0)

    def _publish_state(self, state):
        self.state_pub.publish(String(data=state))

    def _publish_joint_state(self):
        if self.joint_pub is None:
            return
        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.name = self.all_joint_names
        msg.position = self._current_positions
        self.joint_pub.publish(msg)

    def _publish_trajectory_command(self, positions):
        if self.command_mode != "trajectory":
            return

        arm_msg = JointTrajectory()
        arm_msg.joint_names = self.joint_names
        arm_point = JointTrajectoryPoint()
        arm_point.positions = list(positions[: len(self.joint_names)])
        arm_point.time_from_start = Duration(seconds=self._active_step_duration).to_msg()
        arm_msg.points = [arm_point]
        self.arm_command_pub.publish(arm_msg)

        gripper_msg = JointTrajectory()
        gripper_point = JointTrajectoryPoint()
        if self.explicit_gripper_control:
            gripper_msg.joint_names = [
                self.gripper_joint,
                "gripper_base_to_gripper_left2",
                "gripper_left3_to_gripper_left1",
                "gripper_base_to_gripper_right3",
                "gripper_base_to_gripper_right2",
                "gripper_right3_to_gripper_right1",
            ]
            # In contact mode keep a visible mechanical finger gap at the
            # grasp waypoint.  The YAML still marks the step as "closed";
            # this only maps that state to a safe intermediate jaw angle.
            requested_master = float(positions[-1])
            # The adaptive-gripper mimic geometry is inverted at the positive
            # end of its range: 0.12 visually closes the jaws.  Keep the same
            # validated jaw clearance during contact-mode release; the actual
            # opening is provided by the contact pads returning to zero stroke.
            # Keep the validated pick clearance and use the YAML open value
            # for release; it stays below the linkage hard stop at +0.15.
            # Slightly reduce the pick gap again while leaving the release
            # opening at the validated YAML open value (+0.12).
            # Keep a slightly tighter contact gap at pick while preserving the
            # YAML waypoint and the wider release opening.
            master = 0.12 if requested_master >= float(self.config["robot"]["gripper_open"]) else max(requested_master, -0.04)
            closed = float(self.config["robot"]["gripper_closed"])
            opened = float(self.config["robot"]["gripper_open"])
            span = max(abs(closed - opened), 1e-9)
            closure = min(max(abs(master - opened) / span, 0.0), 1.0)
            gripper_point.positions = [master, master, -master, -master, -master, master]
            if self.contact_pad_effort_pub is not None:
                # The prismatic pads use effort interfaces in Gazebo.  Push
                # inward while holding and retract on release.
                pad_effort = -4.0 + 12.0 * closure
                self.contact_pad_effort_pub.publish(
                    Float64MultiArray(data=[pad_effort, pad_effort])
                )
        else:
            gripper_msg.joint_names = [self.gripper_joint]
            gripper_point.positions = [float(positions[-1])]
        gripper_point.time_from_start = Duration(seconds=self._active_step_duration).to_msg()
        gripper_msg.points = [gripper_point]
        self.gripper_command_pub.publish(gripper_msg)

    def _publish_scene(self):
        markers = MarkerArray()
        markers.markers.extend(
            [
                self._cube_marker(
                    1,
                    "table",
                    self.config["scene"]["table"]["pose_xyz"],
                    self.config["scene"]["table"]["size"],
                    (0.45, 0.45, 0.45, 0.85),
                ),
                self._cylinder_marker(
                    2,
                    "pick_point_a",
                    self.config["scene"]["pick_point_a"]["pose_xyz"],
                    0.035,
                    0.004,
                    (0.1, 0.35, 0.9, 0.70),
                ),
                self._cube_marker(
                    3,
                    "place_point_b",
                    self.config["scene"]["place_point_b"]["pose_xyz"],
                    [0.10, 0.10, 0.004],
                    (0.0, 0.65, 0.25, 0.55),
                ),
                self._target_marker(),
                self._text_marker(5, "label_a", "A", self.config["scene"]["pick_point_a"]["pose_xyz"]),
                self._text_marker(6, "label_b", "B", self.config["scene"]["place_point_b"]["pose_xyz"]),
            ]
        )
        self.marker_pub.publish(markers)

    def _marker_base(self, marker_id, namespace, marker_type):
        marker = Marker()
        marker.header.frame_id = self.config["scene"].get("fixed_frame", "base")
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = namespace
        marker.id = marker_id
        marker.type = marker_type
        marker.action = Marker.ADD
        marker.pose.orientation.w = 1.0
        return marker

    def _cube_marker(self, marker_id, namespace, xyz, size, color):
        marker = self._marker_base(marker_id, namespace, Marker.CUBE)
        marker.pose.position.x = float(xyz[0])
        marker.pose.position.y = float(xyz[1])
        marker.pose.position.z = float(xyz[2])
        marker.scale.x = float(size[0])
        marker.scale.y = float(size[1])
        marker.scale.z = float(size[2])
        marker.color.r, marker.color.g, marker.color.b, marker.color.a = color
        return marker

    def _cylinder_marker(self, marker_id, namespace, xyz, radius, height, color):
        marker = self._marker_base(marker_id, namespace, Marker.CYLINDER)
        marker.pose.position.x = float(xyz[0])
        marker.pose.position.y = float(xyz[1])
        marker.pose.position.z = float(xyz[2])
        marker.scale.x = float(radius) * 2.0
        marker.scale.y = float(radius) * 2.0
        marker.scale.z = float(height)
        marker.color.r, marker.color.g, marker.color.b, marker.color.a = color
        return marker

    def _target_marker(self):
        obj = self.config["scene"]["object"]
        xyz = self._target_position()
        return self._cylinder_marker(
            4,
            "target_object",
            xyz,
            float(obj["radius"]),
            float(obj["height"]),
            (0.85, 0.08, 0.08, 1.0),
        )

    def _text_marker(self, marker_id, namespace, text, xyz):
        marker = self._marker_base(marker_id, namespace, Marker.TEXT_VIEW_FACING)
        marker.text = text
        marker.pose.position.x = float(xyz[0])
        marker.pose.position.y = float(xyz[1])
        marker.pose.position.z = float(xyz[2]) + 0.06
        marker.scale.z = 0.04
        marker.color.r = 1.0
        marker.color.g = 1.0
        marker.color.b = 1.0
        marker.color.a = 1.0
        return marker

    def _target_position(self):
        pick = list(self.config["scene"]["pick_point_a"]["pose_xyz"])
        place = list(self.config["scene"]["place_point_b"]["pose_xyz"])
        safe_height = float(self.config["scene"]["safe_height_m"])

        if self._phase in ("idle", "approach_pick", "at_pick"):
            return pick
        if self._phase == "grasped":
            return [pick[0], pick[1], pick[2] + safe_height]
        if self._phase == "carried":
            midpoint = [
                (pick[0] + place[0]) / 2.0,
                (pick[1] + place[1]) / 2.0,
                max(pick[2], place[2]) + safe_height,
            ]
            if self._last_step_name in ("descend_to_place",):
                return [place[0], place[1], place[2] + 0.04]
            return midpoint
        return place


def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = FixedPickTask()
        while rclpy.ok() and not node.should_exit:
            rclpy.spin_once(node, timeout_sec=0.1)
    except Exception as exc:
        if node is not None:
            node.get_logger().error(str(exc))
            node.result_pub.publish(
                String(data=json.dumps({"status": "failed", "failure_reason": str(exc)}))
            )
        else:
            print(f"fixed_pick_task failed: {exc}")
        raise
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
