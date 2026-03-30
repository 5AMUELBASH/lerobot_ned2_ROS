# Copyright 2026
# Licensed under the Apache License, Version 2.0

import logging
import threading
import time
from typing import Any

import rclpy
from rclpy.action import ActionClient
from rclpy.executors import MultiThreadedExecutor
from rclpy.qos import qos_profile_sensor_data

from builtin_interfaces.msg import Duration
from control_msgs.action import FollowJointTrajectory
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

from lerobot.cameras.utils import make_cameras_from_configs
from niryo_ned_ros2_interfaces.msg import Tool
from niryo_ned_ros2_interfaces.srv import ToolCommand, Trigger

from lerobot.robots.robot import Robot

from .config_ned2_ros2_follower import NED2ROS2FollowerConfig

logger = logging.getLogger(__name__)


def _resolve_topic(namespace: str, topic: str) -> str:
    if topic.startswith("/"):
        return topic
    if not namespace:
        return f"/{topic}"
    if namespace.endswith("/"):
        return f"{namespace}{topic}"
    return f"{namespace}/{topic}"


def _ensure_rclpy_init() -> None:
    try:
        if not rclpy.ok():
            rclpy.init()
    except Exception:
        rclpy.init()


class NED2ROS2Follower(Robot):
    config_class = NED2ROS2FollowerConfig
    name = "ned2_ros2_follower"

    def __init__(self, config: NED2ROS2FollowerConfig):
        super().__init__(config)
        self.config = config

        self.cameras = make_cameras_from_configs(self.config.cameras)

        self._node = None
        self._executor = None
        self._executor_thread = None

        self._client = None
        self._goal_handle = None

        self._update_tool_client = None
        self._open_client = None
        self._close_client = None

        self._joint_state = None
        self._tool_position = None

        self._lock = threading.Lock()
        self._gripper_closed = None
        self._warned_missing = set()
        self._warned_camera_read = set()

        # Non-blocking gripper worker state.
        self._pending_gripper_open: bool | None = None
        self._gripper_busy = False
        self._gripper_timer = None

    @property
    def _camera_ft(self) -> dict[str, tuple]:
        return {
            cam_name: (int(cam_cfg.height), int(cam_cfg.width), 3)
            for cam_name, cam_cfg in self.cameras.items()
        }

    @property
    def observation_features(self) -> dict[str, type]:
        features = {}
        if self.config.joint_names:
            for name in self.config.joint_names:
                features[f"{name}.pos"] = float
        if self.config.gripper_key:
            features[self.config.gripper_key] = float

        features.update(self._camera_ft)
        return features

    @property
    def action_features(self) -> dict[str, type]:
        features = {}
        if self.config.joint_names:
            for name in self.config.joint_names:
                features[f"{name}.pos"] = float
        if self.config.gripper_key:
            features[self.config.gripper_key] = float
        return features

    @property
    def is_connected(self) -> bool:
        return self._node is not None

    def connect(self, calibrate: bool = True) -> None:
        del calibrate
        _ensure_rclpy_init()

        node_name = f"LeRobot_ROS_Node"
        self._node = rclpy.create_node(node_name)

        joint_states_topic = _resolve_topic(self.config.namespace, self.config.joint_states_topic)
        tool_motor_topic = _resolve_topic(self.config.namespace, self.config.tool_motor_topic)

        self._node.create_subscription(
            JointState, joint_states_topic, self._on_joint_state, qos_profile_sensor_data
        )
        self._node.create_subscription(Tool, tool_motor_topic, self._on_tool_motor, qos_profile_sensor_data)

        action_name = _resolve_topic(self.config.namespace, self.config.action_name)
        self._client = ActionClient(self._node, FollowJointTrajectory, action_name)

        self._update_tool_client = self._node.create_client(
            Trigger, _resolve_topic(self.config.namespace, self.config.update_tool_service)
        )
        self._open_client = self._node.create_client(
            ToolCommand, _resolve_topic(self.config.namespace, self.config.open_gripper_service)
        )
        self._close_client = self._node.create_client(
            ToolCommand, _resolve_topic(self.config.namespace, self.config.close_gripper_service)
        )

        self._executor = MultiThreadedExecutor(num_threads=2)
        self._executor.add_node(self._node)
        self._executor_thread = threading.Thread(target=self._executor.spin, daemon=True)
        self._executor_thread.start()

        self._wait_for_server_and_services()

        if self.config.wait_for_joint_states:
            self._wait_for_joint_states()

        if self.config.update_tool_on_connect:
            self._update_tool()

        self._gripper_timer = self._node.create_timer(
            self.config.gripper_worker_period_s,
            self._flush_gripper_command,
        )

        for cam in self.cameras.values():
            cam.connect()

        logger.info("%s connected.", self)

    @property
    def is_calibrated(self) -> bool:
        return True

    def calibrate(self) -> None:
        return

    def configure(self) -> None:
        return

    def _wait_for_server_and_services(self) -> None:
        start = time.perf_counter()
        while time.perf_counter() - start < self.config.startup_timeout_s:
            if self._client.wait_for_server(timeout_sec=0.1):
                break
        else:
            logger.warning("Timed out waiting for FollowJointTrajectory action server.")

        for client, label in [
            (self._update_tool_client, "update_tool"),
            (self._open_client, "open_gripper"),
            (self._close_client, "close_gripper"),
        ]:
            start = time.perf_counter()
            while time.perf_counter() - start < self.config.startup_timeout_s:
                if client.wait_for_service(timeout_sec=0.1):
                    break
            else:
                logger.warning("Timed out waiting for service: %s", label)

    def _wait_for_joint_states(self) -> None:
        start = time.perf_counter()
        while time.perf_counter() - start < self.config.startup_timeout_s:
            with self._lock:
                if self._joint_state is not None:
                    return
            time.sleep(0.05)
        logger.warning("Timed out waiting for joint states.")

    def _update_tool(self) -> None:
        try:
            self._update_tool_client.call_async(Trigger.Request())
        except Exception as exc:
            logger.warning("update_tool call failed: %s", exc)

    def _on_joint_state(self, msg: JointState) -> None:
        with self._lock:
            self._joint_state = msg

    def _on_tool_motor(self, msg: Tool) -> None:
        with self._lock:
            try:
                self._tool_position = 1 if int(msg.position) > 2400 else 0
            except Exception:
                self._tool_position = None

    def _resolve_joint_names(self) -> list[str] | None:
        if self.config.joint_names:
            return self.config.joint_names
        with self._lock:
            if self._joint_state is None:
                return None
            return list(self._joint_state.name)

    def get_observation(self) -> dict[str, Any]:
        joint_names = self._resolve_joint_names()
        obs = {}
        with self._lock:
            js = self._joint_state
            if js is None or joint_names is None:
                return obs
            name_to_pos = dict(zip(js.name, js.position))

        for name in joint_names:
            if name in name_to_pos:
                obs[f"{name}.pos"] = float(name_to_pos[name])
            else:
                obs[f"{name}.pos"] = 0.0

        if self.config.gripper_key and self._tool_position is not None:
            obs[self.config.gripper_key] = float(self._tool_position)

        for cam_name, cam in self.cameras.items():
            try:
                obs[cam_name] = cam.read()
            except Exception as exc:
                if cam_name not in self._warned_camera_read:
                    logger.warning("Camera '%s' read failed: %s", cam_name, exc)
                    self._warned_camera_read.add(cam_name)

        return obs

    def _build_goal(self, joint_names: list[str], positions: list[float]) -> FollowJointTrajectory.Goal:
        goal = FollowJointTrajectory.Goal()
        traj = JointTrajectory()
        traj.joint_names = list(joint_names)

        pt = JointTrajectoryPoint()
        pt.positions = list(positions)

        dur = Duration()
        dur.sec = int(self.config.point_time)
        dur.nanosec = int((self.config.point_time - int(self.config.point_time)) * 1e9)
        pt.time_from_start = dur

        traj.points = [pt]
        goal.trajectory = traj
        return goal

    def send_action(self, action: dict[str, Any]) -> dict[str, Any]:
        if not self.is_connected:
            raise RuntimeError("Robot is not connected.")

        joint_names = self._resolve_joint_names()
        if not joint_names:
            raise RuntimeError("Joint names are unknown. Provide config.joint_names or wait for JointState.")

        with self._lock:
            js = self._joint_state
            name_to_pos = dict(zip(js.name, js.position)) if js else {}

        positions = []
        for name in joint_names:
            key = f"{name}.pos"
            if key in action:
                positions.append(float(action[key]))
            elif name in action:
                positions.append(float(action[name]))
            elif name in name_to_pos:
                positions.append(float(name_to_pos[name]))
            else:
                if name not in self._warned_missing:
                    logger.warning("Missing joint '%s' in action; using 0.0", name)
                    self._warned_missing.add(name)
                positions.append(0.0)

        goal = self._build_goal(joint_names, positions)

        if self._goal_handle is not None:
            try:
                self._goal_handle.cancel_goal_async()
            except Exception:
                pass

        try:
            send_future = self._client.send_goal_async(goal)
            send_future.add_done_callback(self._on_goal_response)
        except Exception as exc:
            logger.error("send_goal_async failed: %s", exc)

        if self.config.gripper_key and self.config.gripper_key in action:
            self._handle_gripper(float(action[self.config.gripper_key]))

        return action

    def _on_goal_response(self, fut) -> None:
        try:
            goal_handle = fut.result()
            if not goal_handle.accepted:
                logger.warning("Goal rejected by action server.")
            self._goal_handle = goal_handle
        except Exception as exc:
            logger.error("send_goal_async failed: %s", exc)

    def _handle_gripper(self, value: float) -> None:
        want_open = True if value == 1 else False
        want_closed = not want_open

        with self._lock:
            if self._pending_gripper_open is not None and self._pending_gripper_open == want_open:
                return

            if (self._pending_gripper_open is None
                and self._gripper_closed is not None
                and self._gripper_closed == want_closed
            ):
                return

            self._pending_gripper_open = want_open

    def _build_gripper_request(self, opening: bool) -> ToolCommand.Request:
        req = ToolCommand.Request()
        req.id = self.config.tool_id
        req.speed = self.config.gripper_speed
        req.hold_torque = self.config.gripper_hold_torque
        req.max_torque = self.config.gripper_max_torque
        req.position = self.config.gripper_open_pos if opening else self.config.gripper_close_pos
        return req

    def _flush_gripper_command(self) -> None:
        with self._lock: 
            if self._gripper_busy:
                return
            if self._pending_gripper_open is None:
                return

            opening = self._pending_gripper_open
            self._pending_gripper_open = None
            desired_closed = not opening

            if self._gripper_closed is not None and self._gripper_closed == desired_closed:
                return

            self._gripper_busy = True

        self._dispatch_gripper_command(opening)

    def _dispatch_gripper_command(self, opening: bool) -> None:
        req = self._build_gripper_request(opening)
        client = self._open_client if opening else self._close_client
        label = "OPEN" if opening else "CLOSE"

        logger.debug("Gripper %s -> tool_id=%s target=%s", label, req.id, req.position)
        try:
            fut = client.call_async(req)
            fut.add_done_callback(
                lambda fut, opening=opening, label=label: self._on_gripper_command_done(
                    fut,
                    opening,
                    label,
                )
            )
        except Exception as exc:
            logger.error("Gripper service call failed: %s", exc)
            with self._lock:
                self._gripper_busy = False

    def _on_gripper_command_done(self, fut, opening: bool, label: str) -> None:
        try:
            fut.result()
            with self._lock:
                self._gripper_closed = not opening
            logger.debug("Gripper %s done", label)
        except Exception as exc:
            logger.error("Gripper %s failed: %s", label, exc)
        finally:
            with self._lock:
                self._gripper_busy = False

    def disconnect(self) -> None:
        if self._gripper_timer is not None:
            try:
                self._gripper_timer.cancel()
            except Exception:
                pass
            self._gripper_timer = None

        for cam in self.cameras.values():
            try:
                cam.disconnect()
            except Exception:
                pass

        if self._executor:
            self._executor.shutdown()
        if self._executor_thread:
            self._executor_thread.join(timeout=1.0)
        if self._node:
            self._node.destroy_node()

        with self._lock:
            self._pending_gripper_open = None
            self._gripper_busy = False

        self._executor = None
        self._executor_thread = None
        self._node = None

        #if self.config.shutdown_rclpy_on_disconnect and rclpy.ok():
        #    rclpy.shutdown()

        logger.info("%s disconnected.", self)
