# Copyright 2026
# Licensed under the Apache License, Version 2.0

import logging
import threading
import time

import rclpy
from niryo_ned_ros2_interfaces.msg import EEButtonStatus
from rclpy.executors import MultiThreadedExecutor
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import JointState

from lerobot.teleoperators.teleoperator import Teleoperator

from .config_ned2_ros2_leader import NED2ROS2LeaderConfig

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


class NED2ROS2Leader(Teleoperator):
    config_class = NED2ROS2LeaderConfig
    name = "ned2_ros2_leader"

    def __init__(self, config: NED2ROS2LeaderConfig):
        super().__init__(config)
        self.config = config

        self._node = None
        self._executor = None
        self._executor_thread = None

        self._joint_state = None
        self._lock = threading.Lock()

        self._gripper_value = self.config.gripper_open_value

    @property
    def action_features(self) -> dict[str, type]:
        features = {}
        for name in self.config.joint_names:
            features[f"{name}.pos"] = float
        if self.config.gripper_key:
            features[self.config.gripper_key] = float
        return features

    @property
    def feedback_features(self) -> dict[str, type]:
        return {}

    @property
    def is_connected(self) -> bool:
        return self._node is not None

    def connect(self, calibrate: bool = True) -> None:
        del calibrate
        _ensure_rclpy_init()

        node_name = f"ned2_ros2_leader_{self.id or 'default'}"
        self._node = rclpy.create_node(node_name)

        joint_states_topic = _resolve_topic(self.config.namespace, self.config.joint_states_topic)
        self._node.create_subscription(
            JointState, joint_states_topic, self._on_joint_state, qos_profile_sensor_data
        )

        if self.config.enable_button_gripper:
            button_topic = _resolve_topic(self.config.namespace, self.config.leader_button_topic)
            self._node.create_subscription(
                EEButtonStatus, button_topic, self._on_leader_button, qos_profile_sensor_data
            )
            logger.info("Listening for leader button events on: %s", button_topic)
            logger.info("Single press -> TOGGLE gripper")

        self._executor = MultiThreadedExecutor(num_threads=2)
        self._executor.add_node(self._node)
        self._executor_thread = threading.Thread(target=self._executor.spin, daemon=True)
        self._executor_thread.start()

        if self.config.wait_for_joint_states:
            self._wait_for_joint_states()

        logger.info("%s connected.", self)

    @property
    def is_calibrated(self) -> bool:
        return True

    def calibrate(self) -> None:
        return

    def configure(self) -> None:
        return

    def _wait_for_joint_states(self) -> None:
        start = time.perf_counter()
        while time.perf_counter() - start < self.config.startup_timeout_s:
            with self._lock:
                if self._joint_state is not None:
                    return
            time.sleep(0.05)
        logger.warning("Timed out waiting for joint states.")

    def _on_joint_state(self, msg: JointState) -> None:
        with self._lock:
            self._joint_state = msg

    def _toggle_gripper(self) -> None:
        if self._gripper_value == self.config.gripper_open_value:
            self._gripper_value = self.config.gripper_close_value
        else:
            self._gripper_value = self.config.gripper_open_value

    def _on_leader_button(self, msg: EEButtonStatus) -> None:
        if msg.action != EEButtonStatus.SINGLE_PUSH_ACTION:
            return
        self._toggle_gripper()
        logger.info("Leader button SINGLE press -> TOGGLE gripper")

    def get_action(self) -> dict[str, float]:
        with self._lock:
            js = self._joint_state
            if js is None:
                raise RuntimeError("No JointState received yet.")
            name_to_pos = dict(zip(js.name, js.position))

        action = {}
        for name in self.config.joint_names:
            if name in name_to_pos:
                action[f"{name}.pos"] = float(name_to_pos[name])
            else:
                action[f"{name}.pos"] = 0.0

        if self.config.gripper_key:
            action[self.config.gripper_key] = float(self._gripper_value)

        return action

    def send_feedback(self, feedback: dict[str, float]) -> None:
        raise NotImplementedError

    def disconnect(self) -> None:
        if self._executor:
            self._executor.shutdown()
        if self._executor_thread:
            self._executor_thread.join(timeout=1.0)
        if self._node:
            self._node.destroy_node()

        self._executor = None
        self._executor_thread = None
        self._node = None

        if self.config.shutdown_rclpy_on_disconnect and rclpy.ok():
            rclpy.shutdown()

        logger.info("%s disconnected.", self)
