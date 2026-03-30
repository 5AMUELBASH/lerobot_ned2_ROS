#!/usr/bin/env python

# Copyright 2026
# Licensed under the Apache License, Version 2.0

from dataclasses import dataclass, field
from typing import TypeAlias

from lerobot.teleoperators.config import TeleoperatorConfig


@TeleoperatorConfig.register_subclass("ned2_ros2_leader")
@dataclass
class NED2ROS2LeaderConfig(TeleoperatorConfig):
    """Configuration for the Niryo NED2 leader (joint states via ROS2)."""

    namespace: str = "/leader"

    # Topics (relative to namespace if not absolute)
    joint_states_topic: str = "joint_states"

    # Leader custom button topic (relative to namespace if not absolute)
    leader_button_topic: str = (
        "niryo_robot_hardware_interface/end_effector_interface/custom_button_status"
    )

    # Explicit joint order from the URDF
    joint_names: list[str] = field(
        default_factory=lambda: [
            "joint_1",
            "joint_2",
            "joint_3",
            "joint_4",
            "joint_5",
            "joint_6",
        ]
    )

    # Gripper action emitted by this teleoperator
    gripper_key: str = "gripper"
    gripper_open_value: float = 1.0
    gripper_close_value: float = 0.0

    # Gripper control source. The keyboard flag is retained as a no-op so older
    # CLI/config inputs still parse cleanly after keyboard support removal.
    enable_button_gripper: bool = True

    # Startup behavior
    startup_timeout_s: float = 10.0
    wait_for_joint_states: bool = True

    # Advanced: set to True only if this plugin owns the ROS2 context
    shutdown_rclpy_on_disconnect: bool = False


NED2ROS2LeaderConfigAlias: TypeAlias = NED2ROS2LeaderConfig
