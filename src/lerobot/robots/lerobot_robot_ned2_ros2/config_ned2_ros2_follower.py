#!/usr/bin/env python

# Copyright 2026
# Licensed under the Apache License, Version 2.0

from dataclasses import dataclass, field
from typing import TypeAlias

from lerobot.robots.config import RobotConfig
from lerobot.cameras.opencv import OpenCVCameraConfig
from lerobot.cameras import CameraConfig
from lerobot.cameras.configs import Cv2Rotation


@RobotConfig.register_subclass("ned2_ros2_follower")
@dataclass
class NED2ROS2FollowerConfig(RobotConfig):
    """Configuration for the Niryo NED2 follower controlled via ROS2."""

    namespace: str = "/follower"

    # Topics / actions (relative to namespace if not absolute)
    joint_states_topic: str = "joint_states"
    action_name: str = "niryo_robot_follow_joint_trajectory_controller/follow_joint_trajectory"

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

    
    cameras : dict[str, CameraConfig] = field(
        default_factory=lambda: {
            "front_cam": OpenCVCameraConfig(
                index_or_path="/dev/v4l/by-id/usb-BC-250311-ZW_USB_2.0_Camera-video-index0",
                width=640,
                height=480,
                fps=30,
            ),
            "hand_cam": OpenCVCameraConfig(
                index_or_path="/dev/v4l/by-id/usb-H264_USB_Camera_H264_USB_Camera_2020032801-video-index0",
                width=640,
                height=480,
                fps=30,
                rotation=Cv2Rotation.ROTATE_180,
            ),
        }
    )       


    # Trajectory timing (seconds)
    point_time: float = 0.4     

    # Gripper services / topics
    update_tool_service: str = "niryo_robot_tools_commander/update_tool"
    open_gripper_service: str = "niryo_robot/tools/open_gripper"
    close_gripper_service: str = "niryo_robot/tools/close_gripper"
    tool_motor_topic: str = "niryo_robot_hardware/tools/motor"
    
    # Gripper behavior
    gripper_key: str = "gripper"
    gripper_open_value: float = 1.0
    gripper_close_value: float = 0.0
    gripper_toggle_threshold: float = 0.5

    tool_id: int = 13
    gripper_open_pos: int = 2900
    gripper_close_pos: int = 1900
    gripper_speed: int = 900
    gripper_hold_torque: int = 150
    gripper_max_torque: int = 150

    gripper_worker_period_s: float = 0.01
    update_tool_on_connect: bool = True
    update_tool_each_toggle: bool = False

    # Startup behavior
    startup_timeout_s: float = 10.0
    wait_for_joint_states: bool = True

    # Advanced: set to True only if this plugin owns the ROS2 context
    #shutdown_rclpy_on_disconnect: bool = False


NED2ROS2FollowerConfigAlias: TypeAlias = NED2ROS2FollowerConfig
