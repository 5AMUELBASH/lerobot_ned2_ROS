"""LeRobot robot plugin for Niryo NED2 over ROS2.

Import the teleoperator package here so a single third-party plugin distribution
registers both robot and teleoperator config subclasses during discovery.
"""

from .config_ned2_ros2_follower import NED2ROS2FollowerConfig
from .ned2_ros2_follower import NED2ROS2Follower
from lerobot.teleoperators import lerobot_teleoperator_ned2_ros2 as _teleoperator_plugin

__all__ = ["NED2ROS2Follower", "NED2ROS2FollowerConfig", "_teleoperator_plugin"]
