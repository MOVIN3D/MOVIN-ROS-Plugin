"""
Receive, record and replay MOVIN Studio motion capture in ROS 2.

The ROS-free core lives in :mod:`movin_ros.convert` (coordinate conversion,
pure Python), :mod:`movin_ros.frame_source` (receiver construction and frame
polling on top of ``movin_sdk_python``) and :mod:`movin_ros.recording`
(raw-OSC recording management).  :mod:`movin_ros.mocap_node` is the rclpy node.
"""

from __future__ import annotations
