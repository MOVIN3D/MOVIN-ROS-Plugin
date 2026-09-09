"""
Optional Unitree G1 retargeting of MOVIN Studio motion capture frames.

:mod:`movin_retarget.retarget_core` holds the ROS-free helpers (bone-dict
rebuild, joint-name enumeration, the retargeting call and the root-pose
conversion); :mod:`movin_retarget.retarget_node` is the rclpy node that
subscribes ``mocap_frame`` and publishes ``joint_states`` plus the
``world -> pelvis`` transform.  Retargeting needs the SDK's optional
``[retargeting]`` extra (numpy, scipy, mujoco, mink).
"""

from __future__ import annotations
