# Third-party notices

This repository is licensed under the Apache License, Version 2.0 (see `LICENSE`
and `NOTICE`). It redistributes the following third-party components.

## MOVIN SDK Python

`movin_sdk_python_vendor/MOVIN-SDK-Python` is a git submodule of
[MOVIN-SDK-Python](https://github.com/MOVIN3D/MOVIN-SDK-Python). Building this
workspace copies the SDK's Python package and assets into the colcon
`install/` tree, which is a redistribution of the SDK.

```
MOVIN SDK Python
Copyright 2026 MOVIN Inc.

This product is licensed under the Apache License, Version 2.0.
```

The full license text is in `movin_sdk_python_vendor/MOVIN-SDK-Python/LICENSE`
and the notice file in `movin_sdk_python_vendor/MOVIN-SDK-Python/NOTICE`.

## Unitree G1 robot description

`movin_g1_description/urdf` and `movin_g1_description/meshes` contain the
Unitree G1 URDF and STL meshes from
[unitree_ros](https://github.com/unitreerobotics/unitree_ros) and
[unitree_mujoco](https://github.com/unitreerobotics/unitree_mujoco) by Unitree
Robotics. They are subject to Unitree's own licensing terms; refer to those
repositories for details.

## GMR: General Motion Retargeting

The retargeting approach used by the MOVIN SDK `Retargeter` is derived from
[GMR](https://github.com/YanjieZe/GMR). See the SDK repository for details.
