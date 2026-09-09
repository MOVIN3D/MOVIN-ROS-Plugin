"""
Make the package, the vendored SDK and ``movin_ros`` importable without ROS.

``colcon test`` runs pytest with everything installed; plain
``python -m pytest movin_retarget/test`` on a host without ROS finds the
sources through the repository layout instead.  Package paths are only added
when the real module cannot be found (a bare directory picked up as a
namespace package does not count).  The SDK pinned in
``movin_sdk_python_vendor/MOVIN-SDK-Python`` is always preferred over another
``movin_sdk_python`` installation so the tests exercise the vendored version.
"""

from __future__ import annotations

import importlib.util
import os
import sys

_TEST_DIR = os.path.dirname(os.path.abspath(__file__))
_PACKAGE_DIR = os.path.dirname(_TEST_DIR)
_REPO_DIR = os.path.dirname(_PACKAGE_DIR)


def _importable(dotted: str) -> bool:
    try:
        return importlib.util.find_spec(dotted) is not None
    except (ImportError, ValueError):
        return False


def _ensure(package: str, probe: str, path: str) -> None:
    if _importable(probe) or not os.path.isdir(path):
        return
    module = sys.modules.get(package)
    if module is not None and getattr(module, '__file__', None) is None:
        del sys.modules[package]  # namespace-package remnant from the probe
    if path not in sys.path:
        sys.path.insert(0, path)


def _prefer_vendored_sdk(path: str) -> None:
    init = os.path.join(path, 'movin_sdk_python', '__init__.py')
    if not os.path.isfile(init):
        return
    try:
        spec = importlib.util.find_spec('movin_sdk_python')
    except (ImportError, ValueError):
        spec = None
    origin = getattr(spec, 'origin', None)
    if origin and os.path.realpath(origin) == os.path.realpath(init):
        return
    sys.modules.pop('movin_sdk_python', None)
    if path in sys.path:
        sys.path.remove(path)
    sys.path.insert(0, path)


_ensure('movin_retarget', 'movin_retarget.retarget_core', _PACKAGE_DIR)
_prefer_vendored_sdk(os.path.join(_REPO_DIR, 'movin_sdk_python_vendor', 'MOVIN-SDK-Python'))
_ensure('movin_ros', 'movin_ros.convert', os.path.join(_REPO_DIR, 'movin_ros'))
