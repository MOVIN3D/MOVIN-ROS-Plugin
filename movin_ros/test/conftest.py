"""
Pytest configuration for the ROS-free ``movin_ros`` tests.

Makes ``movin_ros`` and the vendored MOVIN SDK importable when the tests run
outside a ROS workspace (plain ``python3 -m pytest movin_ros/test`` on a host
without ROS or numpy) and provides the fixtures.  The OSC/UDP helpers live in
``osc_helpers.py`` (importable by name from any test module).
"""

from __future__ import annotations

import importlib.util
import os
import sys

import pytest

TEST_DIR = os.path.dirname(os.path.abspath(__file__))
PACKAGE_DIR = os.path.dirname(TEST_DIR)
REPO_DIR = os.path.dirname(PACKAGE_DIR)
SDK_DIR = os.path.join(REPO_DIR, 'movin_sdk_python_vendor', 'MOVIN-SDK-Python')

# Host runs must not leave __pycache__ behind in the source tree (colcon/Docker mount it
# read-only anyway); the pyc of this file itself is written before this line runs.
sys.dont_write_bytecode = True


def _prefer_checkout(name: str, path: str) -> None:
    """
    Make ``import <name>`` resolve to the package under ``path`` (this repository).

    Without ROS nothing is installed, and a developer's interpreter may carry
    another copy of the SDK (or a namespace package picked up from the repo
    root); the tests pin the vendored submodule, so the checkout wins whenever
    the importable package does not already live there.
    """
    local_pkg = os.path.join(path, name)
    if not os.path.isdir(local_pkg):
        return
    module = sys.modules.get(name)
    origin = getattr(module, '__file__', None) if module is not None else None
    if origin is None:
        spec = importlib.util.find_spec(name)
        origin = spec.origin if spec is not None else None
    if origin and os.path.dirname(os.path.abspath(origin)) == local_pkg:
        return
    for loaded in [m for m in sys.modules if m == name or m.startswith(name + '.')]:
        del sys.modules[loaded]
    if path in sys.path:
        sys.path.remove(path)
    sys.path.insert(0, path)


_prefer_checkout('movin_ros', PACKAGE_DIR)
_prefer_checkout('movin_sdk_python', SDK_DIR)
if TEST_DIR not in sys.path:  # `import osc_helpers` from the test modules
    sys.path.insert(0, TEST_DIR)

import osc_helpers  # noqa: E402,I100  (needs the sys.path setup above)

LIVE_FIXTURE = osc_helpers.LIVE_FIXTURE
LEGACY_FIXTURE = osc_helpers.LEGACY_FIXTURE


@pytest.fixture(scope='session')
def live_fixture_path() -> str:
    """Path of the real 1 s MOVINManV3 recording captured from MOVIN Studio."""
    assert os.path.exists(LIVE_FIXTURE), LIVE_FIXTURE
    return LIVE_FIXTURE


@pytest.fixture(scope='session')
def legacy_fixture_path() -> str:
    """Path of the synthetic 52-bone legacy MOVINMan recording (``make_fixture.py``)."""
    assert os.path.exists(LEGACY_FIXTURE), f'{LEGACY_FIXTURE} missing; run test/make_fixture.py'
    return LEGACY_FIXTURE


@pytest.fixture(scope='session')
def live_messages(live_fixture_path) -> list:
    """Every ``(address, args)`` message of the live fixture."""
    return osc_helpers.recorded_messages(live_fixture_path)


@pytest.fixture(scope='session')
def live_frames(live_messages) -> list:
    """Every complete frame of the live fixture, oldest first."""
    frames = osc_helpers.assemble_frames(live_messages)
    assert len(frames) >= 30
    return frames


@pytest.fixture(scope='session')
def legacy_frames(legacy_fixture_path) -> list:
    """Every complete frame of the synthetic legacy fixture, oldest first."""
    return osc_helpers.assemble_frames(osc_helpers.recorded_messages(legacy_fixture_path))


@pytest.fixture
def local_receiver():
    """Start a ``MocapReceiver`` bound to ``127.0.0.1`` on an ephemeral port; stop it after."""
    from movin_sdk_python import MocapReceiver

    receiver = MocapReceiver(host='127.0.0.1', port=0)
    receiver.start()
    try:
        yield receiver
    finally:
        receiver.stop()
