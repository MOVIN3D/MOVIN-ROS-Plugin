"""The ROS-free core must not pull numpy/scipy/mujoco (or rclpy) into the process."""

from __future__ import annotations

import ast
import json
import os
import subprocess
import sys

import movin_ros

import movin_sdk_python

import pytest

CORE_MODULES = ('movin_ros.convert', 'movin_ros.frame_source', 'movin_ros.recording',
                'movin_ros.recording_info')
FORBIDDEN = ('numpy', 'scipy', 'mujoco', 'mink', 'rclpy')

_PROBE = """
import importlib, json, sys
for name in {modules!r}:
    importlib.import_module(name)
loaded = sorted(m for m in sys.modules if m.split('.')[0] in {forbidden!r})
print(json.dumps({{'loaded': loaded, 'modules': list({modules!r})}}))
"""


def _package_root(module):
    return os.path.dirname(os.path.dirname(os.path.abspath(module.__file__)))


def test_core_import_leaves_numeric_stacks_out():
    """Verify core import leaves numeric stacks out."""
    env = dict(os.environ)
    paths = [_package_root(movin_ros), _package_root(movin_sdk_python)]
    if env.get('PYTHONPATH'):
        paths.append(env['PYTHONPATH'])
    env['PYTHONPATH'] = os.pathsep.join(paths)
    env['PYTHONDONTWRITEBYTECODE'] = '1'
    code = _PROBE.format(modules=CORE_MODULES, forbidden=FORBIDDEN)
    result = subprocess.run([sys.executable, '-c', code], env=env, capture_output=True,
                            text=True, timeout=60, check=False)
    assert result.returncode == 0, result.stderr
    report = json.loads(result.stdout.strip().splitlines()[-1])
    assert report['loaded'] == []


def test_core_modules_declare_no_numeric_imports():
    """Verify core modules declare no numeric imports."""
    package_dir = os.path.dirname(os.path.abspath(movin_ros.__file__))
    for name in CORE_MODULES:
        path = os.path.join(package_dir, name.split('.')[-1] + '.py')
        with open(path, encoding='utf-8') as f:
            tree = ast.parse(f.read(), path)
        for node in ast.walk(tree):
            if isinstance(node, ast.Import):
                roots = [alias.name.split('.')[0] for alias in node.names]
            elif isinstance(node, ast.ImportFrom):
                roots = [(node.module or '').split('.')[0]]
            else:
                continue
            for root in roots:
                assert root not in FORBIDDEN, f'{name} imports {root}'


def test_convert_is_stdlib_only():
    """Verify convert is stdlib only."""
    stdlib = getattr(sys, 'stdlib_module_names', None)
    if stdlib is None:
        pytest.skip('sys.stdlib_module_names needs Python 3.10+')
    package_dir = os.path.dirname(os.path.abspath(movin_ros.__file__))
    path = os.path.join(package_dir, 'convert.py')
    with open(path, encoding='utf-8') as f:
        tree = ast.parse(f.read(), path)
    for node in ast.walk(tree):
        if isinstance(node, ast.Import):
            roots = [alias.name.split('.')[0] for alias in node.names]
        elif isinstance(node, ast.ImportFrom):
            roots = [(node.module or '').split('.')[0]]
        else:
            continue
        for root in roots:
            assert root in stdlib or root == '__future__', f'convert.py imports {root}'
