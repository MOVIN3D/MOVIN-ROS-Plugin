"""
The installed ``movin_sdk_python`` carries every file of the SDK package.

``ament_python_install_package`` copies the submodule's package directory
into the install tree; the SDK's own setup.py / pyproject.toml (and their
package-data patterns) are never used.  This test compares the installed copy
with the submodule, so an SDK asset that does not reach the install tree
(skeleton MJCF, mesh NPZ, robot models, IK configs) fails here instead of at
runtime.  CMakeLists.txt passes the installed package directory in
``MOVIN_SDK_INSTALLED_PACKAGE``.
"""

import json
import os
import subprocess
import sys

import pytest

SOURCE_PACKAGE = os.path.join(
    os.path.dirname(os.path.dirname(os.path.abspath(__file__))),
    'MOVIN-SDK-Python', 'movin_sdk_python')
INSTALLED_PACKAGE = os.environ.get('MOVIN_SDK_INSTALLED_PACKAGE', '')


def _package_files(root):
    """Return the package's files relative to ``root`` (byte-code caches excluded)."""
    files = set()
    for directory, subdirs, names in os.walk(root, followlinks=True):
        subdirs[:] = [d for d in subdirs if d != '__pycache__']
        for name in names:
            if not name.endswith(('.pyc', '.pyo')):
                files.add(os.path.relpath(os.path.join(directory, name), root))
    return files


@pytest.fixture(scope='module')
def installed():
    """Return the installed package directory (the test only runs from CMake)."""
    if not INSTALLED_PACKAGE:
        pytest.skip('MOVIN_SDK_INSTALLED_PACKAGE not set (run through colcon test)')
    assert os.path.isfile(os.path.join(INSTALLED_PACKAGE, '__init__.py')), INSTALLED_PACKAGE
    return INSTALLED_PACKAGE


def test_every_sdk_file_is_installed(installed):
    """Each file of the submodule's package exists in the install tree."""
    source = _package_files(SOURCE_PACKAGE)
    assert any(name.endswith('.npz') for name in source)  # the SDK ships its mesh assets
    missing = sorted(source - _package_files(installed))
    assert not missing, f'not installed: {missing}'


def test_preset_assets_resolve_inside_the_install_tree(installed):
    """``SkeletonPreset.mjcf_path`` / ``mesh_npz_path`` of the installed SDK name existing files."""
    code = (
        'import json\n'
        'from movin_sdk_python.utils.skeleton_presets import get_preset\n'
        'paths = []\n'
        'for name in ("movinman", "movinman_v3"):\n'
        '    preset = get_preset(name)\n'
        '    paths += [preset.mjcf_path, preset.mesh_npz_path]\n'
        'print(json.dumps(paths))\n'
    )
    site_packages = os.path.dirname(installed)
    env = dict(os.environ, PYTHONPATH=site_packages)
    out = subprocess.run([sys.executable, '-c', code], env=env, cwd=site_packages, check=True,
                         capture_output=True, text=True).stdout
    paths = json.loads(out.strip().splitlines()[-1])
    assert len(paths) == 4
    for path in paths:
        assert path is not None and os.path.isfile(path), path
    # A copied install resolves into the install tree; a --symlink-install one may point back
    # into the submodule through the symlinked module, which is fine as well.
    if not os.path.islink(os.path.join(installed, 'utils', 'skeleton_presets.py')):
        root = os.path.realpath(installed) + os.sep
        assert all(os.path.realpath(path).startswith(root) for path in paths), paths
