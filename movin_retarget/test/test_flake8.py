"""ament_flake8 lint test (skipped where ament_flake8 is not installed)."""

from __future__ import annotations

import pytest

ament_flake8 = pytest.importorskip('ament_flake8.main')


@pytest.mark.flake8
@pytest.mark.linter
def test_flake8():
    """Run ament_flake8 on the package."""
    rc, errors = ament_flake8.main_with_errors(argv=[])
    assert rc == 0, \
        'Found %d code style errors / warnings:\n' % len(errors) + \
        '\n'.join(errors)
