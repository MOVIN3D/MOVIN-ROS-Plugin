"""ament_pep257 docstring test (skipped where ament_pep257 is not installed)."""

from __future__ import annotations

import pytest

ament_pep257 = pytest.importorskip('ament_pep257.main')


@pytest.mark.linter
@pytest.mark.pep257
def test_pep257():
    """Run ament_pep257 on the package."""
    rc = ament_pep257.main(argv=['.', 'test'])
    assert rc == 0, 'Found code style errors / warnings'
