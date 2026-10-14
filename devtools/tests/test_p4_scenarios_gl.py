"""Phase 4 scenarios: the framework's rasterization against the GL-free raster.

Needs the fixtures written by ``python -m devtools.simlab.fixtures --out
_local/fixtures/p4 --maps <P4 names>`` (built with the framework's own map
construction and ``walls_and_boundary_grid``, which requires OpenGL); the
comparison itself is CPU-only. A missing fixture skips its case, and the
count of compared fixtures is asserted separately so that an empty fixture
directory cannot pass silently.
"""

from __future__ import annotations

import json
import sys
from pathlib import Path

import numpy as np
import pytest

REPO = Path(__file__).resolve().parents[2]
if str(REPO) not in sys.path:
    sys.path.insert(0, str(REPO))

from devtools.simlab import p4_scenarios as S  # noqa: E402

FIXTURES = REPO / "_local" / "fixtures" / "p4"
PRESENT = sorted(p.stem[len("truth_"):] for p in FIXTURES.glob("truth_P4*.npz")) \
    if FIXTURES.exists() else []


def test_fixture_set_is_complete_when_present():
    if not PRESENT:
        pytest.skip("no Phase 4 fixtures extracted in this checkout")
    assert PRESENT == sorted(S.registered_names(seeds=(1, 2))), PRESENT


@pytest.mark.parametrize("name", PRESENT)
def test_framework_raster_matches_the_specification(name):
    data = np.load(FIXTURES / f"truth_{name}.npz", allow_pickle=False)
    info = json.loads(str(data["info"]))
    spec = S.spec_by_name(name)
    assert info["spec_sha256"] == S.spec_sha256(spec)
    assert info["start_area"] == spec["start_area"]
    walls = data["walls"] > 0
    mine = S.raster(spec)["walls"] > 0
    inter = (walls & mine).sum()
    union = (walls | mine).sum()
    assert inter / union > 0.95, (name, inter / union)
    # Every box is solid in the framework's raster (orientation of NormalBox).
    w, h = spec["size_area"]
    for x0, y0, x1, y1 in spec["boxes"]:
        r = int(round(h / 2 - (y0 + y1) / 2))
        c = int(round((x0 + x1) / 2 + w / 2))
        assert walls[r, c], (name, (x0, y0, x1, y1))
