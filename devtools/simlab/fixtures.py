#!/usr/bin/env python3
"""Extract walls-only truth fixtures from the supplied maps.

For each map the script builds the map object once (no simulation step is
executed), rasterizes its real wall entities with the framework's own
``walls_and_boundary_grid`` (the grids the red-team scorer uses), records the
disposal-center geometry and the drone start positions, and writes
``_local/fixtures/truth_<Map>.npz`` plus a JSON summary.

Building a map needs an OpenGL context (the playground creates a hidden
window), so run it where simulations run, e.g.::

    PYGLET_HEADLESS=True PYGLET_HEADLESS_DEVICE=7 python -m devtools.simlab.fixtures

The fixtures are ground truth. They are used only by development tools
(probes, synthetic layouts, the offline simulator and offline analysis) and
must never be read by agent code.
"""

from __future__ import annotations

import argparse
import json
import sys
from pathlib import Path

import numpy as np

REPO_ROOT = Path(__file__).resolve().parents[2]
DEFAULT_OUT = REPO_ROOT / "_local" / "fixtures"
MAPS = ("Map01", "Map02", "Map03", "Map04", "Map05")


def extract(map_name: str, out_dir: Path) -> dict:
    """Fixture of an official map, or of a Phase 4 custom scenario
    (``p4_scenarios``), whose zones and region annotations are added to the
    summary for evaluation tooling."""
    import swarm_rescue.launcher as launcher_module
    extra = {}
    if map_name in MAPS:
        map_class = getattr(launcher_module, map_name)
    else:
        from . import p4_scenarios
        spec = p4_scenarios.spec_by_name(map_name)
        map_class = p4_scenarios.make_map_class(spec)
        extra = {"zones": spec["zones"], "annotations": spec["annotations"],
                 "spec_sha256": p4_scenarios.spec_sha256(spec),
                 "partition": spec["partition"], "synthetic": True}
    return extract_class(map_name, map_class, out_dir, extra)


def extract_class(map_name: str, map_class, out_dir: Path, extra: dict = None) -> dict:
    from swarm_rescue.simulation.drone.drone_motionless import DroneMotionless
    from swarm_rescue.simulation.reporting.place_score_manager import (
        walls_and_boundary_grid,
    )

    the_map = map_class(drone_type=DroneMotionless, zones_config=())
    playground = the_map.playground
    obstacle_render = the_map.explored_map.truth_obstacle_grid()
    walls, boundary = walls_and_boundary_grid(playground,
                                              truth_grid=obstacle_render)
    dc = the_map.disposal_center
    (dc_x, dc_y), _ = the_map._disposal_center_pos
    starts = np.array([[pos[0][0], pos[0][1]] for pos in the_map._drones_pos])
    info = {
        "map": map_name,
        "size_area": list(the_map.size_area),
        "max_timestep_limit": the_map.max_timestep_limit,
        "max_walltime_limit": the_map.max_walltime_limit,
        "disposal_center": {"x": float(dc_x), "y": float(dc_y),
                            "width": float(dc.width), "height": float(dc.height)},
        "start_area": list(the_map._start_area_drones),
        "default_drones": int(the_map.number_drones),
        "wall_pixels": int(walls.sum()),
        "boundary_pixels": int(boundary.sum()),
        "obstacle_render_pixels": int((obstacle_render > 0).sum()),
        **(extra or {}),
    }
    np.savez_compressed(
        out_dir / f"truth_{map_name}.npz",
        walls=walls.astype(np.uint8), boundary=boundary.astype(np.uint8),
        obstacle_render=(obstacle_render > 0).astype(np.uint8),
        starts=starts, info=np.array(json.dumps(info)))
    playground.cleanup()
    try:
        playground.close_window()
    except Exception:
        pass
    return info


def load_fixture(map_name: str, root: Path = DEFAULT_OUT) -> dict:
    """Load a fixture written by :func:`extract` (no GL needed).

    Phase 4 custom scenarios live in ``<root>/p4`` (written with
    ``--out _local/fixtures/p4``), apart from the official maps' fixtures.
    """
    if map_name not in MAPS and root == DEFAULT_OUT:
        root = DEFAULT_OUT / "p4"
    data = np.load(root / f"truth_{map_name}.npz", allow_pickle=False)
    info = json.loads(str(data["info"]))
    return {"walls": data["walls"], "boundary": data["boundary"],
            "obstacle_render": data["obstacle_render"],
            "starts": data["starts"], **info}


def main(argv=None) -> int:
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument("--out", default=str(DEFAULT_OUT))
    parser.add_argument("--maps", nargs="*", default=list(MAPS))
    args = parser.parse_args(argv)
    out_dir = Path(args.out)
    out_dir.mkdir(parents=True, exist_ok=True)
    summary = {}
    for name in args.maps:
        summary[name] = extract(name, out_dir)
        print(json.dumps(summary[name]))
    (out_dir / "fixtures.json").write_text(json.dumps(summary, indent=1) + "\n",
                                           encoding="utf-8")
    return 0


if __name__ == "__main__":
    sys.exit(main())
