#!/usr/bin/env python3
"""Offline analysis of DynamicsProbe runs (no GL needed).

Usage::

    python -m devtools.simlab.analyze_probe <run_dir> [--compare <run_dir>]

Prints one JSON report with the measured dynamics, sensor noise, sensor
conventions, collision damage, message latency, placement rules and, with
``--compare``, a determinism check between two runs of the same seed.
"""

from __future__ import annotations

import argparse
import json
import math
import sys
from pathlib import Path

import cv2
import numpy as np

from .fixtures import load_fixture
from .probe import STEP_FIELDS

F = {name: i for i, name in enumerate(STEP_FIELDS)}


def wrap(a):
    return np.arctan2(np.sin(a), np.cos(a))


def load(run_dir: Path):
    data = np.load(run_dir / "probe.npz")
    extra = json.loads((run_dir / "probe_extra.json").read_text(encoding="utf-8"))
    return data["steps"], data["lidar"], extra


def drone_rows(steps: np.ndarray, ident: int) -> np.ndarray:
    rows = steps[steps[:, F["drone"]] == ident]
    return rows[np.argsort(rows[:, F["step"]])]


def ar1(x: np.ndarray) -> dict:
    x = x[np.isfinite(x)]
    if len(x) < 10:
        return {}
    x = x - x.mean()
    lag1 = float(np.dot(x[1:], x[:-1]) / np.dot(x, x))
    return {"std": round(float(x.std()), 3), "lag1_autocorr": round(lag1, 4), "n": int(len(x))}


def dynamics(steps: np.ndarray) -> dict:
    """Fit v[t+1] = a v[t] + b u[t] per axis in the body frame.

    The command issued at control step t is applied during the physics step
    that follows; the true velocity logged at step t+1 reflects it.
    """
    out = {}
    pairs = {"forward": [], "lateral": []}
    for ident in range(10):
        rows = drone_rows(steps, ident)
        if len(rows) < 3:
            continue
        ang = rows[:, F["ta"]]
        vx, vy = rows[:, F["tvx"]], rows[:, F["tvy"]]
        v_fwd = vx * np.cos(ang) + vy * np.sin(ang)
        v_lat = -vx * np.sin(ang) + vy * np.cos(ang)
        for name, v, u in (("forward", v_fwd, rows[:, F["c_fwd"]]),
                           ("lateral", v_lat, rows[:, F["c_lat"]])):
            # Use steps without rotation so the body frame is stable.
            rot = rows[:, F["c_rot"]]
            ok = (np.abs(rot[:-1]) < 1e-9) & (rows[1:, F["lidar_min"]] > 25)
            # Normalization: only rows where the command vector norm <= 1.
            norm = np.hypot(rows[:, F["c_fwd"]], rows[:, F["c_lat"]])
            ok &= norm[:-1] <= 1.0 + 1e-9
            for i in np.flatnonzero(ok):
                pairs[name].append((v[i], u[i], v[i + 1]))
    for name, rows in pairs.items():
        arr = np.array(rows)
        if len(arr) < 10:
            continue
        design = arr[:, :2]
        coef, *_ = np.linalg.lstsq(design, arr[:, 2], rcond=None)
        resid = arr[:, 2] - design @ coef
        out[name] = {"a_decay": round(float(coef[0]), 5),
                     "b_gain_px_per_step": round(float(coef[1]), 5),
                     "terminal_speed_at_1": round(float(coef[1] / (1 - coef[0])), 3),
                     "resid_std": round(float(resid.std()), 4), "n": int(len(arr))}
    # Lateral sign: velocity direction relative to heading while lateral=+1.
    rows = drone_rows(steps, 3)
    sel = (rows[:-1, F["c_lat"]] > 0.5)
    if np.any(sel):
        vel_dir = np.arctan2(rows[1:, F["tvy"]], rows[1:, F["tvx"]])[sel]
        rel = wrap(vel_dir - rows[1:, F["ta"]][sel])
        speed = np.hypot(rows[1:, F["tvx"]], rows[1:, F["tvy"]])[sel]
        out["lateral_plus_direction_deg"] = round(float(np.degrees(
            np.median(rel[speed > 1.0]))), 1) if np.any(speed > 1.0) else None
    # Rotation response.
    rows = drone_rows(steps, 2)
    dang = wrap(np.diff(rows[:, F["ta"]]))
    cmd = rows[:-1, F["c_rot"]]
    rot = {}
    for level in (1.0, -0.5, 0.25):
        sel = np.isclose(cmd, level)
        if np.any(sel):
            rot[str(level)] = {"dtheta_per_step_rad": round(float(np.median(dang[sel])), 5),
                               "n": int(sel.sum())}
    out["rotation"] = rot
    out["true_angular_velocity_at_cmd1"] = round(float(np.median(
        rows[1:, F["tw"]][np.isclose(cmd, 1.0)])), 5) if np.any(np.isclose(cmd, 1.0)) else None
    # Normalization check (drone 5): speed reached with forward=1, lateral=1.
    rows = drone_rows(steps, 5)
    sel = (rows[:-1, F["c_fwd"]] > 0.99) & (rows[:-1, F["c_lat"]] > 0.99)
    if np.any(sel):
        sp = np.hypot(rows[1:, F["tvx"]], rows[1:, F["tvy"]])[sel]
        out["diagonal_max_speed"] = round(float(sp.max()), 3)
    # Speed ladder (drone 7) and braking (drone 0).
    for ident, levels in ((7, (0.25, 0.75)), (1, (0.5,))):
        rows = drone_rows(steps, ident)
        sp = np.hypot(rows[:, F["tvx"]], rows[:, F["tvy"]])
        for level in levels:
            sel = np.isclose(rows[:-1, F["c_fwd"]], level)
            if np.any(sel):
                out[f"max_speed_cmd_{level}"] = round(float(sp[1:][sel].max()), 3)
    rows = drone_rows(steps, 0)
    sp = np.hypot(rows[:, F["tvx"]], rows[:, F["tvy"]])
    brake = np.flatnonzero(rows[:-1, F["c_fwd"]] < -0.5)
    if len(brake):
        i0 = brake[0]
        dist = float(np.sum(np.hypot(np.diff(rows[i0:brake[-1] + 2, F["tx"]]),
                                     np.diff(rows[i0:brake[-1] + 2, F["ty"]]))))
        out["braking"] = {"speed_at_start": round(float(sp[i0]), 3),
                          "steps": int(len(brake)), "distance_px": round(dist, 1)}
    return out


def sensor_noise(steps: np.ndarray) -> dict:
    out = {"gps_x": [], "gps_y": [], "compass": []}
    odo = {"d": [], "a": [], "t": []}
    for ident in range(10):
        rows = drone_rows(steps, ident)
        out["gps_x"].append(ar1(rows[:, F["gx"]] - rows[:, F["tx"]]))
        out["gps_y"].append(ar1(rows[:, F["gy"]] - rows[:, F["ty"]]))
        out["compass"].append(ar1(np.degrees(wrap(rows[:, F["compass"]] - rows[:, F["ta"]]))))
        dx = np.diff(rows[:, F["tx"]])
        dy = np.diff(rows[:, F["ty"]])
        d_true = np.hypot(dx, dy)
        a_true = wrap(np.arctan2(dy, dx) - rows[:-1, F["ta"]])
        t_true = wrap(np.diff(rows[:, F["ta"]]))
        odo["d"].extend((rows[1:, F["odo_d"]] - d_true).tolist())
        moving = d_true > 2.0
        odo["a"].extend(np.degrees(wrap(rows[1:, F["odo_a"]] - a_true))[moving].tolist())
        odo["t"].extend(np.degrees(wrap(rows[1:, F["odo_t"]] - t_true)).tolist())
    summary = {}
    for key in ("gps_x", "gps_y", "compass"):
        stds = [r["std"] for r in out[key] if r]
        lags = [r["lag1_autocorr"] for r in out[key] if r]
        summary[key] = {"std_median": round(float(np.median(stds)), 3),
                        "lag1_median": round(float(np.median(lags)), 4)}
    for key, values in odo.items():
        arr = np.array(values)
        arr = arr[np.isfinite(arr)]
        summary[f"odometer_{key}_error"] = {"mean": round(float(arr.mean()), 4),
                                            "std": round(float(arr.std()), 4),
                                            "n": int(len(arr))}
    return summary


def lidar_conventions(lidar: np.ndarray, walls: np.ndarray, steps: np.ndarray) -> dict:
    h, w = walls.shape
    dist_to_wall = cv2.distanceTransform((1 - walls).astype(np.uint8), cv2.DIST_L2,
                                         cv2.DIST_MASK_PRECISE)
    angles = np.linspace(-math.pi, math.pi, 181)
    out = {}
    values = lidar[:, 5:]
    for sign_name, sign in (("ccw_positive", 1.0), ("cw_positive", -1.0)):
        errs = []
        for row, vals in zip(lidar, values):
            x, y, a = row[2], row[3], row[4]
            hit = vals < 290
            wa = a + sign * angles[hit]
            px = x + vals[hit] * np.cos(wa)
            py = y + vals[hit] * np.sin(wa)
            col = np.round(px + w / 2).astype(int)
            r = np.round(h / 2 - py).astype(int)
            inside = (col >= 0) & (col < w) & (r >= 0) & (r < h)
            errs.extend(dist_to_wall[r[inside], col[inside]].tolist())
        errs = np.array(errs)
        out[sign_name] = {"median_px": round(float(np.median(errs)), 2),
                          "p90_px": round(float(np.percentile(errs, 90)), 2),
                          "frac_within_5px": round(float(np.mean(errs <= 5)), 3)}
    nohit = values[values > 280]
    out["no_hit_value"] = {"mean": round(float(nohit.mean()), 2),
                           "std": round(float(nohit.std()), 3), "n": int(nohit.size)}
    # Range noise on the static drone (4): per-ray std over time.
    static = lidar[lidar[:, 1] == 4][:, 5:]
    if len(static) > 10:
        per_ray = static.std(axis=0)
        out["static_drone_per_ray_std_median"] = round(float(np.median(per_ray)), 3)
    # Do rays hit other drones? Count hits far from walls but near a drone.
    near_drone = 0
    far_from_wall = 0
    for row, vals in zip(lidar, values):
        x, y, a = row[2], row[3], row[4]
        hit = vals < 290
        wa = a + angles[hit]
        px = x + vals[hit] * np.cos(wa)
        py = y + vals[hit] * np.sin(wa)
        col = np.clip(np.round(px + w / 2).astype(int), 0, w - 1)
        r = np.clip(np.round(h / 2 - py).astype(int), 0, h - 1)
        far = dist_to_wall[r, col] > 8
        if not np.any(far):
            continue
        step = row[0]
        others = steps[(steps[:, F["step"]] == step) & (steps[:, F["drone"]] != row[1])]
        for qx, qy in zip(px[far], py[far]):
            far_from_wall += 1
            if len(others) and np.min(np.hypot(others[:, F["tx"]] - qx,
                                               others[:, F["ty"]] - qy)) < 25:
                near_drone += 1
    out["hits_far_from_walls"] = far_from_wall
    out["of_which_within_25px_of_a_drone"] = near_drone
    return out


def semantic_conventions(extra: dict, fixture: dict, steps: np.ndarray) -> dict:
    dc = fixture["disposal_center"]
    x0, x1 = dc["x"] - dc["width"] / 2, dc["x"] + dc["width"] / 2
    y0, y1 = dc["y"] - dc["height"] / 2, dc["y"] + dc["height"] / 2

    def dist_to_box_boundary(px, py):
        dx = max(x0 - px, 0, px - x1)
        dy = max(y0 - py, 0, py - y1)
        outside = math.hypot(dx, dy)
        if outside > 0:
            return outside
        return min(px - x0, x1 - px, py - y0, y1 - py)

    res = {"dc": {"ccw_positive": [], "cw_positive": []}, "drone_dist_err": [],
           "types": {}}
    for entry in extra["semantic"]:
        for d, ang, kind, grasped in entry["hits"]:
            res["types"][kind] = res["types"].get(kind, 0) + 1
            if kind == "DISPOSAL_CENTER":
                for name, sign in (("ccw_positive", 1), ("cw_positive", -1)):
                    wa = entry["ta"] + sign * ang
                    px = entry["tx"] + d * math.cos(wa)
                    py = entry["ty"] + d * math.sin(wa)
                    res["dc"][name].append(dist_to_box_boundary(px, py))
            if kind == "DRONE":
                wa = entry["ta"] + ang
                px = entry["tx"] + d * math.cos(wa)
                py = entry["ty"] + d * math.sin(wa)
                others = steps[(steps[:, F["step"]] == entry["step"]) &
                               (steps[:, F["drone"]] != entry["drone"])]
                if len(others):
                    dist_c = np.hypot(others[:, F["tx"]] - entry["tx"],
                                      others[:, F["ty"]] - entry["ty"])
                    j = int(np.argmin(np.hypot(others[:, F["tx"]] - px,
                                               others[:, F["ty"]] - py)))
                    res["drone_dist_err"].append(d - dist_c[j])
    out = {"types": res["types"]}
    for name, vals in res["dc"].items():
        if vals:
            arr = np.array(vals)
            out[f"dc_boundary_err_{name}"] = {"median": round(float(np.median(arr)), 2),
                                             "p90": round(float(np.percentile(arr, 90)), 2),
                                             "n": len(arr)}
    if res["drone_dist_err"]:
        arr = np.array(res["drone_dist_err"])
        out["drone_semantic_distance_minus_center_distance"] = {
            "median": round(float(np.median(arr)), 2),
            "p10": round(float(np.percentile(arr, 10)), 2),
            "p90": round(float(np.percentile(arr, 90)), 2), "n": len(arr)}
    return out


def collisions(steps: np.ndarray) -> dict:
    out = {}
    for ident in range(10):
        rows = drone_rows(steps, ident)
        health = rows[:, F["health"]]
        drops = np.flatnonzero(np.diff(health) < 0) + 1
        if len(drops):
            out[str(ident)] = {"health_lost": int(health[0] - health[-1]),
                               "drop_steps": rows[drops, F["step"]].astype(int).tolist(),
                               "lidar_min_at_drops": np.round(rows[drops, F["lidar_min"]], 1).tolist()}
    rows = drone_rows(steps, 6)
    out["wall_pusher_min_lidar"] = round(float(np.nanmin(rows[:, F["lidar_min"]])), 2)
    return out


def messages(extra: dict) -> dict:
    msgs = np.array(extra["messages"], dtype=float)
    if not len(msgs):
        return {}
    latency = msgs[:, 0] - msgs[:, 3]
    values, counts = np.unique(latency, return_counts=True)
    return {"latency_steps_histogram": {str(int(v)): int(c) for v, c in zip(values, counts)},
            "max_distance_received": round(float(msgs[:, 4].max()), 1),
            "n": int(len(msgs))}


def placement(steps: np.ndarray, extra: dict) -> dict:
    out = {"events": [e for e in extra["events"] if e["drone"] in (8, 9)]}
    for ident in (8, 9):
        rows = drone_rows(steps, ident)
        inv = rows[:, F["inventory"]]
        changes = np.flatnonzero(np.diff(inv) != 0) + 1
        out[f"inventory_changes_{ident}"] = [(int(rows[i, F["step"]]), int(inv[i]))
                                             for i in changes]
        grasped = rows[:, F["grasped"]]
        if np.any(grasped > 0):
            sel = grasped[:-1] > 0
            sp = np.hypot(rows[1:, F["tvx"]], rows[1:, F["tvy"]])[sel]
            out[f"max_speed_while_grasping_{ident}"] = round(float(sp.max()), 3)
            out[f"grasp_steps_{ident}"] = [int(rows[np.flatnonzero(grasped > 0)[0], F["step"]]),
                                           int(rows[np.flatnonzero(grasped > 0)[-1], F["step"]])]
    return out


def determinism(a: Path, b: Path) -> dict:
    sa, la, _ = load(a)
    sb, lb, _ = load(b)
    out = {"shapes_equal": sa.shape == sb.shape and la.shape == lb.shape}
    if out["shapes_equal"]:
        diff = np.abs(np.nan_to_num(sa) - np.nan_to_num(sb))
        out["max_abs_diff_steps"] = float(diff.max())
        cols = {name: float(diff[:, i].max()) for name, i in F.items()}
        out["max_abs_diff_by_field"] = {k: v for k, v in cols.items() if v > 0}
        out["max_abs_diff_lidar"] = float(np.abs(la - lb).max())
        first = np.flatnonzero(diff.max(axis=1) > 0)
        out["first_differing_row"] = int(first[0]) if len(first) else None
    return out


def main(argv=None) -> int:
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument("run_dir")
    parser.add_argument("--compare", default=None)
    parser.add_argument("--map", default="Map01")
    args = parser.parse_args(argv)
    run = Path(args.run_dir)
    steps, lidar, extra = load(run)
    fixture = load_fixture(args.map)
    report = {
        "run": run.name,
        "dynamics": dynamics(steps),
        "sensor_noise": sensor_noise(steps),
        "lidar": lidar_conventions(lidar, fixture["walls"], steps),
        "semantic": semantic_conventions(extra, fixture, steps),
        "collisions": collisions(steps),
        "messages": messages(extra),
        "placement": placement(steps, extra),
    }
    if args.compare:
        report["determinism"] = determinism(run, Path(args.compare))
    print(json.dumps(report, indent=1, default=float))
    return 0


if __name__ == "__main__":
    sys.exit(main())
