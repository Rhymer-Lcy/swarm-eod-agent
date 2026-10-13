#!/usr/bin/env python3
"""Occupancy gate for runs on a shared Linux host.

Prints one JSON object describing GPU and CPU occupancy and exits 1 when the
host is not free for a run under the rules in docs/ENVIRONMENT.md:

* every GPU must have no compute process and near-zero memory in use (an
  EGL graphics context does not appear among compute processes, but it does
  occupy memory, so memory is checked on every GPU, not only the target);
* the CPUs this run will be pinned to must be essentially idle and all on
  NUMA node 0 (node 1 is never used by this project).

``--cpu-only`` (CPU-only Python work such as tests or the offline simulator)
skips the GPU requirement; GPU state is still reported.

``--report-only`` always exits 0 (used after a run, for the record). The
check never inspects or touches other users' processes beyond reading
``/proc/stat`` and ``nvidia-smi`` summaries.
"""

from __future__ import annotations

import argparse
import json
import os
import subprocess
import sys
import time

GPU_MEMORY_BUSY_MIB = 50
GPU_UTIL_BUSY_PCT = 5
CPU_BUSY_FRACTION = 0.25


def parse_cpu_list(text: str) -> list:
    cpus = []
    for part in text.split(","):
        part = part.strip()
        if "-" in part:
            lo, hi = part.split("-")
            cpus.extend(range(int(lo), int(hi) + 1))
        elif part:
            cpus.append(int(part))
    return cpus


def _cpu_times() -> dict:
    times = {}
    with open("/proc/stat", encoding="utf-8") as f:
        for line in f:
            if line.startswith("cpu") and line[3].isdigit():
                fields = line.split()
                values = [int(v) for v in fields[1:]]
                idle = values[3] + values[4]
                times[int(fields[0][3:])] = (sum(values), idle)
    return times


def cpu_busy(cpus: list, interval: float = 1.0) -> dict:
    first = _cpu_times()
    time.sleep(interval)
    second = _cpu_times()
    busy = {}
    for cpu in cpus:
        if cpu in first and cpu in second:
            total = second[cpu][0] - first[cpu][0]
            idle = second[cpu][1] - first[cpu][1]
            busy[cpu] = 0.0 if total <= 0 else 1.0 - idle / total
    return busy


def gpu_state() -> dict:
    gpus = subprocess.run(
        ["nvidia-smi", "--query-gpu=index,uuid,utilization.gpu,memory.used",
         "--format=csv,noheader,nounits"],
        check=True, capture_output=True, text=True).stdout.strip().splitlines()
    apps = subprocess.run(
        ["nvidia-smi", "--query-compute-apps=gpu_uuid,pid,used_memory",
         "--format=csv,noheader"],
        check=True, capture_output=True, text=True).stdout.strip().splitlines()
    rows = []
    for line in gpus:
        index, uuid, util, mem = [p.strip() for p in line.split(",")]
        rows.append({"index": int(index), "uuid": uuid, "util_pct": int(util),
                     "memory_mib": int(mem)})
    return {"gpus": rows, "compute_apps": [a for a in apps if a.strip()]}


def node0_cpus() -> list:
    """CPUs of NUMA node 0 as reported by the kernel (empty if unknown)."""
    try:
        with open("/sys/devices/system/node/node0/cpulist", encoding="utf-8") as f:
            return parse_cpu_list(f.read().strip())
    except OSError:
        return []


def main(argv=None) -> int:
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument("--cpus", required=True, help="CPU list this run uses")
    target = parser.add_mutually_exclusive_group(required=True)
    target.add_argument("--egl-device", type=int)
    target.add_argument("--cpu-only", action="store_true",
                        help="CPU-only work: GPUs are reported but not required to be free")
    parser.add_argument("--report-only", action="store_true")
    args = parser.parse_args(argv)

    cpus = parse_cpu_list(args.cpus)
    gpu = gpu_state()
    busy = cpu_busy(cpus)
    mean_busy = sum(busy.values()) / max(len(busy), 1)
    reasons = []
    node0 = node0_cpus()
    if not node0 or not set(cpus) <= set(node0):
        reasons.append("pinned CPUs are not all on NUMA node 0 (or node 0 is unknown)")
    if not args.cpu_only:
        if gpu["compute_apps"]:
            reasons.append("compute processes present on a GPU")
        for row in gpu["gpus"]:
            if row["memory_mib"] > GPU_MEMORY_BUSY_MIB or row["util_pct"] > GPU_UTIL_BUSY_PCT:
                reasons.append(f"GPU {row['index']} in use "
                               f"({row['memory_mib']} MiB, {row['util_pct']}%)")
    if mean_busy > CPU_BUSY_FRACTION:
        reasons.append(f"pinned CPUs {mean_busy:.0%} busy")
    report = {
        "time_unix": round(time.time(), 1),
        "loadavg": os.getloadavg(),
        "cpus": args.cpus,
        "cpu_mean_busy": round(mean_busy, 3),
        "cpu_max_busy": round(max(busy.values()) if busy else 0.0, 3),
        "egl_device": args.egl_device,
        "cpu_only": bool(args.cpu_only),
        "gpu": gpu,
        "free": not reasons,
        "reasons": reasons,
    }
    print(json.dumps(report))
    if args.report_only:
        return 0
    return 0 if not reasons else 1


if __name__ == "__main__":
    sys.exit(main())
