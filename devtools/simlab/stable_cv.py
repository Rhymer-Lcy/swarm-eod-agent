"""OpenCV helpers whose results do not depend on OpenCV's thread setting.

Measured in Phase 4 (attempts P4-A009 to P4-A012, docs/EXPERIMENTS.md
section 9), OpenCV 5.0.0 with the pthreads backend:
``cv2.distanceTransform`` with ``DIST_MASK_PRECISE`` gives one result with
two or more threads (2, 8 and 32 threads agree, and repeated calls agree)
and a different one with a single thread, where it is not even repeatable:
a second call on the same input in the same process differs from the
first, by up to 1.5e-5 px. In the offline simulator that difference changes
a contact decision now and then and the run diverges: R1 on Map05 seed 1
scored 56.48 with two or more OpenCV threads and 54.74 with one, which is
the unexplained Phase 3 difference between attempts P3-A008 and P3-A010.
OpenCV takes its thread count from the CPU affinity unless
``OPENCV_FOR_THREADS_NUM`` is set (it ignores ``OPENCV_NUM_THREADS``), so the
result depended on how a job was pinned.

The helper therefore runs the transform with two threads, whatever the
caller's setting or affinity. Evaluation tooling uses it; agent code does not
call ``distanceTransform`` (its OpenCV operations were bit-identical across
thread counts in the same measurements).
"""

from __future__ import annotations

import cv2
import numpy as np

THREADS = 2


def distance_transform(src: np.ndarray, distance_type: int = cv2.DIST_L2,
                       mask_size: int = cv2.DIST_MASK_PRECISE) -> np.ndarray:
    """``cv2.distanceTransform`` computed with ``THREADS`` OpenCV threads."""
    previous = cv2.getNumThreads()
    cv2.setNumThreads(THREADS)
    try:
        return cv2.distanceTransform(src, distance_type, mask_size)
    finally:
        cv2.setNumThreads(previous)
