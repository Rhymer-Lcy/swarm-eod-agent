"""Thread-setting independence of the evaluation tooling's distance transform."""

from __future__ import annotations

import sys
from pathlib import Path

import cv2
import numpy as np

REPO = Path(__file__).resolve().parents[2]
if str(REPO) not in sys.path:
    sys.path.insert(0, str(REPO))

from devtools.simlab.stable_cv import distance_transform  # noqa: E402


def _src():
    rng = np.random.default_rng(5)
    return 1 - (rng.random((750, 1100)) < 0.01).astype(np.uint8)


def _raw(src, threads):
    cv2.setNumThreads(threads)
    return cv2.distanceTransform(src, cv2.DIST_L2, cv2.DIST_MASK_PRECISE)


def test_raw_single_threaded_precise_transform_is_not_repeatable():
    """The effect the helper exists for (Phase 4, Risk C). If a future OpenCV
    removes it, this test says so instead of the helper silently mattering."""
    src = _src()
    before = cv2.getNumThreads()
    try:
        first, second = _raw(src, 1), cv2.distanceTransform(src, cv2.DIST_L2,
                                                             cv2.DIST_MASK_PRECISE)
        many_a, many_b = _raw(src, 8), _raw(src, 8)
        two = _raw(src, 2)
    finally:
        cv2.setNumThreads(before)
    assert not np.array_equal(first, second)               # one thread: not repeatable
    assert np.array_equal(many_a, many_b) and np.array_equal(many_a, two)
    assert not np.array_equal(first, many_a)
    assert float(np.max(np.abs(first.astype(np.float64) - many_a))) < 1e-3


def test_helper_is_identical_for_any_thread_setting_and_restores_it():
    src = _src()
    before = cv2.getNumThreads()
    try:
        results = []
        for n in (1, 1, 2, 8, 1):
            cv2.setNumThreads(n)
            results.append(distance_transform(src))
            assert cv2.getNumThreads() == n
        assert all(np.array_equal(results[0], r) for r in results[1:])
        assert np.array_equal(results[0], _raw(src, 8))
    finally:
        cv2.setNumThreads(before)
