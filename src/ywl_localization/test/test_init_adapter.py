#!/usr/bin/env python3
import importlib.machinery
import importlib.util
from pathlib import Path
import time
import unittest

import numpy as np

path = Path(__file__).resolve().parents[1] / "scripts/ywl_init_adapter"
loader = importlib.machinery.SourceFileLoader("ywl_init_adapter", str(path))
spec = importlib.util.spec_from_loader(loader.name, loader)
adapter = importlib.util.module_from_spec(spec)
loader.exec_module(adapter)


class InitContract(unittest.TestCase):
    def test_reference_and_map_inverse(self):
        local = adapter.pose([-1, 2, 0.4], [0.0, 0.0, 0.0, 1.0])
        q = np.array([0.02, -0.03, np.sqrt(0.5), np.sqrt(0.5)])
        expected = adapter.pose([5, 3, 2], q / np.linalg.norm(q))
        mapped = expected @ local
        got = mapped @ np.linalg.inv(local)
        np.testing.assert_allclose(got, expected, atol=1e-12)
        point = np.array([3., -2., 4., 1])
        np.testing.assert_allclose(got @ (np.linalg.inv(got) @ point), point, atol=1e-12)
        np.testing.assert_allclose(got[:3, :3] @ [1, 0, 0], expected[:3, 0])

    def test_bounded_interpolation_no_extrapolation(self):
        a = adapter.pose([0, 0, 0], [0, 0, 0, 1])
        b = adapter.pose([2, 0, 0], [0, 0, np.sqrt(0.5), np.sqrt(0.5)])
        history = [(100_000_000, a, 0), (200_000_000, b, 0)]
        p = adapter.at_stamp(history, 150_000_000)
        np.testing.assert_allclose(p[:3, 3], [1, 0, 0])
        np.testing.assert_allclose(p[:3, 0], [np.sqrt(0.5), np.sqrt(0.5), 0])
        self.assertIsNone(adapter.at_stamp(history, 200_000_001))
        self.assertIsNone(adapter.at_stamp(history, 99_000_000))
        self.assertIsNone(adapter.at_stamp([(0, a, 0), (200_000_000, b, 0)], 100_000_000))

    def test_stationary_requires_history_and_fresh_input(self):
        now = time.monotonic()
        settings = dict(still_m=.03, still_deg=2)
        still = [(i * 100_000_000, np.eye(4), now) for i in range(12)]
        anchor = still[-1][0]
        self.assertTrue(adapter.stationary(still, anchor, settings))
        self.assertFalse(adapter.stationary(still[-5:], anchor, settings))
        self.assertFalse(adapter.stationary([(n, t, now - .6) for n, t, _ in still], anchor, settings))
        still[-3][1][0, 3] = .031
        self.assertFalse(adapter.stationary(still, anchor, settings))

    def test_relocalizer_confirmation(self):
        config = {"confirmations": 2}
        result = dict(schema_version=2, confirmed=True, ok=True, stamp=100.0)
        status = dict(schema_version=2, state="LOCALIZED", valid=True, confirmations=2, stamp=100.1)
        self.assertTrue(adapter.confirmed(result, status, config))
        for changed in (dict(confirmations=1), dict(valid=False), dict(stamp=101), dict(stamp=99)):
            self.assertFalse(adapter.confirmed(result, dict(status, **changed), config))
        self.assertFalse(adapter.confirmed(dict(result, confirmed=False), status, config))


if __name__ == "__main__":
    unittest.main()
