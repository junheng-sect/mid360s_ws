#!/usr/bin/env python3
import importlib.machinery
import importlib.util
import json
from pathlib import Path
from tempfile import TemporaryDirectory
import time
import unittest
from unittest.mock import patch

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

    def test_crop_checks_both_map_identities(self):
        with TemporaryDirectory() as folder:
            source = Path(folder) / "original"
            prior = Path(folder) / "crop"
            for path in (source, prior):
                path.mkdir()
                for name in adapter.MAP_FILES:
                    (path / name).write_text(str(path))
            identity = adapter.map_identity(source)
            crop_identity = adapter.map_identity(prior)
            (prior / "manifest.json").write_text(json.dumps({
                "map_frame": "drone_map", "map_dir": str(prior),
                "source": {"map_dir": str(source), "map_sha256": identity["map_world.pcd"],
                           "meta_sha256": identity["meta.json"]},
                "sha256": crop_identity,
            }))
            self.assertEqual(adapter.verify_maps(prior, source)[0], crop_identity)
            (source / "meta.json").write_text("different source metadata")
            with self.assertRaisesRegex(ValueError, "does not match"):
                adapter.verify_maps(prior, source)

    def test_live_result_rejects_old_sessions_and_mismatched_tf(self):
        node = adapter.Adapter.__new__(adapter.Adapter)
        ns = adapter.NS
        now_wall = time.monotonic()
        publisher = "/drone_reloc"
        node.started_ns = node.gravity_stamp = 100 * ns
        node.first_bootstrap_stamp = 101 * ns
        node.world_from_camera = np.eye(4)
        node.settings = dict(confirmations=2, still_m=.03, still_deg=2)
        node.capture = False
        node.source_dir = Path(__file__).resolve().parent
        node.base = node.ywl = [(ns * 103 + i * 100_000_000, np.eye(4), now_wall)
                                for i in range(11)]
        now = adapter.rospy.Time(104)
        node.status = (dict(schema_version=2, state="LOCALIZED", valid=True,
                            confirmations=2, stamp=104.0), now_wall, publisher)
        result = dict(schema_version=2, confirmed=True, ok=True, stamp=103.8,
                      map=str(node.source_dir), transform=adapter.encode_pose(np.eye(4)))
        node.result = (result, now_wall, publisher)
        good_tf = (np.eye(4), 104 * ns, now_wall, publisher)
        with patch.object(adapter.rospy.Time, "now", return_value=now):
            node.map_tf = good_tf
            self.assertEqual(node.live_target()[0], 104 * ns)
            node.map_tf = (*good_tf[:-1], "/different_relocalizer")
            self.assertIsNone(node.live_target())
            wrong_tf = np.eye(4)
            wrong_tf[0, 3] = .1
            node.map_tf = (wrong_tf, *good_tf[1:])
            self.assertIsNone(node.live_target())
            node.map_tf = good_tf
            node.result = (dict(result, stamp=99.0), now_wall, publisher)
            self.assertIsNone(node.live_target())


if __name__ == "__main__":
    unittest.main()
