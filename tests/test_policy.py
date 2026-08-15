#!/usr/bin/env python3

import sys
import unittest
from pathlib import Path

import numpy as np


ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "runtime"))

from go2_policy import (  # noqa: E402
    DEFAULT_DOF_POS,
    Go2Observation,
    OnnxBackend,
    TorchBackend,
    actions_to_joint_targets,
    projected_gravity,
)


class PolicyTest(unittest.TestCase):
    def test_nominal_observation(self):
        obs = Go2Observation().vector()
        self.assertEqual(obs.shape, (45,))
        np.testing.assert_allclose(obs[:3], 0.0)
        np.testing.assert_allclose(obs[3:6], [0.0, 0.0, -1.0], atol=1e-6)
        np.testing.assert_allclose(obs[9:21], 0.0)

    def test_projected_gravity_normalizes_quaternion(self):
        np.testing.assert_allclose(projected_gravity([2.0, 0.0, 0.0, 0.0]), [0.0, 0.0, -1.0])

    def test_projected_gravity_for_quarter_turn_roll(self):
        half_sqrt_two = np.sqrt(0.5)
        np.testing.assert_allclose(
            projected_gravity([half_sqrt_two, half_sqrt_two, 0.0, 0.0]),
            [0.0, -1.0, 0.0],
            atol=1e-6,
        )

    def test_action_postprocessing(self):
        np.testing.assert_allclose(actions_to_joint_targets(np.zeros(12)), DEFAULT_DOF_POS)
        targets = actions_to_joint_targets(np.full(12, 100.0))
        self.assertTrue(np.isfinite(targets).all())
        self.assertTrue(np.all(targets[:3] <= np.array([1.0472, 3.4907, -0.83776]) + 1e-6))

    def test_torch_and_onnx_match(self):
        try:
            import torch  # noqa: F401
        except ImportError:
            self.skipTest("PyTorch is optional in the lightweight CPU environment")
        pt_path = ROOT / "models/go2_robot_lab/policy.pt"
        onnx_path = ROOT / "models/go2_robot_lab/policy.onnx"
        if not onnx_path.exists():
            self.skipTest("ONNX model has not been exported")
        obs = Go2Observation(command=(0.3, -0.1, 0.2)).vector()
        pt_action = TorchBackend(pt_path).infer(obs)
        onnx_action = OnnxBackend(onnx_path).infer(obs)
        self.assertEqual(pt_action.shape, (12,))
        np.testing.assert_allclose(pt_action, onnx_action, rtol=1e-5, atol=1e-5)


if __name__ == "__main__":
    unittest.main()
