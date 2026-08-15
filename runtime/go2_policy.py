#!/usr/bin/env python3
"""CPU inference helpers for the RobotLab Unitree Go2 locomotion policy."""

from __future__ import annotations

import argparse
import json
import math
import statistics
import time
from dataclasses import dataclass
from pathlib import Path
from typing import Protocol, Sequence

import numpy as np


ROOT = Path(__file__).resolve().parents[1]
DEFAULT_PT_MODEL = ROOT / "models" / "go2_robot_lab" / "policy.pt"
DEFAULT_ONNX_MODEL = ROOT / "models" / "go2_robot_lab" / "policy.onnx"

JOINT_NAMES = (
    "FR_hip_joint", "FR_thigh_joint", "FR_calf_joint",
    "FL_hip_joint", "FL_thigh_joint", "FL_calf_joint",
    "RR_hip_joint", "RR_thigh_joint", "RR_calf_joint",
    "RL_hip_joint", "RL_thigh_joint", "RL_calf_joint",
)
DEFAULT_DOF_POS = np.asarray([0.0, 0.8, -1.5] * 4, dtype=np.float32)
ACTION_SCALE = np.asarray([0.125, 0.25, 0.25] * 4, dtype=np.float32)
JOINT_LOWER = np.asarray(
    [-1.0472, -1.5708, -2.7227] * 2 + [-1.0472, -0.5236, -2.7227] * 2,
    dtype=np.float32,
)
JOINT_UPPER = np.asarray(
    [1.0472, 3.4907, -0.83776] * 2 + [1.0472, 4.5379, -0.83776] * 2,
    dtype=np.float32,
)


def _array(name: str, value: Sequence[float], size: int) -> np.ndarray:
    result = np.asarray(value, dtype=np.float32)
    if result.shape != (size,):
        raise ValueError(f"{name} must have shape ({size},), got {result.shape}")
    if not np.isfinite(result).all():
        raise ValueError(f"{name} contains NaN or infinity")
    return result


def projected_gravity(quaternion_wxyz: Sequence[float]) -> np.ndarray:
    """Rotate world gravity [0, 0, -1] into the Go2 body frame."""
    q = _array("quaternion_wxyz", quaternion_wxyz, 4).astype(np.float64)
    norm = float(np.linalg.norm(q))
    if norm < 1e-8:
        raise ValueError("quaternion_wxyz has zero norm")
    w, x, y, z = q / norm
    return np.asarray(
        [
            2.0 * (w * y - x * z),
            -2.0 * (w * x + y * z),
            -(1.0 - 2.0 * (x * x + y * y)),
        ],
        dtype=np.float32,
    )


@dataclass(frozen=True)
class Go2Observation:
    angular_velocity: Sequence[float] = (0.0, 0.0, 0.0)
    quaternion_wxyz: Sequence[float] = (1.0, 0.0, 0.0, 0.0)
    command: Sequence[float] = (0.0, 0.0, 0.0)
    joint_position: Sequence[float] = tuple(DEFAULT_DOF_POS)
    joint_velocity: Sequence[float] = (0.0,) * 12
    previous_action: Sequence[float] = (0.0,) * 12

    def vector(self) -> np.ndarray:
        """Build RobotLab's exact 45-float actor input in training joint order."""
        parts = (
            _array("angular_velocity", self.angular_velocity, 3) * np.float32(0.25),
            projected_gravity(self.quaternion_wxyz),
            _array("command", self.command, 3),
            _array("joint_position", self.joint_position, 12) - DEFAULT_DOF_POS,
            _array("joint_velocity", self.joint_velocity, 12) * np.float32(0.05),
            _array("previous_action", self.previous_action, 12),
        )
        return np.clip(np.concatenate(parts), -100.0, 100.0).astype(np.float32)


def actions_to_joint_targets(actions: Sequence[float], *, enforce_limits: bool = True) -> np.ndarray:
    """Convert 12 raw policy actions to SDK-order joint position targets."""
    targets = DEFAULT_DOF_POS + _array("actions", actions, 12) * ACTION_SCALE
    if enforce_limits:
        targets = np.clip(targets, JOINT_LOWER, JOINT_UPPER)
    return targets.astype(np.float32)


class Backend(Protocol):
    model_path: Path

    def infer(self, observation: np.ndarray) -> np.ndarray: ...


class TorchBackend:
    def __init__(self, model_path: Path, threads: int = 1) -> None:
        import torch

        self.torch = torch
        self.model_path = model_path
        torch.set_num_threads(threads)
        torch.set_num_interop_threads(1)
        self.model = torch.jit.load(str(model_path), map_location="cpu").eval()

    def infer(self, observation: np.ndarray) -> np.ndarray:
        tensor = self.torch.from_numpy(observation.reshape(1, -1))
        with self.torch.inference_mode():
            output = self.model(tensor)
        return output.detach().cpu().numpy().reshape(-1).astype(np.float32)


class OnnxBackend:
    def __init__(self, model_path: Path, threads: int = 1) -> None:
        import onnxruntime as ort

        options = ort.SessionOptions()
        options.intra_op_num_threads = threads
        options.inter_op_num_threads = 1
        options.graph_optimization_level = ort.GraphOptimizationLevel.ORT_ENABLE_ALL
        self.model_path = model_path
        self.session = ort.InferenceSession(
            str(model_path), sess_options=options, providers=["CPUExecutionProvider"]
        )
        self.input_name = self.session.get_inputs()[0].name

    def infer(self, observation: np.ndarray) -> np.ndarray:
        result = self.session.run(None, {self.input_name: observation.reshape(1, -1)})[0]
        return np.asarray(result, dtype=np.float32).reshape(-1)


def load_backend(name: str, model_path: Path | None, threads: int) -> Backend:
    if name == "auto":
        name = "onnx" if DEFAULT_ONNX_MODEL.exists() else "torch"
    if model_path is None:
        model_path = DEFAULT_ONNX_MODEL if name == "onnx" else DEFAULT_PT_MODEL
    if not model_path.is_file():
        raise FileNotFoundError(f"model not found: {model_path}")
    if name == "onnx":
        return OnnxBackend(model_path, threads)
    if name == "torch":
        return TorchBackend(model_path, threads)
    raise ValueError(f"unsupported backend: {name}")


def benchmark(backend: Backend, observation: np.ndarray, warmup: int, iterations: int) -> dict[str, float]:
    for _ in range(warmup):
        backend.infer(observation)
    samples_ms: list[float] = []
    for _ in range(iterations):
        start = time.perf_counter_ns()
        backend.infer(observation)
        samples_ms.append((time.perf_counter_ns() - start) / 1_000_000.0)
    ordered = sorted(samples_ms)
    p95_index = min(len(ordered) - 1, math.ceil(0.95 * len(ordered)) - 1)
    return {
        "mean_ms": statistics.fmean(samples_ms),
        "median_ms": statistics.median(samples_ms),
        "p95_ms": ordered[p95_index],
        "min_ms": ordered[0],
        "max_ms": ordered[-1],
        "hz": 1000.0 / statistics.fmean(samples_ms),
    }


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--backend", choices=("auto", "onnx", "torch"), default="auto")
    parser.add_argument("--model", type=Path)
    parser.add_argument("--threads", type=int, default=1)
    parser.add_argument("--warmup", type=int, default=100)
    parser.add_argument("--iterations", type=int, default=2000)
    parser.add_argument("--command", type=float, nargs=3, metavar=("VX", "VY", "YAW"), default=(0, 0, 0))
    parser.add_argument("--json", action="store_true")
    args = parser.parse_args()
    if args.threads < 1 or args.warmup < 0 or args.iterations < 1:
        parser.error("threads and iterations must be positive; warmup cannot be negative")

    backend = load_backend(args.backend, args.model, args.threads)
    observation = Go2Observation(command=args.command).vector()
    action = backend.infer(observation)
    if action.shape != (12,) or not np.isfinite(action).all():
        raise RuntimeError(f"invalid model output: shape={action.shape}, finite={np.isfinite(action).all()}")
    result = {
        "backend": backend.__class__.__name__,
        "model": str(backend.model_path),
        "model_size_bytes": backend.model_path.stat().st_size,
        "input_shape": [1, 45],
        "output_shape": [1, 12],
        "action": action.tolist(),
        "joint_targets": actions_to_joint_targets(action).tolist(),
        "benchmark": benchmark(backend, observation, args.warmup, args.iterations),
    }
    if args.json:
        print(json.dumps(result, indent=2))
        return
    print(f"backend: {result['backend']}")
    print(f"model: {result['model']} ({result['model_size_bytes'] / 1024:.1f} KiB)")
    print(f"input/output: {result['input_shape']} -> {result['output_shape']}")
    print("actions:", np.array2string(action, precision=5))
    print("joint targets:", np.array2string(actions_to_joint_targets(action), precision=5))
    print("latency:", json.dumps(result["benchmark"], indent=2))


if __name__ == "__main__":
    main()
