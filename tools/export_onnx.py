#!/usr/bin/env python3
"""Export the bundled Go2 TorchScript policy to a fixed-shape ONNX model."""

from __future__ import annotations

import argparse
from pathlib import Path

import numpy as np
import torch


ROOT = Path(__file__).resolve().parents[1]


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--input", type=Path, default=ROOT / "models/go2_robot_lab/policy.pt")
    parser.add_argument("--output", type=Path, default=ROOT / "models/go2_robot_lab/policy.onnx")
    args = parser.parse_args()

    model = torch.jit.load(str(args.input), map_location="cpu").eval()
    example = torch.zeros(1, 45, dtype=torch.float32)
    example[0, 5] = -1.0
    args.output.parent.mkdir(parents=True, exist_ok=True)
    with torch.inference_mode():
        expected = model(example).detach().cpu().numpy()
        torch.onnx.export(
            model,
            example,
            str(args.output),
            export_params=True,
            opset_version=17,
            do_constant_folding=True,
            input_names=["observations"],
            output_names=["actions"],
            dynamo=False,
        )

    import onnx
    import onnxruntime as ort

    graph = onnx.load(str(args.output))
    onnx.checker.check_model(graph)
    session = ort.InferenceSession(str(args.output), providers=["CPUExecutionProvider"])
    actual = session.run(None, {session.get_inputs()[0].name: example.numpy()})[0]
    max_error = float(np.max(np.abs(expected - actual)))
    if max_error > 1e-5:
        raise RuntimeError(f"ONNX verification failed: max error {max_error}")
    print(f"exported: {args.output}")
    print(f"size: {args.output.stat().st_size} bytes")
    print(f"max_abs_error: {max_error:.9g}")


if __name__ == "__main__":
    main()
