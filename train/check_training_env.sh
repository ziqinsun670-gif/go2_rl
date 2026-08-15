#!/usr/bin/env bash
set -euo pipefail

script_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
project_dir="$(cd "${script_dir}/.." && pwd)"
robot_lab_dir="${ROBOT_LAB_DIR:-${project_dir}/upstream/robot_lab}"
python_bin="${ISAACLAB_PYTHON:-python3}"

if [[ ! -f "${robot_lab_dir}/scripts/reinforcement_learning/rsl_rl/train.py" ]]; then
    echo "ERROR: RobotLab checkout not found: ${robot_lab_dir}" >&2
    exit 2
fi

if ! command -v "${python_bin}" >/dev/null 2>&1; then
    echo "ERROR: Python interpreter not found: ${python_bin}" >&2
    echo "Set ISAACLAB_PYTHON to the interpreter with Isaac Lab installed." >&2
    exit 2
fi

expected_commit="500399ed75f510aeaff28705a8ce736c514dbec3"
actual_commit="$(git -C "${robot_lab_dir}" rev-parse HEAD 2>/dev/null || true)"
if [[ "${actual_commit}" != "${expected_commit}" ]]; then
    echo "ERROR: RobotLab commit mismatch." >&2
    echo "  expected: ${expected_commit}" >&2
    echo "  actual:   ${actual_commit:-not a Git checkout}" >&2
    exit 3
fi

export PYTHONPATH="${robot_lab_dir}/source/robot_lab${PYTHONPATH:+:${PYTHONPATH}}"
"${python_bin}" - <<'PY'
import sys

try:
    import torch
    import isaaclab  # noqa: F401
    import isaaclab_rl  # noqa: F401
    import robot_lab  # noqa: F401
except ImportError as error:
    print(f"ERROR: incomplete Isaac Lab environment: {error}", file=sys.stderr)
    sys.exit(4)

if not torch.cuda.is_available():
    print("ERROR: CUDA GPU is unavailable; GO2 Isaac Lab training is not practical on CPU.", file=sys.stderr)
    sys.exit(5)

print(f"OK: CUDA={torch.version.cuda}, GPU={torch.cuda.get_device_name(0)}")
PY
