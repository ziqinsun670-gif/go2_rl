#!/usr/bin/env bash
set -euo pipefail

script_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
project_dir="$(cd "${script_dir}/.." && pwd)"
robot_lab_dir="${ROBOT_LAB_DIR:-${project_dir}/upstream/robot_lab}"
python_bin="${ISAACLAB_PYTHON:-python3}"
checkpoint="${1:-}"

if [[ -z "${checkpoint}" ]]; then
    echo "Usage: ROBOT_LAB_DIR=/path/to/robot_lab $0 /path/to/model.pt" >&2
    exit 2
fi
if [[ ! -f "${checkpoint}" ]]; then
    echo "ERROR: checkpoint not found: ${checkpoint}" >&2
    exit 2
fi
checkpoint="$(realpath "${checkpoint}")"

"${script_dir}/check_training_env.sh"

cd "${robot_lab_dir}"
export PYTHONPATH="${robot_lab_dir}/source/robot_lab${PYTHONPATH:+:${PYTHONPATH}}"
exec "${python_bin}" scripts/reinforcement_learning/rsl_rl/play.py \
    --task RobotLab-Isaac-Velocity-Rough-Unitree-Go2-v0 \
    --num_envs 1 \
    --checkpoint "${checkpoint}" \
    --device cuda:0 \
    --headless \
    --export-only
