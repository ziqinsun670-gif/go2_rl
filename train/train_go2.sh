#!/usr/bin/env bash
set -euo pipefail

script_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
project_dir="$(cd "${script_dir}/.." && pwd)"
robot_lab_dir="${ROBOT_LAB_DIR:-${project_dir}/upstream/robot_lab}"
python_bin="${ISAACLAB_PYTHON:-python3}"
num_envs="${NUM_ENVS:-4096}"

if [[ ! -f "${robot_lab_dir}/scripts/reinforcement_learning/rsl_rl/train.py" ]]; then
    echo "ERROR: ROBOT_LAB_DIR does not point to an installed robot_lab checkout: ${robot_lab_dir}" >&2
    exit 2
fi

"${script_dir}/check_training_env.sh"

cd "${robot_lab_dir}"
export PYTHONPATH="${robot_lab_dir}/source/robot_lab${PYTHONPATH:+:${PYTHONPATH}}"
exec "${python_bin}" scripts/reinforcement_learning/rsl_rl/train.py \
    --task RobotLab-Isaac-Velocity-Rough-Unitree-Go2-v0 \
    --headless \
    --num_envs "${num_envs}" \
    --device cuda:0 \
    "$@"
