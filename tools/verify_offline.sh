#!/usr/bin/env bash
set -euo pipefail

script_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
project_dir="$(cd "${script_dir}/.." && pwd)"

PYTHONNOUSERSITE=1 "${project_dir}/.venv/bin/python" \
    -m unittest discover -s "${project_dir}/tests" -v

cmake -S "${project_dir}/cpp" -B "${project_dir}/cpp/build" -DCMAKE_BUILD_TYPE=Release
cmake --build "${project_dir}/cpp/build" -j "$(nproc)"
"${project_dir}/cpp/build/go2_policy_benchmark" \
    "${project_dir}/models/go2_robot_lab/policy.onnx" "${ITERATIONS:-20000}"
