#!/usr/bin/env bash
set -euo pipefail

script_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
project_dir="$(cd "${script_dir}/.." && pwd)"
export_dir="${1:-}"

if [[ -z "${export_dir}" ]]; then
    echo "Usage: $0 /path/to/checkpoint/exported" >&2
    exit 2
fi

pt_source="${export_dir}/policy.pt"
onnx_source="${export_dir}/policy.onnx"
if [[ ! -f "${pt_source}" || ! -f "${onnx_source}" ]]; then
    echo "ERROR: expected policy.pt and policy.onnx below ${export_dir}" >&2
    exit 2
fi

PYTHONNOUSERSITE=1 "${project_dir}/.venv/bin/python" \
    "${project_dir}/runtime/go2_policy.py" \
    --backend onnx --model "${onnx_source}" --warmup 10 --iterations 20 >/dev/null

install -m 0644 "${pt_source}" "${project_dir}/models/go2_robot_lab/policy.pt"
install -m 0644 "${onnx_source}" "${project_dir}/models/go2_robot_lab/policy.onnx"
install -m 0644 "${onnx_source}" "${project_dir}/upstream/rl_sar/policy/go2/robot_lab/policy.onnx"

sha256sum \
    "${project_dir}/models/go2_robot_lab/policy.pt" \
    "${project_dir}/models/go2_robot_lab/policy.onnx"
echo "Published the validated policy to the CPU runtime and rl_sar deployment tree."
