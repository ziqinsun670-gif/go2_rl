#!/usr/bin/env bash
set -euo pipefail

script_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
project_dir="$(cd "${script_dir}/.." && pwd)"
if [[ "$#" != 0 && "$#" != 2 && "$#" != 5 ]]; then
    echo "usage: $0 [INTERFACE DURATION] [INTERFACE DURATION VX VY YAW]" >&2
    exit 2
fi
interface="${1:-enp99s0}"
duration="${2:-3}"
robot_ip="${ROBOT_IP:-192.168.123.161}"

"${script_dir}/check_connection.sh" "${interface}" "${robot_ip}"

cmake -S "${project_dir}/cpp" -B "${project_dir}/cpp/build" -DCMAKE_BUILD_TYPE=Release
cmake --build "${project_dir}/cpp/build" --target go2_shadow_inference -j "$(nproc)"

if [[ "$#" == 5 ]]; then
    exec "${project_dir}/cpp/build/go2_shadow_inference" \
        "${interface}" \
        "${project_dir}/models/go2_robot_lab/policy.onnx" \
        "${duration}" "${3}" "${4}" "${5}"
else
    exec "${project_dir}/cpp/build/go2_shadow_inference" \
        "${interface}" \
        "${project_dir}/models/go2_robot_lab/policy.onnx" \
        "${duration}"
fi
