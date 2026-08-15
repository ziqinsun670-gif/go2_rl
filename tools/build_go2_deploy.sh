#!/usr/bin/env bash
set -euo pipefail

script_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
project_dir="$(cd "${script_dir}/.." && pwd)"
rl_sar_dir="${project_dir}/upstream/rl_sar"
ort_dir="${project_dir}/third_party/onnxruntime"

if [[ ! -f "${ort_dir}/lib/libonnxruntime.so" ]]; then
    echo "ERROR: ONNX Runtime not found: ${ort_dir}" >&2
    exit 2
fi

git -C "${rl_sar_dir}" submodule update --init \
    src/rl_sar/library/thirdparty/robot_sdk/unitree/unitree_sdk2

mkdir -p "${rl_sar_dir}/library"
if [[ ! -L "${rl_sar_dir}/library/inference_runtime" ]]; then
    echo "ERROR: expected inference_runtime symlink is missing." >&2
    echo "Create ${rl_sar_dir}/library/inference_runtime -> ../../../third_party" >&2
    exit 3
fi

cmake -S "${rl_sar_dir}/src/rl_sar" -B "${rl_sar_dir}/cmake_build" \
    -DUSE_CMAKE=ON \
    -DBUILD_GO2_ONLY=ON \
    -DCMAKE_BUILD_TYPE=Release
cmake --build "${rl_sar_dir}/cmake_build" --target rl_real_go2 -j "$(nproc)"

binary="${rl_sar_dir}/cmake_build/bin/rl_real_go2"
test -x "${binary}"
echo "Built (do not run without completing the hardware checklist): ${binary}"
