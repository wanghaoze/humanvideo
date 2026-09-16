#!/usr/bin/env bash
set -euo pipefail
ROOT="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
cd "$ROOT"
if [[ -z "${CONDA_PREFIX:-}" || "${CONDA_DEFAULT_ENV:-}" == "base" ]]; then
  echo "Activate the R1 Pro Conda environment first." >&2
  exit 2
fi
GPU_MAPPING="$("$CONDA_PREFIX/bin/python" scripts/select_r1pro_gpu.py --gpu "${R1PRO_GPU:-0}" --fields)"
read -r EGL_INDEX GPU_UUID <<< "$GPU_MAPPING"
export MUJOCO_GL=egl MUJOCO_EGL_DEVICE_ID="$EGL_INDEX"
export CUDA_VISIBLE_DEVICES="$GPU_UUID" CUDA_DEVICE_ORDER=PCI_BUS_ID
exec "$CONDA_PREFIX/bin/python" "$@"
