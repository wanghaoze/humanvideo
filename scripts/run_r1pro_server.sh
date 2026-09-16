#!/usr/bin/env bash
set -euo pipefail
ROOT="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
cd "$ROOT"
if [[ -z "${CONDA_PREFIX:-}" || "${CONDA_DEFAULT_ENV:-}" == "base" ]]; then
  echo "Activate the R1 Pro Conda environment first." >&2
  exit 2
fi
exec bash scripts/run_r1pro_gpu.sh -m r1pro_teleop.server "$@"
