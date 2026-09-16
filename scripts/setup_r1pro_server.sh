#!/usr/bin/env bash
set -euo pipefail
ROOT="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
cd "$ROOT"
if [[ "${1:-}" != "" && "${1:-}" != "--with-lerobot" ]]; then
  echo "Usage: conda activate r1pro-tray; bash scripts/setup_r1pro_server.sh [--with-lerobot]" >&2
  exit 2
fi
if [[ -z "${CONDA_PREFIX:-}" || "${CONDA_DEFAULT_ENV:-}" == "base" ]]; then
  echo "Activate a non-base Conda environment first. No system/venv installation is supported." >&2
  exit 2
fi
P="$CONDA_PREFIX/bin/python"
"$P" -c 'import os,sys; from pathlib import Path; assert Path(sys.prefix).resolve()==Path(os.environ["CONDA_PREFIX"]).resolve(); assert (3,10)<=sys.version_info[:2]<=(3,12), "Use Python 3.10–3.12"'
mkdir -p runs/environment_snapshots
STAMP="$(date +%Y%m%d_%H%M%S)"
"$P" -m pip freeze > "runs/environment_snapshots/before_${STAMP}.txt"
"$P" -m pip install -r requirements-r1pro-sim.txt
if [[ "${1:-}" == "--with-lerobot" ]]; then
  "$P" -m pip install 'torch==2.8.0' 'torchvision==0.23.0' --index-url https://download.pytorch.org/whl/cu129
  "$P" -m pip install 'lerobot==0.4.4' 'torchcodec==0.7.0'
fi
"$P" -m pip check
"$P" -m pip freeze > server-installed-versions.txt
echo "Installed inside Conda: $CONDA_PREFIX"
echo "Run in the same Conda environment: bash scripts/run_r1pro_server.sh"
