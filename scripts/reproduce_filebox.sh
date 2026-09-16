#!/usr/bin/env bash
# Installed environment required. No installation or model download unless requested.
set -euo pipefail
ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
DATA_ROOT="${DATA_ROOT:-/home/michelle/haoze/EgoDemo/EgoProStandard-body/lerobot/Return File Box to Place}"
RUN_DIR="${RUN_DIR:-$ROOT/runs/filebox}"
PIPELINE="${PIPELINE:-512}"
export CUDA_VISIBLE_DEVICES="${CUDA_VISIBLE_DEVICES:-0}"
export ATTN_BACKEND=xformers
export SPARSE_ATTN_BACKEND=xformers
export TORCH_EXTENSIONS_DIR="${TORCH_EXTENSIONS_DIR:-$ROOT/tmp/torch_extensions_cu129}"
cd "$ROOT"
case "${1:-prepare}" in
  prepare)
    python scripts/prepare_filebox.py --data-root "$DATA_ROOT" --output "$RUN_DIR"
    ;;
  download)
    python scripts/humanvideo_trellis2.py download
    ;;
  run|run-all)
    MANIFEST="$RUN_DIR/objects_primary.json"
    [[ "$1" != run-all ]] || MANIFEST="$RUN_DIR/objects.json"
    [[ -f "$MANIFEST" ]] || { echo 'Run the prepare stage first.' >&2; exit 1; }
    mkdir -p "$RUN_DIR/logs"
    LOG="$RUN_DIR/logs/${1}_${PIPELINE}_$(date +%Y%m%d_%H%M%S).log"
    python -u scripts/humanvideo_trellis2.py reconstruct \
      --manifest "$MANIFEST" --output "$RUN_DIR/outputs_${1}_${PIPELINE}" \
      --pipeline "$PIPELINE" --seed 42 --faces 100000 --texture-size 2048 2>&1 | tee "$LOG"
    ;;
  *)
    echo 'Usage: bash scripts/reproduce_filebox.sh [prepare|download|run|run-all]' >&2
    exit 2
    ;;
esac
