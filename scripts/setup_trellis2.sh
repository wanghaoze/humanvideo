#!/usr/bin/env bash
# Run inside an activated, dedicated Python 3.10 conda environment.
set -eo pipefail
ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
REPO="${TRELLIS2_DIR:-$ROOT/third_party/TRELLIS.2}"
command -v nvidia-smi >/dev/null || exit 1
if [[ -z "${CONDA_PREFIX:-}" && -z "${VIRTUAL_ENV:-}" ]]; then
  echo 'Activate a dedicated conda/venv environment first.' >&2
  exit 1
fi
# Target: RTX PRO 6000 Blackwell, CUDA Toolkit 12.9, PyTorch cu129.
# Honor explicit paths; support both spellings seen on the user's server.
if [[ -n "${CUDA_HOME:-}" ]]; then
  [[ -x "$CUDA_HOME/bin/nvcc" ]] || { echo "No nvcc at CUDA_HOME=$CUDA_HOME" >&2; exit 1; }
elif [[ -x /usr/local/cuda12.9/bin/nvcc ]]; then
  export CUDA_HOME=/usr/local/cuda12.9
elif [[ -x /usr/local/cuda-12.9/bin/nvcc ]]; then
  export CUDA_HOME=/usr/local/cuda-12.9
elif [[ -n "${CONDA_PREFIX:-}" && -x "$CONDA_PREFIX/bin/nvcc" ]]; then
  export CUDA_HOME="$CONDA_PREFIX"
elif command -v nvcc >/dev/null; then
  CUDA_HOME="$(dirname "$(dirname "$(readlink -f "$(command -v nvcc)")")")"
  export CUDA_HOME
else
  echo 'Set CUDA_HOME to your CUDA 12.9 Toolkit directory.' >&2
  exit 1
fi
export PATH="$CUDA_HOME/bin:$PATH"
CUDA_RELEASE="$(nvcc --version | sed -n 's/.*release \([0-9]*\.[0-9]*\).*/\1/p')"
if [[ "$CUDA_RELEASE" != 12.9 ]]; then
  echo "Detected Toolkit $CUDA_RELEASE at $CUDA_HOME; this installer pins PyTorch cu129." >&2
  echo 'Select your existing CUDA 12.9 Toolkit before building extensions:' >&2
  echo '  export CUDA_HOME=/usr/local/cuda12.9  # or /usr/local/cuda-12.9' >&2
  echo '  export PATH="$CUDA_HOME/bin:$PATH"' >&2
  exit 1
fi
echo "Using CUDA Toolkit $CUDA_RELEASE at $CUDA_HOME"
export CUDA_VISIBLE_DEVICES="${CUDA_VISIBLE_DEVICES:-0}"
export TORCH_CUDA_ARCH_LIST="${TORCH_CUDA_ARCH_LIST:-12.0}"
export ATTN_BACKEND=xformers
export SPARSE_ATTN_BACKEND=xformers
export MAX_JOBS="${MAX_JOBS:-4}"
# Segregate JIT extensions from builds made with the previous torch/CUDA ABI.
export TORCH_EXTENSIONS_DIR="${TORCH_EXTENSIONS_DIR:-$ROOT/tmp/torch_extensions_cu129}"
python -m pip install torch==2.8.0 torchvision==0.23.0 xformers==0.0.32.post2 --index-url https://download.pytorch.org/whl/cu129
python "$ROOT/scripts/check_trellis2_gpu.py"
if [[ ! -d "$REPO" ]]; then
  git clone --recursive https://github.com/microsoft/TRELLIS.2.git "$REPO"
fi
cd "$REPO"
git rev-parse HEAD | tee "$ROOT/trellis2-version.txt"
# Official installer uses sudo apt for libjpeg-dev. Run on Ubuntu/Debian.
# Do not use --new-env or --flash-attn: these pin the older upstream stack.
source ./setup.sh --basic --nvdiffrast --nvdiffrec --cumesh --o-voxel --flexgemm
python -m pip install 'transformers==4.57.3' 'opencv-python-headless<5'
python "$ROOT/scripts/check_dinov3_api.py" --repo "$REPO"
python -c 'import torch, trellis2, o_voxel; assert torch.cuda.is_available(); print(torch.cuda.get_device_name(0))'
python "$ROOT/scripts/check_trellis2_gpu.py"
echo "Ready. Download: python $ROOT/scripts/humanvideo_trellis2.py download"
