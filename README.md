# humanvideo：TRELLIS.2 交互物体重建试验

实物托盘的相机多视角重建、米制尺度恢复和独立尺寸验证，见 [RECONSTRUCT_REAL_TRAY.md](RECONSTRUCT_REAL_TRAY.md)。入口为 `scripts/tray_photogrammetry.py`，与下面的单图生成流程独立。

针对图片和视频中的手持或待交互物体，生成带材质的 GLB。当前工作区没有实际媒体数据，因此提供可在 Linux 服务器执行的完整脚本；尚未进行真实 GPU 推理。

这是**指定物体后的单图生成式重建**。视频抽帧后，人工选遮挡少、清晰的帧，并用物体框或掩码指定目标。不自动判断手在操作哪个物体，不进行跨帧跟踪、多视角融合或真实尺度恢复。遮挡背面由模型推测，输出不能直接当作精确碰撞几何或抓取接触面。

## 1. Linux 环境

本安装脚本针对用户的 **RTX PRO 6000 Blackwell（96GB）**，使用现有 **CUDA Toolkit 12.9 + PyTorch 2.8.0/cu129 + torchvision 0.23.0 + xFormers 0.0.32.post2**。不需要安装 CUDA 12.4。组合依据 [PyTorch 官方版本表](https://pytorch.org/get-started/previous-versions/) 和 [cu129 的 xFormers 安装包](https://download.pytorch.org/whl/cu129/xformers/)，TRELLIS.2 允许使用自定义 PyTorch/CUDA 环境。下面以 Ubuntu/Debian 为例；安装器会调用 sudo 安装 libjpeg-dev。完整 TRELLIS.2 扩展和推理仍需在目标服务器验证。

Transformers 固定为 **4.57.3**，以匹配上游特征提取代码直接访问的 `DINOv3ViTModel.layer`。旧环境遇到缺少 `layer` 时，执行 `python -m pip install 'transformers==4.57.3'`；无需重新安装 PyTorch 或下载权重。参考 [4.57.3 的模型实现](https://github.com/huggingface/transformers/blob/v4.57.3/src/transformers/models/dinov3_vit/modeling_dinov3_vit.py)。安装末尾的小模型 CPU 检查会验证实际 TRELLIS.2 特征提取接口。

把此目录复制到服务器，进入项目目录：

```bash
sudo apt-get update
sudo apt-get install -y git build-essential libjpeg-dev
conda create -n humanvideo-trellis2 python=3.10 -y
conda activate humanvideo-trellis2
export CUDA_HOME=/usr/local/cuda12.9  # 如果实际目录有连字符，改成 /usr/local/cuda-12.9
export PATH="$CUDA_HOME/bin:$PATH"
export CUDA_VISIBLE_DEVICES=0
nvcc --version
bash scripts/setup_trellis2.sh
```

安装脚本下载官方仓库，使用其 setup.sh 编译扩展，将实际代码提交号记录到 trellis2-version.txt。它不会更新已存在的仓库。建议使用全新环境；官方扩展安装器不是严格幂等的，失败重跑时需检查 `/tmp/extensions` 中的残留和安装日志。

如果你已按旧说明配置过 CUDA_HOME，必须先改成上述 12.9 路径。脚本也可在未设置 CUDA_HOME 时自动查找 `/usr/local/cuda12.9` 和 `/usr/local/cuda-12.9`。使用已有环境时会替换 PyTorch/torchvision，并重新构建扩展；应确保这是专用环境，没有正在运行的任务。

脚本默认只使用 GPU 0（用户提供的状态中约 94GiB 空闲），为 RTX Blackwell 设置 `TORCH_CUDA_ARCH_LIST=12.0`，且设置 `ATTN_BACKEND=xformers`、`SPARSE_ATTN_BACKEND=xformers`。不会调用官方安装器的 `--new-env` 或 `--flash-attn`，避免重新安装旧 PyTorch 或固定的 FlashAttention 2.7.3。普通注意力和稀疏注意力都需要配置，单独设置 SDPA 不覆盖官方稀疏注意力。

安装器会在编译扩展之前和安装结束时进行 GPU 矩阵乘法、FP16/BF16 普通/变长 xFormers 注意力检查。也可单独复验：

```bash
CUDA_VISIBLE_DEVICES=0 python scripts/check_trellis2_gpu.py
```

若旧版检查在 `flash-attention/hopper/flash_fwd_launch_template.h` 报 `invalid argument`，这是本次服务器实测遇到的 xFormers 自动选择 FA3 内核问题。更新 scripts 目录中的 `trellis2_attention.py`、`check_trellis2_gpu.py` 和 `humanvideo_trellis2.py`，然后用新的 Python 进程重跑上述检查。无需重装已成功安装的 PyTorch/CUDA 包。

当前两个入口共享进程内 CUTLASS 显式选择逻辑，通过 xFormers 的 `op` 参数绕过自动 FlashAttention 调度，不修改 site-packages。检查覆盖 FP16/BF16、64/128 维注意力头、普通及不等长分块交叉注意力，并与 FP32 参考结果比较。CUTLASS 在目标 GPU 上尚待复验；若仍失败，应保留新日志继续诊断，不要跳过检查。直接运行上游 example.py 不会应用此修正。

`nvidia-smi` 中的 CUDA Version 13.2 表示驱动支持的 CUDA 版本上限，不要求应用使用 Toolkit 13.2；安装脚本以实际 `nvcc --version` 为准，本配置要求 12.9。GPU 检查通过仅代表这些基础内核可运行，不等于完整重建验证通过。

## 2. 下载模型

先在 Hugging Face 登录，并按模型页要求申请/接受 DINOv3 和 RMBG-2.0 的访问条件。403 通常需要检查账号权限；脚本不会绕过访问限制。

```bash
export HF_HOME=/data/cache/huggingface
hf auth login
python scripts/humanvideo_trellis2.py download
```

下载包含主模型、来自 TRELLIS-image-large 的稀疏解码器，以及 pipeline.json 指定的 DINOv3、RMBG-2.0。使用 Hugging Face 缓存，重复运行会复用已下载文件；快照路径记录到 model-downloads.json。即使输入有透明通道，官方加载器仍初始化背景移除模型，因此一并下载。

## 3. 收集图片和视频帧

```bash
python scripts/humanvideo_trellis2.py extract \
  --input /data/humanvideo --output ./frames \
  --every 2 --max-frames 32
```

递归读取图片及常见视频。每个视频最多均匀采样 32 帧，输出 PNG 和 frames.json（原始路径、帧号、时间）。达到上限时实际间隔会超过 2 秒。帧输出目录必须放在数据输入目录之外。

## 4. 指定交互物体

复制 examples/objects.json 为自己的清单，删除不用的示例条目，替换路径和坐标。相对路径均以**清单所在目录**为基准；也支持绝对路径。

```json
[
  {
    "id": "cup",
    "image": "/data/humanvideo/cup.jpg",
    "bbox": [120, 80, 420, 500]
  }
]
```

- `bbox`：原图像素坐标 `[左, 上, 右, 下]`，右/下不包含；必须在图像内并留出框外背景。脚本使用 GrabCut 做粗分割，手与物体相接时可能一起保留。
- `mask`：推荐使用；与原图同尺寸的灰度 PNG，白色为可见物体、黑色为背景，优先于 bbox。
- `exclude_mask`：可选，同尺寸灰度 PNG，白色区域从物体掩码中剔除，可用于排除手。它不会自动补回被手遮挡的像素。
- 也可直接输入仅保留物体的透明 PNG，省略 bbox 和 mask。
- 一个条目对应一个独立 3D 候选。相同物体的不同帧需使用不同 id，不会融合。

先用 CPU 检查物体提取结果：

```bash
python scripts/humanvideo_trellis2.py reconstruct \
  --manifest examples/my_objects.json --output previews --prepare-only
```

检查 `previews/<id>/object.png`。若残留手、背景或物体被切掉，调整掩码后重跑。可在普通 CPU 环境运行 `pip install numpy pillow 'opencv-python-headless<5'` 使用抽帧和预处理功能（本脚本用 OpenCV 4.x 验证）。

## 5. 生成 3D

```bash
python scripts/humanvideo_trellis2.py reconstruct \
  --manifest examples/my_objects.json --output outputs --pipeline 512
```

质量提升试验（显存和耗时增加）：

```bash
python scripts/humanvideo_trellis2.py reconstruct \
  --manifest examples/my_objects.json --output outputs_1024 \
  --pipeline 1024_cascade --texture-size 4096 --faces 300000
```

每个物体保存 object.png、object.glb 和 result.json。GLB 可用 Blender 打开。失败记录包含错误，整个命令返回非零；已存在的 GLB 不会覆盖，重试失败条目时可使用只包含失败条目的清单，或选择新输出目录。

全部权重下载成功后，可用 `HF_HUB_OFFLINE=1 TRANSFORMERS_OFFLINE=1` 前缀检查离线推理。若仍提示缺文件，联网补全对应缓存。不要在推理时改变 HF_HOME，否则找不到先前下载的模型。

常见问题：CUDA 编译失败检查 nvcc、CUDA_HOME 和 PyTorch CUDA 版本；OOM 先使用 512、缩小纹理，仍失败需更大显存。框分割错误优先改用精确物体 mask。真实尺度、坐标系对齐、碰撞网格和手物接触约束需要后续标定与验证。

接口参考：[官方推理代码](https://github.com/microsoft/TRELLIS.2/blob/main/trellis2/pipelines/trellis2_image_to_3d.py)、[官方导出示例](https://github.com/microsoft/TRELLIS.2/blob/main/example.py)、[模型配置](https://huggingface.co/microsoft/TRELLIS.2-4B/blob/main/pipeline.json)。

## 本地验证

已通过 Python 编译检查、Bash 语法检查，以及合成媒体的物体掩码、手部排除、GrabCut、空掩码拒绝、预处理 CLI、视频抽帧检查。可运行 `python scripts/check_preprocessing.py` 复验。未验证 Linux 安装、模型下载和 GPU 推理；这些需要在目标服务器执行。
