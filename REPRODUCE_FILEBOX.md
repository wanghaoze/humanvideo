# Return File Box to Place：服务器复现

安装已经完成，以下步骤不会重新安装环境。把本复现包解压到 `/home/michelle/haoze/humanvideo`，保留 scripts 和 examples 的相对目录关系。

## 已核对的数据

- 服务器任务目录：`/home/michelle/haoze/EgoDemo/EgoProStandard-body/lerobot/Return File Box to Place/`
- Episode：`fb075fd0-12dd-40bc-93cd-341468f56a61`
- 视频：`videos/observation.images.head_left/chunk-000/file-000.mp4`
- 本地实际解码：1920×1456，544 帧，30.006 fps，约 18.13 秒。
- 四路视频均查看过抽样帧。腕部相机手掌遮挡重，首次复现使用头部左相机。

|候选|从 0 开始的帧号|时间|说明|
|---|---:|---:|---|
|filebox_closed|300|约 9.998 秒|默认首选，合盖 FILE BOX|
|filebox_open_with_contents|135|约 4.499 秒|打开的白盒，包含可见内装物|
|lid_inner_surface|135|约 4.499 秒|棕色内侧、白色边框的疑似盒盖|

这些是同一任务中的部件/状态候选，不代表三个独立物体的真值标注。annotation.json 的文字与视觉部件身份不完全等价，所以不把棕色区域断言成另一个独立文件盒。

## 复制执行

```bash
conda activate humanvideo-trellis2
cd /home/michelle/haoze/humanvideo
export CUDA_VISIBLE_DEVICES=0
export CUDA_HOME=/usr/local/cuda12.9
# 若实际目录是 /usr/local/cuda-12.9，请相应修改上一行。
export PATH="$CUDA_HOME/bin:$PATH"

# 1. 在服务器的原视频中提取已选帧，生成掩码、预览和物体清单。
bash scripts/reproduce_filebox.sh prepare

# 2. 下载主模型及辅助权重；已有完整缓存可以跳过。
# 沿用之前的 HF_HOME；不要切换到另一个空缓存目录。
# DINOv3/RMBG 若提示 401/403，先用有访问权限的账号执行 hf auth login。
bash scripts/reproduce_filebox.sh download

# 3. 首次只重建合盖盒子，512 分辨率，seed=42。
bash scripts/reproduce_filebox.sh run
```

第一步应生成 `runs/filebox/previews/contact_sheet.jpg`，可下载查看。清单和 PNG 路径在服务器生成，无需把本机 C: 路径改成 Linux 路径，也不用手工填 bbox。

默认 GLB：

```text
/home/michelle/haoze/humanvideo/runs/filebox/outputs_run_512/filebox_closed/object.glb
```

同目录有 `object.png` 和 `result.json`；日志在 `runs/filebox/logs/`。`source.json` 记录视频 SHA256、fps、帧数和本次选帧/多边形配置。保留这些文件可追溯输入。

## 其他实验

如果遇到 `AttributeError: 'DINOv3ViTModel' object has no attribute 'layer'`，是 Transformers 的模型内部接口与此 TRELLIS.2 版本不匹配。当前安装脚本已固定到包含 `.layer` 接口的 4.57.3。已安装环境仅执行下面的依赖修复后重跑，无需重新安装 CUDA、编译扩展或下载模型：

```bash
python -m pip install 'transformers==4.57.3'
python scripts/check_dinov3_api.py
bash scripts/reproduce_filebox.sh run
```

`check_dinov3_api.py` 使用随机小模型在 CPU 上调用实际 TRELLIS.2 特征提取方法，不下载任何预训练权重。服务器原有脚本不包含此检查文件时，可以先同步更新的 scripts 目录，或省略检查命令直接重跑。

三个候选都跑一遍（与首选输出目录分开）：

```bash
bash scripts/reproduce_filebox.sh run-all
```

输出位于 `runs/filebox/outputs_run-all_512/<候选ID>/object.glb`。

提高主候选的生成分辨率：

```bash
PIPELINE=1024_cascade bash scripts/reproduce_filebox.sh run
```

输出位于 `runs/filebox/outputs_run_1024_cascade/filebox_closed/object.glb`。这不会覆盖 512 结果。已完成的同名 GLB 不会覆盖；完全重跑可使用新实验目录，必须对两个阶段使用同一 RUN_DIR：

```bash
export RUN_DIR="$PWD/runs/filebox_trial2"
bash scripts/reproduce_filebox.sh prepare
bash scripts/reproduce_filebox.sh run
```

如服务器 Episode 并非此任务目录的直接子目录，可显式指定实际 Episode 目录：

```bash
DATA_ROOT="/actual/path/fb075fd0-12dd-40bc-93cd-341468f56a61" \
  bash scripts/reproduce_filebox.sh prepare
```

## 结果解释与验证状态

已用用户下载的真实视频在本地执行 prepare，生成并查看了三个物体预览，也验证了它们可以经通用重建脚本的 `--prepare-only` 路径处理。Linux Bash 语法、Python 编译检查通过。GPU 安装由用户确认完成，这里尚未在服务器实际生成 GLB。

掩码是根据这段视频人工选定的粗多边形，不是自动分割模型的结果。被手臂遮住的边缘、盒盖近侧会缺失，TRELLIS.2 也可能把缺口误当作真实形状。默认合盖候选相对完整，适合先验证流程。若用于精确人手交互，还需要更少遮挡的参考、精细掩码、尺度标定和碰撞面验证；本次不会融合双目/腕部视频或将网格对齐到 Parquet 中的手部世界坐标。
