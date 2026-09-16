# Quest 3 → R1 Pro 仿真 → LeRobot

2026-09-14 更新：[服务器优化与工位诊断](R1PRO_SERVER_OPTIMIZATION.md)。原固定工位的预选侧边抓取点超出臂链长度上界，不能将管线验收视作可完成搬盘；新增候选布局与离线 IK 检查，尚未替换默认场景。

本包控制 MuJoCo 仿真，不含真机 ROS 指令发布器。原来的静态场景和 Blender 文件保留。入口为 `outputs/r1pro_tray_scene/scene_physics.xml`。

## 包内完成的工作

- 托盘：保留 64 块 CoACD 详细凸分解网格；运行环境使用 17 块开口接触网格，按当前模型截面近似底板和倾斜侧壁，去除浮雕文字的接触细节。
- 桌子：桌面及结构的复合凸碰撞网格，修复 OBJ 材质/法线接缝导致的平面漏失。
- 动力学：托盘自由运动；固定底盘/躯干；14 个臂关节目标和 2 个夹爪开度目标；夹爪双指联动；关节限位及模拟执行器。
- 观测：head、left_wrist、right_wrist 三个训练相机，另有 overview 操作视角。相机是仿真配置，未声称等于真机标定。
- 交互：Quest 原生 APK 输入桥接；另有浏览器 WebXR 输入与头显画面；相对位姿 IK、侧握离合、扳机夹爪、输入过期保持目标。
- 数据：20 Hz 仿真时间，同步 RGB、实测模拟关节状态、下一控制区间的目标指令；HDF5 原始记录，通过官方 LeRobot 0.4.4 API 输出 v3 数据。

托盘质量 1.2 kg、摩擦系数和执行器增益是仿真假设。几何来自当前资产，尚未用实物尺寸/质量校准。接触网格是近似，不用于宣称真实抓取精度。完整抓取成功和 Quest 头显联调仍需操作员执行验收。

## 1. Linux 服务器安装

将 `r1pro-tray-server.zip` 上传到服务器。以下解压得到独立子目录，不覆盖已有 TRELLIS.2 环境。

```bash
cd /home/michelle/haoze/humanvideo
unzip -n /你的上传目录/r1pro-tray-server.zip
cd r1pro-tray-server

# 所有依赖安装在 Conda 环境内，不使用 sudo、系统 pip 或 .venv。
conda create -n r1pro-tray python=3.11 pip -y
conda activate r1pro-tray
bash scripts/setup_r1pro_server.sh --with-lerobot
```

安装器只使用当前激活的非 base Conda 环境，通过该环境的 `python -m pip` 安装依赖；未激活环境时会拒绝执行。仿真依赖固定为 MuJoCo 3.13.0 等版本；转换安装 LeRobot 0.4.4、PyTorch 2.8/cu129、torchvision 0.23、torchcodec 0.7。保留原 `humanvideo-trellis2` 环境。torchcodec 与 torch 的版本组合见 [官方兼容表](https://github.com/meta-pytorch/torchcodec)。

若暂时只测试仿真，可不传 `--with-lerobot`，稍后用相同命令补齐。首次安装会下载依赖，不是离线 wheel 包。

先运行无头验收：

```bash
bash scripts/run_r1pro_gpu.sh -m r1pro_teleop.smoke \
  --output runs/server_smoke_v1 --frames 40
```

成功时输出 `cavity_open: true`、`rim_contact: true`，托盘静置于桌面，保存三相机图片、原始测试轨迹和 `smoke_report.json`。这里是管线测试，结果标签为 `smoke`，不会默认导入成功示教集。

启动服务：

```bash
bash scripts/run_r1pro_server.sh
```

终端会打印随机访问令牌，复制到浏览器或设置桥接程序的 `R1PRO_TOKEN`。也可启动前手动设置该环境变量。脚本按 PCI 地址和 UUID 将物理 GPU 0 映射到 EGL 设备；在 xinmatrix 上对应 EGL 1，不能直接假定 EGL 0 等于 nvidia-smi GPU 0。MuJoCo C 引擎的物理步进在 CPU 上。

电脑浏览器打开 `http://服务器IP:8765/`，输入令牌查看画面和状态。`processing_ms` 是单周期处理时间；20 Hz 的周期预算为 50 ms。若持续超过预算，仿真时间会慢于现实时间，应先看 CPU/渲染负载，不继续把这种时延当作真机控制性能。

若 EGL 初始化失败，先检查 `nvidia-smi` 与系统 EGL/OpenGL 驱动。本项目不安装或修改系统图形驱动；若系统驱动缺失，需先由服务器管理员处理。

## 2. Quest 3 原生 APK

包内文件：

- `deliverables/quest3/oculus-reader-quest3.apk`
- `deliverables/quest3/provenance.json`：下载地址、固定提交和 SHA256。
- `deliverables/quest3/platform-tools-windows.zip`：Google Android Platform Tools。
- `scripts/install_quest_apk.ps1`：设备识别、校验和安装。

APK 来自 Oculus Reader 原作者维护的 [Quest 3 分支](https://github.com/jborbik/oculus_reader)，提交 `9689484d319c4798e54d59509b192436647b7427`。上游将 Quest 3 支持标为 **Beta**。本地已检查 APK 容器、AndroidManifest 和 arm64 库，尚未在你的头显上安装验证。原生 APK 用于传控制器数据，本项目没有给它增加头显视频接收功能；原生模式配合电脑观察窗。需要在头显里看服务器画面时，使用下面的 WebXR 模式。

先在 Meta 设备上启用开发者模式，USB 连接电脑，并在头显确认 USB 调试授权，参考 [Meta 设备设置](https://developers.meta.com/horizon/documentation/native/android/mobile-device-setup/)。

在 Windows 解压本包后的根目录运行：

```powershell
Expand-Archive -LiteralPath .\deliverables\quest3\platform-tools-windows.zip -DestinationPath .\deliverables\quest3\adb
$Adb = (Resolve-Path .\deliverables\quest3\adb\platform-tools\adb.exe).Path
& $Adb devices
.\scripts\install_quest_apk.ps1 -Adb $Adb -InspectOnly
.\scripts\install_quest_apk.ps1 -Adb $Adb
```

若有多台 ADB 设备，追加 `-Serial 设备序列号`。S3A、M3 Pro 目前只有名称，品牌与 OpenXR 平台未确认，安装器不会把它们当作已验证的 Quest 3 设备；先读取 `ro.product.model` 和 `ro.product.manufacturer`。

### 原生输入桥接：USB 到电脑，电脑联网到服务器

在连接 Quest 的电脑上建立一个 Python 3.11 虚拟环境，安装桥接依赖：

```powershell
py -3.11 -m venv .venv-quest
.\.venv-quest\Scripts\python.exe -m pip install numpy==2.2.6 scipy==1.15.3 requests==2.32.5
$env:R1PRO_TOKEN = '服务器显示的令牌'
.\.venv-quest\Scripts\python.exe -m r1pro_teleop.quest_bridge `
  --server http://服务器IP:8765 --serial 设备序列号 --adb $Adb
```

桥接程序启动 APK，读取 ADB 实时控制器日志，通过 HTTP 发给服务器。每次只发送最新采样；没有新日志就不伪造新采样。请求带令牌，服务器拒绝重复序号与同时抢占的输入源。不要同时打开原生桥接和 WebXR 输入。

首次可追加 `--check-only`，确认左右手位置和扳机值变化，再连接仿真。无线 ADB 可在 USB 授权后使用 `adb tcpip 5555`、`adb connect 头显IP:5555`，随后将 `--serial` 改为 `头显IP:5555`。是否支持以头显实际系统为准。

## 3. Quest 浏览器内看画面并遥操作

此模式使用 Quest Browser 的 WebXR，不依赖上面的 APK。网页及 A-Frame 库已经随包提供，不需要联网加载 CDN。操作视图为头显内的平面视频面板，不是完整双目立体重建。

WebXR 要求安全上下文：使用可信 HTTPS，或者通过 ADB 反向端口将服务映射到头显的 `localhost`。下面的方法无需在头显导入证书：

1. 电脑开一个终端，将服务器端口转到电脑本地，保持该 SSH 会话运行：

```powershell
ssh -N -L 8765:127.0.0.1:8765 michelle@xinmatrix
```

2. 另一个电脑终端，用已授权的 Quest USB 连接设置反向端口：

```powershell
& $Adb -s 设备序列号 reverse tcp:8765 tcp:8765
```

3. Quest Browser 打开 `http://localhost:8765/vr`。输入服务器令牌，点击“连接”，再点右下角进入 VR。

主机名不能解析时，将 SSH 命令中的 `xinmatrix` 换为你的服务器实际地址。SSH 主机密钥需通过正常验证；本包不关闭主机密钥检查。HTTP 页面直接使用远端 IP 不满足 WebXR 安全上下文要求，即使普通观察页能打开。

## 4. 操作与录制

| 操作 | 行为 |
|---|---|
| 握住左/右侧握键 | 对应机械臂跟随该手柄相对位姿 |
| 松开侧握键 | 保持目标，可重新摆放手柄；再次握住重新锚定 |
| 食指扳机 | 对应夹爪闭合程度；仅在侧握激活时更新 |
| X | 开始一条原始记录 |
| A | 结束记录并由操作者标为成功 |
| B | 暂停/恢复控制；记录期间仍会继续记录 |
| 电脑页面“保存失败” | 结束并保留失败轨迹 |
| 电脑页面“重置场景” | 当前记录记为失败，然后重置托盘和机器人 |

初始为手臂下垂、夹爪打开的姿态。先空手练习离合、抬手与向前伸手，确认方向；IK 使用关节限位和每周期目标变化限制。这里尚未实现无碰路径规划；碰到桌子时应退回和调整姿态。

正确的一条成功示教应包含接近、夹住、抬起、移动、落稳、松手、撤离。A 按钮只是人工标签，不自动证明完成任务。

原始输出在 `runs/r1pro_raw/`，默认每次服务启动是一个采集 session：

```text
episode_时间_标识.h5
  images/head          uint8 [T,240,320,3]
  images/left_wrist     uint8 [T,240,320,3]
  images/right_wrist    uint8 [T,240,320,3]
  state                float32 [T,16]
  action               float32 [T,16]
  timestamp            仿真时间，间隔 0.05 s
  wall_monotonic       墙钟时间，供延迟诊断
  input_age_s          输入在服务器的年龄，-1 表示没有输入
  qpos / qvel          完整仿真状态
  tray_pose            托盘世界位姿
```

状态和 RGB 是 t 时刻的观测；`action[t]` 是随后控制区间的绝对目标。左右臂分别 7 维弧度，加一个以米表示的双指滑动关节位移之和。这是当前模拟夹爪的开度定义，不能未经校准映射到真实夹爪协议。

异常退出保留 `.partial.h5`，默认不转换。仿真质量、相机位置、控制频率或动作定义改变后，使用新的数据集版本。

## 5. 转换成 LeRobot v3

服务器包根目录运行：

```bash
# 只检查原始数据
python -m r1pro_teleop.dataset --raw runs/r1pro_raw

# 默认只导出人工标记为 success 的完整轨迹；输出目录必须尚不存在
python -m r1pro_teleop.dataset \
  --raw runs/r1pro_raw \
  --output datasets/r1pro_tray_sim_v1 \
  --repo-id local/r1pro-tray-sim-v1
```

输出包含 Parquet、三路 MP4、`meta/info.json`、统计、episode 元信息与 `source_manifest.json`。转换完成后，程序用真正的 `LeRobotDataset` 加载第一帧和最后一帧，检查视频张量与 16 维动作。不会上传 Hugging Face。

管线验收用 smoke 数据显式允许非成功记录：

```bash
python -m r1pro_teleop.dataset \
  --raw runs/server_smoke_v1/raw \
  --output datasets/pipeline_smoke_v1 \
  --repo-id local/pipeline-smoke-v1 --include-all
```

`--include-all` 用于诊断，不把失败/测试数据自动变成专家示教。数据明确标识来源为 simulation。划分训练/验证集时按 `source_manifest.json` 的 session 分组，并只用训练部分计算归一化统计。参考 [LeRobotDataset v3 官方文档](https://huggingface.co/docs/lerobot/lerobot-dataset-v3)。

## 6. 验收顺序与已知边界

1. `smoke`：场景可加载、托盘不掉穿桌面、空腔开放、边沿可接触、三相机非空、状态与动作不是错误地复制同一数组。
2. 控制输入：先 `--check-only` 或 WebXR，核对两侧手柄；松开离合重新握住不跳变；停止输入后保持目标。
3. 原始数据：至少录两条短轨迹，分别保存成功和失败，回看动作与图像。
4. 转换：用官方加载器读取完整输出；检查默认导出确实排除了失败记录。
5. 实际搬盘：手动重复抓取，记录完整成功率后再判断是否适合批量采集。

2026-09-11 已在 xinmatrix 的 r1pro-tray Conda 环境完成 Linux NVIDIA EGL、CUDA 矩阵运算、物理碰撞、三相机、HTTP 控制/录制及 LeRobot 转换验收，证据见 `validation/server/`。实测服务运行在物理 GPU 0，录制周期中位耗时约 7.5 ms、P95 约 8.5 ms。头显硬件连接和完整抓取尚未验收。

完整机理限制：碰撞分解/接触参数近似；无实测动力学校准；无真机适配；没有自动学习策略；没有证明完整抓取成功；没有把未识别的 S3A/M3 Pro 宣称为兼容头显。

## 7. 重建碰撞资产（通常不需要）

包内已经包含生成好的网格。要重新构建时另装构建依赖：

```bash
python -m pip install trimesh==4.11.2 coacd==1.0.14 networkx==3.4.2
python -m r1pro_teleop.build_assets
```

默认复用详细凸分解缓存。`--rebuild-collisions` 会重新分解，耗时明显增加。不要在录制中替换场景文件。机器人网格来自 OpenGalaxea/GalaxeaManipSim 固定提交 `abe7f5161eeaa150e6eaffdf443af5df7f23f356`，许可及 NOTICE 已保留。

## 8. 已部署服务器的日常操作

地址：`http://100.104.0.108:8765/`（电脑需能访问该 Tailscale 地址）。服务已后台启动，关闭 SSH 不会停止；服务器重启后需重新启动。

```bash
cd /home/michelle/haoze/humanvideo/r1pro-tray-server
conda activate r1pro-tray
cat runs/server.token                 # 复制到观察页或 VR 页；不要公开令牌
python scripts/control_r1pro_service.py status
python scripts/control_r1pro_service.py stop
python scripts/control_r1pro_service.py start
```

原始示教保存在 `runs/r1pro_raw/`。已有的 unlabeled 和 smoke 轨迹是验收数据，默认不会纳入成功示教集。正式录制请在完成搬盘后使用“保存成功”。

```bash
python -m r1pro_teleop.dataset --raw runs/r1pro_raw \
  --output datasets/tray_success_v1 --repo-id local/tray-success-v1
```

后台启动脚本自动选取显卡。手动执行 CUDA/EGL 检查或 smoke 时，使用 `bash scripts/run_r1pro_gpu.sh -m ...` 包装 Python 模块，以使用相同显卡映射。安装依赖版本保存在 `server-installed-versions.txt`；Conda 环境导出在 `runs/server_conda_environment.yml`。
