# 实物托盘：相机多视角重建与尺寸验证

这是现有 TRELLIS.2 流程之外的测量路线。输出为摄影测量点云、可选候选网格和尺度验证报告；不自动生成已验收的 CAD 或物理碰撞模型，也不保证亚毫米精度。

## 1. 环境（Linux 服务器）

使用单独环境，不改 TRELLIS.2 环境：

```bash
conda create -n tray-photo python=3.10 -y
conda activate tray-photo
python -m pip install numpy trimesh
colmap -h
colmap patch_match_stereo -h
```

须另外安装具有 CUDA 稠密重建支持的 COLMAP，并将 `colmap` 加入 PATH。安装方式见 https://colmap.github.io/install.html 。某些发行版包没有 CUDA，不能完成此脚本的稠密阶段；上述帮助命令不代表 GPU 已通过验证。命令流程依据 https://colmap.github.io/cli.html 。本机没有 COLMAP，因此尚未实跑真实重建。

## 2. 拍摄与目录

```text
/data/tray/top/       正面同一静止姿态的照片
/data/tray/bottom/    翻面后另一组照片
```

固定同一个相机、焦距、对焦和图片尺寸；关闭数码变焦，不混入裁剪图。脚本使用共享 OPENCV 内参并由 COLMAP 优化，不是实验室预标定。相机绕静止托盘拍摄，保持纹理、标尺和背景不动。均匀照明，环绕多个高度，有重叠地补拍边沿下方。无纹理表面添加可移除薄标记，关键接触面避免被覆盖。

正反面分别运行，不能将翻面后的背景当成与托盘共同静止。建议先做一组 20–40 张局部试拍，确认有足够匹配，再拍完整数据。保留用于恢复尺度的可识别标尺端点，以及至少两组不参与尺度拟合的实测尺寸。

## 3. 重建

在项目根目录运行：

```bash
python scripts/tray_photogrammetry.py reconstruct \
  --images /data/tray/top \
  --output runs/tray_top_v1 --max-image-size 4000 --mesh
```

输出：

- `dense/fused.ply`：主要测量参考，任意尺度，包含背景。
- `dense/mesh_candidate.ply`：可选 Poisson 候选，可能补出不存在的面、封孔或连薄壁，不能直接作为碰撞真值。
- `inputs.json`：照片路径与 SHA256。
- `commands.jsonl` 和各阶段日志：复现与排错。
- `model_analyzer.log`：检查注册图像数量、重投影误差。重投影误差小不代表物理尺寸准确。

不自动挑选多个不连通模型。如果出现多个模型，脚本停止，需要改善照片重叠。即使只得到一个模型，也要人工检查是否丢失大量照片、是否仅重建了背景。输出目录必须为空，失败后保留日志，用新目录重试。

## 4. 选点、恢复尺度、验证

在能读取 PLY 坐标的三维查看器（例如 CloudCompare）打开**未缩放**的 `fused.ply`，挑选与实物测量位置完全对应的端点，抄录其 XYZ 原始坐标。保持坐标系不变；不要启用会改变导出坐标的平移或缩放。像素坐标不能填入此文件。

复制 `examples/tray_measurements.json`，替换**全部**示例坐标及尺寸。每个条目中 `a`、`b` 为原始 PLY 的三维坐标，`distance_mm` 为实测毫米距离。`calibration` 只负责拟合统一尺度；`validation` 必须用预留的独立尺寸，建议覆盖不同方向以及抓取边、堆叠间隙。软件只检查完全重复的端点对，实际独立性由采集者保证。

```bash
python scripts/tray_photogrammetry.py metric \
  --input runs/tray_top_v1/dense/fused.ply \
  --measurements examples/my_tray_measurements.json \
  --output runs/tray_top_v1/tray_scene_meters.ply \
  --tolerance-mm 0.5
```

`0.5` 是你指定的验收阈值，不是精度承诺。工具以校准距离做最小二乘尺度拟合，仅缩放、不平移或旋转。输出以**米**为单位，保持颜色及网格拓扑。`*.report.json` 包含尺度、各测量的有符号误差、独立验证最大绝对误差与 RMSE。验证超限会返回非零退出码，只写报告、不导出模型。

若对 `mesh_candidate.ply` 缩放，应在同一原始坐标系中选点并独立检查候选表面，避免忽略网格化偏差。输入点云和网格均支持，输出必须是 PLY。

## 5. 后续 CAD 和仿真

在米制点云上裁掉背景，依据关键截面及实测尺寸建立 CAD。正反面用共同侧壁和标记做刚性配准，禁止用自由缩放消除差异；翻面引起的形变需要另行判断。保存未经平滑的点云，不能把补洞区域当成测量结果。

最终碰撞几何还需检查边沿、壁厚、凹槽与堆叠间隙，并标定质量、摩擦和夹爪接触。通过少量尺寸检查不等于整个表面或物理行为已经验收。

## 本地测试

```bash
python -m unittest discover -s scripts -p test_tray_photogrammetry.py -v
```
