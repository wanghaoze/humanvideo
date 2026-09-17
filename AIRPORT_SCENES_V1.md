# 机场安检托盘场景 v1

入口：`scripts/generate_airport_scenes.py`。仅新增生成器、schema、测试及新输出目录，不修改 vendor，也不覆盖原场景/轨迹。

## 生成与审核

Windows（项目根目录）：

```powershell
.\.venv-r1pro\Scripts\python.exe scripts/generate_airport_scenes.py --output outputs/airport_new_run --seed 20260916 --workers 4
.\.venv-r1pro\Scripts\python.exe scripts/audit_airport_scenes.py --root outputs/airport_new_run
```

Linux（使用现有 Conda，不安装系统包）：

```bash
conda activate r1pro-tray
export MUJOCO_GL=egl
export MUJOCO_EGL_DEVICE_ID=1
python scripts/generate_airport_scenes.py --output outputs/airport_new_run --seed 20260916 --workers 4
python scripts/audit_airport_scenes.py --root outputs/airport_new_run
```

默认先生成并验证 12 个 smoke，再生成正式 400 个。`--smoke-only` 只运行前者。输出目录存在即拒绝执行；修改 seed 时仍须选择新目录。原始资产依赖 `outputs/r1pro_tray_scene/assets`，无需复制或修改 vendor。

最终交付目录：`outputs/airport_trays_final_v1`。其他 `airport_smoke_dev01`、`airport_trays_v1`、`airport_trays_release_v1` 是开发中保留的检查批次，不作为正式训练场景集。

## 场景与坐标

- 三张桌子，每张长 1.2 m、宽 0.6 m、桌面高 0.8 m。两张前桌无间距相邻。
- 三只托盘，长 0.56 m、宽 0.43 m、高 0.105 m，沿用原质量、碰撞网格及摩擦。
- 履带原点在左端近机器人一侧桌面角，世界坐标 `[0.55, 0.6, 0.8]`。
- 履带 `+X = 世界 -Y`，`+Y = 世界 +X`，`+Z = 世界 +Z`。JSON 保存完整变换矩阵。
- 前后桌距离指**相对桌边之间的净通道宽度**，均匀采样 1～2 m；不是两桌中心距。
- 槽位：`x_i = 0.005 + 0.43/2 + i*(0.43+0.015)`，`y_i = 0.6/2`，`z_i = 0`（桌面坐标），`yaw_i = π/2`。长边垂直于履带长边，左侧留 5 mm，相邻间隙 15 mm。
- 后桌容纳三只普通托盘时采用受约束的随机排布，端部约 6～7 cm 悬边；质心保持在有效桌面内，并通过动力学验证。不能在固定桌面尺寸下同时承诺任意 yaw、三只不重叠和不悬边。
- 复杂集使用轻微错位的双层堆叠，第三只分开放置；高度由支撑层级确定，初始 roll/pitch 都是零。此版不生成大角度斜搭场景。
- 世界安装的 head 相机随初始 base pose 转换；手腕相机局部位姿、720p framebuffer、关节初始值、夹爪增益/力限制保持原配置。
- 地面与可见桌面共用同一 material；日夜、光方向和强度分别随机化。

## 数量与文件

Smoke：front_count 0/1/2/3 各 3 个，共普通 10、复杂 2。

正式：front_count 0/1/2/3 各 100 个，共普通 380、复杂 20。按 scene 划分 train/validation/test 为 320/40/40；复杂场景也分布在三个 split。两个批次 seed 不重叠。

每个批次的 `ordinary/`、`complex/` 下，每场景有 `scene.json` 和 `scene.xml`。批次 `manifest.json` 汇总索引、拒绝原因及重采样次数。每批选 12 个生成原生 720p 单场景预览及 `preview_grid.jpg`。`audit.json` 保存全量覆盖和重建校验。

Schema：`schemas/airport_scene_v1.schema.json`，版本 `airport_trays/1.0.0`。记录 seed、接受候选的 attempt、scene_id、difficulty、split、支撑关系、灯光、目标槽位、运行库版本、源场景和 XML 哈希。相同 seed、生成器版本、模板及 MuJoCo/NumPy 环境可确定性复现；更换物理引擎版本应重新验收。

## 自动验收

每候选先检查数量、质心有效区域（桌边内缩 15 mm）、普通布局的二维旋转矩形不重叠、槽位边界。随后加载实际碰撞网格，按原始关节目标静置 2 秒：

- 检查机器人与桌/托盘碰撞；固定机器人与固定桌子的接触还用 `mj_geomDistance` 检查，避免静态碰撞过滤漏检。
- 拒绝托盘接触地面、滑出桌面、倾斜超过 10°、穿透超过 3 mm。
- 结束时质心仍在指定桌面区域，接触图必须连到指定桌面；允许经下层托盘形成支撑链。
- 托盘末态线速度不超过 3 cm/s；普通布局末态不得重叠。
- 每次失败保存原因和 attempt，最多 250 次。达到上限保留 failure.json 并失败退出，不把无效候选标为成功。

这是**场景生成阶段**。机器人 base 仅随机初始化，`base_control_available=false`；不产生虚假的底盘动作，也不生成遥操标签/轨迹。新场景有三只具名托盘，尚不能直接替换依赖单个 `tray` body 的旧在线 Recorder；三托盘任务状态机、移动和分段录制留给后续任务。
