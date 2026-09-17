# 明亮室内多光源场景 v1.1

输出：`outputs/airport_trays_indoor_v1_1`，12 个 smoke + 400 个正式场景。

四盏室内顶灯覆盖前后桌，另有环境补光和视角补光。全部使用 `indoor_bright`，取消暗夜采样。顶灯强度仅在 0.15～0.18 范围变化，方向小幅变化；一盏灯投影，其他灯补光，避免多重硬阴影。保留 720p 相机。

每个场景的 lighting 字段保存四盏灯的完整参数。布局、机器人初始姿态、材料、质量、摩擦、夹爪参数及目标槽位与上一版逐项一致；迁移脚本比较除 light/headlight 外的完整 XML 树，复用上一版两秒物理验收结果，并写入 lighting_migration 记录。原版目录未修改。

重新生成新随机场景（默认已使用室内灯光）：

```bash
python scripts/generate_airport_scenes.py --output outputs/airport_indoor_new --seed 20260916 --workers 4
```

仅更新已验收的 v1 布局灯光：

```bash
python scripts/relight_airport_scenes.py --source outputs/airport_trays_final_v1 --output outputs/airport_indoor_copy
python scripts/audit_airport_scenes.py --root outputs/airport_indoor_copy
```

使用现有 `.venv-r1pro` 或服务器 `r1pro-tray` Conda 环境。Linux 渲染沿用 `MUJOCO_GL=egl MUJOCO_EGL_DEVICE_ID=1`。输出目录必须不存在。场景包依赖原项目 `outputs/r1pro_tray_scene/assets`。
