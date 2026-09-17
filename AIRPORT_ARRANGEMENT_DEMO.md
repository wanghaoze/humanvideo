# 三托盘完整整理示例

在服务器 MuJoCo 中，由程序控制器完整执行一个普通 smoke 场景：先整理履带上的一只托盘，再取回后桌两只，最终在履带左端并排摆放。没有人工遥操，没有物体附着约束、瞬移或直接对托盘施力。

控制器使用仿真位姿与接触真值，数据来源为 `program_generated` / `simulation`。这是指定场景的程序示教，不代表已训练策略或真机测试，也未验证全部 400 场景的泛化。

## 本次服务器产物

项目：`/home/michelle/haoze/humanvideo/r1pro-tray-server`

数据根目录：`runs/airport_arrangement_demo_20260916`

最终执行：`attempt_04/`。`attempt_03/` 完成了任务，但导航姿态的腕部黑帧未通过训练数据质检，已保留作诊断。

- `trajectory.npz`：完整连续状态、双臂与底盘动作、接触证据、程序阶段。
- `acceptance.json`：任务物理验收、回放误差、手腕相机方向检查。
- `full_arrangement_720p.mp4`：完整 20 fps 回放，约 6 分 35 秒。
- `preview_4x_720p.mp4`：覆盖完整过程的 4 倍速预览。
- `initial.png`、`final.png`：初末状态。
- `episode.json`：原始 Episode 与官方 LeRobot 导出位置。

`episodes/` 保存唯一 raw HDF5，`annotations/`、`views/` 引用帧范围；技能视图不复制视频。三路训练视频保持 1280×720、20 fps、H.264 CRF 18。额外概览视频只用于查看效果。

## 再执行一次

```bash
cd /home/michelle/haoze/humanvideo/r1pro-tray-server
conda activate r1pro-tray
MUJOCO_GL=egl MUJOCO_EGL_DEVICE_ID=1 python scripts/airport_teleop_agent.py run-arrangement \
  --root runs/airport_arrangement_demo_20260916 \
  --scene-id airport_0009_20260925
```

每次新建 UUID 目录，不覆盖已有 Episode。当前完整任务策略只允许这个已验证场景。Quest 不参与控制，不需要佩戴或授权。

## 任务与验收

程序先抬臂避让、调整手腕朝前下方再导航；双臂抓取后退出桌腿区域，再转向搬运。后桌托盘先以单臂在桌上分开，避免夹带另一只。放置时先留出夹爪退出空间，再以单臂推移、反馈修正间隙。

成功要求三个目标槽位误差 ≤3 cm、长轴方向误差 ≤5°、倾角 ≤5°、相邻间隙 0～3 cm，松爪后稳定至少 1 秒，全程无落地或非法碰撞、穿透不超过既有 3 mm 接触容差。

本任务只规定矩形托盘长边方向，没有指定带标记的正面朝向。因此这个独立数据集合显式设置 `yaw_period_rad=pi`，180°掉头视为同一长轴方向；未修改旧场景默认的 360°方向判据，未放宽位置、间隙、倾角或稳定性标准。

底盘是仿真平面伺服驱动，不是真机轮组接口。双臂动作保持 16 维，底盘动作单独为 3 维。仿真世界位姿、接触和槽位信息属于 privileged state，不伪装成真机部署观察。
