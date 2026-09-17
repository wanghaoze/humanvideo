# Agent 控制的移动底盘

当前采集不需要人工遥操或 Quest。底盘由 agent 控制 MuJoCo 平面关节驱动器，支持世界 X/Y 平移和 yaw 转向；保留双臂初始关节、托盘物理参数和三路 720p 相机。头部视角随底盘移动。

这是理想平面伺服底盘模型，不是 R1 Pro 真机轮组驱动。未接入 ROS 真机底盘。运动连续积分，禁止通过修改 qpos 瞬移完成任务。控制器限制速度，并在接触检查失败时停止任务。

## 服务器调用

```bash
cd /home/michelle/haoze/humanvideo/r1pro-tray-server
conda activate r1pro-tray
MUJOCO_GL=egl MUJOCO_EGL_DEVICE_ID=1 python scripts/airport_teleop_agent.py run-base-smoke \
  --root runs/airport_mobile_agent_v2_20260916
```

agent 自动在三个 smoke 场景中平移 10 cm、转向约 6.9°并返回，生成新 UUID Episode、视频、分段、LeRobot 和质量报告。重复执行不会覆盖旧 Episode。该命令是底盘诊断，不执行完整托盘整理，不启动正式 400 场景采集。

创建独立移动场景集合：

```bash
python scripts/airport_teleop_agent.py init --source outputs/airport_trays_indoor_v1_1 \
  --root runs/airport_mobile_new --mobile-base
```

场景原 XML 保留，移动版本保存为 `mobile.xml`，独立哈希和模型版本写入 scene JSON。split 仍按原 scene_id 分组。

## 数据字段

- `action[16]`、`observation.state[16]`：原双臂格式。
- `action.base[3]`：实际施加的平面关节目标；X/Y 为相对初始世界原点的位移，yaw 为相对初始朝向的角度，单位 m/m/rad。
- raw `base_state[3]`：世界 X/Y/yaw；raw `base_joint_state[3]`：平面关节位置。两者明确为 privileged 仿真真值，不导出为可部署观察。
- `base_control_available=true`，`controller_source=program_generated`，`environment_source=simulation`。

无底盘的旧数据保持 `base_control_available=false`，不会补造底盘动作。回放同时施加双臂与底盘动作，核对全部 qpos。

## 验收范围

底盘运动、三场景回归、同步记录与回放是本次验收目标。跨桌避障导航、随机姿态托盘抓取、携物移动与三个托盘完整整理还需要联调验收。底盘诊断 Episode 始终 `training_candidate=false`，不能计为任务成功示教。

`runs/airport_mobile_agent_20260916` 是已被替代的诊断原型，见其中 `SUPERSEDED.json`；使用 v2 数据根目录。
