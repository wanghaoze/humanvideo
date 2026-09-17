> 当前以 [移动底盘 agent 说明](AIRPORT_MOBILE_AGENT.md) 为准：全部由 agent 控制，无人工遥操环节。下文保留旧固定底盘基线说明。

# 机场托盘采集 agent：Prompt 2/3/4 基线

**当前能力边界：这是采集与验收 agent，不是已训练好的全任务整理策略。** 固定底盘无法导航；默认程序队列只做明确标记的短时诊断保持。只有提供与 scene_id、XML 哈希匹配的已验证关节计划时才执行动作。三托盘完整移动整理尚未实现，不能把诊断 Episode 当成成功示教。

## 服务器位置与启动

项目：`/home/michelle/haoze/humanvideo/r1pro-tray-server`

数据：`/home/michelle/haoze/humanvideo/r1pro-tray-server/runs/airport_collection_20260916`

```bash
cd /home/michelle/haoze/humanvideo/r1pro-tray-server
conda activate r1pro-tray
python scripts/control_airport_service.py start --scene-id airport_0001_20260917
python scripts/control_airport_service.py status
```

服务端口 8767，保留旧单托盘服务 8765。访问令牌沿用 `runs/server.token`，不要写入共享命令或日志。切换后台服务初始 scene_id 前先 stop 再 start；运行中可通过界面“下一场景”切换。

本机 Quest 已允许 USB 调试后，在项目根目录运行：

```powershell
powershell -ExecutionPolicy Bypass -File scripts/start_airport_quest.ps1
```

进入 VR → ENABLE → RECORD → 双手柄操作 → FINISH / ABORT。FINISH 只是操作者成功申请，必须通过集中物理标准才会成为成功。另有保存失败、暂停、重置、下一场景。底盘不可控提示显示在监视画面和 HUD。没有新鲜 Quest 输入时拒绝启动 human Episode。

## 调用采集 agent

```bash
# 只跑三个 smoke 场景的管线诊断，不生成成功整理示教
MUJOCO_GL=egl MUJOCO_EGL_DEVICE_ID=1 python scripts/airport_teleop_agent.py run-smoke \
  --root runs/airport_collection_20260916 --seconds 3

# 查看队列阻塞原因、质量统计和产物位置
python scripts/airport_teleop_agent.py status --root runs/airport_collection_20260916
```

也可使用 `run-queue --scene-ids ...`，一次只允许 1～3 个已注册 smoke 场景。不会启动正式 400 场景采集。

自定义程序动作使用 `--action-file plan.npz`，文件含 `scene_id`、`scene_sha256`、`action[N,16]`。必须是绝对关节目标，且与指定场景完全匹配；不能直接把旧单托盘轨迹套到随机三托盘布局。程序来源始终标为 program_generated，环境来源始终标为 simulation。当前不包含实机驱动。

新建独立数据根目录：

```bash
python scripts/airport_teleop_agent.py init \
  --source outputs/airport_trays_indoor_v1_1 --root runs/airport_collection_new
```

已存在的根目录会拒绝初始化。每次 Episode 使用新 UUID 目录，不覆盖旧数据。

## 数据与标注

- `scenes/`：场景 JSON/XML 快照与索引。
- `episodes/`：唯一原始 HDF5 Episode；同步 state/action/RGB、物理状态和接触证据。
- `annotations/<episode>/`：events、segments、outcome。
- `views/{adjust,pick,place,full_arrangement}.json`：只引用 Episode 与半开帧区间，不复制视频。
- `splits/`：smoke 按 scene_id 确定性 8/2/2；正式场景预留 300/50/50 计划。
- `lerobot/<episode>/`：官方 LeRobot 0.4.4 / v3 导出，16 维双臂状态和动作，三路 1280×720、20 fps、H.264 CRF 18。
- `quality/`：逐 Episode 报告、来源/技能/phase/难度/光照/front_count 数量和时长、泄漏与场景拒绝原因。
- `agent/`：队列状态、服务状态、日志；`diagnostics/`：回归和视频验证。

状态机只依据可观测接触、运动及释放证据标注；不能确定目标或 phase 时标 needs_review，目标槽位未知则为 null。仿真真值统一放在 `privileged/`，qpos/qvel/旧兼容 tray_pose 也明确声明为 privileged，不导出为策略部署观察。当前无 base action，`base_control_available=false`。

成功标准在 `r1pro_teleop/airport_annotations.py:Criteria` 集中定义：位置 ≤3 cm、yaw 环绕误差 ≤5°、倾角 ≤5°、相邻间隙 0～3 cm、稳定至少 1 秒（20 Hz 下至少 21 个连续采样点）、无落地/非法碰撞/超容差穿透，并松开夹爪。操作者声明成功但不满足标准会标 needs_review。

结束 Episode 后后台自动分段、验收、完整物理回放及导出。NaN、空白帧、错误尺寸/时序、大动作跳变或回放失败会阻止训练候选导出，并保留原始证据。失败但格式有效的数据可以导出作诊断，`training_candidate=false`；训练前必须按质量报告过滤。

## 本次验证与仍缺能力

- 三个 smoke 场景已完成 3 秒诊断 Episode：共 180 帧、9 路视频文件，全部 720p/20 fps，官方 loader 首末帧可读，回放 qpos 误差为 0，scene 泄漏为 0。
- 这些是 program_generated 的诊断保持片段，人工 Episode 数为 0，不是三个完成整理的示范。
- 另有真实 MuJoCo 单托盘抓取—下放—开爪 fixture，共 620 帧。检测到抓取接触、离桌超过 2 cm、桌面支撑恢复、开爪释放；没有将手工伪造状态当作物理测试。
- 单托盘初始姿态、不翻腕、720p、Quest 数据解析等 14 项既有测试及 4 项新管线测试通过。
- 缺少可控移动底盘、跨工位导航和已验证的三托盘自主整理策略。Quest 本轮 USB 仍未授权，因此尚未验证真实手柄到新采集器的人工 Episode；代码与服务器验证不等同于该硬件验收。
