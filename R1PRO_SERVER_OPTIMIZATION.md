# 服务器优化与下一步验收（2026-09-14）

部署目录：`/home/michelle/haoze/humanvideo/r1pro-tray-server`，Conda 环境 `r1pro-tray`，GPU 0。当前仍是仿真，尚无正式搬盘示教。

## 已补齐

- `r1pro_teleop.quality`：检查数据结构、相机空帧、时间一致性、输入过期、指令跳变，以及托盘抬起/到达/放稳/松夹爪的运动学条件。保留原始标签，不自动制造成功示教。
- `dataset --require-task-checks`：严格导出会拒绝质量检查不合格或任务检查未通过的成功标签。默认导出行为兼容旧版；训练前建议使用严格模式。
- `r1pro_teleop.replay`：从第一帧状态恢复，逐帧执行已记录的控制目标，生成物理回放 MP4 和状态误差报告。不会直接把每帧 qpos 设置为录制值来伪造一致性。
- 新录制补充求解器 warm-start 状态、场景目标坐标；兼容旧记录。
- 服务管理检查 HTTP/渲染心跳，启动失败会报错，修复快速重启时 TIME_WAIT 导致的端口误判。
- `workspace_check`：对预选抓取/放置点比较现有在线 IK、离线多初值 IK 和放松姿态后的结果。

## 最重要的发现：原工位距离不合适

原场景侧边抓取点距肩部约 1.08 m，超过模型约 0.92 m 的臂链长度上界。在固定底盘、固定躯干的前提下，这些点不可能直接到达。前沿点也未通过原在线 IK；不能把服务运行成功理解成已经能够搬盘。

新增 `outputs/r1pro_tray_scene/scene_workspace_candidate.xml`：底盘向前移动 0.35 m，目标横移距离改为 0.16 m。该场景通过静置、碰撞空腔、渲染与录制检查；双臂前沿的四个抓取/放置测试点通过离线多初值 IK 的粗验收（位置误差 <2 cm，姿态误差 <10°）。这只是可达性筛查，尚未验证运动路径无碰撞或夹持稳定性。原在线 IK 在该候选场景仍会失败，因此**没有替换默认服务场景**。

后续优先顺序：确定前沿夹持姿态 → 将离线解用于可靠的接近姿态/在线 IK 初始化 → 检查完整双臂路径碰撞 → 验证抬起、搬运、放下 → 再接 Quest 与采集训练示教。不能现在用接口测试轨迹训练“搬盘成功”策略。

## 在服务器直接运行

```bash
cd /home/michelle/haoze/humanvideo/r1pro-tray-server
conda activate r1pro-tray
python scripts/control_r1pro_service.py status

python -m r1pro_teleop.quality --raw runs/r1pro_raw --output runs/quality_latest.json

# --output 必须使用新目录；下面的原始记录属于测试轨迹。
bash scripts/run_r1pro_gpu.sh -m r1pro_teleop.replay \
  --raw runs/r1pro_raw/episode_20260911_172136_93557a78.h5 \
  --output runs/replay_review_$(date +%Y%m%d_%H%M%S)

python -m r1pro_teleop.workspace_check \
  --scene outputs/r1pro_tray_scene/scene_workspace_candidate.xml \
  --output runs/workspace_candidate_latest.json

# 只有录到真实成功的仿真示教并通过检查后再执行：
python -m r1pro_teleop.dataset --raw runs/r1pro_raw \
  --output datasets/tray_success_v1 --repo-id local/tray-success-v1 \
  --require-task-checks
```

默认搬盘判据：相对首帧抬起至少 4 cm、目标 XY 误差不超过 4 cm、底部高度误差不超过 2 cm、倾斜小于 15°、末尾至少 1 s 速度小于 3 cm/s、两夹爪开度均不少于 6 cm。阈值针对当前空托盘场景，是工程假设，并非真机标定值或接触证明。改变任务时应调整判据；新记录从场景读取目标，旧记录采用原目标 `[0.85,0.32,0.8]`。

验收结果在 `runs/quality_20260914.json`、`runs/replay_20260914_v1/`、`runs/workspace_multistart_20260914.json`、`runs/workspace_candidate_20260914.json`、`runs/http_acceptance_20260914.json`。升级前文件备份在 `runs/backups/pre_20260914.tgz`。
