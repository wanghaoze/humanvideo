# R1 Pro 托盘遥操数据：分模块 Codex Prompts

按顺序发送。Codex 已继承当前会话上下文，不需要重新解释已有实现。

## Prompt 1：场景定义与生成

```text
基于当前 R1 Pro MuJoCo 场景，实现机场安检托盘任务的场景 schema 和程序化生成器。

要求：
- 三张桌子：前方两张相邻桌子组成履带，机器人后方一张桌子。
- 三只托盘，只允许 XY 和 yaw 随机，始终朝上；重心必须落在桌面有效区域，禁止落地。
- 定义唯一履带坐标系：+X 从履带左端到右端，+Y 沿短边指向桌内，+Z 向上。
- 用公式生成 3 个整理槽位；默认相邻托盘目标间隙 1.5 cm。
- 随机化机器人 base pose/yaw、前后桌距离 1～2 m、托盘位置/yaw、光照角度/强度及日夜模式。
- 前方履带初始托盘数量支持 0/1/2/3。
- 普通布局和堆叠/搭接复杂布局分目录；复杂布局仍要求每只托盘重心由桌面支撑。
- 保留现有机器人初始关节、托盘质量/摩擦、夹爪参数、720p 相机及不翻腕配置。
- 每个场景保存版本化 JSON、seed、scene_id、difficulty、split 和生成后的 MuJoCo XML。
- 同 seed 必须确定性复现。

先生成 12 个 smoke scenes：front_count 0/1/2/3 各 3 个，其中普通 10 个、复杂 2 个。不要生成正式 400 个场景。

自动拒绝：重心无支撑、落地、非法穿透、普通布局重叠、静置 2 秒滑落/倾覆、槽位越界、机器人初始碰撞。记录拒绝原因和重采样次数。

完成代码、必要测试、场景预览和一个命令行入口。不要改 vendor，不要覆盖已有场景和数据。完成后简要报告改动和命令。
```

## Prompt 2：遥操 Episode 与自动分段

```text
在当前 Recorder 和 Quest 仿真遥操链路上，实现机场托盘任务的完整 Episode 记录与自动分段。保持旧数据兼容，不复制视频来制作技能数据。

规范标签：
- 状态：SEARCH_TARGET, ALIGN_BASE, APPROACH_TRAY, ADJUST_TRAY, GRASP_TRAY, TRANSPORT_TRAY, PLACE_TRAY, VERIFY_PLACEMENT, RECOVER, TASK_COMPLETE
- 技能：navigate_to_tray, push_adjust_tray, grasp_lift_tray, carry_tray, place_release_tray, verify_or_recover
- 主类：adjust, pick, place
- 抓取 phase：pregrasp, approach, contact, close, load_transfer, lift, stabilize
- 放置 phase：transport, preplace, descend, support_contact, unload, open, retreat

原始 Episode 是唯一数据源。新增 episodes、segments、events manifest；segment 使用半开帧区间 [start_frame, end_frame)。每段至少保存 episode_id、scene_id、start/end、skill、phase、target_tray_id、target_slot_id、success、recovery、failure_reason、controller_source、annotation_source/version。

利用仿真真值自动检测：夹爪接触、双手闭合、离桌超过 2 cm、重新接触桌面、开爪脱离、桌面推动/旋转、进入槽位并稳定 1 秒、接触丢失、掉落、重抓及人工中止。不能可靠判断时标记 needs_review，不能猜标签。

保留完整整理轨迹，同时生成 adjust/pick/place/full_arrangement 四个 manifest 视图，只引用原 Episode 帧范围。

当前没有真实底盘控制时明确记录 base_control_available=false，不得生成虚假 base action。仿真托盘 pose、接触和目标槽位标为 privileged state。

使用现有程序轨迹验证抓取/抬升边界，再增加一个最小程序放置 fixture 验证放置边界。完成实现、必要测试和一个端到端示例，不要声称程序轨迹是人类遥操。
```

## Prompt 3：LeRobot 数据组织、切分与质量检查

```text
把当前机场托盘 raw Episode、segments 和 events 接入现有 LeRobot 0.4.4 数据管线。

要求：
- 继续使用官方 LeRobot API；保留当前 16 维双臂状态/动作兼容性。
- head、left_wrist、right_wrist 均为原生 1280x720、20 fps、H.264 CRF 18。
- 状态、动作、图像、时间戳严格同步；官方 loader 能读取首末帧。
- segment manifest 仅引用 Episode 帧范围，不复制视频。
- 数据来源严格区分 human_teleoperation、program_generated、simulation、real_robot。
- privileged state 不能伪装成部署时可用观察。
- train/val/test 必须按 scene_id 分组，禁止同一 scene 的 Episode 或 segment 跨 split。
- smoke set 使用确定性 8/2/2 split；为未来 400 场景保留 300/50/50 配置。

输出结构至少包含 scenes、episodes、annotations、views、splits 和 LeRobot 数据。

质量报告至少统计：各 skill/phase/difficulty/light/front_count 的数量和时长；成功/恢复/失败/needs_review；分辨率、帧率、空白帧、NaN、时间戳、动作跳变；scene 泄漏；无效场景原因；human 与 program_generated 分开统计。

成功放置标准集中配置：位置误差 <=3 cm、yaw 误差 <=5°（正确处理角度环绕）、倾角 <=5°、相邻间隙 0～3 cm、稳定 >=1 秒、无落地/穿透/非法碰撞。

完成代码、必要测试、一个 smoke Episode 的 LeRobot 导出和质量报告。不要覆盖已有数据。
```

## Prompt 4：贯通遥操采集基线

```text
把已经完成的三桌三托盘场景、Quest 遥操 Recorder、自动 segment、LeRobot 导出和质量检查贯通为可操作的数据采集基线。

提供一个命令启动指定 scene_id 的 Quest 仿真遥操；支持开始/结束 Episode、标记成功/失败/中止、重置同一场景、切换下一个场景。结束后自动生成 raw Episode、events、segments、adjust/pick/place/full manifests、LeRobot 数据和质量报告。

先只对 3 个 smoke scenes 做端到端验证，不启动正式 400 场景采集。验证：
- 初始场景与 JSON/seed 一致；
- 完整 Episode 可回放；
- 自动分段帧范围有效；
- manifest 没有复制视频；
- LeRobot loader 可读；
- controller_source 标记准确；
- 现有单托盘直接抓取、不翻腕和 720p 回归测试继续通过。

如果底盘仍不可控，遥操界面和数据中明确提示，不要伪造完整移动机器人能力。最后给出实际可执行命令、输出目录和仍缺少的能力，保持简短。
```
