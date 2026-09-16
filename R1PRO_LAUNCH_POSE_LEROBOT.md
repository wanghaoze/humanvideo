# R1 Pro：按指定启动文件初始化的 LeRobot 抬盘数据

服务器已部署初始姿态：双臂 [0,0,0,-1.62,0,0,0]；躯干 [0.2,-0.5,-0.72,0]。躯干固定在此姿态，底座仍为 X=0.5 m，动作和状态各 16 维，包含双臂关节及夹爪开度。源文件为用户提供的 vr_teleoperation_whole_body_launch.py，哈希保存在 scene/scene_launch_pose.json。仅应用其 R1 Pro 初始关节数组，没有启动 ROS 真机节点。

## 数据

lerobot/ 是可由 LeRobotDataset 直接读取的 v3.0 数据集，生成工具为 lerobot==0.4.4。1 个 episode，456 帧，20 Hz，22.8 秒，包含 observation.state、action、observation.images.head、observation.images.left_wrist、observation.images.right_wrist。state/RGB 在 action[t] 执行前同步采样。来源明确为 program_generated + simulation，success_operator=false，不是人类 VR 示教。

服务器数据目录：
/home/michelle/haoze/humanvideo/r1pro-tray-server/runs/tray_lift_launch_pose/dataset/lerobot

服务器原始 HDF5 留存在相邻 raw/ 目录；下载包省略大体积原始 RGB HDF5，包含训练用 Parquet/视频、来源清单、质量报告和相机截图。

## 验证

服务器实际执行新轨迹并导出，LeRobot 官方加载器首末帧读取通过；抬起并保持验收通过。review_validation.json 验证全部 456 帧动作重放 qpos 误差为 0；在双手接触托盘的全部帧，两只腕部相机光轴沿机器人向桌子的 +X 方向。camera_previews/ 和 review.mp4 包含实际腕部图像，朝向托盘和桌面，并修正了夹持时的上下方向。

任务是夹起并保持，不包括放下或全身移动。整个过程为仿真。单个程序生成 episode 只能作为管线样例，未验证学习策略表现。

## 接触设置

空托盘质量 1.2 kg、指尖滑动摩擦 1.2、夹爪 kp=2000/kv=40、力上限 40 N 保留。场景使用 impratio=10 抑制软接触数值滑移。源场景资产依赖已有 outputs/r1pro_tray_scene/assets；下载包 scene/ 下的 XML 需放回该目录运行，不能单独从解压后的 scene/ 目录加载。

## 在已配置的服务器继续生成

```bash
cd /home/michelle/haoze/humanvideo/r1pro-tray-server
MUJOCO_GL=egl MUJOCO_EGL_DEVICE_ID=1 conda run --no-capture-output -n r1pro-tray python scripts/run_tray_lift_pipeline.py --output runs/tray_lift_new_001
```

必须选择未存在的输出目录。该入口执行规划、物理验收、同步记录、LeRobot 导出和回放校验；验收失败会停止，不输出成功标记。它运行独立仿真，不控制真机，也不向正在运行的 Quest 服务注入轨迹。
