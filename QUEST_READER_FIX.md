# Quest Reader 修复版

修复版包名 `org.humanvideo.questreader`，显示名 `Humanvideo Quest Reader`。
它与旧版 `com.rail.oculus.teleop` 并存，不覆盖旧签名或清除头显数据。

## 改动

- 修正 `Remotes.h` 左右手追踪条件；未追踪的手不输出矩阵。
- 使用 Meta SDK 默认 Touch 手柄绑定，移除原有 Touch Pro 震动演示及其额外扩展。
- 打包 SDK 默认 `efigs.fnt` 和 `efigs_sdf.ktx`，使用英文界面，避免 SDK 的其他语言资源引用未提供的字体。
- 修改 SDK `TinyUI.cpp`，直接加载 `apk:///res/raw/efigs.fnt`；实际设备日志证实旧的 `apk://font` 系统路径加载失败，仅打包字体仍不够。
- 显示左右手追踪状态、EXIT 按钮；长按左手菜单键 2 秒请求正常退出。
- 支持 Android UI 线程上的限时诊断退出；电脑端诊断结束后另行确认应用停止。
- 桥接默认使用新包名，拒绝追踪标志不匹配和单位矩阵占位数据。

这仍是手柄输入应用，没有机器人视频回传。坐标沿用原版相对头部坐标，尚未验证与仿真坐标及真机控制的标定。

## 构建

工具位于本机 Conda `quest-build` 环境中，JDK 17、Gradle 8.5、Android SDK 32、NDK 27.0.12077973、CMake 3.22.1。
源码位于 `third_party/oculus_reader_fixed`，基于上游 `9689484d319c4798e54d59509b192436647b7427`。
Meta SDK v85 固定为 `bbed2f20e38a5df7113630771c83cb8279e4fc26`。
SDK 本地另有上述字体 URI 补丁，重建时必须保留。

```powershell
cd C:\Users\HAOZEW\Documents\code\humanvideo
.\scripts\build_quest_reader.ps1
```

## 限时检查

先保持头显唤醒、双手柄可见。以下命令只读取头显，不控制仿真或机器人；输出目录必须是新目录。

```powershell
.\.venv-r1pro\Scripts\python.exe scripts/check_quest_reader.py --adb deliverables/quest3/adb/platform-tools/adb.exe --serial 2G97C5ZH5D006C --seconds 20 --output runs/quest_fixed_check_manual
```

如需从电脑退出：

```powershell
& .\deliverables\quest3\adb\platform-tools\adb.exe -s 2G97C5ZH5D006C shell am force-stop org.humanvideo.questreader
```

安装成功、构建成功和收到日志都不等于可遥操。需分别确认：界面正常、左右手数据随移动变化、按键读数变化、退出正常，然后才验证仿真中的坐标映射。

## 2026-09-15 验证结果

- APK 编译、v2 签名校验、Quest 3 安装通过。
- 字体路径已在设备日志验证：`FontInfoType load SUCCESS`。
- 20 秒诊断自动退出，随后确认应用进程已停止。
- 第四次测试：收到 1395 组有效数据，每组均有左右手；左右手位置均变化，左扳机范围 0–1，右扳机范围 0–0.99243。OpenXR 会话进入 FOCUSED，诊断后应用已停止。
- 佩戴者已确认文字、手柄和 EXIT 按钮正常可见，且限时测试结束后自动返回 Quest 主页。Android screencap 捕获全黑图不反映这次佩戴者实际所见。点击 EXIT 和长按菜单键这两种退出方式尚未单独验收。
- 证据：`runs/quest_fixed_check_20260915_04/`。诊断会在启动前检查头显是否唤醒。
