# 2026 电赛 H 题：滚球控制系统

> TC264 控制、K230 视觉与 Android 图传录像客户端的软件工程。

## 项目简介

系统使用 K230 识别钢球位置和触摸目标，通过 UART 把坐标发送给 TC264；TC264 同时控制小车循迹与滚球步进机构。Android 客户端用于查看 K230 RTSP 画面并录像。仓库不包含官方题面，请从官方渠道获取比赛要求。

## 项目状态

- TC264：保留当前控制、菜单、通信和步进执行源码；主机静态检查不能替代 ADS 构建和板端验收。
- K230：包含 CanMV 视觉、LCD、触摸、UART 与可选 RTSP 功能。
- Android：9/9 单元测试通过，Lint `0 errors / 6 warnings`，`assembleDebug` 成功。
- APK：已在 GitHub Release 提供 Debug Preview。
- 整机：未完成当前版本的 TC264/K230 重新烧录、通信波形、机构和完整赛场验收。

## 仓库结构

```text
.
├── tc264/              # TC264D 控制、菜单、通信与步进执行代码
├── k230/               # K230 CanMV 视觉、LCD、触摸、UART 与 RTSP
├── android/            # K230 图传录像 Android 客户端
├── LICENSE
└── THIRD_PARTY_NOTICES.md
```

## 硬件与软件环境

- 控制器：Infineon AURIX TC264D
- TC264 基础库：逐飞科技开源库 V3.4.4
- TC264 工具链：AURIX Development Studio / TASKING
- 视觉端：K230，CanMV v1.8-0-gc2d1f5c
- Android：Android 10+，包名 `com.k230.camera`
- 客户端版本：1.0.2（`versionCode=3`）
- 播放组件：LibVLC Android 3.7.5

## 快速开始

### K230

1. 将 `k230/main.py` 复制到开发板 `/sdcard/main.py`。
2. 默认启用钢球识别、LCD、UART2 和触摸，RTSP 默认关闭。
3. 需要图传时，把 `ENABLE_RTSP` 改为 `True`，并填写自己的 Wi-Fi 名称和强密码；不要提交真实凭据。
4. 重启后先检查启动日志、LCD、触摸标记和 GPIO11 串口帧，再连接 TC264。

### TC264

1. 用 ADS 打开 `tc264/` 工程并执行 Refresh。
2. 依次执行 Clean 和 Build Project。
3. 确认 Console、ELF、HEX 和 Map 均来自本次构建。
4. 烧录前复核供电、共地、UART、步进机构限位和急停。

### Android

1. 在 [Release android-v1.0.2](https://github.com/hyxsbcjg/NUEDC-2026-H-Rolling-Ball-Control-System/releases/tag/android-v1.0.2) 下载 APK 和校验文件。
2. 推荐现代手机安装 `K230Camera-v1.0.2-arm64-v8a-debug.apk`；需要多 ABI 时使用 universal 版本。
3. 手机连接 K230 热点并保持“无互联网”连接。
4. 打开“K230 图传录像”，显示“预览正常”后再开始录像。
5. 停止后到 `DCIM/K230` 核对最终文件；出现 `RECOVERY_REQUIRED` 时保留应用数据并执行恢复检查。

应用署名：`bilibili@fuuu1688`。详细说明见 [Android README](android/README.md) 和 [安全说明](android/SECURITY.md)。

## 通信与接口

- K230 UART2 GPIO11 TX → TC264 UART1 P02.3 RX。
- K230 UART2 GPIO12 RX ← TC264 UART1 P02.2 TX。
- 参数：115200 baud、8N1、3.3 V TTL、必须共地。
- 帧格式：`40 Flag Data1 Data2 50`；`0x00` 为 `Ball_X`，`0x0A` 为 `Touch_X`，数据为 16 位大端。
- TC264 UART3 P15.6 单向连接步进驱动 RX；没有 ACK 或到位反馈。

完整帧格式、无有效识别时的行为和步进指令见[串口通信协议](tc264/code/K230_TC264_步进电机串口通信协议.md)。

## 验证记录

- Android：9/9 单元测试、Lint 和 Debug APK 构建已完成。
- 仓库已完成凭据占位符和发布文件检查。
- 尚未完成新鲜 ADS/TASKING Clean/Build、TC264/K230 烧录、UART 波形和整机实测。
- 尚未对任何 FPS、控制周期或赛场性能作保证。

## 已知问题

- RTSP 明文、无认证且未加密，只能用于受控网络。
- 步进链路无 ACK 和到位反馈，本地发送成功不代表机构到位。
- 录像调用成功不代表文件已安全保存，必须在系统相册复核。
- 视觉阈值、像素坐标映射、机械中心和控制参数必须按实物标定。

## 开源与许可

本仓库原创部分按 [GNU GPL v3 或更高版本](LICENSE)（SPDX：`GPL-3.0-or-later`）发布。逐飞库、Infineon iLLD、CanMV、Android 和 LibVLC 依赖保留各自版权及许可条款，详见 [第三方说明](THIRD_PARTY_NOTICES.md)。
