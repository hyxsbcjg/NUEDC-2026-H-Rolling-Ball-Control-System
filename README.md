# 2026 电赛 H 题滚球控制系统

本仓库保存一套滚球控制系统的 TC264 控制端、K230 视觉端与 Android 图传客户端源文件。仓库不包含官方题面；使用者应自行从官方渠道获取比赛要求。

## 仓库结构

~~~text
.
├── tc264/  # TC264D 控制、菜单、通信与步进执行代码
├── k230/   # K230 CanMV 视觉识别、LCD、触摸、UART 与 RTSP 代码
└── android/ # K230 图传录像 Android 客户端
~~~

## 开发环境

- 控制器：Infineon AURIX TC264D
- TC264 基础库：逐飞科技开源库 V3.4.4
- TC264 工具链：AURIX Development Studio（ADS）中的 TASKING 工具链
- 视觉端：K230
- K230 固件：CanMV v1.8-0-gc2d1f5c
- Android 应用：K230 图传录像
- Android 包名：`com.k230.camera`
- Android 版本：1.0.2（`versionCode = 3`）
- Android 系统要求：Android 10 或更高版本（`minSdk = 29`）
- 播放组件：LibVLC Android 3.7.5

版本差异可能影响外设 API、工程配置和编译结果。请使用上述版本复现，再单独评估升级。

## 系统通信

### K230 与 TC264

两端使用 3.3 V TTL 串口，参数为 UART2/UART1、115200 baud、8N1。接线必须交叉并共地：

- K230 GPIO11（UART2 TX）→ TC264 P02.3（UART1 RX）
- K230 GPIO12（UART2 RX）← TC264 P02.2（UART1 TX）
- K230 GND ↔ TC264 GND

不要把 5 V 串口电平直接接入任一端。

固定帧长为 5 字节：

~~~text
40 Flag Data1 Data2 50
~~~

- `Flag = 0x00`：`Ball_X`
- `Flag = 0x0A`：`Touch_X`
- `Data1` 为高字节，`Data2` 为低字节，即 16 位大端数据
- 每次新的触摸事件把同一 `Touch_X` 帧连续发送 3 次

### TC264 与步进驱动

TC264 UART3 的 P15.6 TX 单向连接步进驱动 RX。该链路没有 ACK，也没有到位反馈；一次本地发送成功只能证明字节已交给串口发送路径，不能证明电机或机构已到达目标位置。

## K230 默认行为

`k230/main.py` 默认启用钢球识别、LCD 显示、UART2 发送和触摸输入。RTSP 默认关闭。

启用 RTSP 前，使用者必须在本地配置自己的 Wi-Fi AP 名称与强密码，并保持板端 IP、端口和会话名与 Android 应用一致：

~~~python
ENABLE_RTSP = True
AP_SSID = "CHANGE_ME"
AP_PASSWORD = "CHANGE_ME"
~~~

`CHANGE_ME` 仅为占位符。仓库不提供可用的实际 Wi-Fi 凭据；请勿把个人或现场凭据提交到公开仓库。配置后将 `k230/main.py` 复制为板端 `/sdcard/main.py` 并重启 K230。

## Android 图传与录像

Android 客户端名为“K230 图传录像”，要求 Android 10 或更高版本。默认地址为 `rtsp://192.168.169.1:8554/live`；若板端网络参数不同，修改 [`AppConfig.kt`](android/app/src/main/java/com/k230/camera/AppConfig.kt) 后重新构建和安装。`WIFI_SSID_HINT` 只控制界面提示，应用不会据此验证热点身份。

手机先通过系统 Wi-Fi 设置连接 K230 热点，并在“无互联网”提示下保持连接。应用不读取或保存热点密码，也不申请危险运行时权限。RTSP 流量为明文、未认证且未加密，只应在受控网络内使用。

### 下载 Debug Preview

Release tag：[`android-v1.0.2`](https://github.com/hyxsbcjg/NUEDC-2026-H-Ball-Control/releases/tag/android-v1.0.2)。以下 APK 是 Debug Preview，不是正式发布签名：

- [`K230Camera-v1.0.2-arm64-v8a-debug.apk`](https://github.com/hyxsbcjg/NUEDC-2026-H-Ball-Control/releases/download/android-v1.0.2/K230Camera-v1.0.2-arm64-v8a-debug.apk)：推荐多数现代手机
- [`K230Camera-v1.0.2-universal-debug.apk`](https://github.com/hyxsbcjg/NUEDC-2026-H-Ball-Control/releases/download/android-v1.0.2/K230Camera-v1.0.2-universal-debug.apk)：包含四种 ABI
- [`SHA256SUMS.txt`](https://github.com/hyxsbcjg/NUEDC-2026-H-Ball-Control/releases/download/android-v1.0.2/SHA256SUMS.txt)：安装前用于校验下载文件

客户端支持预览、开始/停止录像和中断事务恢复。开始录像要求应用私有存储至少有 1 GiB 可用空间；停止后通过校验的 MP4/H.264 文件发布到 `DCIM/K230`。录像调用被接受不等于文件已经安全保存，应在系统相册中核对最终文件。

基本使用步骤：

1. 在 K230 端配置自己的 AP 名称和强密码，启用 RTSP，部署脚本并重启。
2. 安装与手机 ABI 匹配的 APK；允许该安装来源，或使用 `adb install -r <apk>`。
3. 用 Android 系统 Wi-Fi 设置连接该热点，并在无互联网提示下保持连接。
4. 打开“K230 图传录像”，等待状态显示“预览正常”后再开始录像。
5. 结束时点“停止并保存”，再到系统相册的 `DCIM/K230` 核对文件。
6. 若出现 `RECOVERY_REQUIRED`/“恢复检查”，保留应用数据并点“恢复检查”。这表示应用无法安全确认事务已收尾，不等于录像必然丢失；应用会保留临时文件并拒绝开始新录像。

Android 项目说明与安全边界：

- [android/README.md](android/README.md)
- [SECURITY.md](android/SECURITY.md)
- [Android THIRD_PARTY_NOTICES.md](android/THIRD_PARTY_NOTICES.md)

应用署名：`bilibili@fuuu1688`。

## K230 CanMV 部署

1. 给 K230 烧录 CanMV v1.8-0-gc2d1f5c，并确认串口、LCD、触摸和摄像头硬件与脚本匹配。
2. 在 CanMV IDE 中连接开发板。
3. 将 `k230/main.py` 复制到开发板的 `/sdcard/main.py`。
4. RTSP 保持关闭时，无需填写 AP 凭据；需要 RTSP 时，先按上一节配置自己的凭据。
5. 重启开发板，观察串口启动日志、LCD 图像和触摸标记。
6. 用串口分析工具或示波器检查 GPIO11 上的 5 字节帧，再连接 TC264。

## TC264 ADS 构建

1. 用 AURIX Development Studio 打开 `tc264/` 工程。
2. 在 Project Explorer 中选中工程，按 `F5` 或执行 `Refresh`。
3. 执行 `Project > Clean...`，清理旧产物。
4. 执行 `Project > Build Project`。
5. 检查 Console 的错误与警告，并确认 ELF、HEX、Map 等产物的时间戳来自本次构建。
6. 烧录前复核引脚、供电、共地和步进机构安全边界。

## 当前验证边界

本次 Android 主机验证结果为：9/9 单元测试通过、Lint 0 errors/6 warnings、`assembleDebug` 成功。它们只证明当前主机上的源码检查与构建结果，不代表：

- 已完成一次新鲜的 ADS/TASKING Clean/Build；
- 已把固件烧录到 TC264D 或 K230；
- 已测得 UART1、UART2、UART3 的实际波形；
- 已验证步进驱动 ACK 或到位状态——该链路本身不提供这两类反馈；
- 已完成整机、赛场环境或正式比赛验证；
- 已在本次发布中完成 Android 手机与 K230 的整机预览、录像或恢复实测；
- 已取得任何特定 FPS 或实时性能保证。

请在目标硬件上分别验证视觉输出、完整控制周期、通信波形、机械限位和异常停机。

## 许可证

本仓库原创部分按 [GNU GPL v3 或更高版本](LICENSE)（SPDX：`GPL-3.0-or-later`）发布。逐飞库、Infineon iLLD、CanMV、Android 和 LibVLC 相关依赖保留各自的版权及许可条款，详见 [THIRD_PARTY_NOTICES.md](THIRD_PARTY_NOTICES.md)。
