# K230 图传录像 Android 客户端

本目录保存“K230 图传录像”的 Android Gradle 工程源码、Gradle Wrapper 和许可证文件。APK 通过 GitHub Releases 提供，不纳入源码目录；仓库不包含发布签名密钥。

配套 K230 脚本位于 [`../k230/main.py`](../k230/main.py)。

## 项目元数据

- 应用名：K230 图传录像
- 包名：`com.k230.camera`
- 版本：1.0.2
- `versionCode`：3
- 最低系统：Android 10（`minSdk = 29`）
- 播放组件：LibVLC Android 3.7.5
- 作者：`bilibili@fuuu1688`

实际配置以 [`app/build.gradle.kts`](app/build.gradle.kts) 和源码为准。

## 下载与安装

Release tag：[`android-v1.0.2`](https://github.com/hyxsbcjg/NUEDC-2026-H-Ball-Control/releases/tag/android-v1.0.2)。这些 APK 是 **Debug Preview**，使用调试签名，不是正式发布签名：

- [`K230Camera-v1.0.2-arm64-v8a-debug.apk`](https://github.com/hyxsbcjg/NUEDC-2026-H-Ball-Control/releases/download/android-v1.0.2/K230Camera-v1.0.2-arm64-v8a-debug.apk)：推荐多数现代 Android 手机
- [`K230Camera-v1.0.2-universal-debug.apk`](https://github.com/hyxsbcjg/NUEDC-2026-H-Ball-Control/releases/download/android-v1.0.2/K230Camera-v1.0.2-universal-debug.apk)：包含 `arm64-v8a`、`armeabi-v7a`、`x86`、`x86_64` 四种 ABI
- [`SHA256SUMS.txt`](https://github.com/hyxsbcjg/NUEDC-2026-H-Ball-Control/releases/download/android-v1.0.2/SHA256SUMS.txt)：下载后校验 APK 完整性

Android 10 或更高版本可在系统设置中允许浏览器或文件管理器“安装未知应用”，然后打开 APK。也可启用 USB 调试后执行：

~~~powershell
adb install -r .\K230Camera-v1.0.2-arm64-v8a-debug.apk
~~~

应用只声明 `INTERNET` 和 `ACCESS_NETWORK_STATE`，不申请相机、麦克风、位置或传统外部存储等危险运行时权限。热点密码只在 Android 系统 Wi-Fi 设置中输入，应用不会读取或保存密码。

## 功能

客户端连接 K230 热点后，通过 LibVLC 播放配置的 RTSP 流，并提供预览、开始录像、停止保存和未完成录像事务恢复。

录像调用被接受不代表文件已经保存。应用停止录像后等待 LibVLC 事件，核对实际文件，再通过 MediaStore 发布到系统相册。恢复逻辑使用 `PREPARING → PENDING → COMMITTED` 状态处理被中断的发布事务；已经确认公开到相册的文件不由恢复流程删除。

## RTSP 与热点配置

默认 RTSP 地址在 [`AppConfig.kt`](app/src/main/java/com/k230/camera/AppConfig.kt) 中配置：

~~~text
rtsp://192.168.169.1:8554/live
~~~

如需修改地址，请编辑 [`AppConfig.kt`](app/src/main/java/com/k230/camera/AppConfig.kt) 中的 `HOST`、`RTSP_PORT` 或 `RTSP_URL`，再重新构建并安装 APK。板端的 `AP_IP`、`RTSP_PORT`、`RTSP_SESSION` 必须与之保持一致。`WIFI_SSID_HINT` 只控制状态提示，应用不会用它验证热点身份。

该 RTSP 流未加密，也不提供应用层认证，只应在受控网络中使用。详细限制见 [`SECURITY.md`](SECURITY.md)。

K230 的热点和 RTSP 开关位于 [`../k230/main.py`](../k230/main.py)。源码默认关闭 RTSP，且不包含可用热点密码。下面的 `CHANGE_ME` 只是占位符：

~~~python
ENABLE_RTSP = True
AP_SSID = "CHANGE_ME"
AP_PASSWORD = "CHANGE_ME"
~~~

启用 RTSP 前，必须把 `AP_SSID` 和 `AP_PASSWORD` 改为自己的私有名称和强密码。不要把真实凭据提交到仓库。

将配置后的 [`../k230/main.py`](../k230/main.py) 复制为板端 `/sdcard/main.py`，重启 K230 并检查启动日志。手机随后在 Android 系统 Wi-Fi 设置中连接该热点；若系统提示该网络“无互联网”，请选择保持连接。

## 构建

准备 JDK 17 和 Android SDK 后，在 `android/` 目录执行：

~~~powershell
.\gradlew.bat testDebugUnitTest lintDebug
.\gradlew.bat assembleDebug
~~~

`assembleDebug` 的输出位于 `app/build/outputs/apk/`。构建产物不会纳入源码仓库。

本次主机验证已完成：9/9 单元测试通过、Lint 0 errors/6 warnings、`assembleDebug` 成功。这些结果不包含 APK 安装、签名身份或真机功能验证。

## 预览与录像

1. 打开应用后，它会等待 Wi-Fi，并探测配置的 K230 主机和 RTSP 端口。
2. 端口验证通过后，应用建立 RTSP/TCP 连接。只有状态显示“预览正常”时，“开始录像”按钮才会启用。
3. 开始录像要求应用私有存储至少有 1 GiB 可用空间。按钮操作被接受后，应用仍会等待 LibVLC 的录像开始确认。
4. 结束时点“停止并保存”。应用等待文件封装完成，并检查文件边界、大小稳定性、MP4 容器、H.264 视频轨、关键帧、时间戳和有效时长；录像不得包含音轨。
5. 校验通过后，应用使用 MediaStore 发布到系统公共目录 `DCIM/K230`。文件名格式为 `K230_yyyyMMdd_HHmmss_SSS_<recordingId>.mp4`。
6. 只有界面报告发布成功，并且系统相册或文件管理器中能看到且播放最终文件，才算保存完成。

应用切到后台或 K230 Wi-Fi 断开时，会自动请求停止录像并尝试收尾。不要把“开始录像调用已接受”或“停止按钮已点击”当作文件已经保存。

## `RECOVERY_REQUIRED` 与恢复检查

`RECOVERY_REQUIRED` 表示应用无法安全确认录像或 MediaStore 发布事务已经完整收尾，不表示录像必然丢失。此时应用会：

- 保留应用私有目录中的临时文件和事务日志；
- 拒绝开始下一段录像，避免覆盖或重复发布；
- 显示“恢复检查”按钮。

应用启动时会先自动协调上次 MediaStore 事务。若仍需人工触发，请保留应用数据并点“恢复检查”。恢复流程只有在事务日志可读且能唯一确定一个本次候选文件时，才会重新验证和发布；候选为 0 个或多个、日志损坏、MediaStore 状态未知或发现重复记录时，应用会拒绝猜测和覆盖。

已经确认公开到相册的文件不会被恢复流程删除。未公开的 MediaStore pending 条目可以清理后从私有临时文件重新恢复。不要在确认相册文件完整前清除应用数据或卸载应用；这会删除私有临时录像、事务日志和恢复状态。

## 常见故障

- 一直等待热点：确认手机已连接 K230 Wi-Fi，并在“无互联网”提示下保持连接。
- 提示当前 Wi-Fi 不可用：确认 K230 已把 `ENABLE_RTSP` 设为 `True`，热点凭据已配置，板端脚本已部署并重启。
- 端口验证失败：核对板端与 App 的 IP、端口和 session；默认组合为 `192.168.169.1`、`8554`、`live`。
- 能连接热点但没有预览：检查 K230 启动日志、RTSP/VENC 初始化和手机到端口 8554 的可达性。
- RTSP 中断：应用按 1 秒、2 秒、5 秒自动重连；连续失败后修复网络并点“重新连接”。
- 无法开始录像：先等待“预览正常”，并确认应用私有存储至少有 1 GiB 可用空间、没有待恢复事务。
- 停止后进入恢复状态：保留应用数据和临时文件，点“恢复检查”；不要反复开始新录像或清除数据。

## 验证边界

Release 提供 Debug Preview APK，但不是正式发布签名。本次主机验证已完成 9/9 单元测试、Lint 0 errors/6 warnings 和 `assembleDebug`；尚未完成：

- Release 资产在目标手机上的下载安装和调试签名身份核验；
- 手机连接 K230 热点后的预览、录像与恢复实测；
- K230 与手机的端到端延迟、稳定性或长时间录像验证。

源码检查或本机构建不能替代手机与 K230 整机证明。

## 许可证与安全

- [Android 第三方声明](THIRD_PARTY_NOTICES.md)
- [GUST 字体许可证](app/src/main/assets/licenses/GUST_FONT_LICENSE.txt)
- [安全说明](SECURITY.md)

第三方组件保留各自的版权和许可证。本目录没有重新许可 LibVLC、Android 工具链或字体资源。
