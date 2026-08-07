# K230 图传录像 Android 客户端

本目录保存“K230 图传录像”的 Android Gradle 工程源码、Gradle Wrapper 和许可证文件。它不包含 APK、ZIP、`dist/`、`archive/`、签名密钥或其他签名产物。

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

## 功能

客户端连接 K230 热点后，通过 LibVLC 播放配置的 RTSP 流，并提供预览、开始录像、停止保存和未完成录像事务恢复。

录像调用被接受不代表文件已经保存。应用停止录像后等待 LibVLC 事件，核对实际文件，再通过 MediaStore 发布到系统相册。恢复逻辑使用 `PREPARING → PENDING → COMMITTED` 状态处理被中断的发布事务；已经确认公开到相册的文件不由恢复流程删除。

## RTSP 与热点配置

默认 RTSP 地址在 [`AppConfig.kt`](app/src/main/java/com/k230/camera/AppConfig.kt) 中配置：

~~~text
rtsp://192.168.169.1:8554/live
~~~

该 RTSP 流未加密，也不提供应用层认证，只应在受控网络中使用。详细限制见 [`SECURITY.md`](SECURITY.md)。

K230 的热点和 RTSP 开关位于 [`../k230/main.py`](../k230/main.py)。源码默认关闭 RTSP，且不包含可用热点密码。下面的 `CHANGE_ME` 只是密码占位符：

~~~python
ENABLE_RTSP = True
AP_PASSWORD = "CHANGE_ME"
~~~

启用 RTSP 前，必须把热点名称和 `AP_PASSWORD` 改为自己的配置。不要把真实密码提交到仓库。Android 应用不读取或保存热点密码；请通过系统 Wi-Fi 设置连接热点。

## 构建

准备 JDK 17 和 Android SDK 后，在 `android/` 目录执行：

~~~powershell
.\gradlew.bat testDebugUnitTest lintDebug
.\gradlew.bat assembleDebug
~~~

`assembleDebug` 的输出位于 `app/build/outputs/apk/`。构建产物不会纳入源码仓库。

这些命令是本项目的检查与构建入口；本轮文档修正没有运行测试、Lint 或 APK 构建，因此不声称它们已经通过。

## 使用

1. 在 [`../k230/main.py`](../k230/main.py) 中配置自己的热点名称和密码，并启用 RTSP。
2. 将 K230 脚本部署为板端 `/sdcard/main.py`，重启后检查启动日志。
3. 构建并安装 Android 调试 APK。
4. 在 Android 系统 Wi-Fi 设置中连接 K230 热点，并在“无互联网”提示下保持连接。
5. 打开“K230 图传录像”，等待预览正常。
6. 开始录像；结束时执行“停止并保存”，再到系统相册核对文件。
7. 若应用提示恢复检查，先保留应用数据并按提示核对事务和录像文件。

不要在未确认相册文件完整前清除应用数据。清除应用数据会同时删除应用私有目录中的临时录像与恢复记录。

## 验证边界

本目录提供源码与构建入口，不提供现成 APK 或签名发布物。本轮只修正文档，没有完成：

- Android 单元测试或 Lint 的本轮执行；
- APK 的本轮构建、安装或签名核验；
- 手机连接 K230 热点后的预览、录像与恢复实测；
- K230 与手机的端到端延迟、稳定性或长时间录像验证。

源码检查或本机构建不能替代手机与 K230 整机证明。

## 许可证与安全

- [Android 第三方声明](THIRD_PARTY_NOTICES.md)
- [GUST 字体许可证](app/src/main/assets/licenses/GUST_FONT_LICENSE.txt)
- [安全说明](SECURITY.md)

第三方组件保留各自的版权和许可证。本目录没有重新许可 LibVLC、Android 工具链或字体资源。
