package com.k230.camera

import android.content.Context
import android.net.ConnectivityManager
import android.net.Network
import android.net.NetworkCapabilities
import android.net.NetworkRequest
import android.os.Handler
import android.os.Looper
import java.net.InetSocketAddress
import java.util.concurrent.Executors
import java.util.concurrent.atomic.AtomicLong

class WifiRouteController(
    context: Context,
    private val onReady: (Network) -> Unit,
    private val onLost: () -> Unit,
    private val onStatus: (String) -> Unit,
) {
    private val connectivity =
        context.getSystemService(ConnectivityManager::class.java)
    private val mainHandler = Handler(Looper.getMainLooper())
    private val probeExecutor = Executors.newCachedThreadPool()
    private val probeGeneration = AtomicLong(0L)

    @Volatile
    private var started = false
    private var boundNetwork: Network? = null

    private val callback = object : ConnectivityManager.NetworkCallback() {
        override fun onAvailable(network: Network) {
            val ticket = probeGeneration.incrementAndGet()
            onStatusOnMain("发现 Wi-Fi，正在验证 K230 RTSP 端口…")
            probeExecutor.execute {
                val reachable = runCatching {
                    network.socketFactory.createSocket().use { socket ->
                        socket.connect(
                            InetSocketAddress(AppConfig.HOST, AppConfig.RTSP_PORT),
                            AppConfig.NETWORK_PROBE_TIMEOUT_MS,
                        )
                    }
                    true
                }.getOrDefault(false)

                mainHandler.post {
                    if (!started || ticket != probeGeneration.get() || !reachable) {
                        if (started && !reachable) {
                            onStatus(
                                "当前 Wi-Fi 不是可用的 ${AppConfig.WIFI_SSID_HINT}，请在系统设置中连接热点",
                            )
                        }
                        return@post
                    }
                    if (!connectivity.bindProcessToNetwork(network)) {
                        onStatus("Wi-Fi 可达，但进程网络绑定失败")
                        return@post
                    }
                    boundNetwork = network
                    onReady(network)
                }
            }
        }

        override fun onLost(network: Network) {
            mainHandler.post {
                if (network != boundNetwork) return@post
                probeGeneration.incrementAndGet()
                boundNetwork = null
                connectivity.bindProcessToNetwork(null)
                onLost()
            }
        }
    }

    fun start() {
        if (started) return
        started = true
        onStatus("等待系统连接 ${AppConfig.WIFI_SSID_HINT}（无需向 App 提供密码）")
        val request = NetworkRequest.Builder()
            .addTransportType(NetworkCapabilities.TRANSPORT_WIFI)
            .removeCapability(NetworkCapabilities.NET_CAPABILITY_INTERNET)
            .build()
        connectivity.registerNetworkCallback(request, callback)
    }

    fun retry() {
        probeGeneration.incrementAndGet()
        stop(unbind = true)
        start()
    }

    fun stop(unbind: Boolean = true) {
        if (started) {
            started = false
            runCatching { connectivity.unregisterNetworkCallback(callback) }
        }
        probeGeneration.incrementAndGet()
        boundNetwork = null
        if (unbind) connectivity.bindProcessToNetwork(null)
    }

    fun close() {
        stop(unbind = true)
        probeExecutor.shutdownNow()
    }

    private fun onStatusOnMain(message: String) {
        mainHandler.post { if (started) onStatus(message) }
    }
}
