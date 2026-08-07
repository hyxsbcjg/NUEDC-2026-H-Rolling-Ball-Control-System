package com.k230.camera

object AppConfig {
    const val HOST = "192.168.169.1"
    const val RTSP_PORT = 8554
    const val RTSP_URL = "rtsp://$HOST:$RTSP_PORT/live"
    const val WIFI_SSID_HINT = "K230-CAM"
    const val RTSP_NETWORK_CACHING_MS = 100
    const val RTSP_CLOCK_JITTER_MS = 0
    val RTSP_MEDIA_OPTIONS = listOf(
        ":rtsp-tcp",
        ":network-caching=$RTSP_NETWORK_CACHING_MS",
        ":clock-jitter=$RTSP_CLOCK_JITTER_MS",
    )
    const val PREFER_HARDWARE_DECODER = true
    const val FORCE_HARDWARE_DECODER = false

    const val NETWORK_PROBE_TIMEOUT_MS = 1_500
    const val RECORD_START_TIMEOUT_MS = 3_000L
    const val RECORD_STOP_TIMEOUT_MS = 5_000L
    const val FILE_STABILITY_DELAY_MS = 800L
    const val MINIMUM_FREE_BYTES = 1L shl 30

    const val TEMP_DIRECTORY = "recordings"
    const val MEDIASTORE_DIRECTORY = "K230"
}
