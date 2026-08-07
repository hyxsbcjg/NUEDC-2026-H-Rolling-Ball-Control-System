package com.k230.camera

import org.junit.Assert.assertEquals
import org.junit.Test

class AppConfigTest {
    @Test
    fun locksStableLowLatencyRtspProfile() {
        assertEquals("rtsp://192.168.169.1:8554/live", AppConfig.RTSP_URL)
        assertEquals(100, AppConfig.RTSP_NETWORK_CACHING_MS)
        assertEquals(0, AppConfig.RTSP_CLOCK_JITTER_MS)
        assertEquals(
            listOf(
                ":rtsp-tcp",
                ":network-caching=100",
                ":clock-jitter=0",
            ),
            AppConfig.RTSP_MEDIA_OPTIONS,
        )
        assertEquals(true, AppConfig.PREFER_HARDWARE_DECODER)
        assertEquals(false, AppConfig.FORCE_HARDWARE_DECODER)
    }
}
