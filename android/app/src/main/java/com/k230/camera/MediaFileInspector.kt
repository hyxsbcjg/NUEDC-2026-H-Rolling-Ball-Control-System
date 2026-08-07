package com.k230.camera

import android.media.MediaExtractor
import android.media.MediaFormat
import java.io.File

data class MediaInspection(
    val accepted: Boolean,
    val reason: String,
    val durationUs: Long = 0L,
)

object MediaFileInspector {
    fun inspectExpectedMp4(file: File): MediaInspection {
        val container = ContainerSniffer.detect(file)
        if (container != ContainerKind.MP4) {
            return MediaInspection(
                false,
                "实际容器为 $container，不伪装成 MP4；临时文件已保留",
            )
        }

        val extractor = MediaExtractor()
        return try {
            extractor.setDataSource(file.absolutePath)
            var videoTrack = -1
            var durationUs = 0L
            var audioTracks = 0

            for (index in 0 until extractor.trackCount) {
                val format = extractor.getTrackFormat(index)
                val mime = format.getString(MediaFormat.KEY_MIME).orEmpty()
                if (mime.startsWith("audio/")) audioTracks += 1
                if (mime == "video/avc" && videoTrack < 0) {
                    videoTrack = index
                    if (format.containsKey(MediaFormat.KEY_DURATION)) {
                        durationUs = format.getLong(MediaFormat.KEY_DURATION)
                    }
                }
            }
            if (videoTrack < 0) {
                return MediaInspection(false, "MP4 内没有 H.264 视频轨")
            }
            if (audioTracks != 0) {
                return MediaInspection(false, "录像包含意外的音频轨")
            }

            extractor.selectTrack(videoTrack)
            var samples = 0
            var firstPts = -1L
            var previousPts = -1L
            var lastPts = -1L
            var hasKeyFrame = false
            while (true) {
                val pts = extractor.sampleTime
                if (pts < 0L) break
                if (firstPts < 0L) firstPts = pts
                if (previousPts > pts) {
                    return MediaInspection(false, "录像时间戳发生倒退")
                }
                previousPts = pts
                lastPts = pts
                if (extractor.sampleFlags and MediaExtractor.SAMPLE_FLAG_SYNC != 0) {
                    hasKeyFrame = true
                }
                samples += 1
                if (!extractor.advance()) break
            }

            if (samples < 2) return MediaInspection(false, "录像有效帧不足")
            if (!hasKeyFrame) return MediaInspection(false, "录像没有可定位的关键帧")
            if (durationUs <= 0L && firstPts >= 0L && lastPts > firstPts) {
                durationUs = lastPts - firstPts
            }
            if (durationUs <= 0L) return MediaInspection(false, "录像时长无效")

            MediaInspection(true, "MP4/H.264 检查通过", durationUs)
        } catch (error: Throwable) {
            MediaInspection(false, "媒体解析失败：${error.message ?: error.javaClass.simpleName}")
        } finally {
            extractor.release()
        }
    }
}
