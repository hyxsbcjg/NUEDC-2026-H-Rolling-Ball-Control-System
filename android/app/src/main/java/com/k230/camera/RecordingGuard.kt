package com.k230.camera

import java.io.File
import java.io.RandomAccessFile

data class FileSnapshot(
    val length: Long,
    val lastModified: Long,
)

data class CandidateValidation(
    val accepted: Boolean,
    val reason: String,
    val fingerprint: String? = null,
)

object RecordingGuard {
    fun snapshot(directory: File): Map<String, FileSnapshot> {
        directory.mkdirs()
        return directory.listFiles()
            .orEmpty()
            .filter { it.isFile }
            .associate { file ->
                file.canonicalPath to FileSnapshot(file.length(), file.lastModified())
            }
    }

    fun fingerprint(file: File): String =
        "${file.canonicalPath}|${file.length()}|${file.lastModified()}"

    fun validateBasic(
        root: File,
        candidate: File?,
        baseline: Map<String, FileSnapshot>,
        startedAtMs: Long,
        consumed: Set<String>,
    ): CandidateValidation {
        if (candidate == null) {
            return CandidateValidation(false, "LibVLC 未返回本次录像路径")
        }

        val rootPath = root.canonicalFile.path
        val file = candidate.canonicalFile
        val insideRoot = file.path.startsWith(rootPath + File.separator)
        if (!insideRoot) {
            return CandidateValidation(false, "返回路径越出应用临时目录")
        }
        if (!file.isFile) {
            return CandidateValidation(false, "录像文件不存在")
        }
        if (file.length() <= 0L) {
            return CandidateValidation(false, "录像文件为空")
        }

        val before = baseline[file.path]
        if (before != null &&
            before.length == file.length() &&
            before.lastModified == file.lastModified()
        ) {
            return CandidateValidation(false, "LibVLC 返回了开始前已存在的旧文件")
        }
        if (file.lastModified() < startedAtMs - 2_000L) {
            return CandidateValidation(false, "录像修改时间不属于本次会话")
        }

        val fingerprint = fingerprint(file)
        if (fingerprint in consumed) {
            return CandidateValidation(false, "该临时文件已经发布过")
        }
        return CandidateValidation(true, "基础检查通过", fingerprint)
    }

    fun hasStableSize(file: File, firstLength: Long): Boolean =
        file.isFile && firstLength > 0L && file.length() == firstLength
}

enum class ContainerKind {
    MP4,
    MPEG_TS,
    MATROSKA,
    MPEG_PS,
    UNKNOWN,
}

object ContainerSniffer {
    fun detect(file: File): ContainerKind {
        if (!file.isFile || file.length() < 8L) return ContainerKind.UNKNOWN

        val header = ByteArray(384)
        val count = RandomAccessFile(file, "r").use { it.read(header) }
        if (count >= 12 &&
            header[4] == 'f'.code.toByte() &&
            header[5] == 't'.code.toByte() &&
            header[6] == 'y'.code.toByte() &&
            header[7] == 'p'.code.toByte()
        ) {
            return ContainerKind.MP4
        }
        if (count >= 4 &&
            header[0] == 0x1A.toByte() &&
            header[1] == 0x45.toByte() &&
            header[2] == 0xDF.toByte() &&
            header[3] == 0xA3.toByte()
        ) {
            return ContainerKind.MATROSKA
        }
        if (count >= 4 &&
            header[0] == 0x00.toByte() &&
            header[1] == 0x00.toByte() &&
            header[2] == 0x01.toByte() &&
            header[3] == 0xBA.toByte()
        ) {
            return ContainerKind.MPEG_PS
        }
        if (count >= 189 && header[0] == 0x47.toByte() && header[188] == 0x47.toByte()) {
            return ContainerKind.MPEG_TS
        }
        return ContainerKind.UNKNOWN
    }
}
