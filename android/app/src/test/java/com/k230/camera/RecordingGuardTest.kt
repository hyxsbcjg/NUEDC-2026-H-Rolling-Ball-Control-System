package com.k230.camera

import org.junit.Assert.assertEquals
import org.junit.Assert.assertFalse
import org.junit.Assert.assertTrue
import org.junit.Rule
import org.junit.Test
import org.junit.rules.TemporaryFolder

class RecordingGuardTest {
    @get:Rule
    val temporaryFolder = TemporaryFolder()

    @Test
    fun rejectsPathOutsideRecordingRoot() {
        val root = temporaryFolder.newFolder("root")
        val outside = temporaryFolder.newFile("outside.mp4").apply {
            writeBytes(mp4Header())
        }

        val result = RecordingGuard.validateBasic(
            root,
            outside,
            emptyMap(),
            outside.lastModified(),
            emptySet(),
        )

        assertFalse(result.accepted)
        assertTrue(result.reason.contains("越出"))
    }

    @Test
    fun rejectsUnchangedBaselineAndConsumedCandidate() {
        val root = temporaryFolder.newFolder("root")
        val candidate = root.resolve("record.mp4").apply { writeBytes(mp4Header()) }
        val snapshot = RecordingGuard.snapshot(root)

        val unchanged = RecordingGuard.validateBasic(
            root,
            candidate,
            snapshot,
            candidate.lastModified(),
            emptySet(),
        )
        assertFalse(unchanged.accepted)

        candidate.appendBytes(byteArrayOf(1))
        candidate.setLastModified(System.currentTimeMillis())
        val consumedFingerprint = RecordingGuard.fingerprint(candidate)
        val consumed = RecordingGuard.validateBasic(
            root,
            candidate,
            snapshot,
            candidate.lastModified(),
            setOf(consumedFingerprint),
        )
        assertFalse(consumed.accepted)
        assertTrue(consumed.reason.contains("发布过"))
    }

    @Test
    fun acceptsNewNonEmptyChild() {
        val root = temporaryFolder.newFolder("root")
        val started = System.currentTimeMillis()
        val candidate = root.resolve("new.mp4").apply {
            writeBytes(mp4Header())
            setLastModified(started)
        }

        val result = RecordingGuard.validateBasic(
            root,
            candidate,
            emptyMap(),
            started,
            emptySet(),
        )

        assertTrue(result.accepted)
        assertTrue(result.fingerprint?.contains(candidate.canonicalPath) == true)
    }

    @Test
    fun sniffsSupportedHeadersWithoutTrustingExtension() {
        val fakeTs = temporaryFolder.newFile("fake.mp4").apply {
            val bytes = ByteArray(189)
            bytes[0] = 0x47
            bytes[188] = 0x47
            writeBytes(bytes)
        }
        val realMp4 = temporaryFolder.newFile("real.bin").apply {
            writeBytes(mp4Header())
        }

        assertEquals(ContainerKind.MPEG_TS, ContainerSniffer.detect(fakeTs))
        assertEquals(ContainerKind.MP4, ContainerSniffer.detect(realMp4))
    }

    private fun mp4Header(): ByteArray =
        byteArrayOf(
            0, 0, 0, 24,
            'f'.code.toByte(),
            't'.code.toByte(),
            'y'.code.toByte(),
            'p'.code.toByte(),
            'i'.code.toByte(),
            's'.code.toByte(),
            'o'.code.toByte(),
            'm'.code.toByte(),
        )
}
