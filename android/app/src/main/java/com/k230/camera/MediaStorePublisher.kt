package com.k230.camera

import android.content.ContentUris
import android.content.ContentValues
import android.content.Context
import android.net.Uri
import android.os.Environment
import android.provider.MediaStore
import org.json.JSONObject
import java.io.File
import java.text.SimpleDateFormat
import java.util.Date
import java.util.Locale

data class PublicationReceipt(
    val uri: Uri,
    val fingerprint: String,
    val sourcePath: String,
    val displayName: String,
)

class MediaStorePublisher(private val context: Context) {
    private data class DurableState(
        val phase: PublicationPhase,
        val displayName: String,
        val fingerprint: String,
        val sourcePath: String,
        val uri: Uri? = null,
        val verifiedPublished: Boolean = false,
    ) {
        fun receipt(): PublicationReceipt =
            PublicationReceipt(
                requireNotNull(uri) { "已发布事务缺少 MediaStore URI" },
                fingerprint,
                sourcePath,
                displayName,
            )
    }

    private data class MediaRow(
        val uri: Uri,
        val state: MediaRowState,
    )

    private val preferences =
        context.getSharedPreferences("media-publisher", Context.MODE_PRIVATE)
    private val resolver = context.contentResolver
    private val collection =
        MediaStore.Video.Media.getContentUri(MediaStore.VOLUME_EXTERNAL_PRIMARY)
    private val relativePath =
        "${Environment.DIRECTORY_DCIM}/${AppConfig.MEDIASTORE_DIRECTORY}"

    fun reconcile(): PublicationReceipt? = serialized {
        reconcileLocked()
    }

    fun hasUnfinishedTransaction(): Boolean = serialized {
        preferences.contains(KEY_STATE) ||
            preferences.contains(KEY_LEGACY_PENDING_URI)
    }

    private fun reconcileLocked(): PublicationReceipt? {
        val state = loadState()
        if (state == null) {
            reconcileLegacyPendingUri()
            return null
        }

        val row = findRow(state)
        val rowState = row?.state ?: MediaRowState.ABSENT
        return when (
            PublicationRecoveryPolicy.decide(
                state.phase,
                rowState,
                state.verifiedPublished,
            )
        ) {
            PublicationRecoveryAction.CLEAR_STATE -> {
                clearState()
                null
            }
            PublicationRecoveryAction.DELETE_PENDING -> {
                val pending = requireNotNull(row)
                val deleted = resolver.delete(pending.uri, null, null)
                check(deleted == 1) { "MediaStore 待发布记录清理数量为 $deleted" }
                clearState()
                null
            }
            PublicationRecoveryAction.RETURN_COMMITTED -> {
                val committed = state.copy(
                    phase = PublicationPhase.COMMITTED,
                    uri = row?.uri ?: state.uri,
                    verifiedPublished = true,
                )
                saveState(committed)
                committed.receipt()
            }
            PublicationRecoveryAction.RETRY_LATER ->
                error("无法确认上次 MediaStore 事务状态，已保留恢复记录")
        }
    }

    fun publishMp4(
        source: File,
        fingerprint: String,
        recordingId: String,
        startedAtMs: Long,
    ): PublicationReceipt = serialized {
        val canonicalSource = source.canonicalFile
        reconcileLocked()?.let { previous ->
            check(previous.fingerprint == fingerprint) {
                "仍有另一段已发布录像等待本地事务收尾"
            }
            check(File(previous.sourcePath).canonicalFile == canonicalSource) {
                "已发布事务的源文件与当前候选不一致"
            }
            return@serialized previous
        }

        val stamp =
            SimpleDateFormat("yyyyMMdd_HHmmss_SSS", Locale.US).format(Date(startedAtMs))
        val displayName = "K230_${stamp}_${recordingId}.mp4"
        var state = DurableState(
            PublicationPhase.PREPARING,
            displayName,
            fingerprint,
            canonicalSource.path,
        )
        saveState(state)

        try {
            val values = ContentValues().apply {
                put(MediaStore.Video.Media.DISPLAY_NAME, displayName)
                put(MediaStore.Video.Media.MIME_TYPE, "video/mp4")
                put(MediaStore.Video.Media.RELATIVE_PATH, relativePath)
                put(MediaStore.Video.Media.IS_PENDING, 1)
            }
            val uri = resolver.insert(collection, values)
                ?: error("MediaStore 无法创建待发布条目")
            state = state.copy(phase = PublicationPhase.PENDING, uri = uri)
            saveState(state)

            resolver.openOutputStream(uri, "w").use { output ->
                requireNotNull(output) { "MediaStore 无法打开输出流" }
                canonicalSource.inputStream().use { input -> input.copyTo(output) }
            }
            val published = resolver.update(
                uri,
                ContentValues().apply { put(MediaStore.Video.Media.IS_PENDING, 0) },
                null,
                null,
            )
            check(published == 1) { "MediaStore 发布门控更新数量为 $published" }
            val readback = queryUri(uri)
            check(readback?.state == MediaRowState.PUBLISHED) {
                "MediaStore 发布后未读回 IS_PENDING=0，已保留临时源文件"
            }

            state = state.copy(
                phase = PublicationPhase.COMMITTED,
                verifiedPublished = true,
            )
            saveState(state)
            return@serialized state.receipt()
        } catch (error: Throwable) {
            val recovered = runCatching { reconcileLocked() }.getOrNull()
            if (recovered != null &&
                recovered.fingerprint == fingerprint &&
                File(recovered.sourcePath).canonicalFile == canonicalSource
            ) {
                return@serialized recovered
            }
            throw error
        }
    }

    fun acknowledge(receipt: PublicationReceipt) = serialized {
        val state = loadState() ?: return@serialized
        check(state.phase == PublicationPhase.COMMITTED && state.verifiedPublished) {
            "不能确认未经实际读回的 MediaStore 事务"
        }
        check(state.uri == receipt.uri && state.fingerprint == receipt.fingerprint) {
            "MediaStore 确认凭据与持久化事务不一致"
        }
        clearState()
    }

    private fun findRow(state: DurableState): MediaRow? {
        state.uri?.let { uri ->
            queryUri(uri)?.let { return it }
        }

        val projection = arrayOf(
            MediaStore.Video.Media._ID,
            MediaStore.Video.Media.IS_PENDING,
        )
        val rows = mutableListOf<MediaRow>()
        resolver.query(
            collection,
            projection,
            "${MediaStore.Video.Media.DISPLAY_NAME} = ?",
            arrayOf(state.displayName),
            null,
        )?.use { cursor ->
            val idIndex = cursor.getColumnIndexOrThrow(MediaStore.Video.Media._ID)
            val pendingIndex =
                cursor.getColumnIndexOrThrow(MediaStore.Video.Media.IS_PENDING)
            while (cursor.moveToNext()) {
                val uri = ContentUris.withAppendedId(collection, cursor.getLong(idIndex))
                val pending =
                    if (cursor.getInt(pendingIndex) == 1) {
                        MediaRowState.PENDING
                    } else {
                        MediaRowState.PUBLISHED
                    }
                rows += MediaRow(uri, pending)
            }
        } ?: return MediaRow(
            state.uri ?: collection,
            MediaRowState.UNKNOWN,
        )

        check(rows.size <= 1) {
            "同一录像事务发现 ${rows.size} 条 MediaStore 记录，拒绝自动处理"
        }
        return rows.singleOrNull()
    }

    private fun queryUri(uri: Uri): MediaRow? {
        val projection = arrayOf(MediaStore.Video.Media.IS_PENDING)
        val cursor = resolver.query(uri, projection, null, null, null)
            ?: return MediaRow(uri, MediaRowState.UNKNOWN)
        cursor.use {
            if (!it.moveToFirst()) return null
            val pending =
                if (it.getInt(it.getColumnIndexOrThrow(MediaStore.Video.Media.IS_PENDING)) == 1) {
                    MediaRowState.PENDING
                } else {
                    MediaRowState.PUBLISHED
                }
            return MediaRow(uri, pending)
        }
    }

    private fun loadState(): DurableState? {
        val text = preferences.getString(KEY_STATE, null) ?: return null
        val json = JSONObject(text)
        return DurableState(
            PublicationPhase.valueOf(json.getString("phase")),
            json.getString("displayName"),
            json.getString("fingerprint"),
            json.getString("sourcePath"),
            json.optString("uri").takeIf { it.isNotEmpty() }?.let(Uri::parse),
            json.optBoolean("verifiedPublished", false),
        )
    }

    private fun saveState(state: DurableState) {
        val json = JSONObject()
            .put("phase", state.phase.name)
            .put("displayName", state.displayName)
            .put("fingerprint", state.fingerprint)
            .put("sourcePath", state.sourcePath)
            .put("verifiedPublished", state.verifiedPublished)
        state.uri?.let { json.put("uri", it.toString()) }
        check(preferences.edit().putString(KEY_STATE, json.toString()).commit()) {
            "无法持久化 MediaStore 发布事务"
        }
    }

    private fun clearState() {
        check(preferences.edit().remove(KEY_STATE).commit()) {
            "无法清除 MediaStore 发布事务"
        }
    }

    private fun reconcileLegacyPendingUri() {
        val text = preferences.getString(KEY_LEGACY_PENDING_URI, null) ?: return
        val uri = Uri.parse(text)
        val row = queryUri(uri)
        when (row?.state ?: MediaRowState.ABSENT) {
            MediaRowState.PENDING -> {
                val deleted = resolver.delete(uri, null, null)
                check(deleted == 1) { "旧版 MediaStore 待发布记录清理数量为 $deleted" }
            }
            MediaRowState.PUBLISHED ->
                error(
                    "检测到旧版录像已经公开到相册；为避免重复发布，" +
                        "已保留旧版事务标记，请先人工核对",
                )
            MediaRowState.ABSENT -> Unit
            MediaRowState.UNKNOWN ->
                error("无法确认旧版 MediaStore 事务状态，已保留恢复记录")
        }
        check(preferences.edit().remove(KEY_LEGACY_PENDING_URI).commit()) {
            "无法清除旧版 MediaStore 事务记录"
        }
    }

    companion object {
        private val PROCESS_LOCK = Any()

        internal fun <T> serialized(block: () -> T): T =
            synchronized(PROCESS_LOCK) { block() }

        const val KEY_STATE = "publication_state"
        const val KEY_LEGACY_PENDING_URI = "pending_uri"
    }
}
