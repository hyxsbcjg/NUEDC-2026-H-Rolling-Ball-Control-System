package com.k230.camera

import android.content.Context
import android.util.AtomicFile
import org.json.JSONObject

data class JournalEntry(
    val recordingId: String,
    val playerGeneration: Long,
    val startedAtMs: Long,
    val baseline: Map<String, FileSnapshot>,
)

class RecordingJournal(context: Context) {
    private val file = AtomicFile(context.filesDir.resolve("recording-transaction.json"))

    fun hasStoredTransaction(): Boolean {
        val base = file.baseFile
        val parent = requireNotNull(base.parentFile)
        return base.exists() ||
            parent.resolve("${base.name}.bak").exists() ||
            parent.resolve("${base.name}.new").exists()
    }

    fun begin(entry: JournalEntry) {
        val baselineJson = JSONObject()
        entry.baseline.forEach { (path, snapshot) ->
            baselineJson.put(
                path,
                JSONObject()
                    .put("length", snapshot.length)
                    .put("lastModified", snapshot.lastModified),
            )
        }
        val json = JSONObject()
            .put("recordingId", entry.recordingId)
            .put("playerGeneration", entry.playerGeneration)
            .put("startedAtMs", entry.startedAtMs)
            .put("baseline", baselineJson)

        val output = file.startWrite()
        try {
            output.write(json.toString().toByteArray(Charsets.UTF_8))
            output.fd.sync()
            file.finishWrite(output)
        } catch (error: Throwable) {
            file.failWrite(output)
            throw error
        }
    }

    fun load(): JournalEntry? {
        if (!file.baseFile.isFile) return null
        return runCatching {
            val json = file.openRead().bufferedReader(Charsets.UTF_8).use { it.readText() }
            val root = JSONObject(json)
            val baselineObject = root.getJSONObject("baseline")
            val baseline = buildMap {
                val keys = baselineObject.keys()
                while (keys.hasNext()) {
                    val path = keys.next()
                    val value = baselineObject.getJSONObject(path)
                    put(
                        path,
                        FileSnapshot(
                            value.getLong("length"),
                            value.getLong("lastModified"),
                        ),
                    )
                }
            }
            JournalEntry(
                root.getString("recordingId"),
                root.getLong("playerGeneration"),
                root.getLong("startedAtMs"),
                baseline,
            )
        }.getOrNull()
    }

    fun clear() {
        file.delete()
        val base = file.baseFile
        val parent = requireNotNull(base.parentFile)
        check(!base.exists()) { "录像事务日志删除失败" }
        check(!parent.resolve("${base.name}.bak").exists()) {
            "录像事务日志备份删除失败"
        }
        check(!parent.resolve("${base.name}.new").exists()) {
            "录像事务日志临时文件删除失败"
        }
    }
}
