package com.k230.camera

import android.app.Activity
import android.net.Uri
import android.os.Bundle
import android.os.Handler
import android.os.Looper
import android.os.StatFs
import android.view.View
import android.widget.Button
import android.widget.TextView
import org.videolan.libvlc.Dialog
import org.videolan.libvlc.LibVLC
import org.videolan.libvlc.Media
import org.videolan.libvlc.MediaPlayer
import org.videolan.libvlc.util.VLCVideoLayout
import org.json.JSONArray
import java.io.File
import java.util.UUID
import java.util.concurrent.Executors

class MainActivity : Activity() {
    private enum class PlaybackState {
        WAITING_NETWORK,
        CONNECTING,
        PREVIEWING,
        ERROR,
    }

    private enum class RecordingState {
        IDLE,
        STARTING,
        RECORDING,
        STOPPING,
        PUBLISHING,
        RECOVERY_REQUIRED,
        ERROR,
    }

    private data class ActiveRecording(
        val id: String,
        val playerGeneration: Long,
        val startedAtMs: Long,
        val baseline: Map<String, FileSnapshot>,
    )

    private data class PublishOutcome(
        val success: Boolean,
        val message: String,
        val destination: Uri? = null,
    )

    private lateinit var videoLayout: VLCVideoLayout
    private lateinit var statusText: TextView
    private lateinit var startButton: Button
    private lateinit var stopButton: Button
    private lateinit var retryButton: Button
    private lateinit var recoverButton: Button

    private lateinit var routeController: WifiRouteController
    private lateinit var journal: RecordingJournal
    private lateinit var publisher: MediaStorePublisher
    private lateinit var temporaryDirectory: File

    private val mainHandler = Handler(Looper.getMainLooper())
    private val worker = Executors.newSingleThreadExecutor()
    private val consumedPreferences by lazy {
        getSharedPreferences("consumed-recordings", MODE_PRIVATE)
    }

    private var libVLC: LibVLC? = null
    private var mediaPlayer: MediaPlayer? = null
    private var playerGeneration = 0L
    private var playbackState = PlaybackState.WAITING_NETWORK
    private var recordingState = RecordingState.IDLE
    private var activeRecording: ActiveRecording? = null
    private var releaseAfterStop = false
    private var candidateProcessing = false
    private var reconnectAttempt = 0
    private var reconnectScheduledForGeneration: Long? = null
    private var destroyRequested = false

    private val vlcDialogs = object : Dialog.Callbacks {
        override fun onDisplay(dialog: Dialog.ErrorMessage) {
            mainHandler.post {
                val message = "LibVLC 写入错误：${dialog.text}"
                setStatus(message)
                if (isRecordingActive()) {
                    requestStopRecording(message, releasePlayerAfterwards = false)
                }
            }
        }

        override fun onDisplay(dialog: Dialog.LoginDialog) {
            dialog.dismiss()
        }

        override fun onDisplay(dialog: Dialog.QuestionDialog) {
            dialog.dismiss()
        }

        override fun onDisplay(dialog: Dialog.ProgressDialog) {
            if (dialog.isCancelable) dialog.dismiss()
        }

        override fun onCanceled(dialog: Dialog) = Unit

        override fun onProgressUpdate(dialog: Dialog.ProgressDialog) = Unit
    }

    override fun onCreate(savedInstanceState: Bundle?) {
        super.onCreate(savedInstanceState)
        setContentView(R.layout.activity_main)

        videoLayout = findViewById(R.id.video_layout)
        statusText = findViewById(R.id.status_text)
        startButton = findViewById(R.id.start_button)
        stopButton = findViewById(R.id.stop_button)
        retryButton = findViewById(R.id.retry_button)
        recoverButton = findViewById(R.id.recover_button)

        temporaryDirectory = filesDir.resolve(AppConfig.TEMP_DIRECTORY).apply { mkdirs() }
        journal = RecordingJournal(this)
        publisher = MediaStorePublisher(this)
        routeController = WifiRouteController(
            context = this,
            onReady = {
                playbackState = PlaybackState.CONNECTING
                setStatus("K230 端口验证通过，正在建立 RTSP/TCP 预览…")
                createAndStartPlayer()
            },
            onLost = {
                playbackState = PlaybackState.WAITING_NETWORK
                setStatus("K230 Wi-Fi 已断开；临时录像会先尝试收尾")
                if (isRecordingActive()) {
                    requestStopRecording("网络断开", releasePlayerAfterwards = true)
                } else {
                    releasePlayer()
                }
            },
            onStatus = { setStatus(it) },
        )

        startButton.setOnClickListener { startRecording() }
        stopButton.setOnClickListener {
            requestStopRecording("用户停止录像", releasePlayerAfterwards = false)
        }
        retryButton.setOnClickListener {
            if (isRecordingActive()) return@setOnClickListener
            releasePlayer()
            if (recordingState == RecordingState.ERROR) {
                recordingState = RecordingState.IDLE
            }
            reconnectAttempt = 0
            reconnectScheduledForGeneration = null
            routeController.retry()
            updateControls()
        }
        recoverButton.setOnClickListener { recoverInterruptedRecording() }

        resumePublicationTransaction()
        updateControls()
    }

    override fun onStart() {
        super.onStart()
        routeController.start()
    }

    override fun onStop() {
        if (isRecordingActive()) {
            requestStopRecording("应用进入后台", releasePlayerAfterwards = true)
        } else {
            releasePlayer()
        }
        routeController.stop(unbind = true)
        super.onStop()
    }

    override fun onDestroy() {
        destroyRequested = true
        routeController.close()
        if (!isRecordingActive() && !candidateProcessing) {
            disposeController()
        }
        super.onDestroy()
    }

    private fun createAndStartPlayer() {
        if (isRecordingActive()) return
        releasePlayer()
        val generation = ++playerGeneration
        val engine = LibVLC(applicationContext, arrayListOf("--no-input-record-native"))
        Dialog.setCallbacks(engine, vlcDialogs)
        val player = MediaPlayer(engine)
        player.attachViews(videoLayout, null, false, false)
        player.setEventListener { event ->
            mainHandler.post { handlePlayerEvent(generation, event) }
        }

        val media = Media(engine, Uri.parse(AppConfig.RTSP_URL)).apply {
            AppConfig.RTSP_MEDIA_OPTIONS.forEach { addOption(it) }
            setHWDecoderEnabled(
                AppConfig.PREFER_HARDWARE_DECODER,
                AppConfig.FORCE_HARDWARE_DECODER,
            )
        }
        player.media = media
        media.release()

        libVLC = engine
        mediaPlayer = player
        playbackState = PlaybackState.CONNECTING
        player.play()
        updateControls()
    }

    private fun handlePlayerEvent(generation: Long, event: MediaPlayer.Event) {
        if (generation != playerGeneration) return
        when (event.type) {
            MediaPlayer.Event.Opening -> setStatus("RTSP/TCP 正在连接…")
            MediaPlayer.Event.Playing -> {
                reconnectAttempt = 0
                reconnectScheduledForGeneration = null
                playbackState = PlaybackState.PREVIEWING
                setStatus("预览正常：${AppConfig.RTSP_URL}")
            }
            MediaPlayer.Event.RecordChanged -> {
                handleRecordChanged(generation, event.recording, event.recordPath)
            }
            MediaPlayer.Event.EncounteredError -> {
                if (isRecordingActive()) {
                    requestStopRecording(
                        "播放链路错误，正在保全临时录像",
                        releasePlayerAfterwards = true,
                    )
                } else {
                    playbackState = PlaybackState.ERROR
                    schedulePlayerReconnect()
                }
            }
            MediaPlayer.Event.EndReached,
            MediaPlayer.Event.Stopped,
            -> {
                if (!isRecordingActive() && playbackState != PlaybackState.WAITING_NETWORK) {
                    playbackState = PlaybackState.ERROR
                    schedulePlayerReconnect()
                }
            }
        }
        updateControls()
    }

    private fun startRecording() {
        if (playbackState != PlaybackState.PREVIEWING ||
            recordingState != RecordingState.IDLE
        ) {
            return
        }
        val player = mediaPlayer ?: return
        val availableBytes = StatFs(temporaryDirectory.absolutePath).availableBytes
        if (availableBytes < AppConfig.MINIMUM_FREE_BYTES) {
            recordingState = RecordingState.ERROR
            setStatus("可用空间不足 1 GiB，拒绝开始录像")
            updateControls()
            return
        }
        val session = try {
            MediaStorePublisher.serialized {
                val hasJournal = journal.hasStoredTransaction()
                val hasPublication = publisher.hasUnfinishedTransaction()
                check(!hasJournal && !hasPublication) {
                    "存在未完成录像或发布事务，请先执行恢复检查"
                }
                val created = ActiveRecording(
                    id = UUID.randomUUID().toString(),
                    playerGeneration = playerGeneration,
                    startedAtMs = System.currentTimeMillis(),
                    baseline = RecordingGuard.snapshot(temporaryDirectory),
                )
                journal.begin(
                    JournalEntry(
                        created.id,
                        created.playerGeneration,
                        created.startedAtMs,
                        created.baseline,
                    ),
                )
                created
            }
        } catch (error: Throwable) {
            recordingState = RecordingState.RECOVERY_REQUIRED
            recoverButton.visibility = View.VISIBLE
            setStatus("拒绝开始录像：${error.message}")
            updateControls()
            return
        }

        activeRecording = session
        recordingState = RecordingState.STARTING
        candidateProcessing = false
        releaseAfterStop = false
        setStatus("正在启动录像；等待 LibVLC 确认…")
        updateControls()

        val recordAccepted =
            runCatching { player.record(temporaryDirectory.absolutePath) }.getOrDefault(false)
        if (!recordAccepted) {
            activeRecording = null
            val cleared = runCatching {
                MediaStorePublisher.serialized { journal.clear() }
            }.isSuccess
            if (cleared) {
                recordingState = RecordingState.ERROR
                setStatus("LibVLC 拒绝启动录像；预览输入可能尚未就绪")
            } else {
                recordingState = RecordingState.RECOVERY_REQUIRED
                recoverButton.visibility = View.VISIBLE
                setStatus("LibVLC 拒绝启动录像，且事务日志清理失败；请执行恢复检查")
            }
            updateControls()
            return
        }

        mainHandler.postDelayed(
            {
                val current = activeRecording
                if (current?.id == session.id &&
                    current.playerGeneration == session.playerGeneration &&
                    recordingState == RecordingState.STARTING
                ) {
                    requestStopRecording(
                        "3 秒内未收到录像启动确认",
                        releasePlayerAfterwards = false,
                    )
                }
            },
            AppConfig.RECORD_START_TIMEOUT_MS,
        )
    }

    private fun requestStopRecording(
        reason: String,
        releasePlayerAfterwards: Boolean,
    ) {
        val session = activeRecording ?: return
        if (releasePlayerAfterwards) releaseAfterStop = true
        if (recordingState == RecordingState.STOPPING ||
            recordingState == RecordingState.PUBLISHING
        ) {
            return
        }
        if (recordingState != RecordingState.STARTING &&
            recordingState != RecordingState.RECORDING
        ) {
            return
        }

        recordingState = RecordingState.STOPPING
        setStatus("$reason；正在停止并等待文件封装完成…")
        updateControls()
        val accepted = runCatching { mediaPlayer?.record(null) == true }.getOrDefault(false)
        if (!accepted) {
            recordingState = RecordingState.RECOVERY_REQUIRED
            setStatus("LibVLC 拒绝停止控制；原始临时文件与事务日志已保留")
            recoverButton.visibility = View.VISIBLE
            if (releaseAfterStop) releasePlayer()
            disposeControllerIfRequested()
            updateControls()
            return
        }

        mainHandler.postDelayed(
            {
                val current = activeRecording
                if (current?.id == session.id &&
                    recordingState == RecordingState.STOPPING
                ) {
                    recordingState = RecordingState.RECOVERY_REQUIRED
                    setStatus("5 秒内未收到停止事件；保留临时文件，稍后点“恢复检查”")
                    recoverButton.visibility = View.VISIBLE
                    if (releaseAfterStop) releasePlayer()
                    disposeControllerIfRequested()
                    updateControls()
                }
            },
            AppConfig.RECORD_STOP_TIMEOUT_MS,
        )
    }

    private fun handleRecordChanged(
        generation: Long,
        isRecording: Boolean,
        path: String?,
    ) {
        val session = activeRecording ?: return
        if (generation != session.playerGeneration) return

        if (isRecording) {
            if (recordingState == RecordingState.STARTING) {
                recordingState = RecordingState.RECORDING
                setStatus("正在录像；停止后才会验证并发布到 DCIM/K230")
                updateControls()
            }
            return
        }

        // A false event received before this app requested stop can be a stale
        // global record-file event from LibVLC 3.7.5.  Never publish it.
        if (recordingState != RecordingState.STOPPING || candidateProcessing) return
        processCandidate(session, path?.let(::File))
    }

    private fun processCandidate(session: ActiveRecording, candidate: File?) {
        candidateProcessing = true
        recordingState = RecordingState.PUBLISHING
        setStatus("正在验证文件边界、稳定性、容器和 H.264 轨道…")
        updateControls()

        worker.execute {
            val outcome = runCatching {
                validateAndPublish(session, candidate)
            }.getOrElse { error ->
                PublishOutcome(
                    false,
                    "验证或发布流程异常：${error.message ?: error.javaClass.simpleName}",
                )
            }
            mainHandler.post {
                candidateProcessing = false
                if (activeRecording?.id != session.id) return@post

                if (outcome.success) {
                    activeRecording = null
                    recordingState = RecordingState.IDLE
                    recoverButton.visibility = View.GONE
                    setStatus(
                        "录像已发布到 DCIM/K230：${outcome.destination}\n${outcome.message}",
                    )
                } else {
                    recordingState = RecordingState.RECOVERY_REQUIRED
                    recoverButton.visibility = View.VISIBLE
                    setStatus("${outcome.message}\n临时文件未删除，可稍后恢复检查")
                }
                if (releaseAfterStop) {
                    releaseAfterStop = false
                    releasePlayer()
                }
                disposeControllerIfRequested()
                updateControls()
            }
        }
    }

    private fun validateAndPublish(
        session: ActiveRecording,
        candidate: File?,
    ): PublishOutcome {
        val initial = RecordingGuard.validateBasic(
            root = temporaryDirectory,
            candidate = candidate,
            baseline = session.baseline,
            startedAtMs = session.startedAtMs,
            consumed = consumedFingerprints(),
        )
        if (!initial.accepted || candidate == null) {
            return PublishOutcome(false, initial.reason)
        }

        val firstLength = candidate.length()
        Thread.sleep(AppConfig.FILE_STABILITY_DELAY_MS)
        if (!RecordingGuard.hasStableSize(candidate, firstLength)) {
            return PublishOutcome(false, "录像文件仍在变化，拒绝提前发布")
        }

        val inspection = MediaFileInspector.inspectExpectedMp4(candidate)
        if (!inspection.accepted) {
            return PublishOutcome(false, inspection.reason)
        }

        return try {
            MediaStorePublisher.serialized {
                val receipt = publisher.publishMp4(
                    candidate,
                    requireNotNull(initial.fingerprint),
                    session.id,
                    session.startedAtMs,
                )
                finishCommittedPublication(
                    receipt,
                    "校验通过，时长约 %.2f 秒".format(
                        inspection.durationUs / 1_000_000.0,
                    ),
                )
            }
        } catch (error: Throwable) {
            PublishOutcome(
                false,
                "发布失败：${error.message ?: error.javaClass.simpleName}",
            )
        }
    }

    private fun recoverInterruptedRecording() {
        if (isRecordingActive() || candidateProcessing) return
        candidateProcessing = true
        recordingState = RecordingState.PUBLISHING
        setStatus("正在核对上次 MediaStore 发布事务…")
        updateControls()

        worker.execute {
            val result = runCatching {
                MediaStorePublisher.serialized {
                    publisher.reconcile()?.let {
                        finishCommittedPublication(it, "已恢复上次发布事务")
                    }
                }
            }
            val outcome = result.getOrNull()
            mainHandler.post {
                candidateProcessing = false
                when {
                    result.isFailure -> {
                        recordingState = RecordingState.RECOVERY_REQUIRED
                        recoverButton.visibility = View.VISIBLE
                        setStatus(
                            "发布事务状态暂时无法确认：${result.exceptionOrNull()?.message}",
                        )
                    }
                    outcome != null -> applyRecoveredPublicationOutcome(outcome)
                    else -> recoverInterruptedRecordingCandidate()
                }
                disposeControllerIfRequested()
                updateControls()
            }
        }
    }

    private fun recoverInterruptedRecordingCandidate() {
        val entry = journal.load()
        if (entry == null) {
            if (journal.hasStoredTransaction()) {
                recordingState = RecordingState.RECOVERY_REQUIRED
                recoverButton.visibility = View.VISIBLE
                setStatus("录像事务日志无法读取；已保留文件，拒绝自动发布或覆盖")
            } else {
                recordingState = RecordingState.IDLE
                setStatus("没有可验证的事务日志；未知临时文件不会自动发布")
            }
            updateControls()
            return
        }

        val candidates = temporaryDirectory.listFiles()
            .orEmpty()
            .filter { file ->
                val before = runCatching { entry.baseline[file.canonicalPath] }.getOrNull()
                file.isFile &&
                    file.lastModified() >= entry.startedAtMs - 2_000L &&
                    (before == null ||
                        before.length != file.length() ||
                        before.lastModified != file.lastModified())
            }
        if (candidates.size != 1) {
            recordingState = RecordingState.RECOVERY_REQUIRED
            setStatus("恢复要求恰好一个本次候选文件，当前找到 ${candidates.size} 个；未自动发布")
            updateControls()
            return
        }

        val session = ActiveRecording(
            entry.recordingId,
            entry.playerGeneration,
            entry.startedAtMs,
            entry.baseline,
        )
        activeRecording = session
        processCandidate(session, candidates.single())
    }

    private fun resumePublicationTransaction() {
        candidateProcessing = true
        recordingState = RecordingState.PUBLISHING
        setStatus("正在检查上次录像发布事务…")
        updateControls()

        worker.execute {
            val result = runCatching {
                MediaStorePublisher.serialized {
                    publisher.reconcile()?.let {
                        finishCommittedPublication(it, "已自动完成上次发布事务")
                    }
                }
            }
            val outcome = result.getOrNull()
            mainHandler.post {
                candidateProcessing = false
                when {
                    result.isFailure -> {
                        recordingState = RecordingState.RECOVERY_REQUIRED
                        recoverButton.visibility = View.VISIBLE
                        setStatus(
                            "发布事务状态暂时无法确认：${result.exceptionOrNull()?.message}",
                        )
                    }
                    outcome != null -> applyRecoveredPublicationOutcome(outcome)
                    else -> {
                        recordingState = RecordingState.IDLE
                        showRecoveryIfNeeded()
                    }
                }
                disposeControllerIfRequested()
                updateControls()
            }
        }
    }

    private fun finishCommittedPublication(
        receipt: PublicationReceipt,
        message: String,
    ): PublishOutcome = MediaStorePublisher.serialized {
        try {
            rememberConsumed(receipt.fingerprint)
            val root = temporaryDirectory.canonicalFile
            val source = File(receipt.sourcePath).canonicalFile
            check(source.path.startsWith(root.path + File.separator)) {
                "发布事务源文件越出应用临时目录"
            }
            check(!source.exists() || source.delete()) {
                "已发布录像的临时源文件无法删除"
            }
            journal.clear()
            publisher.acknowledge(receipt)
            PublishOutcome(true, message, receipt.uri)
        } catch (error: Throwable) {
            PublishOutcome(
                false,
                "录像已进入 MediaStore，但本地事务收尾失败：${error.message}",
                receipt.uri,
            )
        }
    }

    private fun applyRecoveredPublicationOutcome(outcome: PublishOutcome) {
        activeRecording = null
        if (outcome.success) {
            recordingState = RecordingState.IDLE
            recoverButton.visibility = View.GONE
            setStatus(
                "录像已发布到 DCIM/K230：${outcome.destination}\n${outcome.message}",
            )
        } else {
            recordingState = RecordingState.RECOVERY_REQUIRED
            recoverButton.visibility = View.VISIBLE
            setStatus("${outcome.message}\n请再次点“恢复检查”")
        }
    }

    private fun showRecoveryIfNeeded() {
        val entry = journal.load()
        val hasJournal = journal.hasStoredTransaction()
        val hasTemporaryFiles = temporaryDirectory.listFiles().orEmpty().any { it.isFile }
        recoverButton.visibility =
            if (hasJournal || hasTemporaryFiles) View.VISIBLE else View.GONE
        if (hasJournal) {
            recordingState = RecordingState.RECOVERY_REQUIRED
            setStatus(
                if (entry != null) {
                    "发现上次未完成的录像事务，可点“恢复检查”"
                } else {
                    "录像事务日志无法读取；已保留文件，拒绝自动发布或覆盖"
                },
            )
        }
    }

    private fun releasePlayer() {
        val player = mediaPlayer
        val engine = libVLC
        mediaPlayer = null
        libVLC = null
        playerGeneration += 1
        if (player != null) {
            runCatching { player.setEventListener(null) }
            runCatching { player.stop() }
            runCatching { player.detachViews() }
            runCatching { player.release() }
        }
        if (engine != null) {
            runCatching { Dialog.setCallbacks(engine, null) }
            runCatching { engine.release() }
        }
        if (playbackState != PlaybackState.ERROR) {
            playbackState = PlaybackState.WAITING_NETWORK
        }
        updateControls()
    }

    private fun disposeControllerIfRequested() {
        if (destroyRequested && !isRecordingActive() && !candidateProcessing) {
            disposeController()
        }
    }

    private fun disposeController() {
        releasePlayer()
        if (!worker.isShutdown) worker.shutdown()
    }

    private fun schedulePlayerReconnect() {
        if (reconnectScheduledForGeneration == playerGeneration) return
        val delays = longArrayOf(1_000L, 2_000L, 5_000L)
        if (reconnectAttempt >= delays.size) {
            setStatus("RTSP 连续重连失败；确认 K230-CAM 后点“重新连接”")
            updateControls()
            return
        }
        val delay = delays[reconnectAttempt++]
        val expectedGeneration = playerGeneration
        reconnectScheduledForGeneration = expectedGeneration
        setStatus("RTSP 中断，${delay / 1_000} 秒后自动重连…")
        mainHandler.postDelayed(
            {
                if (playbackState == PlaybackState.ERROR &&
                    playerGeneration == expectedGeneration &&
                    !isRecordingActive()
                ) {
                    reconnectScheduledForGeneration = null
                    createAndStartPlayer()
                }
            },
            delay,
        )
    }

    private fun isRecordingActive(): Boolean =
        activeRecording != null &&
            (
                recordingState == RecordingState.STARTING ||
                    recordingState == RecordingState.RECORDING ||
                    recordingState == RecordingState.STOPPING ||
                    recordingState == RecordingState.PUBLISHING
                )

    private fun consumedFingerprints(): Set<String> =
        orderedConsumedFingerprints().toSet()

    private fun rememberConsumed(fingerprint: String) {
        val values = orderedConsumedFingerprints()
        values.removeAll { it == fingerprint }
        values += fingerprint
        while (values.size > MAX_CONSUMED_FINGERPRINTS) {
            values.removeAt(0)
        }
        val json = JSONArray()
        values.forEach { json.put(it) }
        check(
            consumedPreferences.edit()
                .putString(KEY_CONSUMED_ORDERED, json.toString())
                .remove(KEY_CONSUMED_LEGACY)
                .commit(),
        ) {
            "无法持久化已发布录像指纹"
        }
    }

    private fun orderedConsumedFingerprints(): MutableList<String> {
        val current = consumedPreferences.getString(KEY_CONSUMED_ORDERED, null)
        if (current != null) {
            val json = JSONArray(current)
            val result = mutableListOf<String>()
            for (index in 0 until json.length()) {
                val value = json.getString(index)
                if (value !in result) result += value
            }
            return result
        }
        return consumedPreferences
            .getStringSet(KEY_CONSUMED_LEGACY, emptySet())
            .orEmpty()
            .toMutableList()
    }

    private fun setStatus(message: String) {
        statusText.text = message
    }

    private fun updateControls() {
        startButton.isEnabled =
            playbackState == PlaybackState.PREVIEWING &&
                recordingState == RecordingState.IDLE
        stopButton.isEnabled =
            recordingState == RecordingState.STARTING ||
                recordingState == RecordingState.RECORDING
        retryButton.isEnabled = !isRecordingActive() && !candidateProcessing
        recoverButton.isEnabled =
            !isRecordingActive() &&
                !candidateProcessing &&
                recordingState != RecordingState.PUBLISHING
    }

    private companion object {
        const val KEY_CONSUMED_LEGACY = "fingerprints"
        const val KEY_CONSUMED_ORDERED = "fingerprints_ordered_v2"
        const val MAX_CONSUMED_FINGERPRINTS = 200
    }
}
