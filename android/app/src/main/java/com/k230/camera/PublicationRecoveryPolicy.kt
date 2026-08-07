package com.k230.camera

enum class PublicationPhase {
    PREPARING,
    PENDING,
    COMMITTED,
}

enum class MediaRowState {
    ABSENT,
    PENDING,
    PUBLISHED,
    UNKNOWN,
}

enum class PublicationRecoveryAction {
    CLEAR_STATE,
    DELETE_PENDING,
    RETURN_COMMITTED,
    RETRY_LATER,
}

object PublicationRecoveryPolicy {
    fun decide(
        phase: PublicationPhase,
        row: MediaRowState,
        verifiedPublished: Boolean = false,
    ): PublicationRecoveryAction {
        if (row == MediaRowState.PUBLISHED) {
            return PublicationRecoveryAction.RETURN_COMMITTED
        }
        if (phase == PublicationPhase.COMMITTED) {
            return if (verifiedPublished && row != MediaRowState.PENDING) {
                PublicationRecoveryAction.RETURN_COMMITTED
            } else {
                PublicationRecoveryAction.RETRY_LATER
            }
        }
        return when (row) {
            MediaRowState.ABSENT -> PublicationRecoveryAction.CLEAR_STATE
            MediaRowState.PENDING -> PublicationRecoveryAction.DELETE_PENDING
            MediaRowState.UNKNOWN -> PublicationRecoveryAction.RETRY_LATER
            MediaRowState.PUBLISHED -> PublicationRecoveryAction.RETURN_COMMITTED
        }
    }
}
