package com.k230.camera

import org.junit.Assert.assertEquals
import org.junit.Test

class PublicationRecoveryPolicyTest {
    @Test
    fun publishedRowsAreNeverDeleted() {
        PublicationPhase.entries.forEach { phase ->
            assertEquals(
                PublicationRecoveryAction.RETURN_COMMITTED,
                PublicationRecoveryPolicy.decide(phase, MediaRowState.PUBLISHED),
            )
        }
    }

    @Test
    fun onlyUncommittedPendingRowsAreDeleted() {
        assertEquals(
            PublicationRecoveryAction.DELETE_PENDING,
            PublicationRecoveryPolicy.decide(
                PublicationPhase.PREPARING,
                MediaRowState.PENDING,
            ),
        )
        assertEquals(
            PublicationRecoveryAction.DELETE_PENDING,
            PublicationRecoveryPolicy.decide(
                PublicationPhase.PENDING,
                MediaRowState.PENDING,
            ),
        )
        assertEquals(
            PublicationRecoveryAction.RETRY_LATER,
            PublicationRecoveryPolicy.decide(
                PublicationPhase.COMMITTED,
                MediaRowState.PENDING,
            ),
        )
        assertEquals(
            PublicationRecoveryAction.RETRY_LATER,
            PublicationRecoveryPolicy.decide(
                PublicationPhase.COMMITTED,
                MediaRowState.PENDING,
                verifiedPublished = true,
            ),
        )
    }

    @Test
    fun missingRowsClearOnlyUncommittedState() {
        assertEquals(
            PublicationRecoveryAction.CLEAR_STATE,
            PublicationRecoveryPolicy.decide(
                PublicationPhase.PENDING,
                MediaRowState.ABSENT,
            ),
        )
        assertEquals(
            PublicationRecoveryAction.RETRY_LATER,
            PublicationRecoveryPolicy.decide(
                PublicationPhase.COMMITTED,
                MediaRowState.ABSENT,
            ),
        )
    }

    @Test
    fun unknownRowsAreRetriedUnlessAlreadyCommitted() {
        assertEquals(
            PublicationRecoveryAction.RETRY_LATER,
            PublicationRecoveryPolicy.decide(
                PublicationPhase.PREPARING,
                MediaRowState.UNKNOWN,
            ),
        )
        assertEquals(
            PublicationRecoveryAction.RETRY_LATER,
            PublicationRecoveryPolicy.decide(
                PublicationPhase.COMMITTED,
                MediaRowState.UNKNOWN,
            ),
        )
        assertEquals(
            PublicationRecoveryAction.RETURN_COMMITTED,
            PublicationRecoveryPolicy.decide(
                PublicationPhase.COMMITTED,
                MediaRowState.UNKNOWN,
                verifiedPublished = true,
            ),
        )
    }
}
