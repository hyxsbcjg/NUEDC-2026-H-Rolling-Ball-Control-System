#include "k230_app.h"

#include "zf_common_interrupt.h"

#define K230_APP_UPDATE_COUNT_MAX                (0xFFFFFFFFU)

#pragma section all "cpu0_dsram"
static volatile k230_app_snapshot_struct k230_app_snapshot =
{
    ZF_FALSE,
    ZF_FALSE,
    ZF_FALSE,
    0U,
    0U,
    0U,
    0U
};
#pragma section all restore

k230_uart_status_enum k230_app_init (void)
{
    uint32 interrupt_state;
    k230_uart_status_enum status;

    interrupt_state = interrupt_global_disable();
    k230_app_snapshot.initialized = ZF_FALSE;
    k230_app_snapshot.position_valid = ZF_FALSE;
    k230_app_snapshot.touch_valid = ZF_FALSE;
    k230_app_snapshot.ball_x = 0U;
    k230_app_snapshot.touch_x = 0U;
    k230_app_snapshot.position_update_count = 0U;
    k230_app_snapshot.touch_update_count = 0U;
    interrupt_global_enable(interrupt_state);

    status = k230_uart_init();
    if (K230_UART_STATUS_OK == status)
    {
        interrupt_state = interrupt_global_disable();
        k230_app_snapshot.initialized = ZF_TRUE;
        interrupt_global_enable(interrupt_state);
    }

    return status;
}

void k230_app_service (void)
{
    uint8 frame_count;
    uint32 interrupt_state;
    uint16 ball_x;
    uint16 touch_x;
    k230_uart_frame_struct frame;

    if (ZF_FALSE == k230_app_snapshot.initialized)
    {
        return;
    }

    for (frame_count = 0U;
         frame_count < K230_APP_MAX_FRAMES_PER_SERVICE;
         frame_count++)
    {
        if (K230_UART_STATUS_OK != k230_uart_read_frame(&frame))
        {
            break;
        }

        if (ZF_TRUE == k230_uart_decode_ball_x(&frame, &ball_x))
        {
            if (K230_APP_BALL_X_MAX >= ball_x)
            {
                interrupt_state = interrupt_global_disable();
                k230_app_snapshot.ball_x = ball_x;
                k230_app_snapshot.position_valid = ZF_TRUE;
                if (K230_APP_UPDATE_COUNT_MAX
                    != k230_app_snapshot.position_update_count)
                {
                    k230_app_snapshot.position_update_count++;
                }
                interrupt_global_enable(interrupt_state);
            }
            continue;
        }

        if (ZF_TRUE == k230_uart_decode_touch_x(&frame, &touch_x))
        {
            if (K230_APP_TOUCH_X_MAX >= touch_x)
            {
                interrupt_state = interrupt_global_disable();
                k230_app_snapshot.touch_x = touch_x;
                k230_app_snapshot.touch_valid = ZF_TRUE;
                if (K230_APP_UPDATE_COUNT_MAX
                    != k230_app_snapshot.touch_update_count)
                {
                    k230_app_snapshot.touch_update_count++;
                }
                interrupt_global_enable(interrupt_state);
            }
        }
    }
}

uint8 k230_app_get_snapshot (k230_app_snapshot_struct *snapshot)
{
    uint32 interrupt_state;

    if ((0 == snapshot) || (ZF_FALSE == k230_app_snapshot.initialized))
    {
        return ZF_FALSE;
    }

    interrupt_state = interrupt_global_disable();
    *snapshot = k230_app_snapshot;
    interrupt_global_enable(interrupt_state);
    return ZF_TRUE;
}
