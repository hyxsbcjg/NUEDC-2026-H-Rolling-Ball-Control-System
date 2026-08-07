#include "menu.h"

#include "ball_control.h"
#include "encoder.h"
#include "gray_app.h"
#include "gw_gray_sensor.h"
#include "imu_app.h"
#include "k230_app.h"
#include "my_battery.h"
#include "my_key.h"
#include "my_screen.h"
#include "my_uart.h"
#include "question_app.h"
#include "step_motor.h"
#include "task.h"

#define MY_MENU_GYRO_ROW                       (2U)
#define MY_MENU_ACCEL_ROW                     (3U)
#define MY_MENU_ENCODER_ROW                    (4U)
#define MY_MENU_QUESTION_ROW                   (5U)
#define MY_MENU_BATTERY_ROW                    (6U)
#define MY_MENU_BALL_DEBUG_ENABLE              (0U)
#define MY_MENU_PULSE_TEST_LOG_PERIOD_MS       (150U)
#define MY_MENU_PULSE_TEST_LOG_PERIOD_TICKS    \
    ((MY_MENU_PULSE_TEST_LOG_PERIOD_MS + MY_TASK_PERIOD_MS - 1U) \
        / MY_TASK_PERIOD_MS)
#if (MY_MENU_BALL_DEBUG_ENABLE != 0U)
#define MY_MENU_BALL_DEBUG_PERIOD_MS           (150U)
#define MY_MENU_BALL_DEBUG_PERIOD_TICKS        \
    ((MY_MENU_BALL_DEBUG_PERIOD_MS + MY_TASK_PERIOD_MS - 1U) \
        / MY_TASK_PERIOD_MS)
#endif

/* Q0 KEY3 每次推进一个驱动器绝对脉冲位置。 */
static const int32 my_menu_pulse_test_positions[] =
{
    0,
    20, 40, 60, 80, 100, 120, 140, 160, 180, 200,
    0,
    -20, -40, -60, -80, -100, -120, -140, -160, -180, -200
};
#define MY_MENU_PULSE_TEST_POSITION_COUNT      \
    ((uint8)(sizeof(my_menu_pulse_test_positions) \
        / sizeof(my_menu_pulse_test_positions[0])))

/*
 * IPS200 的第 1 行固定给灰度，2..6 行分别给角速度、加速度、编码器、
 * 题目状态和电池。
 * 所有绘制都经 my_screen_show_line() 覆盖整行，避免短字符串留下旧字符。
 */
#if (MY_MENU_BATTERY_ROW > MY_SCREEN_ROW_COUNT)
#error "The menu rows do not fit on the screen"
#endif

#pragma section all "cpu0_dsram"
static uint8 my_menu_gray_sample_valid = ZF_FALSE;
static gray_app_sample_struct my_menu_gray_sample =
{
    GW_GRAY_CHANNEL_ALL,
    0.0F,
    0U
};
static uint8 my_menu_encoder_sample_valid = ZF_FALSE;
static encoder_delta_struct my_menu_encoder_delta = {0, 0};
static uint8 my_menu_pulse_test_active = ZF_FALSE;
static uint8 my_menu_pulse_test_next_index = 0U;
static uint8 my_menu_pulse_test_motor_status =
    (uint8)STEPPER_UART_STATUS_NOT_INITIALIZED;
static uint8 my_menu_pulse_test_header_printed = ZF_FALSE;
static uint8 my_menu_pulse_test_start_position_valid = ZF_FALSE;
static int32 my_menu_pulse_test_requested_pulse = 0;
static int32 my_menu_pulse_test_applied_pulse = 0;
static uint16 my_menu_pulse_test_start_ball_x = 0U;
static uint32 my_menu_pulse_test_trial_id = 0U;
static uint32 my_menu_pulse_test_start_tick = 0U;
static uint32 my_menu_pulse_test_last_log_tick = 0U;
static uint32 my_menu_pulse_test_start_position_update_count = 0U;
#if (MY_MENU_BALL_DEBUG_ENABLE != 0U)
static uint8 my_menu_ball_debug_active = ZF_FALSE;
static uint32 my_menu_ball_debug_last_tick = 0U;
#endif
#pragma section all restore

static void my_menu_reset_pulse_test_state (void)
{
    my_menu_pulse_test_active = ZF_FALSE;
    my_menu_pulse_test_next_index = 0U;
    my_menu_pulse_test_motor_status =
        (uint8)STEPPER_UART_STATUS_NOT_INITIALIZED;
    my_menu_pulse_test_header_printed = ZF_FALSE;
    my_menu_pulse_test_start_position_valid = ZF_FALSE;
    my_menu_pulse_test_requested_pulse = 0;
    my_menu_pulse_test_applied_pulse = 0;
    my_menu_pulse_test_start_ball_x = 0U;
    my_menu_pulse_test_trial_id = 0U;
    my_menu_pulse_test_start_tick = 0U;
    my_menu_pulse_test_last_log_tick = 0U;
    my_menu_pulse_test_start_position_update_count = 0U;
}

static void my_menu_print_pulse_test_header (void)
{
    if (ZF_FALSE == my_menu_pulse_test_header_printed)
    {
        my_menu_pulse_test_header_printed = ZF_TRUE;
        (void)my_printf(
            "PT,ID,EL,RQ,AP,MS,PV,BX,DX,PU,DU\r\n");
    }
}

static void my_menu_print_pulse_test_record (uint8 force)
{
    uint8 snapshot_valid = ZF_FALSE;
    uint8 position_valid = ZF_FALSE;
    uint16 ball_x = 0U;
    int32 delta_x = 0;
    uint32 position_update_count = 0U;
    uint32 position_update_delta = 0U;
    uint32 tick_count;
    uint32 elapsed_ms;
    k230_app_snapshot_struct k230_snapshot;

    if (ZF_FALSE == my_menu_pulse_test_active)
    {
        return;
    }

    tick_count = my_task_get_tick_count();
    if ((ZF_FALSE == force)
        && (MY_MENU_PULSE_TEST_LOG_PERIOD_TICKS
            > (uint32)(tick_count - my_menu_pulse_test_last_log_tick)))
    {
        return;
    }
    my_menu_pulse_test_last_log_tick = tick_count;

    if (ZF_TRUE == k230_app_get_snapshot(&k230_snapshot))
    {
        snapshot_valid = ZF_TRUE;
        position_valid = k230_snapshot.position_valid;
        ball_x = k230_snapshot.ball_x;
        position_update_count = k230_snapshot.position_update_count;
    }
    if ((ZF_TRUE == position_valid)
        && (ZF_TRUE == my_menu_pulse_test_start_position_valid))
    {
        delta_x = (int32)ball_x
            - (int32)my_menu_pulse_test_start_ball_x;
    }
    if (ZF_TRUE == snapshot_valid)
    {
        position_update_delta = position_update_count
            - my_menu_pulse_test_start_position_update_count;
    }
    elapsed_ms =
        (uint32)(tick_count - my_menu_pulse_test_start_tick)
        * MY_TASK_PERIOD_MS;

    my_menu_print_pulse_test_header();
    (void)my_printf(
        "PT,%lu,%lu,%ld,%ld,%u,%u,%u,%ld,%lu,%lu\r\n",
        (unsigned long)my_menu_pulse_test_trial_id,
        (unsigned long)elapsed_ms,
        (long)my_menu_pulse_test_requested_pulse,
        (long)my_menu_pulse_test_applied_pulse,
        (unsigned int)my_menu_pulse_test_motor_status,
        (unsigned int)position_valid,
        (unsigned int)ball_x,
        (long)delta_x,
        (unsigned long)position_update_count,
        (unsigned long)position_update_delta);
}

static stepper_uart_status_enum my_menu_advance_pulse_test (void)
{
    int32 requested_pulse;
    stepper_uart_status_enum motor_status;
    k230_app_snapshot_struct k230_snapshot;

    requested_pulse =
        my_menu_pulse_test_positions[my_menu_pulse_test_next_index];
    motor_status = Step_Motor_Set_Position(requested_pulse);
    my_menu_pulse_test_requested_pulse = requested_pulse;
    my_menu_pulse_test_motor_status = (uint8)motor_status;

    if (STEPPER_UART_STATUS_OK != motor_status)
    {
        my_menu_print_pulse_test_header();
        (void)my_printf(
            "PT_ERR,%ld,%u\r\n",
            (long)requested_pulse,
            (unsigned int)motor_status);
        return motor_status;
    }

    my_menu_pulse_test_active = ZF_TRUE;
    my_menu_pulse_test_applied_pulse = requested_pulse;
    my_menu_pulse_test_start_tick = my_task_get_tick_count();
    my_menu_pulse_test_last_log_tick = my_menu_pulse_test_start_tick;
    my_menu_pulse_test_start_position_valid = ZF_FALSE;
    my_menu_pulse_test_start_ball_x = 0U;
    my_menu_pulse_test_start_position_update_count = 0U;
    if (ZF_TRUE == k230_app_get_snapshot(&k230_snapshot))
    {
        my_menu_pulse_test_start_position_valid =
            k230_snapshot.position_valid;
        my_menu_pulse_test_start_ball_x = k230_snapshot.ball_x;
        my_menu_pulse_test_start_position_update_count =
            k230_snapshot.position_update_count;
    }

    if (0xFFFFFFFFU != my_menu_pulse_test_trial_id)
    {
        my_menu_pulse_test_trial_id++;
    }
    my_menu_pulse_test_next_index++;
    if (MY_MENU_PULSE_TEST_POSITION_COUNT
        <= my_menu_pulse_test_next_index)
    {
        my_menu_pulse_test_next_index = 0U;
    }

    my_menu_print_pulse_test_record(ZF_TRUE);
    return STEPPER_UART_STATUS_OK;
}

static uint8 my_menu_pulse_test_can_leave_q0 (void)
{
    return ((ZF_FALSE == my_menu_pulse_test_active)
        || (0 == my_menu_pulse_test_applied_pulse))
        ? ZF_TRUE
        : ZF_FALSE;
}

static uint8 my_menu_adjust_q6_runtime_center (int32 delta_pulse)
{
    int32 current_center_pulse;
    int32 requested_center_pulse;
    float32 positive_breakaway_absolute_pulse;
    float32 negative_breakaway_absolute_pulse;
    ball_control_status_enum ball_status;
    stepper_uart_status_enum motor_status;

    ball_status = ball_control_get_q6_runtime_center_pulse(
        &current_center_pulse);
    if (BALL_CONTROL_STATUS_OK != ball_status)
    {
        (void)my_printf(
            "Q6_REF_ERR,GET,%u\r\n",
            (unsigned int)ball_status);
        return ZF_FALSE;
    }

    requested_center_pulse = current_center_pulse + delta_pulse;
    positive_breakaway_absolute_pulse =
        (float32)requested_center_pulse
        + BALL_CONTROL_Q4_Q5_BREAKAWAY_POSITIVE_MIN_PULSE;
    negative_breakaway_absolute_pulse =
        (float32)requested_center_pulse
        + BALL_CONTROL_Q4_Q5_BREAKAWAY_NEGATIVE_MAX_PULSE;
    if (((int32)BALL_CONTROL_MIN_PULSE > requested_center_pulse)
        || ((int32)BALL_CONTROL_MAX_PULSE < requested_center_pulse)
        || ((float32)BALL_CONTROL_MAX_PULSE
            < positive_breakaway_absolute_pulse)
        || ((float32)BALL_CONTROL_MIN_PULSE
            > negative_breakaway_absolute_pulse))
    {
        (void)my_printf(
            "Q6_REF_LIMIT,%ld\r\n",
            (long)requested_center_pulse);
        return ZF_FALSE;
    }

    motor_status = Step_Motor_Set_Position(requested_center_pulse);
    (void)my_printf(
        "Q6_REF,%ld,%u\r\n",
        (long)requested_center_pulse,
        (unsigned int)motor_status);
    if (STEPPER_UART_STATUS_OK != motor_status)
    {
        return ZF_FALSE;
    }

    ball_status = ball_control_set_q6_runtime_center_pulse(
        requested_center_pulse);
    if (BALL_CONTROL_STATUS_OK != ball_status)
    {
        (void)my_printf(
            "Q6_REF_ERR,SET,%u\r\n",
            (unsigned int)ball_status);
        return ZF_FALSE;
    }

    return ZF_TRUE;
}

#if (MY_MENU_BALL_DEBUG_ENABLE != 0U)
static int32 my_menu_scale_debug_x10 (float32 value)
{
    float32 scaled = value * 10.0F;

    return (int32)(scaled + ((0.0F <= scaled) ? 0.5F : -0.5F));
}

static void my_menu_print_ball_control_debug (void)
{
    uint32 tick_count;
    ball_control_snapshot_struct snapshot;

    if ((ZF_FALSE == ball_control_get_snapshot(&snapshot))
        || (ZF_FALSE == snapshot.enabled)
        || (ZF_FALSE == snapshot.position_valid))
    {
        my_menu_ball_debug_active = ZF_FALSE;
        return;
    }

    tick_count = my_task_get_tick_count();
    if (ZF_FALSE == my_menu_ball_debug_active)
    {
        my_menu_ball_debug_active = ZF_TRUE;
        my_menu_ball_debug_last_tick = tick_count;
        (void)my_printf(
            "T,A,E10,OI10,TV10,FV10,VE10,II10,PO10,"
            "RQ,AP,S,MS,PU,CU,OU\r\n");
    }
    else if (MY_MENU_BALL_DEBUG_PERIOD_TICKS
        > (uint32)(tick_count - my_menu_ball_debug_last_tick))
    {
        return;
    }
    else
    {
        my_menu_ball_debug_last_tick = tick_count;
    }

    (void)my_printf(
        "%u,%u,%d,%d,%d,%d,%d,%d,%d,%d,%d,%u,%u,%lu,%lu,%lu\r\n",
        (unsigned int)snapshot.target_x,
        (unsigned int)snapshot.actual_x,
        (int)my_menu_scale_debug_x10(snapshot.error),
        (int)my_menu_scale_debug_x10(
            snapshot.position_integral_velocity_pixel_per_s),
        (int)my_menu_scale_debug_x10(
            snapshot.target_velocity_pixel_per_s),
        (int)my_menu_scale_debug_x10(
            snapshot.filtered_velocity_pixel_per_s),
        (int)my_menu_scale_debug_x10(
            snapshot.velocity_error_pixel_per_s),
        (int)my_menu_scale_debug_x10(snapshot.integral_output_pulse),
        (int)my_menu_scale_debug_x10(snapshot.pid_output_pulse),
        (int)snapshot.requested_absolute_pulse,
        (int)snapshot.applied_absolute_pulse,
        (unsigned int)snapshot.status,
        (unsigned int)snapshot.motor_status,
        (unsigned long)snapshot.position_update_count,
        (unsigned long)snapshot.control_update_count,
        (unsigned long)snapshot.outer_update_count);
}
#endif

static void my_menu_show_battery (void)
{
    uint32 millivolts;

    if (ZF_FALSE == my_battery_is_ready())
    {
        (void)my_screen_show_line(MY_MENU_BATTERY_ROW, "BAT ADC INIT ERR");
        return;
    }

    millivolts = my_battery_get_millivolts();
    (void)my_screen_show_line(
        MY_MENU_BATTERY_ROW,
        "BAT:%lu.%03luV ADC:%u",
        (unsigned long)(millivolts / 1000U),
        (unsigned long)(millivolts % 1000U),
        (unsigned int)my_battery_get_raw());
}


static void my_menu_show_question_state (void)
{
    k230_app_snapshot_struct k230_snapshot;
    question_app_snapshot_struct snapshot;
    const char *state_text;
    int32 q6_runtime_center_pulse;

    if (ZF_FALSE == question_app_get_snapshot(&snapshot))
    {
        (void)my_screen_show_line(MY_MENU_QUESTION_ROW, "QUESTION INIT ERR");
        return;
    }

    if (QUESTION_APP_QUESTION_K230_MONITOR
        == snapshot.selected_question)
    {
        if ((QUESTION_APP_STATE_IDLE == snapshot.state)
            && (ZF_TRUE == my_menu_pulse_test_active))
        {
            if ((ZF_TRUE == k230_app_get_snapshot(&k230_snapshot))
                && (ZF_TRUE == k230_snapshot.position_valid))
            {
                (void)my_screen_show_line(
                    MY_MENU_QUESTION_ROW,
                    "Q:0 P:%ld BX:%u",
                    (long)my_menu_pulse_test_applied_pulse,
                    (unsigned int)k230_snapshot.ball_x);
            }
            else
            {
                (void)my_screen_show_line(
                    MY_MENU_QUESTION_ROW,
                    "Q:0 P:%ld BX:---",
                    (long)my_menu_pulse_test_applied_pulse);
            }
            return;
        }

#if (QUESTION_APP_Q0_DEBUG_MODE \
    == QUESTION_APP_Q0_DEBUG_MODE_GYRO_LOOP)
        (void)my_screen_show_line(
            MY_MENU_QUESTION_ROW,
            (QUESTION_APP_STATE_RUNNING == snapshot.state)
                ? "Q:0 GYRO RUN"
                : ((QUESTION_APP_STATE_FAULT == snapshot.state)
                    ? "Q:0 GYRO FAULT"
                    : "Q:0 GYRO READY"));
        return;
#elif (QUESTION_APP_Q0_DEBUG_MODE \
    == QUESTION_APP_Q0_DEBUG_MODE_ACCEL_FF)
        (void)my_screen_show_line(
            MY_MENU_QUESTION_ROW,
            (QUESTION_APP_STATE_RUNNING == snapshot.state)
                ? "Q:0 ACC FF RUN"
                : ((QUESTION_APP_STATE_FAULT == snapshot.state)
                    ? "Q:0 ACC FF FAULT"
                    : "Q:0 ACC READY"));
        return;
#elif (QUESTION_APP_Q0_DEBUG_MODE \
    == QUESTION_APP_Q0_DEBUG_MODE_SOFT_START)
        (void)my_screen_show_line(
            MY_MENU_QUESTION_ROW,
            (QUESTION_APP_STATE_RUNNING == snapshot.state)
                ? "Q:0 SOFT RUN"
                : ((QUESTION_APP_STATE_FAULT == snapshot.state)
                    ? "Q:0 SOFT FAULT"
                    : "Q:0 SOFT READY"));
        return;
#endif

        if (ZF_FALSE == k230_app_get_snapshot(&k230_snapshot))
        {
            (void)my_screen_show_line(
                MY_MENU_QUESTION_ROW,
                "Q:0 K230 INIT ERR");
        }
        else if (ZF_FALSE == k230_snapshot.position_valid)
        {
            (void)my_screen_show_line(
                MY_MENU_QUESTION_ROW,
                "Q:0 K230 WAIT");
        }
        else
        {
            (void)my_screen_show_line(
                MY_MENU_QUESTION_ROW,
                "Q:0 BX:%u",
                (unsigned int)k230_snapshot.ball_x);
        }
        return;
    }

    /*
     * Q2 第二次停车线确认后已在 3 ms ISR 内冻结总时间。运行态不写屏；
     * 退出后复用现有 150 ms 刷新事件，把第 5 行切换为两位小数的比赛时间。
     */
    if ((QUESTION_APP_QUESTION_TRACK_AND_STOP
            == snapshot.selected_question)
        && (QUESTION_APP_STATE_FINISHED == snapshot.state))
    {
        (void)my_screen_show_elapsed_time(
            MY_MENU_QUESTION_ROW,
            snapshot.q2_elapsed_ms);
        return;
    }

    if (QUESTION_APP_STATE_RUNNING == snapshot.state)
    {
        state_text = "RUN";
    }
    else if (QUESTION_APP_STATE_FINISHED == snapshot.state)
    {
        state_text = "DONE";
    }
    else if (QUESTION_APP_STATE_FAULT == snapshot.state)
    {
        state_text = "FAULT";
    }
    else if (QUESTION_APP_QUESTION_TRACK_AND_STOP
        == snapshot.selected_question)
    {
        state_text = "READY";
    }
    else if ((QUESTION_APP_QUESTION_BALL_FIRST
            <= snapshot.selected_question)
        && (QUESTION_APP_QUESTION_BALL_LAST
            >= snapshot.selected_question))
    {
        state_text = "READY";
    }
    else
    {
        state_text = "SELECT Q2-6";
    }

    if ((QUESTION_APP_QUESTION_BALL_FIRST
            <= snapshot.selected_question)
        && (QUESTION_APP_QUESTION_BALL_LAST
            >= snapshot.selected_question))
    {
        if (QUESTION_APP_QUESTION_BALL_Q6
            == snapshot.selected_question)
        {
            if (BALL_CONTROL_STATUS_OK
                != ball_control_get_q6_runtime_center_pulse(
                    &q6_runtime_center_pulse))
            {
                (void)my_screen_show_line(
                    MY_MENU_QUESTION_ROW,
                    "Q6 REF ERR");
                return;
            }

            if (ZF_FALSE == snapshot.q6_target_valid)
            {
                (void)my_screen_show_line(
                    MY_MENU_QUESTION_ROW,
                    "Q6 C:%ld AUTO BX",
                    (long)q6_runtime_center_pulse);
                return;
            }

            (void)my_screen_show_line(
                MY_MENU_QUESTION_ROW,
                "Q6 C:%ld %s T:%u",
                (long)q6_runtime_center_pulse,
                state_text,
                (unsigned int)snapshot.ball_target_x);
            return;
        }

        (void)my_screen_show_line(
            MY_MENU_QUESTION_ROW,
            "Q:%u %s T:%u",
            (unsigned int)snapshot.selected_question,
            state_text,
            (unsigned int)snapshot.ball_target_x);
        return;
    }

    (void)my_screen_show_line(
        MY_MENU_QUESTION_ROW,
        "Q:%u %s PASS:%u",
        (unsigned int)snapshot.selected_question,
        state_text,
        (unsigned int)snapshot.stop_line_count);
}

static void my_menu_process_key_clicks (void)
{
    key_index_enum key;
    uint8 state_changed = ZF_FALSE;

    /*
     * 停车态 KEY_1 切换题号；KEY_2 在 Q0 触发回到驱动器既有机械零点，
     * 在 Q6 IDLE 将 RAM 运行中心减 20 pulse；KEY_3 在 Q0 IDLE 推进绝对脉冲
     * 机构测试，在 Q6 IDLE 将 RAM 运行中心加 20 pulse；KEY_4 启动 Q0
     * 自定义调试模式或选中的
     * 题目。题目态不再扫描按键；KEY_4 成功发车后立即丢弃同批剩余事件。
     */
    while (ZF_TRUE == my_key_fetch_click(&key))
    {
        if (KEY_1 == key)
        {
            if (ZF_FALSE == my_menu_pulse_test_can_leave_q0())
            {
                (void)my_screen_show_line(
                    MY_MENU_QUESTION_ROW,
                    "Q:0 RETURN P0");
                (void)my_printf(
                    "PT_BLOCK,KEY1,AP=%ld\r\n",
                    (long)my_menu_pulse_test_applied_pulse);
            }
            else
            {
                my_menu_reset_pulse_test_state();
                (void)question_app_select_next();
                state_changed = ZF_TRUE;
            }
        }
        else if (KEY_2 == key)
        {
            question_app_snapshot_struct snapshot;
            stepper_uart_status_enum motor_status;

            if (ZF_TRUE == question_app_get_snapshot(&snapshot))
            {
                if ((QUESTION_APP_QUESTION_K230_MONITOR
                        == snapshot.selected_question)
                    && (QUESTION_APP_STATE_IDLE == snapshot.state))
                {
                    motor_status = Trigger_Return();
                    (void)my_printf(
                        "PT_HOME,%u\r\n",
                        (unsigned int)motor_status);
                    if (STEPPER_UART_STATUS_OK == motor_status)
                    {
                        my_menu_reset_pulse_test_state();
                        state_changed = ZF_TRUE;
                    }
                }
                else if ((QUESTION_APP_QUESTION_BALL_Q6
                        == snapshot.selected_question)
                    && (QUESTION_APP_STATE_IDLE == snapshot.state))
                {
                    state_changed = my_menu_adjust_q6_runtime_center(
                        -BALL_CONTROL_Q6_CENTER_ADJUST_STEP_PULSE);
                }
            }
        }
        else if (KEY_3 == key)
        {
            question_app_snapshot_struct snapshot;

            if (ZF_TRUE == question_app_get_snapshot(&snapshot))
            {
                if ((QUESTION_APP_QUESTION_K230_MONITOR
                        == snapshot.selected_question)
                    && (QUESTION_APP_STATE_IDLE == snapshot.state))
                {
                    (void)my_menu_advance_pulse_test();
                    state_changed = ZF_TRUE;
                }
                else if ((QUESTION_APP_QUESTION_BALL_Q6
                        == snapshot.selected_question)
                    && (QUESTION_APP_STATE_IDLE == snapshot.state))
                {
                    state_changed = my_menu_adjust_q6_runtime_center(
                        BALL_CONTROL_Q6_CENTER_ADJUST_STEP_PULSE);
                }
            }
        }
        else if (KEY_4 == key)
        {
            if (ZF_FALSE == my_menu_pulse_test_can_leave_q0())
            {
                (void)my_screen_show_line(
                    MY_MENU_QUESTION_ROW,
                    "Q:0 RETURN P0");
                (void)my_printf(
                    "PT_BLOCK,KEY4,AP=%ld\r\n",
                    (long)my_menu_pulse_test_applied_pulse);
            }
            else
            {
                my_menu_reset_pulse_test_state();
                (void)question_app_start_selected();
                state_changed = ZF_TRUE;
            }
        }

        if (ZF_TRUE == question_app_is_running())
        {
            my_key_clear_clicks();
            return;
        }
    }

    if (ZF_TRUE == state_changed)
    {
        my_menu_show_question_state();
    }
}

static uint32 my_menu_get_imu_magnitude_tenths (
    float32 corrected_lsb,
    char *sign)
{
    int32 tenths;
    uint32 magnitude;

    /*
     * 先四舍五入成定点数，再拆符号和整数/小数位，避免依赖嵌入式 printf
     * 是否启用浮点格式化。显示单位仍是校正后的原始 LSB，不是角速度。
     */
    if (0.0F <= corrected_lsb)
    {
        tenths = (int32)(corrected_lsb * 10.0F + 0.5F);
    }
    else
    {
        tenths = (int32)(corrected_lsb * 10.0F - 0.5F);
    }

    if (0 > tenths)
    {
        *sign = '-';
        magnitude = (uint32)(-tenths);
    }
    else
    {
        *sign = '+';
        magnitude = (uint32)tenths;
    }

    return magnitude;
}

static void my_menu_show_imu (void)
{
    ImuAppData imu_data;
    uint32 x_tenths;
    uint32 y_tenths;
    uint32 z_tenths;
    uint32 acc_x_tenths;
    uint32 acc_y_tenths;
    char x_sign;
    char y_sign;
    char z_sign;
    char acc_x_sign;
    char acc_y_sign;

    if (ZF_FALSE == imu_app_get_snapshot(&imu_data))
    {
        return;
    }

    if (ZF_FALSE == imu_data.initialized)
    {
        (void)my_screen_show_line(
            MY_MENU_GYRO_ROW,
            "GYRO INIT ERR:%u",
            (unsigned int)imu_data.init_status);
        (void)my_screen_show_line(
            MY_MENU_ACCEL_ROW,
            "ACC INIT ERR:%u",
            (unsigned int)imu_data.init_status);
        return;
    }

    if (ZF_FALSE == imu_data.offset_valid)
    {
        (void)my_screen_show_line(MY_MENU_GYRO_ROW, "GYRO CAL ERR");
    }
    else
    {
        x_tenths = my_menu_get_imu_magnitude_tenths(
            imu_data.corrected_x_lsb,
            &x_sign);
        y_tenths = my_menu_get_imu_magnitude_tenths(
            imu_data.corrected_y_lsb,
            &y_sign);
        z_tenths = my_menu_get_imu_magnitude_tenths(
            imu_data.corrected_z_lsb,
            &z_sign);

        (void)my_screen_show_line(
            MY_MENU_GYRO_ROW,
            "GYRO X:%c%lu.%01lu Y:%c%lu.%01lu Z:%c%lu.%01lu",
            x_sign,
            (unsigned long)(x_tenths / 10U),
            (unsigned long)(x_tenths % 10U),
            y_sign,
            (unsigned long)(y_tenths / 10U),
            (unsigned long)(y_tenths % 10U),
            z_sign,
            (unsigned long)(z_tenths / 10U),
            (unsigned long)(z_tenths % 10U));
    }

    if ((ZF_FALSE == imu_data.offset_valid)
        || (ZF_FALSE == imu_data.acceleration_valid))
    {
        (void)my_screen_show_line(MY_MENU_ACCEL_ROW, "ACC CAL/READ WAIT");
        return;
    }

    acc_x_tenths = my_menu_get_imu_magnitude_tenths(
        imu_data.corrected_acc_x_lsb,
        &acc_x_sign);
    acc_y_tenths = my_menu_get_imu_magnitude_tenths(
        imu_data.corrected_acc_y_lsb,
        &acc_y_sign);

    (void)my_screen_show_line(
        MY_MENU_ACCEL_ROW,
        "ACC X:%c%lu.%01lu Y:%c%lu.%01lu LSB",
        acc_x_sign,
        (unsigned long)(acc_x_tenths / 10U),
        (unsigned long)(acc_x_tenths % 10U),
        acc_y_sign,
        (unsigned long)(acc_y_tenths / 10U),
        (unsigned long)(acc_y_tenths % 10U));
}

static void my_menu_show_encoders (void)
{
    if (ZF_FALSE == encoder_is_ready())
    {
        (void)my_screen_show_line(
            MY_MENU_ENCODER_ROW,
            "ENC A/B INIT ERR");
        return;
    }

    if (ZF_FALSE == my_menu_encoder_sample_valid)
    {
        (void)my_screen_show_line(
            MY_MENU_ENCODER_ROW,
            "ENC A/B READ ERR");
        return;
    }

    (void)my_screen_show_line(
        MY_MENU_ENCODER_ROW,
        "ENC A:%+d B:%+d /150ms",
        (int)my_menu_encoder_delta.motor_a,
        (int)my_menu_encoder_delta.motor_b);
}

static void my_menu_sample_encoders (void)
{
    encoder_delta_struct delta;

    /*
     * read-and-clear 把计数窗口截断在本次 150 ms 刷新点；屏幕显示的是该
     * 窗口内增量，不是上电后的累计脉冲数。
     */
    if (ENCODER_STATUS_OK == encoder_read_and_clear_both(&delta))
    {
        my_menu_encoder_delta = delta;
        my_menu_encoder_sample_valid = ZF_TRUE;
    }
    else
    {
        my_menu_encoder_sample_valid = ZF_FALSE;
    }
}

static uint16 my_menu_get_gray_error_hundredths (
    float error,
    char *sign)
{
    int16 hundredths;
    uint16 magnitude;

    if (0.0F <= error)
    {
        hundredths = (int16)(error * 100.0F + 0.5F);
    }
    else
    {
        hundredths = (int16)(error * 100.0F - 0.5F);
    }

    if (0 > hundredths)
    {
        *sign = '-';
        magnitude = (uint16)(-hundredths);
    }
    else
    {
        *sign = '+';
        magnitude = (uint16)hundredths;
    }

    return magnitude;
}

static void my_menu_show_gray_sample (
    const gray_app_sample_struct *sample)
{
    uint16 error_hundredths;
    char error_sign;

    error_hundredths = my_menu_get_gray_error_hundredths(
        sample->error,
        &error_sign);

    /*
     * bit0..bit7 按 CH1..CH8 从左到右显示；业务层约定 0 表示该通道检测到
     * 黑线。E 为有效黑线通道的平均位权，N 为参与平均的通道数。
     */
    (void)my_screen_show_line(
        1U,
        "GRAY:%u%u%u%u%u%u%u%u E:%c%u.%02u N:%u",
        ((unsigned int)sample->digital_value >> 0U) & 0x01U,
        ((unsigned int)sample->digital_value >> 1U) & 0x01U,
        ((unsigned int)sample->digital_value >> 2U) & 0x01U,
        ((unsigned int)sample->digital_value >> 3U) & 0x01U,
        ((unsigned int)sample->digital_value >> 4U) & 0x01U,
        ((unsigned int)sample->digital_value >> 5U) & 0x01U,
        ((unsigned int)sample->digital_value >> 6U) & 0x01U,
        ((unsigned int)sample->digital_value >> 7U) & 0x01U,
        error_sign,
        (unsigned int)(error_hundredths / 100U),
        (unsigned int)(error_hundredths % 100U),
        (unsigned int)sample->error_num);
}

static void my_menu_sample_gray (void)
{
    gray_app_sample_struct sample;

    if (GW_GRAY_STATUS_OK == my_task_get_gray_sample(&sample))
    {
        my_menu_gray_sample = sample;
        my_menu_gray_sample_valid = ZF_TRUE;
    }
    else
    {
        my_menu_gray_sample_valid = ZF_FALSE;
    }
}

void my_menu_init (void)
{
    gw_gray_status_enum status;
    my_screen_status_enum screen_status;

    my_menu_gray_sample_valid = ZF_FALSE;
    my_menu_gray_sample.digital_value = GW_GRAY_CHANNEL_ALL;
    my_menu_gray_sample.error = 0.0F;
    my_menu_gray_sample.error_num = 0U;
    my_menu_encoder_sample_valid = ZF_FALSE;
    my_menu_encoder_delta.motor_a = 0;
    my_menu_encoder_delta.motor_b = 0;
    my_menu_reset_pulse_test_state();
    question_app_init();

    /*
     * PIT 尚未启动，因此初始化阶段不存在周期 ISR 总线竞争。滚球模块在
     * 此处初始化 UART3/步进驱动，只使能并停止。Q0 KEY2 可触发驱动器按
     * 既有机械零点回零；Q6 KEY2/KEY3 只发送绝对位置并调整 RAM 运行中心；
     * 工程不提供写当前位置、设置零点或修改回零配置入口。
     * K230 UART1 放在其它本地设备之后初始化，软件环形缓冲就绪后才开 RX。
     */
    my_key_init();
    my_battery_init();
    if (ZF_FALSE == my_uart_is_ready())
    {
        (void)my_uart_init();
    }
    screen_status = my_screen_init();

    status = gray_app_init();
    if ((GW_GRAY_STATUS_OK == status)
        && (GW_GRAY_STATUS_OK == gray_app_read(&my_menu_gray_sample)))
    {
        my_menu_gray_sample_valid = ZF_TRUE;
    }

    if (ENCODER_STATUS_OK == encoder_clear_all())
    {
        my_menu_encoder_sample_valid = ZF_TRUE;
    }

    (void)ball_control_init();
    (void)k230_app_init();

    if (MY_SCREEN_STATUS_OK != screen_status)
    {
        return;
    }

    (void)my_screen_clear();
    my_menu_show_gray_sample(&my_menu_gray_sample);
    my_menu_show_imu();
    my_menu_show_encoders();
    my_menu_show_question_state();
    my_menu_show_battery();
}

void my_menu_proc (void)
{
    uint8 q6_target_changed;

    /*
     * K230 成帧始终先执行。停车态按键 3 ms、ADC 9 ms、整屏业务数据
     * 150 ms；Q6 只在 IDLE 停车态锁存触摸目标。题目态仍停用按键扫描和屏幕刷新；
     * Q2 停车退出题目态后，下一次刷新显示已冻结的总时间。
    */
    k230_app_service();
    q6_target_changed = question_app_process_k230_touch();
    my_menu_print_pulse_test_record(ZF_FALSE);
#if (MY_MENU_BALL_DEBUG_ENABLE != 0U)
    my_menu_print_ball_control_debug();
#endif

    if (ZF_TRUE == question_app_is_running())
    {
        if (ZF_TRUE == my_task_take_battery_sample_event())
        {
            (void)my_battery_update();
        }
        return;
    }

    my_key_proc();
    my_menu_process_key_clicks();

    if (ZF_TRUE == question_app_is_running())
    {
        return;
    }

    if (ZF_TRUE == q6_target_changed)
    {
        my_menu_show_question_state();
    }

    if (ZF_TRUE == my_task_take_battery_sample_event())
    {
        (void)my_battery_update();
    }

    if (ZF_FALSE == my_task_take_refresh_event())
    {
        return;
    }

    my_menu_show_imu();
    my_menu_sample_encoders();
    my_menu_show_encoders();
    my_menu_show_question_state();
    my_menu_show_battery();

    my_menu_sample_gray();
    if (ZF_TRUE == my_menu_gray_sample_valid)
    {
        my_menu_show_gray_sample(&my_menu_gray_sample);
    }
    /* 灰度快照失败时保留上一帧显示，valid 仅阻止把安全默认值画成有效数据。 */
}
