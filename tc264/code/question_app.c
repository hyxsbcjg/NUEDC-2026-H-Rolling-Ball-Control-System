#include "question_app.h"

#include "k230_app.h"
#include "zf_common_interrupt.h"

#define QUESTION_APP_COUNTER_MAX                    (255U)
#define QUESTION_APP_Q2_ELAPSED_MS_MAX               (0xFFFFFFFFU)

typedef struct
{
    uint16 q3_outbound_target_x;
    uint16 q3_hold_target_x;
    uint16 q3_switch_min_x;
    uint16 q3_switch_max_x;
    uint16 q4_target_x;
    uint16 q5_target_x;
} question_app_ball_parameter_struct;

typedef enum
{
    QUESTION_APP_Q0_SOFT_START_PHASE_WAIT_CH2 = 0,
    QUESTION_APP_Q0_SOFT_START_PHASE_TO_BASE,
    QUESTION_APP_Q0_SOFT_START_PHASE_TO_TARGET
} question_app_q0_soft_start_phase_enum;

/*
 * 滚球题号参数表。目标单位为 K230 上报的像素 X 坐标；
 * 后续完成每道题的实机标定后，只修改本表。
 */
static const question_app_ball_parameter_struct
    question_app_ball_parameter_table =
{
    546U,   /* Q3 第一阶段目标位置。 */
    234U,   /* Q3 第二阶段保持位置。 */
    526U,   /* Q3 目标切换区间下限。 */
    546U,   /* Q3 目标切换区间上限。 */
    396U,   /* Q4 钢球目标位置。 */
    396U    /* Q5 钢球目标位置。 */
};

#pragma section all "cpu0_dsram"
static volatile question_app_snapshot_struct question_app_snapshot =
{
    QUESTION_APP_QUESTION_DEFAULT,
    QUESTION_APP_STATE_IDLE,
    QUESTION_APP_Q3_STATE_INACTIVE,
    QUESTION_APP_STOP_LINE_WAIT_FIRST,
    0U,
    0U,
    0U,
    0U,
    0.0F,
    ZF_FALSE,
    0U,
    ZF_FALSE,
    TRACK_CONTROL_STATUS_NOT_INITIALIZED,
    BALL_CONTROL_STATUS_NOT_INITIALIZED
};
static volatile uint8 question_app_initialized = ZF_FALSE;
static volatile uint8 question_app_q0_soft_start_track_active = ZF_FALSE;
static volatile question_app_q0_soft_start_phase_enum
    question_app_q0_soft_start_phase =
        QUESTION_APP_Q0_SOFT_START_PHASE_WAIT_CH2;
static volatile uint32 question_app_q0_soft_start_phase_elapsed_ms = 0U;
static volatile uint16 question_app_q6_target_x = 0U;
static volatile uint32 question_app_last_touch_update_count = 0U;
#pragma section all restore

static void question_app_reset_q0_soft_start_state (void)
{
    question_app_q0_soft_start_track_active = ZF_FALSE;
    question_app_q0_soft_start_phase =
        QUESTION_APP_Q0_SOFT_START_PHASE_WAIT_CH2;
    question_app_q0_soft_start_phase_elapsed_ms = 0U;
}

static void question_app_reset_q2_state (void)
{
    question_app_snapshot.stop_line_state =
        QUESTION_APP_STOP_LINE_WAIT_FIRST;
    question_app_snapshot.stop_line_count = 0U;
    question_app_snapshot.stop_line_consecutive_frames = 0U;
    question_app_snapshot.stop_line_cooldown_remaining_ms = 0U;
    question_app_snapshot.q2_elapsed_ms = 0U;
    question_app_snapshot.q2_yaw_deg = 0.0F;
    question_app_snapshot.q2_slowdown_active = ZF_FALSE;
}

static void question_app_reset_q3_state (void)
{
    question_app_snapshot.q3_state = QUESTION_APP_Q3_STATE_INACTIVE;
}

static uint8 question_app_is_ball_question (uint8 question)
{
    return ((QUESTION_APP_QUESTION_BALL_FIRST <= question)
        && (QUESTION_APP_QUESTION_BALL_LAST >= question))
        ? ZF_TRUE
        : ZF_FALSE;
}

static uint8 question_app_is_track_ball_question (uint8 question)
{
    return ((QUESTION_APP_QUESTION_BALL_Q4 == question)
        || (QUESTION_APP_QUESTION_BALL_Q5 == question)
        || (QUESTION_APP_QUESTION_BALL_Q6 == question))
        ? ZF_TRUE
        : ZF_FALSE;
}

#if (QUESTION_APP_Q0_DEBUG_MODE \
    == QUESTION_APP_Q0_DEBUG_MODE_SOFT_START)
static motor_status_enum question_app_start_q0_left_motor (void)
{
    motor_status_enum motor_status;

#if (QUESTION_APP_Q0_SOFT_START_LEFT_MOTOR_USE_A != 0U)
    motor_status = motor_set_B(0);
    if (MOTOR_STATUS_OK == motor_status)
    {
        motor_status =
            motor_set_A(QUESTION_APP_Q0_SOFT_START_LEFT_MOTOR_PWM);
    }
#else
    motor_status = motor_set_A(0);
    if (MOTOR_STATUS_OK == motor_status)
    {
        motor_status =
            motor_set_B(QUESTION_APP_Q0_SOFT_START_LEFT_MOTOR_PWM);
    }
#endif

    return motor_status;
}

static track_control_status_enum question_app_start_q0_track_ramp (void)
{
#if (QUESTION_APP_Q0_SOFT_START_LEFT_MOTOR_USE_A != 0U)
    return track_control_enable_motor_ramp(
        QUESTION_APP_Q0_SOFT_START_LEFT_MOTOR_PWM,
        0,
        QUESTION_APP_Q0_SOFT_START_TRACK_PWM_START,
        QUESTION_APP_Q0_SOFT_START_TRACK_PWM_START,
        (uint16)QUESTION_APP_Q0_SOFT_START_TRACK_RAMP_CYCLES);
#else
    return track_control_enable_motor_ramp(
        0,
        QUESTION_APP_Q0_SOFT_START_LEFT_MOTOR_PWM,
        QUESTION_APP_Q0_SOFT_START_TRACK_PWM_START,
        QUESTION_APP_Q0_SOFT_START_TRACK_PWM_START,
        (uint16)QUESTION_APP_Q0_SOFT_START_TRACK_RAMP_CYCLES);
#endif
}
#endif

static uint8 question_app_get_ball_parameter_set (
    uint8 question,
    ball_control_parameter_set_enum *parameter_set)
{
    if (0 == parameter_set)
    {
        return ZF_FALSE;
    }

    switch (question)
    {
        case QUESTION_APP_QUESTION_BALL_Q3:
            *parameter_set = BALL_CONTROL_PARAMETER_SET_Q3;
            return ZF_TRUE;

        case QUESTION_APP_QUESTION_BALL_Q4:
        case QUESTION_APP_QUESTION_BALL_Q5:
            *parameter_set = BALL_CONTROL_PARAMETER_SET_Q4_Q5;
            return ZF_TRUE;

        case QUESTION_APP_QUESTION_BALL_Q6:
            *parameter_set = BALL_CONTROL_PARAMETER_SET_Q6;
            return ZF_TRUE;

        default:
            return ZF_FALSE;
    }
}

static uint8 question_app_get_ball_target_x (
    uint8 question,
    uint16 *target_x)
{
    if (0 == target_x)
    {
        return ZF_FALSE;
    }

    switch (question)
    {
        case QUESTION_APP_QUESTION_BALL_Q3:
            *target_x =
                question_app_ball_parameter_table.q3_outbound_target_x;
            return ZF_TRUE;

        case QUESTION_APP_QUESTION_BALL_Q4:
            *target_x =
                question_app_ball_parameter_table.q4_target_x;
            return ZF_TRUE;

        case QUESTION_APP_QUESTION_BALL_Q5:
            *target_x =
                question_app_ball_parameter_table.q5_target_x;
            return ZF_TRUE;

        case QUESTION_APP_QUESTION_BALL_Q6:
            *target_x = question_app_q6_target_x;
            return ZF_TRUE;

        default:
            return ZF_FALSE;
    }
}

static uint8 question_app_get_track_pwm_base (
    uint8 question,
    int32 *pwm_base)
{
    if (0 == pwm_base)
    {
        return ZF_FALSE;
    }

    switch (question)
    {
        case QUESTION_APP_QUESTION_TRACK_AND_STOP:
            *pwm_base = QUESTION_APP_Q2_TRACK_PWM_HIGH;
            return ZF_TRUE;

        case QUESTION_APP_QUESTION_BALL_Q4:
        case QUESTION_APP_QUESTION_BALL_Q5:
        case QUESTION_APP_QUESTION_BALL_Q6:
            *pwm_base = QUESTION_APP_Q4_Q5_Q6_TRACK_PWM_BASE;
            return ZF_TRUE;

        default:
            return ZF_FALSE;
    }
}

void question_app_init (void)
{
    question_app_initialized = ZF_FALSE;
    question_app_snapshot.selected_question =
        QUESTION_APP_QUESTION_DEFAULT;
    question_app_snapshot.state = QUESTION_APP_STATE_IDLE;
    question_app_snapshot.ball_target_x = 0U;
    question_app_snapshot.control_status =
        TRACK_CONTROL_STATUS_NOT_INITIALIZED;
    question_app_snapshot.ball_control_status =
        BALL_CONTROL_STATUS_NOT_INITIALIZED;
    question_app_snapshot.q6_target_valid = ZF_FALSE;
    question_app_reset_q0_soft_start_state();
    question_app_q6_target_x = 0U;
    question_app_last_touch_update_count = 0U;
    question_app_reset_q3_state();
    question_app_reset_q2_state();
    question_app_initialized = ZF_TRUE;
}

uint8 question_app_select_next (void)
{
    uint32 interrupt_state;
    uint8 selected_question;
    uint16 target_x;

    interrupt_state = interrupt_global_disable();

    if ((ZF_TRUE == question_app_initialized)
        && (QUESTION_APP_STATE_RUNNING != question_app_snapshot.state))
    {
        selected_question = question_app_snapshot.selected_question;
        selected_question++;
        if (QUESTION_APP_QUESTION_LAST < selected_question)
        {
            selected_question = QUESTION_APP_QUESTION_FIRST;
        }

        question_app_snapshot.selected_question = selected_question;
        question_app_snapshot.state = QUESTION_APP_STATE_IDLE;
        question_app_q6_target_x = 0U;
        question_app_snapshot.q6_target_valid = ZF_FALSE;
        target_x = 0U;
        (void)question_app_get_ball_target_x(selected_question, &target_x);
        question_app_snapshot.ball_target_x = target_x;
        question_app_snapshot.control_status =
            track_control_get_status();
        question_app_snapshot.ball_control_status =
            ball_control_get_status();
        question_app_reset_q0_soft_start_state();
        question_app_reset_q3_state();
        question_app_reset_q2_state();
    }

    selected_question = question_app_snapshot.selected_question;
    interrupt_global_enable(interrupt_state);
    return selected_question;
}

question_app_status_enum question_app_start_selected (void)
{
    uint32 interrupt_state;
    uint8 selected_question;
    uint16 target_x;
    uint8 q6_target_valid;
    int32 track_pwm_base;
    ball_control_parameter_set_enum ball_parameter_set;
    ball_control_status_enum ball_status;
    track_control_status_enum control_status;
    k230_app_snapshot_struct k230_snapshot;
#if (QUESTION_APP_Q0_DEBUG_MODE \
    == QUESTION_APP_Q0_DEBUG_MODE_SOFT_START)
    motor_status_enum motor_status;
#endif

    if (ZF_FALSE == question_app_initialized)
    {
        return QUESTION_APP_STATUS_NOT_INITIALIZED;
    }

    interrupt_state = interrupt_global_disable();
    if (QUESTION_APP_STATE_RUNNING == question_app_snapshot.state)
    {
        interrupt_global_enable(interrupt_state);
        return QUESTION_APP_STATUS_ALREADY_RUNNING;
    }

    selected_question = question_app_snapshot.selected_question;
    q6_target_valid = question_app_snapshot.q6_target_valid;
    question_app_snapshot.state = QUESTION_APP_STATE_IDLE;
    question_app_reset_q0_soft_start_state();
    question_app_reset_q3_state();
    question_app_reset_q2_state();
    interrupt_global_enable(interrupt_state);

    if ((QUESTION_APP_QUESTION_BALL_Q6 == selected_question)
        && (ZF_FALSE == q6_target_valid))
    {
        /*
         * 未显式设置 Touch_X 时，以 KEY_4 发车前最新的有效 Ball_X 作为本次
         * Q6 目标。只在这里锁存一次；进入 RUNNING 后不再跟随 Ball_X 变化。
         */
        if ((ZF_FALSE == k230_app_get_snapshot(&k230_snapshot))
            || (ZF_FALSE == k230_snapshot.position_valid))
        {
            return QUESTION_APP_STATUS_TARGET_NOT_SET;
        }

        target_x = k230_snapshot.ball_x;
        interrupt_state = interrupt_global_disable();
        question_app_q6_target_x = target_x;
        question_app_snapshot.ball_target_x = target_x;
        question_app_snapshot.q6_target_valid = ZF_TRUE;
        interrupt_global_enable(interrupt_state);
    }

#if (QUESTION_APP_Q0_DEBUG_MODE \
    == QUESTION_APP_Q0_DEBUG_MODE_GYRO_LOOP)
    if (QUESTION_APP_QUESTION_K230_MONITOR == selected_question)
    {
        /*
         * Q0 调试只使能零目标角速度环。task 在状态提交前仍只采样并复位
         * 分频，不会提前执行一次控制。
         */
        control_status = track_control_enable_gyro_debug();

        interrupt_state = interrupt_global_disable();
        question_app_snapshot.ball_target_x = 0U;
        question_app_snapshot.control_status = control_status;
        question_app_snapshot.ball_control_status =
            ball_control_get_status();
        if (TRACK_CONTROL_STATUS_OK == control_status)
        {
            question_app_snapshot.state = QUESTION_APP_STATE_RUNNING;
        }
        else
        {
            question_app_snapshot.state = QUESTION_APP_STATE_FAULT;
        }
        interrupt_global_enable(interrupt_state);

        return (TRACK_CONTROL_STATUS_OK == control_status)
            ? QUESTION_APP_STATUS_OK
            : QUESTION_APP_STATUS_CONTROL_REJECTED;
    }
#elif (QUESTION_APP_Q0_DEBUG_MODE \
    == QUESTION_APP_Q0_DEBUG_MODE_ACCEL_FF)
    if (QUESTION_APP_QUESTION_K230_MONITOR == selected_question)
    {
        /*
         * Q0 前馈调试固定复用 Q4/Q5 参数表，只验证加速度前馈本身。Q6
         * 正常题目控制使用独立的 Q6 参数表，同样启用前馈。该调试入口不
         * 读取 K230，task 也不会运行循迹或两级滚球 PI。
         */
        ball_status =
            ball_control_enable_acceleration_feedforward_debug();

        interrupt_state = interrupt_global_disable();
        question_app_snapshot.ball_target_x = 0U;
        question_app_snapshot.control_status =
            track_control_get_status();
        question_app_snapshot.ball_control_status = ball_status;
        if (BALL_CONTROL_STATUS_OK == ball_status)
        {
            question_app_snapshot.state = QUESTION_APP_STATE_RUNNING;
        }
        else
        {
            question_app_snapshot.state = QUESTION_APP_STATE_FAULT;
        }
        interrupt_global_enable(interrupt_state);

        return (BALL_CONTROL_STATUS_OK == ball_status)
            ? QUESTION_APP_STATUS_OK
            : QUESTION_APP_STATUS_CONTROL_REJECTED;
    }
#elif (QUESTION_APP_Q0_DEBUG_MODE \
    == QUESTION_APP_Q0_DEBUG_MODE_SOFT_START)
    if (QUESTION_APP_QUESTION_K230_MONITOR == selected_question)
    {
        /*
         * 先撤销任何遗留循迹状态，再在同一个临界区内使能 X=396 滚球闭环、
         * 施加单左轮 500 并提交 RUNNING。状态提交前的 3 ms 中断看不到半成品。
         */
        control_status = track_control_disable();
        if (TRACK_CONTROL_STATUS_OK != control_status)
        {
            interrupt_state = interrupt_global_disable();
            question_app_snapshot.ball_target_x =
                QUESTION_APP_Q0_SOFT_START_BALL_TARGET_X;
            question_app_snapshot.control_status = control_status;
            question_app_snapshot.ball_control_status =
                ball_control_get_status();
            question_app_snapshot.state = QUESTION_APP_STATE_FAULT;
            interrupt_global_enable(interrupt_state);
            return QUESTION_APP_STATUS_CONTROL_REJECTED;
        }

        interrupt_state = interrupt_global_disable();
        ball_status = ball_control_enable(
            QUESTION_APP_Q0_SOFT_START_BALL_TARGET_X,
            BALL_CONTROL_PARAMETER_SET_Q4_Q5);
        motor_status = (BALL_CONTROL_STATUS_OK == ball_status)
            ? question_app_start_q0_left_motor()
            : MOTOR_STATUS_NOT_INITIALIZED;
        question_app_snapshot.ball_target_x =
            QUESTION_APP_Q0_SOFT_START_BALL_TARGET_X;
        question_app_snapshot.control_status =
            track_control_get_status();
        question_app_snapshot.ball_control_status = ball_status;
        if ((BALL_CONTROL_STATUS_OK == ball_status)
            && (MOTOR_STATUS_OK == motor_status))
        {
            question_app_snapshot.state = QUESTION_APP_STATE_RUNNING;
        }
        else
        {
            question_app_snapshot.state = QUESTION_APP_STATE_FAULT;
        }
        interrupt_global_enable(interrupt_state);

        if ((BALL_CONTROL_STATUS_OK != ball_status)
            || (MOTOR_STATUS_OK != motor_status))
        {
            (void)motor_stop_all();
            if (ZF_TRUE == ball_control_is_enabled())
            {
                ball_status = ball_control_disable();
                interrupt_state = interrupt_global_disable();
                question_app_snapshot.ball_control_status =
                    (BALL_CONTROL_STATUS_OK == ball_status)
                    ? BALL_CONTROL_STATUS_DISABLED
                    : ball_status;
                interrupt_global_enable(interrupt_state);
            }
            return QUESTION_APP_STATUS_CONTROL_REJECTED;
        }

        return QUESTION_APP_STATUS_OK;
    }
#endif

    if (QUESTION_APP_QUESTION_TRACK_AND_STOP == selected_question)
    {
        /*
         * task 只有在业务状态变为 RUNNING 后才执行循迹 PID。因此 enable() 与
         * 下面的状态提交之间即使插入一个 3 ms 中断，也只会采样并复位分频。
         */
        if (ZF_FALSE == question_app_get_track_pwm_base(
                selected_question,
                &track_pwm_base))
        {
            return QUESTION_APP_STATUS_UNSUPPORTED_QUESTION;
        }
        control_status = track_control_enable(track_pwm_base);

        interrupt_state = interrupt_global_disable();
        question_app_snapshot.ball_target_x = 0U;
        question_app_snapshot.control_status = control_status;
        question_app_snapshot.ball_control_status =
            ball_control_get_status();
        if (TRACK_CONTROL_STATUS_OK == control_status)
        {
            question_app_snapshot.state = QUESTION_APP_STATE_RUNNING;
        }
        else
        {
            question_app_snapshot.state = QUESTION_APP_STATE_FAULT;
        }
        interrupt_global_enable(interrupt_state);

        return (TRACK_CONTROL_STATUS_OK == control_status)
            ? QUESTION_APP_STATUS_OK
            : QUESTION_APP_STATUS_CONTROL_REJECTED;
    }

    if (ZF_TRUE == question_app_is_track_ball_question(selected_question))
    {
        /*
         * Q4/Q5/Q6 共用“循迹 + 稳球”运行模式和同一套车体缓启动。先使能
         * 从 400 到 1700、共 350 个成功 6 ms 电机输出周期的车体软启动；业务状态
         * 仍为 IDLE，因此两个控制器都不会提前执行。随后在同一临界区使能滚球
         * 并提交 RUNNING，确保第一拍看到完整双控制器状态。滚球参数仍按题号
         * 显式选择，Q6 不与 Q4/Q5 共用滚球参数表。
         */
        if (ZF_FALSE
            == question_app_get_ball_target_x(selected_question, &target_x))
        {
            return QUESTION_APP_STATUS_UNSUPPORTED_QUESTION;
        }
        if (ZF_FALSE == question_app_get_ball_parameter_set(
                selected_question,
                &ball_parameter_set))
        {
            return QUESTION_APP_STATUS_UNSUPPORTED_QUESTION;
        }

        if (ZF_FALSE == question_app_get_track_pwm_base(
                selected_question,
                &track_pwm_base))
        {
            return QUESTION_APP_STATUS_UNSUPPORTED_QUESTION;
        }
        control_status = track_control_enable_ramp(
            QUESTION_APP_Q4_Q5_Q6_TRACK_PWM_START,
            track_pwm_base,
            QUESTION_APP_Q4_Q5_Q6_TRACK_RAMP_INNER_CYCLES);
        if (TRACK_CONTROL_STATUS_OK != control_status)
        {
            interrupt_state = interrupt_global_disable();
            question_app_snapshot.ball_target_x = target_x;
            question_app_snapshot.control_status = control_status;
            question_app_snapshot.ball_control_status =
                ball_control_get_status();
            question_app_snapshot.state = QUESTION_APP_STATE_FAULT;
            interrupt_global_enable(interrupt_state);
            return QUESTION_APP_STATUS_CONTROL_REJECTED;
        }

        interrupt_state = interrupt_global_disable();
        ball_status = ball_control_enable(target_x, ball_parameter_set);
        question_app_snapshot.ball_target_x = target_x;
        question_app_snapshot.control_status = control_status;
        question_app_snapshot.ball_control_status = ball_status;
        if (BALL_CONTROL_STATUS_OK == ball_status)
        {
            question_app_snapshot.state = QUESTION_APP_STATE_RUNNING;
        }
        else
        {
            question_app_snapshot.state = QUESTION_APP_STATE_FAULT;
        }
        interrupt_global_enable(interrupt_state);

        if (BALL_CONTROL_STATUS_OK != ball_status)
        {
            control_status = track_control_disable();
            interrupt_state = interrupt_global_disable();
            question_app_snapshot.control_status =
                (TRACK_CONTROL_STATUS_OK == control_status)
                ? TRACK_CONTROL_STATUS_DISABLED
                : control_status;
            interrupt_global_enable(interrupt_state);
            return QUESTION_APP_STATUS_CONTROL_REJECTED;
        }

        return QUESTION_APP_STATUS_OK;
    }

    if (ZF_FALSE
        == question_app_get_ball_target_x(selected_question, &target_x))
    {
        return QUESTION_APP_STATUS_UNSUPPORTED_QUESTION;
    }
    if (ZF_FALSE == question_app_get_ball_parameter_set(
            selected_question,
            &ball_parameter_set))
    {
        return QUESTION_APP_STATUS_UNSUPPORTED_QUESTION;
    }

    /*
     * ball_control_enable() 装入显式映射的 Q3、Q4/Q5 或 Q6 参数集，但不
     * 发送也不等待。使能与题目状态提交必须位于同一个临界区，防止 18 ms
     * 中断在 RUNNING 状态可见前执行新控制器。
     */
    interrupt_state = interrupt_global_disable();
    ball_status = ball_control_enable(target_x, ball_parameter_set);
    question_app_snapshot.ball_target_x = target_x;
    question_app_snapshot.control_status = track_control_get_status();
    question_app_snapshot.ball_control_status = ball_status;
    if (BALL_CONTROL_STATUS_OK == ball_status)
    {
        question_app_snapshot.state = QUESTION_APP_STATE_RUNNING;
        if (QUESTION_APP_QUESTION_BALL_Q3 == selected_question)
        {
            question_app_snapshot.q3_state =
                QUESTION_APP_Q3_STATE_MOVE_TO_546;
        }
    }
    else
    {
        question_app_snapshot.state = QUESTION_APP_STATE_FAULT;
    }
    interrupt_global_enable(interrupt_state);

    return (BALL_CONTROL_STATUS_OK == ball_status)
        ? QUESTION_APP_STATUS_OK
        : QUESTION_APP_STATUS_CONTROL_REJECTED;
}

uint8 question_app_process_k230_touch (void)
{
    uint32 interrupt_state;
    uint8 target_changed = ZF_FALSE;
    k230_app_snapshot_struct k230_snapshot;

    if ((ZF_FALSE == question_app_initialized)
        || (ZF_FALSE == k230_app_get_snapshot(&k230_snapshot))
        || (ZF_FALSE == k230_snapshot.touch_valid))
    {
        return ZF_FALSE;
    }

    interrupt_state = interrupt_global_disable();
    if (question_app_last_touch_update_count
        == k230_snapshot.touch_update_count)
    {
        interrupt_global_enable(interrupt_state);
        return ZF_FALSE;
    }

    /*
     * 在所有题号和状态下推进已见计数，避免旧点击在后来切到 Q6 或重新停车
     * 时被误用。K230 的三连发携带相同 X；目标未变化时只消费。
     */
    question_app_last_touch_update_count =
        k230_snapshot.touch_update_count;
    if ((QUESTION_APP_QUESTION_BALL_Q6
            != question_app_snapshot.selected_question)
        || (QUESTION_APP_STATE_IDLE != question_app_snapshot.state)
        || ((ZF_TRUE == question_app_snapshot.q6_target_valid)
            && (question_app_q6_target_x == k230_snapshot.touch_x)))
    {
        interrupt_global_enable(interrupt_state);
        return ZF_FALSE;
    }

    question_app_q6_target_x = k230_snapshot.touch_x;
    question_app_snapshot.ball_target_x = k230_snapshot.touch_x;
    question_app_snapshot.q6_target_valid = ZF_TRUE;
    target_changed = ZF_TRUE;

    interrupt_global_enable(interrupt_state);
    return target_changed;
}

void question_app_process_ball_irq (void)
{
    ball_control_snapshot_struct ball_snapshot;
    ball_control_status_enum ball_status;

    if ((ZF_FALSE == question_app_initialized)
        || (QUESTION_APP_STATE_RUNNING != question_app_snapshot.state)
        || (QUESTION_APP_QUESTION_BALL_Q3
            != question_app_snapshot.selected_question)
        || (QUESTION_APP_Q3_STATE_MOVE_TO_546
            != question_app_snapshot.q3_state))
    {
        return;
    }

    if ((ZF_FALSE == ball_control_get_snapshot(&ball_snapshot))
        || (ZF_FALSE == ball_snapshot.enabled)
        || (ZF_FALSE == ball_snapshot.position_valid)
        || (question_app_ball_parameter_table.q3_switch_min_x
            > ball_snapshot.actual_x)
        || (question_app_ball_parameter_table.q3_switch_max_x
            < ball_snapshot.actual_x))
    {
        return;
    }

    /*
     * 在下一个 18 ms 控制步骤前锁存目标切换。切换时保留两级 PI 状态和
     * 速度历史；重新启动题目或复位前，Q3 不会再返回目标 546。
     */
    ball_status = ball_control_set_target(
        question_app_ball_parameter_table.q3_hold_target_x);
    question_app_snapshot.ball_control_status =
        (BALL_CONTROL_STATUS_OK == ball_status)
        ? ball_control_get_status()
        : ball_status;

    if (BALL_CONTROL_STATUS_OK == ball_status)
    {
        question_app_snapshot.ball_target_x =
            question_app_ball_parameter_table.q3_hold_target_x;
        question_app_snapshot.q3_state =
            QUESTION_APP_Q3_STATE_HOLD_244;
        return;
    }

    /* 目标切换失败只保留诊断状态，不关闭滚球或改变题目运行态。 */
}

void question_app_process_imu_irq (
    const volatile ImuAppData *imu_data,
    uint16 elapsed_ms)
{
    float32 yaw_abs_deg;
    float32 yaw_delta_deg;
    track_control_status_enum control_status;

#if (QUESTION_APP_Q0_DEBUG_MODE \
    == QUESTION_APP_Q0_DEBUG_MODE_SOFT_START)
    if ((ZF_TRUE == question_app_initialized)
        && (QUESTION_APP_STATE_RUNNING == question_app_snapshot.state)
        && (QUESTION_APP_QUESTION_K230_MONITOR
            == question_app_snapshot.selected_question)
        && (QUESTION_APP_Q0_SOFT_START_PHASE_TO_BASE
            == question_app_q0_soft_start_phase))
    {
        if ((0xFFFFFFFFU
                - question_app_q0_soft_start_phase_elapsed_ms)
            < (uint32)elapsed_ms)
        {
            question_app_q0_soft_start_phase_elapsed_ms = 0xFFFFFFFFU;
        }
        else
        {
            question_app_q0_soft_start_phase_elapsed_ms +=
                (uint32)elapsed_ms;
        }

        if (question_app_q0_soft_start_phase_elapsed_ms
            < QUESTION_APP_Q0_SOFT_START_TRACK_RAMP_MS)
        {
            return;
        }

        control_status = track_control_set_motor_ramp_irq(
            QUESTION_APP_Q0_SOFT_START_TRACK_PWM_START,
            QUESTION_APP_Q0_SOFT_START_TRACK_PWM_START,
            QUESTION_APP_Q0_SOFT_START_TRACK_PWM_TARGET,
            QUESTION_APP_Q0_SOFT_START_TRACK_PWM_TARGET,
            (uint16)QUESTION_APP_Q0_SOFT_START_TRACK_RAMP_CYCLES);
        question_app_snapshot.control_status = control_status;
        if (TRACK_CONTROL_STATUS_OK == control_status)
        {
            question_app_q0_soft_start_phase =
                QUESTION_APP_Q0_SOFT_START_PHASE_TO_TARGET;
            question_app_q0_soft_start_phase_elapsed_ms = 0U;
            return;
        }

        /* 保持第一段斜坡和两个控制器运行，下一拍继续尝试切换参数。 */
        return;
    }
#endif

    if ((ZF_FALSE == question_app_initialized)
        || (QUESTION_APP_STATE_RUNNING != question_app_snapshot.state)
        || (QUESTION_APP_QUESTION_TRACK_AND_STOP
            != question_app_snapshot.selected_question))
    {
        return;
    }

    if ((QUESTION_APP_Q2_ELAPSED_MS_MAX
            - question_app_snapshot.q2_elapsed_ms)
        < (uint32)elapsed_ms)
    {
        question_app_snapshot.q2_elapsed_ms =
            QUESTION_APP_Q2_ELAPSED_MS_MAX;
    }
    else
    {
        question_app_snapshot.q2_elapsed_ms += (uint32)elapsed_ms;
    }

    if ((0 == imu_data)
        || (ZF_FALSE == imu_data->initialized)
        || (ZF_FALSE == imu_data->offset_valid))
    {
        return;
    }

    yaw_delta_deg = imu_app_gyro_lsb_to_dps(
        imu_data->corrected_z_lsb)
        * ((float32)elapsed_ms / 1000.0F);
    question_app_snapshot.q2_yaw_deg += yaw_delta_deg;

    if (ZF_TRUE == question_app_snapshot.q2_slowdown_active)
    {
        return;
    }

    yaw_abs_deg = question_app_snapshot.q2_yaw_deg;
    if (0.0F > yaw_abs_deg)
    {
        yaw_abs_deg = -yaw_abs_deg;
    }
    if (QUESTION_APP_Q2_SLOWDOWN_YAW_DEG > yaw_abs_deg)
    {
        return;
    }

    control_status = track_control_set_pwm_base_irq(
        QUESTION_APP_Q2_TRACK_PWM_LOW);
    question_app_snapshot.control_status = control_status;
    if (TRACK_CONTROL_STATUS_OK == control_status)
    {
        question_app_snapshot.q2_slowdown_active = ZF_TRUE;
        return;
    }

    /* 保持原基础 PWM 继续运行，后续拍继续尝试降速。 */
}

void question_app_process_gray_irq (
    gw_gray_status_enum gray_status,
    const gray_app_sample_struct *gray_sample,
    uint16 elapsed_ms)
{
    track_control_status_enum control_status;

#if (QUESTION_APP_Q0_DEBUG_MODE \
    == QUESTION_APP_Q0_DEBUG_MODE_SOFT_START)
    if ((ZF_TRUE == question_app_initialized)
        && (QUESTION_APP_STATE_RUNNING == question_app_snapshot.state)
        && (QUESTION_APP_QUESTION_K230_MONITOR
            == question_app_snapshot.selected_question))
    {
        if ((ZF_TRUE == question_app_q0_soft_start_track_active)
            || (GW_GRAY_STATUS_OK != gray_status)
            || (0 == gray_sample)
            || (0U != (gray_sample->digital_value
                & QUESTION_APP_Q0_SOFT_START_TRIGGER_MASK)))
        {
            return;
        }

        /*
         * CH2 的首个有效低电平样本立即接管单轮阶段。task 随后在同一拍重新
         * 检查 track_running，并以本拍灰度快照执行外环和首个
         * A=500/B=0 基础值的循迹内环。
         */
        control_status = question_app_start_q0_track_ramp();
        question_app_snapshot.control_status = control_status;
        if (TRACK_CONTROL_STATUS_OK == control_status)
        {
            question_app_q0_soft_start_track_active = ZF_TRUE;
            question_app_q0_soft_start_phase =
                QUESTION_APP_Q0_SOFT_START_PHASE_TO_BASE;
            question_app_q0_soft_start_phase_elapsed_ms = 0U;
            return;
        }

        /* 保持单左轮与滚球运行，后续有效 CH2 样本继续尝试接管循迹。 */
        return;
    }
#endif

    if ((ZF_FALSE == question_app_initialized)
        || (QUESTION_APP_STATE_RUNNING != question_app_snapshot.state)
        || (QUESTION_APP_QUESTION_TRACK_AND_STOP
            != question_app_snapshot.selected_question))
    {
        return;
    }

    if (QUESTION_APP_STOP_LINE_COOLDOWN
        == question_app_snapshot.stop_line_state)
    {
        question_app_snapshot.stop_line_consecutive_frames = 0U;

        if (elapsed_ms
            >= question_app_snapshot.stop_line_cooldown_remaining_ms)
        {
            question_app_snapshot.stop_line_cooldown_remaining_ms = 0U;
        }
        else
        {
            question_app_snapshot.stop_line_cooldown_remaining_ms =
                (uint16)(
                    question_app_snapshot.stop_line_cooldown_remaining_ms
                    - elapsed_ms);
        }

        if (0U == question_app_snapshot.stop_line_cooldown_remaining_ms)
        {
            question_app_snapshot.stop_line_state =
                QUESTION_APP_STOP_LINE_WAIT_SECOND;
        }
        return;
    }

    if ((QUESTION_APP_STOP_LINE_WAIT_FIRST
            != question_app_snapshot.stop_line_state)
        && (QUESTION_APP_STOP_LINE_WAIT_SECOND
            != question_app_snapshot.stop_line_state))
    {
        question_app_snapshot.stop_line_consecutive_frames = 0U;
        return;
    }

    if ((GW_GRAY_STATUS_OK != gray_status)
        || (0 == gray_sample))
    {
        question_app_snapshot.stop_line_consecutive_frames = 0U;
        return;
    }

    /*
     * 停车线样本必须同时满足“黑线通道数不少于 4”和“CH4、CH5 都为
     * 低电平黑线”。任何不合格样本都会打断连续两帧确认。
     */
    if ((QUESTION_APP_STOP_LINE_ERROR_NUM_THRESHOLD
            > gray_sample->error_num)
        || (0U != (gray_sample->digital_value
            & QUESTION_APP_STOP_LINE_REQUIRED_BLACK_MASK)))
    {
        question_app_snapshot.stop_line_consecutive_frames = 0U;
        return;
    }

    if (QUESTION_APP_COUNTER_MAX
        > question_app_snapshot.stop_line_consecutive_frames)
    {
        question_app_snapshot.stop_line_consecutive_frames++;
    }

    if (QUESTION_APP_STOP_LINE_CONFIRM_FRAMES
        > question_app_snapshot.stop_line_consecutive_frames)
    {
        return;
    }

    question_app_snapshot.stop_line_consecutive_frames = 0U;

    if (QUESTION_APP_COUNTER_MAX
        > question_app_snapshot.stop_line_count)
    {
        question_app_snapshot.stop_line_count++;
    }

    if (QUESTION_APP_STOP_LINE_WAIT_FIRST
        == question_app_snapshot.stop_line_state)
    {
        question_app_snapshot.stop_line_state =
            QUESTION_APP_STOP_LINE_COOLDOWN;
        question_app_snapshot.stop_line_cooldown_remaining_ms =
            QUESTION_APP_STOP_LINE_COOLDOWN_MS;
        return;
    }

    /*
     * 先撤销 RUNNING，再停循迹。task 在本函数返回后会重新检查状态，所以第二
     * 次确认的这一拍不会继续进入外环、内环或重新写入非零电机命令。
     */
    question_app_snapshot.stop_line_state =
        QUESTION_APP_STOP_LINE_COMPLETE;
    question_app_snapshot.state = QUESTION_APP_STATE_FINISHED;
    control_status = track_control_disable();
    question_app_snapshot.control_status =
        (TRACK_CONTROL_STATUS_OK == control_status)
        ? TRACK_CONTROL_STATUS_DISABLED
        : control_status;

    if (TRACK_CONTROL_STATUS_OK != control_status)
    {
        question_app_snapshot.state = QUESTION_APP_STATE_FAULT;
    }
}

void question_app_sync_control_irq (void)
{
    if (QUESTION_APP_STATE_RUNNING != question_app_snapshot.state)
    {
        return;
    }

#if (QUESTION_APP_Q0_DEBUG_MODE \
    == QUESTION_APP_Q0_DEBUG_MODE_GYRO_LOOP)
    if (QUESTION_APP_QUESTION_K230_MONITOR
        == question_app_snapshot.selected_question)
    {
        question_app_snapshot.control_status =
            track_control_get_status();
        return;
    }
#elif (QUESTION_APP_Q0_DEBUG_MODE \
    == QUESTION_APP_Q0_DEBUG_MODE_ACCEL_FF)
    if (QUESTION_APP_QUESTION_K230_MONITOR
        == question_app_snapshot.selected_question)
    {
        question_app_snapshot.ball_control_status =
            ball_control_get_status();
        return;
    }
#elif (QUESTION_APP_Q0_DEBUG_MODE \
    == QUESTION_APP_Q0_DEBUG_MODE_SOFT_START)
    if (QUESTION_APP_QUESTION_K230_MONITOR
        == question_app_snapshot.selected_question)
    {
        question_app_snapshot.control_status =
            track_control_get_status();
        question_app_snapshot.ball_control_status =
            ball_control_get_status();
        return;
    }
#endif

    if (QUESTION_APP_QUESTION_TRACK_AND_STOP
        == question_app_snapshot.selected_question)
    {
        question_app_snapshot.control_status =
            track_control_get_status();
        return;
    }

    if (ZF_TRUE == question_app_is_track_ball_question(
            question_app_snapshot.selected_question))
    {
        question_app_snapshot.control_status =
            track_control_get_status();
        question_app_snapshot.ball_control_status =
            ball_control_get_status();
        return;
    }

    if (ZF_TRUE == question_app_is_ball_question(
            question_app_snapshot.selected_question))
    {
        question_app_snapshot.ball_control_status =
            ball_control_get_status();
    }
}

uint8 question_app_is_running (void)
{
    return ((ZF_TRUE == question_app_initialized)
        && (QUESTION_APP_STATE_RUNNING == question_app_snapshot.state))
        ? ZF_TRUE
        : ZF_FALSE;
}

uint8 question_app_is_gyro_debug_running (void)
{
#if (QUESTION_APP_Q0_DEBUG_MODE \
    == QUESTION_APP_Q0_DEBUG_MODE_GYRO_LOOP)
    return ((ZF_TRUE == question_app_is_running())
        && (QUESTION_APP_QUESTION_K230_MONITOR
            == question_app_snapshot.selected_question))
        ? ZF_TRUE
        : ZF_FALSE;
#else
    return ZF_FALSE;
#endif
}

uint8 question_app_is_acceleration_feedforward_debug_running (void)
{
#if (QUESTION_APP_Q0_DEBUG_MODE \
    == QUESTION_APP_Q0_DEBUG_MODE_ACCEL_FF)
    return ((ZF_TRUE == question_app_is_running())
        && (QUESTION_APP_QUESTION_K230_MONITOR
            == question_app_snapshot.selected_question))
        ? ZF_TRUE
        : ZF_FALSE;
#else
    return ZF_FALSE;
#endif
}

uint8 question_app_is_acceleration_feedforward_active_irq (void)
{
    return ((ZF_TRUE == question_app_initialized)
        && (QUESTION_APP_STATE_RUNNING == question_app_snapshot.state)
        && (ZF_TRUE == question_app_is_track_ball_question(
            question_app_snapshot.selected_question))
        && (ZF_TRUE == track_control_is_ramp_active_irq()))
        ? ZF_TRUE
        : ZF_FALSE;
}

uint8 question_app_is_track_running (void)
{
    return ((ZF_TRUE == question_app_is_running())
        && ((QUESTION_APP_QUESTION_TRACK_AND_STOP
                == question_app_snapshot.selected_question)
            || (ZF_TRUE == question_app_is_track_ball_question(
                question_app_snapshot.selected_question))
#if (QUESTION_APP_Q0_DEBUG_MODE \
    == QUESTION_APP_Q0_DEBUG_MODE_SOFT_START)
            || ((QUESTION_APP_QUESTION_K230_MONITOR
                    == question_app_snapshot.selected_question)
                && (ZF_TRUE
                    == question_app_q0_soft_start_track_active))
#endif
            ))
        ? ZF_TRUE
        : ZF_FALSE;
}

uint8 question_app_is_ball_running (void)
{
    return ((ZF_TRUE == question_app_is_running())
        && ((ZF_TRUE == question_app_is_ball_question(
                question_app_snapshot.selected_question))
#if (QUESTION_APP_Q0_DEBUG_MODE \
    == QUESTION_APP_Q0_DEBUG_MODE_SOFT_START)
            || (QUESTION_APP_QUESTION_K230_MONITOR
                == question_app_snapshot.selected_question)
#endif
            ))
        ? ZF_TRUE
        : ZF_FALSE;
}

uint8 question_app_get_snapshot (
    question_app_snapshot_struct *snapshot)
{
    uint32 interrupt_state;

    if ((0 == snapshot) || (ZF_FALSE == question_app_initialized))
    {
        return ZF_FALSE;
    }

    interrupt_state = interrupt_global_disable();
    *snapshot = question_app_snapshot;
    interrupt_global_enable(interrupt_state);
    return ZF_TRUE;
}
