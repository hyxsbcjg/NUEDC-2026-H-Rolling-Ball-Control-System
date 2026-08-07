#include "ball_control.h"

#include "k230_app.h"
#include "pid.h"
#include "step_motor.h"
#include "zf_common_interrupt.h"

#define BALL_CONTROL_UPDATE_COUNT_MAX           (0xFFFFFFFFU)

typedef struct
{
    float32 position_outer_kp;
    float32 position_outer_ki;
    float32 position_integral_window_pixel;
    float32 position_integral_min_velocity_pixel_per_s;
    float32 position_integral_max_velocity_pixel_per_s;
    float32 target_velocity_limit_pixel_per_s;
    float32 velocity_inner_kp;
    float32 velocity_inner_ki;
    float32 velocity_filter_alpha;
    float32 acceleration_feedforward_gain_pulse_per_lsb;
    float32 acceleration_feedforward_deadband_lsb;
    float32 acceleration_feedforward_input_limit_lsb;
    float32 acceleration_feedforward_output_limit_pulse;
    float32 acceleration_feedforward_slew_limit_pulse_per_step;
    float32 breakaway_positive_min_pulse;
    float32 breakaway_negative_max_pulse;
    float32 breakaway_position_error_deadband_pixel;
    float32 breakaway_stationary_velocity_limit_pixel_per_s;
    float32 position_error_limit_pixel;
    float32 velocity_integral_min_pulse;
    float32 velocity_integral_max_pulse;
} ball_control_parameter_struct;


/*
 * 钢球位置/速度串级控制参数表。
 *
 * 位置误差 = 限幅(目标位置 - 实际位置)
 * 位置积分 = 限幅(原位置积分 + 位置积分系数 × 位置误差 × 外环时间)
 * 目标速度 = 限幅(位置比例系数 × 位置误差 + 位置积分)
 * 滤波速度 = 一阶低通(实际位置变化量 / 速度观察时间)
 * 速度误差 = 目标速度 - 滤波速度
 * 速度积分 = 限幅(原速度积分 + 速度积分系数 × 速度误差 × 内环时间)
 * 加速度前馈 = 带符号增益 × 死区和限幅处理后的去零漂 Y 轴加速度
 * 原始控制器输出 = 速度 PI 输出 + 加速度前馈
 * 起滚补偿 = 钢球近似静止且位置误差超出小窗口时，按位置误差方向把原始输出
 *              提升到对应的最小绝对脉冲
 * 控制器输出 = 起滚补偿(原始控制器输出)
 * 绝对脉冲 = 当前题号绝对中心脉冲 + 控制器输出
 *
 * 位置积分采用积分分离：仅当位置误差绝对值不超过参数表中的积分窗口时
 * 累计，窗口外冻结已有积分；使能、禁用、失帧和电机故障都会清零。位置
 * 积分输出单位为 pixel/s，并受参数表中的独立上下限约束，避免它主导位置
 * 比例输出。当前位置和速度增益已经在原单环 PV 的等效起点上再次整定，
 * 不再要求与旧 PV 完全等效。
 * Q3、Q4/Q5、Q6 的参数所有权严格分离。Q3 关闭加速度前馈；Q4/Q5 与
 * Q6 只在对应车体缓启动尚未完成时允许前馈，并启用各自保存的实测起滚
 * 补偿；位置/速度 PI 参数仍按题号独立整定。
 *
 * 速度正方向定义为摄像头 X 增大，因此位置 Kp/Ki 应为正。若实机连杆方向
 * 形成正反馈，应同时反转速度内环 Kp/Ki。Q3/Q4/Q5 的中心为脉冲 0；Q6
 * 使用停车态提交的 RAM 绝对中心，且不会修改驱动器机械零点。
 */
static const ball_control_parameter_struct ball_control_q3_parameter_table =
{
    1.65F,      /* 位置外环比例系数，单位为 pixel/s 每 pixel。 */
    10.0F,      /* 位置外环积分系数，只在积分窗口内累计。 */
    82.0F,      /* 位置积分窗口，单位为 pixel。 */
    -70.0F,     /* 位置积分输出下限，单位为 pixel/s。 */
    70.0F,      /* 位置积分输出上限，单位为 pixel/s。 */
    1000.0F,    /* 目标速度对称限幅，单位为 pixel/s。 */
    -0.8F,      /* 速度内环比例系数，输出单位为 pulse。 */
    -0.08F,     /* 速度内环积分系数，输出单位为 pulse。 */
    0.25F,      /* 速度一阶低通中新样本的权重。 */
    0.0F,       /* Q3 前馈增益为 0，明确关闭加速度前馈。 */
    64.0F,      /* 前馈死区，单位为去零漂加速度 raw LSB；Q3 不使用。 */
    1250.0F,    /* 前馈输入对称限幅，单位为 raw LSB；Q3 不使用。 */
    0.0F,       /* Q3 前馈输出限幅为 0，因此不会产生前馈脉冲。 */
    36.0F,      /* 前馈每个 18 ms 控制槽的最大变化量；Q3 不使用。 */
    85.0F,      /* 实测正脉冲起滚下限：正脉冲使 Ball_X 减小。 */
    -100.0F,    /* 实测负脉冲起滚上限：负脉冲使 Ball_X 增大。 */
    15.0F,       /* |位置误差| <= 15 pixel 时不做起滚补偿。 */
    30.0F,      /* |滤波速度| <= 30 pixel/s 视为近似静止。 */
    10000.0F,   /* 位置误差对称保护限幅，单位为 pixel。 */
    -80.0F,     /* 速度积分输出下限，单位为 pulse。 */
    80.0F       /* 速度积分输出上限，单位为 pulse。 */
};

static const ball_control_parameter_struct
    ball_control_q4_q5_parameter_table =
{
        0.92F,      /* 位置外环比例系数，单位为 pixel/s 每 pixel。 */
        0.0F,       /* 位置外环积分系数；Q4/Q5 当前不累计位置积分。 */
        60.0F,      /* 位置积分窗口，单位为 pixel。 */
        -30.0F,     /* 位置积分输出下限，单位为 pixel/s。 */
        30.0F,      /* 位置积分输出上限，单位为 pixel/s。 */
        1000.0F,    /* 目标速度对称限幅，单位为 pixel/s。 */
        -0.92F,      /* 速度内环比例系数，输出单位为 pulse。 */
        0,      /* 速度内环积分系数，输出单位为 pulse。 */
        0.25F,      /* 速度一阶低通中新样本的权重。 */
        -1.3F,      /* 前馈增益，单位为 pulse/LSB；正 Y 加速度产生负脉冲。 */
        52.0F,      /* 前馈死区，单位为去零漂加速度 raw LSB。 */
        1250.0F,    /* 前馈输入对称限幅，单位为 raw LSB。 */
        200.0F,    /* 前馈输出对称限幅；仅在 Q4/Q5 车体缓启动阶段启用。 */
        30.0F,      /* 前馈每个 18 ms 控制槽的最大变化量，单位为 pulse。 */
        BALL_CONTROL_Q4_Q5_BREAKAWAY_POSITIVE_MIN_PULSE, /* 实测正脉冲起滚下限：正脉冲使 Ball_X 减小。 */
        BALL_CONTROL_Q4_Q5_BREAKAWAY_NEGATIVE_MAX_PULSE, /* 实测负脉冲起滚上限：负脉冲使 Ball_X 增大。 */
        15.0F,       /* |位置误差| <= 15 pixel 时不做起滚补偿。 */
        30.0F,      /* |滤波速度| <= 30 pixel/s 视为近似静止。 */
        10000.0F,   /* 位置误差对称保护限幅，单位为 pixel。 */
        -60.0F,    /* 速度积分输出下限，单位为 pulse。 */
        60.0F      /* 速度积分输出上限，单位为 pulse。 */
};

static const ball_control_parameter_struct ball_control_q6_parameter_table =
{
        0.92F,      /* 位置外环比例系数，单位为 pixel/s 每 pixel。 */
        0.0F,       /* 位置外环积分系数；Q4/Q5 当前不累计位置积分。 */
        60.0F,      /* 位置积分窗口，单位为 pixel。 */
        -30.0F,     /* 位置积分输出下限，单位为 pixel/s。 */
        30.0F,      /* 位置积分输出上限，单位为 pixel/s。 */
        1000.0F,    /* 目标速度对称限幅，单位为 pixel/s。 */
        -0.92F,      /* 速度内环比例系数，输出单位为 pulse。 */
        0,      /* 速度内环积分系数，输出单位为 pulse。 */
        0.25F,      /* 速度一阶低通中新样本的权重。 */
        -1.3F,      /* 前馈增益，单位为 pulse/LSB；正 Y 加速度产生负脉冲。 */
        52.0F,      /* 前馈死区，单位为去零漂加速度 raw LSB。 */
        1250.0F,    /* 前馈输入对称限幅，单位为 raw LSB。 */
        200.0F,    /* 前馈输出对称限幅；仅在 Q6 车体缓启动阶段启用。 */
        30.0F,      /* 前馈每个 18 ms 控制槽的最大变化量，单位为 pulse。 */
        BALL_CONTROL_Q4_Q5_BREAKAWAY_POSITIVE_MIN_PULSE, /* 实测正脉冲起滚下限：正脉冲使 Ball_X 减小。 */
        BALL_CONTROL_Q4_Q5_BREAKAWAY_NEGATIVE_MAX_PULSE, /* 实测负脉冲起滚上限：负脉冲使 Ball_X 增大。 */
        13.0F,       /* |位置误差| <= 15 pixel 时不做起滚补偿。 */
        30.0F,      /* |滤波速度| <= 30 pixel/s 视为近似静止。 */
        10000.0F,   /* 位置误差对称保护限幅，单位为 pixel。 */
        -60.0F,    /* 速度积分输出下限，单位为 pulse。 */
        60.0F      /* 速度积分输出上限，单位为 pulse。 */
};

static const ball_control_parameter_struct *ball_control_get_parameter_table (
    ball_control_parameter_set_enum parameter_set);

static uint8 ball_control_parameters_are_valid (
    const ball_control_parameter_struct *parameters,
    int32 center_pulse,
    float32 positive_breakaway_min_pulse,
    float32 negative_breakaway_max_pulse);
static ball_control_status_enum ball_control_configure_parameter_set (
    ball_control_parameter_set_enum parameter_set);
static float32 ball_control_clamp_position_error (float32 error);
static float32 ball_control_clamp_target_velocity (float32 velocity);
static float32 ball_control_apply_breakaway_compensation (
    float32 controller_output);
static int32 ball_control_round_and_clamp_position (float32 position);
static void ball_control_reset_cascade_state (void);
static void ball_control_reset_acceleration_feedforward_state (void);
static void ball_control_step_acceleration_feedforward_irq (
    uint8 feedforward_allowed);
static void ball_control_advance_velocity_elapsed_irq (void);
static uint8 ball_control_update_velocity_irq (
    uint16 actual_x,
    uint32 position_update_delta,
    float32 *inner_dt_s);
static pid_status_enum ball_control_update_position_outer_irq (
    float32 outer_dt_s);
static pid_status_enum ball_control_schedule_position_outer_irq (
    uint8 velocity_ready,
    float32 inner_dt_s);
static void ball_control_reset_snapshot (
    ball_control_status_enum status);
static void ball_control_queue_safe_center (
    ball_control_status_enum status);
static void ball_control_refresh_pending_command_irq (void);
static void ball_control_try_send_pending_irq (void);

#pragma section all "cpu0_dsram"
static pid_controller_struct ball_control_position_pid;
static pid_controller_struct ball_control_velocity_pid;
static volatile ball_control_snapshot_struct ball_control_snapshot;
static volatile uint32 ball_control_last_position_update_count = 0U;
static volatile uint8 ball_control_outer_updates_until_due = 0U;
static volatile float32 ball_control_outer_elapsed_s = 0.0F;
static volatile uint8 ball_control_pending_valid = ZF_FALSE;
static volatile int32 ball_control_pending_pulse = 0;
static volatile uint16 ball_control_velocity_previous_x = 0U;
static volatile uint32 ball_control_velocity_elapsed_ms = 0U;
static volatile float32 ball_control_filtered_velocity_pixel_per_s = 0.0F;
static volatile uint8 ball_control_velocity_history_valid = ZF_FALSE;
static volatile float32 ball_control_velocity_pid_output_pulse = 0.0F;
static volatile float32 ball_control_acceleration_feedforward_target_pulse =
    0.0F;
static volatile float32 ball_control_acceleration_feedforward_output_pulse =
    0.0F;
static volatile float32 ball_control_acceleration_y_lsb = 0.0F;
static volatile uint8 ball_control_acceleration_sample_valid = ZF_FALSE;
static volatile uint8 ball_control_acceleration_feedforward_enabled =
    ZF_FALSE;
static volatile uint8
    ball_control_acceleration_feedforward_debug_enabled = ZF_FALSE;
static const ball_control_parameter_struct * volatile
    ball_control_active_parameter_table =
        &ball_control_q3_parameter_table;
static volatile ball_control_parameter_set_enum
    ball_control_active_parameter_set =
        BALL_CONTROL_PARAMETER_SET_Q3;
static volatile int32 ball_control_active_center_pulse =
    BALL_CONTROL_CENTER_PULSE;
static volatile float32 ball_control_active_positive_breakaway_min_pulse =
    BALL_CONTROL_Q4_Q5_BREAKAWAY_POSITIVE_MIN_PULSE;
static volatile float32 ball_control_active_negative_breakaway_max_pulse =
    BALL_CONTROL_Q4_Q5_BREAKAWAY_NEGATIVE_MAX_PULSE;
static volatile int32 ball_control_q6_runtime_center_pulse =
    BALL_CONTROL_CENTER_PULSE;
#pragma section all restore

static const ball_control_parameter_struct *ball_control_get_parameter_table (
    ball_control_parameter_set_enum parameter_set)
{
    const ball_control_parameter_struct *parameters = 0;

    switch (parameter_set)
    {
        case BALL_CONTROL_PARAMETER_SET_Q3:
            parameters = &ball_control_q3_parameter_table;
            break;

        case BALL_CONTROL_PARAMETER_SET_Q4_Q5:
            parameters = &ball_control_q4_q5_parameter_table;
            break;

        case BALL_CONTROL_PARAMETER_SET_Q6:
            parameters = &ball_control_q6_parameter_table;
            break;

        default:
            break;
    }

    return parameters;
}


static uint8 ball_control_parameters_are_valid (
    const ball_control_parameter_struct *parameters,
    int32 center_pulse,
    float32 positive_breakaway_min_pulse,
    float32 negative_breakaway_max_pulse)
{
    float32 output_min =
        (float32)BALL_CONTROL_MIN_PULSE
        - (float32)center_pulse;
    float32 output_max =
        (float32)BALL_CONTROL_MAX_PULSE
        - (float32)center_pulse;
    float32 velocity_output_min;
    float32 velocity_output_max;

    if (0 == parameters)
    {
        return ZF_FALSE;
    }

    velocity_output_min =
        output_min
        + parameters->acceleration_feedforward_output_limit_pulse;
    velocity_output_max =
        output_max
        - parameters->acceleration_feedforward_output_limit_pulse;

    return ((-65535.0F
            <= parameters->position_outer_kp)
        && (65535.0F
            >= parameters->position_outer_kp))
        && (-65535.0F
            <= parameters->position_outer_ki)
        && (65535.0F
            >= parameters->position_outer_ki)
        && (0.0F
            < parameters->position_integral_window_pixel)
        && (parameters->position_error_limit_pixel
            >= parameters->position_integral_window_pixel)
        && (0.0F
            < parameters->target_velocity_limit_pixel_per_s)
        && (65535.0F
            >= parameters->target_velocity_limit_pixel_per_s)
        && (parameters->position_integral_min_velocity_pixel_per_s
            <= parameters->position_integral_max_velocity_pixel_per_s)
        && (parameters->position_integral_min_velocity_pixel_per_s
            <= 0.0F)
        && (parameters->position_integral_max_velocity_pixel_per_s
            >= 0.0F)
        && (-parameters->target_velocity_limit_pixel_per_s
            <= parameters->position_integral_min_velocity_pixel_per_s)
        && (parameters->target_velocity_limit_pixel_per_s
            >= parameters->position_integral_max_velocity_pixel_per_s)
        && (-65535.0F
            <= parameters->velocity_inner_kp)
        && (65535.0F
            >= parameters->velocity_inner_kp)
        && (-65535.0F
            <= parameters->velocity_inner_ki)
        && (65535.0F
            >= parameters->velocity_inner_ki)
        && (0.0F < parameters->velocity_filter_alpha)
        && (1.0F >= parameters->velocity_filter_alpha)
        && (-65535.0F
            <= parameters->acceleration_feedforward_gain_pulse_per_lsb)
        && (65535.0F
            >= parameters->acceleration_feedforward_gain_pulse_per_lsb)
        && (0.0F
            <= parameters->acceleration_feedforward_deadband_lsb)
        && (0.0F
            < parameters->acceleration_feedforward_input_limit_lsb)
        && (parameters->acceleration_feedforward_input_limit_lsb
            >= parameters->acceleration_feedforward_deadband_lsb)
        && (0.0F
            <= parameters->acceleration_feedforward_output_limit_pulse)
        && (0.0F
            < parameters->acceleration_feedforward_slew_limit_pulse_per_step)
        && ((int32)BALL_CONTROL_MIN_PULSE <= center_pulse)
        && ((int32)BALL_CONTROL_MAX_PULSE >= center_pulse)
        && (0.0F <= positive_breakaway_min_pulse)
        && (output_max >= positive_breakaway_min_pulse)
        && (0.0F >= negative_breakaway_max_pulse)
        && (output_min <= negative_breakaway_max_pulse)
        && (0.0F
            <= parameters->breakaway_position_error_deadband_pixel)
        && (parameters->position_error_limit_pixel
            >= parameters->breakaway_position_error_deadband_pixel)
        && (0.0F
            <= parameters->breakaway_stationary_velocity_limit_pixel_per_s)
        && (0.0F
            < parameters->position_error_limit_pixel)
        && (65535.0F
            >= parameters->position_error_limit_pixel)
        && (parameters->velocity_integral_min_pulse
            <= parameters->velocity_integral_max_pulse)
        && (parameters->velocity_integral_min_pulse <= 0.0F)
        && (parameters->velocity_integral_max_pulse >= 0.0F)
        && (velocity_output_min
            <= parameters->velocity_integral_min_pulse)
        && (velocity_output_max
            >= parameters->velocity_integral_max_pulse)
        && (velocity_output_min <= 0.0F)
        && (velocity_output_max >= 0.0F)
        ? ZF_TRUE
        : ZF_FALSE;
}

static ball_control_status_enum ball_control_configure_parameter_set (
    ball_control_parameter_set_enum parameter_set)
{
    const ball_control_parameter_struct *parameters =
        ball_control_get_parameter_table(parameter_set);
    int32 absolute_center_pulse = BALL_CONTROL_CENTER_PULSE;
    float32 positive_breakaway_min_pulse;
    float32 negative_breakaway_max_pulse;
    pid_config_struct pid_config;
    pid_status_enum pid_status;

    if (0 == parameters)
    {
        return BALL_CONTROL_STATUS_INVALID_CONFIG;
    }

    positive_breakaway_min_pulse =
        parameters->breakaway_positive_min_pulse;
    negative_breakaway_max_pulse =
        parameters->breakaway_negative_max_pulse;
    if (BALL_CONTROL_PARAMETER_SET_Q6 == parameter_set)
    {
        absolute_center_pulse = ball_control_q6_runtime_center_pulse;
    }

    if (ZF_FALSE == ball_control_parameters_are_valid(
            parameters,
            absolute_center_pulse,
            positive_breakaway_min_pulse,
            negative_breakaway_max_pulse))
    {
        return BALL_CONTROL_STATUS_INVALID_CONFIG;
    }

    pid_config.kp = parameters->position_outer_kp;
    pid_config.ki = parameters->position_outer_ki;
    pid_config.kd = 0.0F;
    pid_config.output_min =
        -parameters->target_velocity_limit_pixel_per_s;
    pid_config.output_max =
        parameters->target_velocity_limit_pixel_per_s;
    pid_config.integral_min =
        parameters->position_integral_min_velocity_pixel_per_s;
    pid_config.integral_max =
        parameters->position_integral_max_velocity_pixel_per_s;

    pid_status = pid_init(&ball_control_position_pid, &pid_config);
    if (PID_STATUS_OK != pid_status)
    {
        return BALL_CONTROL_STATUS_INVALID_CONFIG;
    }

    pid_config.kp = parameters->velocity_inner_kp;
    pid_config.ki = parameters->velocity_inner_ki;
    pid_config.kd = 0.0F;
    pid_config.output_min =
        (float32)BALL_CONTROL_MIN_PULSE
        - (float32)absolute_center_pulse
        + parameters->acceleration_feedforward_output_limit_pulse;
    pid_config.output_max =
        (float32)BALL_CONTROL_MAX_PULSE
        - (float32)absolute_center_pulse
        - parameters->acceleration_feedforward_output_limit_pulse;
    pid_config.integral_min = parameters->velocity_integral_min_pulse;
    pid_config.integral_max = parameters->velocity_integral_max_pulse;

    pid_status = pid_init(&ball_control_velocity_pid, &pid_config);
    if (PID_STATUS_OK != pid_status)
    {
        return BALL_CONTROL_STATUS_INVALID_CONFIG;
    }

    ball_control_active_parameter_table = parameters;
    ball_control_active_parameter_set = parameter_set;
    ball_control_active_center_pulse = absolute_center_pulse;
    ball_control_active_positive_breakaway_min_pulse =
        positive_breakaway_min_pulse;
    ball_control_active_negative_breakaway_max_pulse =
        negative_breakaway_max_pulse;
    return BALL_CONTROL_STATUS_OK;
}

static float32 ball_control_clamp_position_error (float32 error)
{
    float32 limited = error;

    if (ball_control_active_parameter_table->position_error_limit_pixel
        < limited)
    {
        limited =
            ball_control_active_parameter_table->position_error_limit_pixel;
    }
    else if (-ball_control_active_parameter_table->position_error_limit_pixel
        > limited)
    {
        limited =
            -ball_control_active_parameter_table->position_error_limit_pixel;
    }

    return limited;
}

static float32 ball_control_clamp_target_velocity (float32 velocity)
{
    float32 limited = velocity;
    float32 limit = ball_control_active_parameter_table
        ->target_velocity_limit_pixel_per_s;

    if (limit < limited)
    {
        limited = limit;
    }
    else if (-limit > limited)
    {
        limited = -limit;
    }

    return limited;
}

static float32 ball_control_apply_breakaway_compensation (
    float32 controller_output)
{
    float32 error = ball_control_snapshot.error;
    float32 velocity =
        ball_control_snapshot.filtered_velocity_pixel_per_s;
    float32 error_deadband = ball_control_active_parameter_table
        ->breakaway_position_error_deadband_pixel;
    float32 stationary_velocity_limit = ball_control_active_parameter_table
        ->breakaway_stationary_velocity_limit_pixel_per_s;
    float32 positive_min =
        ball_control_active_positive_breakaway_min_pulse;
    float32 negative_max =
        ball_control_active_negative_breakaway_max_pulse;
    float32 absolute_velocity =
        (0.0F <= velocity) ? velocity : -velocity;

    /*
     * 实测方向：正绝对脉冲使 Ball_X 减小，负绝对脉冲使 Ball_X 增大。
     * 只在钢球近似静止且误差超出保持窗口时，按位置误差方向建立最小
     * 起滚倾角；原始 PI 输出即使暂时为反向，也由位置误差方向纠正。
     */
    if (stationary_velocity_limit < absolute_velocity)
    {
        return controller_output;
    }

    if ((-error_deadband > error)
        && (positive_min > controller_output))
    {
        controller_output = positive_min;
    }
    else if ((error_deadband < error)
        && (negative_max < controller_output))
    {
        controller_output = negative_max;
    }

    return controller_output;
}

static int32 ball_control_round_and_clamp_position (float32 position)
{
    float32 limited = position;
    float32 min_pulse = (float32)BALL_CONTROL_MIN_PULSE;
    float32 max_pulse = (float32)BALL_CONTROL_MAX_PULSE;

    if (max_pulse < limited)
    {
        limited = max_pulse;
    }
    else if (min_pulse > limited)
    {
        limited = min_pulse;
    }

    return (0.0F <= limited)
        ? (int32)(limited + 0.5F)
        : (int32)(limited - 0.5F);
}

static void ball_control_reset_acceleration_feedforward_state (void)
{
    ball_control_acceleration_feedforward_enabled = ZF_FALSE;
    ball_control_acceleration_feedforward_debug_enabled = ZF_FALSE;
    ball_control_acceleration_y_lsb = 0.0F;
    ball_control_acceleration_sample_valid = ZF_FALSE;
    ball_control_acceleration_feedforward_target_pulse = 0.0F;
    ball_control_acceleration_feedforward_output_pulse = 0.0F;
}

static void ball_control_reset_cascade_state (void)
{
    ball_control_velocity_previous_x = 0U;
    ball_control_velocity_elapsed_ms = 0U;
    ball_control_filtered_velocity_pixel_per_s = 0.0F;
    ball_control_velocity_history_valid = ZF_FALSE;
    ball_control_velocity_pid_output_pulse = 0.0F;
    ball_control_outer_updates_until_due = 0U;
    ball_control_outer_elapsed_s = 0.0F;
    ball_control_snapshot.target_velocity_pixel_per_s = 0.0F;
    ball_control_snapshot.position_integral_velocity_pixel_per_s = 0.0F;
    ball_control_snapshot.velocity_error_pixel_per_s = 0.0F;
    ball_control_snapshot.filtered_velocity_pixel_per_s = 0.0F;
    ball_control_snapshot.integral_output_pulse = 0.0F;
    ball_control_snapshot.pid_output_pulse = 0.0F;
    ball_control_snapshot.velocity_output_pulse = 0.0F;
    ball_control_snapshot.controller_output_pulse = 0.0F;
    ball_control_reset_acceleration_feedforward_state();
}

void ball_control_update_acceleration_y_irq (
    float32 acceleration_y_lsb)
{
    if ((ZF_TRUE == ball_control_snapshot.initialized)
        && (acceleration_y_lsb == acceleration_y_lsb))
    {
        ball_control_acceleration_y_lsb = acceleration_y_lsb;
        ball_control_acceleration_sample_valid = ZF_TRUE;
    }
}

static void ball_control_step_acceleration_feedforward_irq (
    uint8 feedforward_allowed)
{
    float32 effective_acceleration;
    float32 target_pulse;
    float32 output_delta;
    float32 slew_limit;
    float32 input_limit;
    float32 output_limit;
    float32 deadband;

    if ((ZF_FALSE == feedforward_allowed)
        || (ZF_FALSE == ball_control_acceleration_feedforward_enabled)
        || (ZF_FALSE == ball_control_acceleration_sample_valid))
    {
        ball_control_acceleration_feedforward_target_pulse = 0.0F;
        ball_control_acceleration_feedforward_output_pulse = 0.0F;
        return;
    }

    deadband = ball_control_active_parameter_table
        ->acceleration_feedforward_deadband_lsb;
    input_limit = ball_control_active_parameter_table
        ->acceleration_feedforward_input_limit_lsb;
    output_limit = ball_control_active_parameter_table
        ->acceleration_feedforward_output_limit_pulse;
    slew_limit = ball_control_active_parameter_table
        ->acceleration_feedforward_slew_limit_pulse_per_step;

    if (ball_control_acceleration_y_lsb > deadband)
    {
        effective_acceleration =
            ball_control_acceleration_y_lsb - deadband;
    }
    else if (ball_control_acceleration_y_lsb < -deadband)
    {
        effective_acceleration =
            ball_control_acceleration_y_lsb + deadband;
    }
    else
    {
        effective_acceleration = 0.0F;
    }

    if (effective_acceleration > input_limit)
    {
        effective_acceleration = input_limit;
    }
    else if (-input_limit > effective_acceleration)
    {
        effective_acceleration = -input_limit;
    }

    target_pulse =
        ball_control_active_parameter_table
            ->acceleration_feedforward_gain_pulse_per_lsb
        * effective_acceleration;
    if (target_pulse > output_limit)
    {
        target_pulse = output_limit;
    }
    else if (-output_limit > target_pulse)
    {
        target_pulse = -output_limit;
    }

    ball_control_acceleration_feedforward_target_pulse = target_pulse;
    output_delta =
        target_pulse
        - ball_control_acceleration_feedforward_output_pulse;
    if (output_delta > slew_limit)
    {
        output_delta = slew_limit;
    }
    else if (-slew_limit > output_delta)
    {
        output_delta = -slew_limit;
    }

    ball_control_acceleration_feedforward_output_pulse += output_delta;
}

static void ball_control_advance_velocity_elapsed_irq (void)
{
    if (ZF_TRUE == ball_control_velocity_history_valid)
    {
        if ((BALL_CONTROL_UPDATE_COUNT_MAX - BALL_CONTROL_PERIOD_MS)
            >= ball_control_velocity_elapsed_ms)
        {
            ball_control_velocity_elapsed_ms += BALL_CONTROL_PERIOD_MS;
        }
        else
        {
            ball_control_velocity_elapsed_ms =
                BALL_CONTROL_UPDATE_COUNT_MAX;
        }
    }
}

static uint8 ball_control_update_velocity_irq (
    uint16 actual_x,
    uint32 position_update_delta,
    float32 *inner_dt_s)
{
    float32 camera_interval_s;
    float32 elapsed_control_s;
    float32 observation_dt_s;
    float32 raw_velocity;
    float32 filtered_velocity;
    int32 delta_x;
    uint32 elapsed_ms;

    if (0 == inner_dt_s)
    {
        return ZF_FALSE;
    }
    *inner_dt_s = 0.0F;

    if (ZF_FALSE == ball_control_velocity_history_valid)
    {
        ball_control_velocity_previous_x = actual_x;
        ball_control_velocity_elapsed_ms = 0U;
        ball_control_filtered_velocity_pixel_per_s = 0.0F;
        ball_control_velocity_history_valid = ZF_TRUE;
        ball_control_snapshot.filtered_velocity_pixel_per_s = 0.0F;
        return ZF_FALSE;
    }

    elapsed_ms = ball_control_velocity_elapsed_ms;
    ball_control_velocity_elapsed_ms = 0U;
    if (0U == elapsed_ms)
    {
        ball_control_velocity_previous_x = actual_x;
        return ZF_FALSE;
    }

    elapsed_control_s = (float32)elapsed_ms / 1000.0F;
    camera_interval_s =
        (float32)position_update_delta
        / (float32)BALL_CONTROL_CAMERA_FRAME_RATE_HZ;
    observation_dt_s = elapsed_control_s;
    if (camera_interval_s > observation_dt_s)
    {
        observation_dt_s = camera_interval_s;
    }

    delta_x = (int32)actual_x
        - (int32)ball_control_velocity_previous_x;
    ball_control_velocity_previous_x = actual_x;
    raw_velocity = (float32)delta_x / observation_dt_s;
    filtered_velocity =
        ball_control_filtered_velocity_pixel_per_s
        + ball_control_active_parameter_table->velocity_filter_alpha
            * (raw_velocity
                - ball_control_filtered_velocity_pixel_per_s);
    ball_control_filtered_velocity_pixel_per_s = filtered_velocity;
    ball_control_snapshot.filtered_velocity_pixel_per_s =
        filtered_velocity;
    *inner_dt_s = elapsed_control_s;
    return ZF_TRUE;
}

static pid_status_enum ball_control_update_position_outer_irq (
    float32 outer_dt_s)
{
    float32 limited_error;
    float32 proportional_velocity;
    float32 target_velocity;
    pid_status_enum pid_status = PID_STATUS_OK;

    limited_error = ball_control_clamp_position_error(
        (float32)ball_control_snapshot.target_x
        - (float32)ball_control_snapshot.actual_x);

    if ((0.0F < outer_dt_s)
        && (-ball_control_active_parameter_table
                ->position_integral_window_pixel
            <= limited_error)
        && (ball_control_active_parameter_table
                ->position_integral_window_pixel
            >= limited_error))
    {
        pid_status = pid_step(
            &ball_control_position_pid,
            limited_error,
            0.0F,
            outer_dt_s,
            &target_velocity);
        if (PID_STATUS_OK != pid_status)
        {
            return pid_status;
        }
    }
    else
    {
        /*
         * 首个新坐标没有完整外环 dt；误差超出积分窗口时也只计算 P，并冻结
         * 已有位置积分。积分仍参与输出，避免进出窗口时目标速度突跳。
         */
        proportional_velocity =
            ball_control_active_parameter_table->position_outer_kp
            * limited_error;
        target_velocity = ball_control_clamp_target_velocity(
            proportional_velocity
            + ball_control_position_pid.state.integral_output);
    }

    ball_control_snapshot.error = limited_error;
    ball_control_snapshot.target_velocity_pixel_per_s =
        target_velocity;
    ball_control_snapshot.position_integral_velocity_pixel_per_s =
        ball_control_position_pid.state.integral_output;
    if (BALL_CONTROL_UPDATE_COUNT_MAX
        != ball_control_snapshot.outer_update_count)
    {
        ball_control_snapshot.outer_update_count++;
    }

    return PID_STATUS_OK;
}

static pid_status_enum ball_control_schedule_position_outer_irq (
    uint8 velocity_ready,
    float32 inner_dt_s)
{
    pid_status_enum pid_status = PID_STATUS_OK;

    if (ZF_TRUE == velocity_ready)
    {
        ball_control_outer_elapsed_s += inner_dt_s;
    }

    if (0U == ball_control_outer_updates_until_due)
    {
        pid_status = ball_control_update_position_outer_irq(
            ball_control_outer_elapsed_s);
        if (PID_STATUS_OK == pid_status)
        {
            ball_control_outer_elapsed_s = 0.0F;
            ball_control_outer_updates_until_due =
                (uint8)(BALL_CONTROL_OUTER_DIVIDER - 1U);
        }
    }
    else
    {
        ball_control_outer_updates_until_due--;
    }

    return pid_status;
}

static void ball_control_reset_snapshot (
    ball_control_status_enum status)
{
    ball_control_snapshot.status = status;
    ball_control_snapshot.initialized = ZF_FALSE;
    ball_control_snapshot.enabled = ZF_FALSE;
    ball_control_snapshot.position_valid = ZF_FALSE;
    ball_control_snapshot.target_x = 0U;
    ball_control_snapshot.actual_x = 0U;
    ball_control_snapshot.error = 0.0F;
    ball_control_snapshot.target_velocity_pixel_per_s = 0.0F;
    ball_control_snapshot.position_integral_velocity_pixel_per_s = 0.0F;
    ball_control_snapshot.velocity_error_pixel_per_s = 0.0F;
    ball_control_snapshot.integral_output_pulse = 0.0F;
    ball_control_snapshot.pid_output_pulse = 0.0F;
    ball_control_snapshot.filtered_velocity_pixel_per_s = 0.0F;
    ball_control_snapshot.velocity_output_pulse = 0.0F;
    ball_control_snapshot.controller_output_pulse = 0.0F;
    ball_control_snapshot.requested_absolute_pulse =
        ball_control_active_center_pulse;
    ball_control_snapshot.applied_absolute_pulse =
        ball_control_active_center_pulse;
    ball_control_snapshot.position_update_count = 0U;
    ball_control_snapshot.control_update_count = 0U;
    ball_control_snapshot.outer_update_count = 0U;
    ball_control_snapshot.motor_status =
        (uint8)STEPPER_UART_STATUS_NOT_INITIALIZED;
    ball_control_reset_cascade_state();
}

static void ball_control_queue_safe_center (
    ball_control_status_enum status)
{
    ball_control_snapshot.enabled = ZF_FALSE;
    ball_control_snapshot.position_valid = ZF_FALSE;
    ball_control_snapshot.status = status;
    ball_control_snapshot.error = 0.0F;
    ball_control_snapshot.target_velocity_pixel_per_s = 0.0F;
    ball_control_snapshot.position_integral_velocity_pixel_per_s = 0.0F;
    ball_control_snapshot.velocity_error_pixel_per_s = 0.0F;
    ball_control_snapshot.integral_output_pulse = 0.0F;
    ball_control_snapshot.pid_output_pulse = 0.0F;
    ball_control_reset_cascade_state();
    ball_control_snapshot.requested_absolute_pulse =
        ball_control_active_center_pulse;
    ball_control_pending_pulse = ball_control_active_center_pulse;
    ball_control_pending_valid = ZF_TRUE;
    (void)pid_reset(&ball_control_position_pid);
    (void)pid_reset(&ball_control_velocity_pid);
}

static void ball_control_refresh_pending_command_irq (void)
{
    float32 controller_output;
    float32 absolute_position;
    int32 absolute_pulse;

    controller_output =
        ball_control_velocity_pid_output_pulse
        + ball_control_acceleration_feedforward_output_pulse;
    controller_output =
        ball_control_apply_breakaway_compensation(controller_output);
    absolute_position =
        (float32)ball_control_active_center_pulse
        + controller_output;
    absolute_pulse =
        ball_control_round_and_clamp_position(absolute_position);

    ball_control_snapshot.controller_output_pulse = controller_output;
    if ((absolute_pulse
            != ball_control_snapshot.requested_absolute_pulse)
        || ((ZF_FALSE == ball_control_pending_valid)
            && (absolute_pulse
                != ball_control_snapshot.applied_absolute_pulse)))
    {
        ball_control_snapshot.requested_absolute_pulse = absolute_pulse;
        ball_control_pending_pulse = absolute_pulse;
        ball_control_pending_valid = ZF_TRUE;
    }
}

static void ball_control_try_send_pending_irq (void)
{
    int32 pending_pulse;
    stepper_uart_status_enum motor_status;

    if (ZF_FALSE == ball_control_pending_valid)
    {
        return;
    }

    pending_pulse = ball_control_pending_pulse;
    motor_status = Step_Motor_Set_Position(pending_pulse);
    ball_control_snapshot.motor_status = (uint8)motor_status;

    if (STEPPER_UART_STATUS_OK == motor_status)
    {
        ball_control_snapshot.applied_absolute_pulse = pending_pulse;
        ball_control_pending_valid = ZF_FALSE;
    }
    else if (STEPPER_UART_STATUS_TX_BUSY != motor_status)
    {
        ball_control_snapshot.enabled = ZF_FALSE;
        ball_control_snapshot.position_valid = ZF_FALSE;
        ball_control_snapshot.status = BALL_CONTROL_STATUS_MOTOR_ERROR;
        ball_control_snapshot.error = 0.0F;
        ball_control_snapshot.target_velocity_pixel_per_s = 0.0F;
        ball_control_snapshot.position_integral_velocity_pixel_per_s = 0.0F;
        ball_control_snapshot.velocity_error_pixel_per_s = 0.0F;
        ball_control_snapshot.integral_output_pulse = 0.0F;
        ball_control_snapshot.pid_output_pulse = 0.0F;
        ball_control_reset_cascade_state();
        ball_control_pending_valid = ZF_FALSE;
        (void)pid_reset(&ball_control_position_pid);
        (void)pid_reset(&ball_control_velocity_pid);
    }
}

ball_control_status_enum ball_control_init (void)
{
    ball_control_status_enum parameter_status;
    stepper_uart_status_enum motor_status;

    ball_control_q6_runtime_center_pulse = BALL_CONTROL_CENTER_PULSE;
    ball_control_active_parameter_set = BALL_CONTROL_PARAMETER_SET_Q3;
    ball_control_active_center_pulse = BALL_CONTROL_CENTER_PULSE;
    ball_control_active_positive_breakaway_min_pulse =
        BALL_CONTROL_Q4_Q5_BREAKAWAY_POSITIVE_MIN_PULSE;
    ball_control_active_negative_breakaway_max_pulse =
        BALL_CONTROL_Q4_Q5_BREAKAWAY_NEGATIVE_MAX_PULSE;
    ball_control_reset_snapshot(BALL_CONTROL_STATUS_NOT_INITIALIZED);
    ball_control_last_position_update_count = 0U;
    ball_control_pending_valid = ZF_FALSE;
    ball_control_pending_pulse = ball_control_active_center_pulse;
    ball_control_velocity_pid_output_pulse = 0.0F;

    parameter_status = ball_control_configure_parameter_set(
        BALL_CONTROL_PARAMETER_SET_Q3);
    if (BALL_CONTROL_STATUS_OK != parameter_status)
    {
        ball_control_snapshot.status = parameter_status;
        return ball_control_snapshot.status;
    }

    /*
     * Step_Motor_Init() 初始化 UART3，等待驱动器上电，随后使能地址 1
     * 并发送停止命令；它不会建立或改变驱动器零点。
     */
    motor_status = Step_Motor_Init();
    ball_control_snapshot.motor_status = (uint8)motor_status;
    if (STEPPER_UART_STATUS_OK != motor_status)
    {
        ball_control_snapshot.status = BALL_CONTROL_STATUS_MOTOR_ERROR;
        return ball_control_snapshot.status;
    }

    ball_control_snapshot.initialized = ZF_TRUE;
    ball_control_snapshot.status = BALL_CONTROL_STATUS_DISABLED;
    return BALL_CONTROL_STATUS_OK;
}

ball_control_status_enum ball_control_enable (
    uint16 target_x,
    ball_control_parameter_set_enum parameter_set)
{
    uint32 interrupt_state;
    uint32 current_update_count = 0U;
    ball_control_status_enum parameter_status;
    k230_app_snapshot_struct position_snapshot;

    if (ZF_FALSE == ball_control_snapshot.initialized)
    {
        return BALL_CONTROL_STATUS_NOT_INITIALIZED;
    }

    if ((ZF_TRUE == k230_app_get_snapshot(&position_snapshot))
        && (ZF_TRUE == position_snapshot.position_valid))
    {
        current_update_count = position_snapshot.position_update_count;
    }

    interrupt_state = interrupt_global_disable();

    parameter_status = ball_control_configure_parameter_set(parameter_set);
    if (BALL_CONTROL_STATUS_OK != parameter_status)
    {
        interrupt_global_enable(interrupt_state);
        return parameter_status;
    }

    (void)pid_reset(&ball_control_position_pid);
    (void)pid_reset(&ball_control_velocity_pid);
    ball_control_last_position_update_count = current_update_count;
    ball_control_pending_valid = ZF_FALSE;
    ball_control_pending_pulse = ball_control_active_center_pulse;

    ball_control_snapshot.enabled = ZF_TRUE;
    ball_control_snapshot.position_valid = ZF_FALSE;
    ball_control_snapshot.target_x = target_x;
    ball_control_snapshot.actual_x = 0U;
    ball_control_snapshot.error = 0.0F;
    ball_control_snapshot.target_velocity_pixel_per_s = 0.0F;
    ball_control_snapshot.position_integral_velocity_pixel_per_s = 0.0F;
    ball_control_snapshot.velocity_error_pixel_per_s = 0.0F;
    ball_control_snapshot.integral_output_pulse = 0.0F;
    ball_control_snapshot.pid_output_pulse = 0.0F;
    ball_control_reset_cascade_state();
    ball_control_acceleration_feedforward_enabled =
        (0.0F
            < ball_control_active_parameter_table
                ->acceleration_feedforward_output_limit_pulse)
        ? ZF_TRUE
        : ZF_FALSE;
    ball_control_snapshot.requested_absolute_pulse =
        ball_control_active_center_pulse;
    ball_control_snapshot.position_update_count = current_update_count;
    ball_control_snapshot.control_update_count = 0U;
    ball_control_snapshot.outer_update_count = 0U;
    ball_control_snapshot.status =
        BALL_CONTROL_STATUS_WAITING_FOR_POSITION;
    interrupt_global_enable(interrupt_state);

    return BALL_CONTROL_STATUS_OK;
}

ball_control_status_enum
    ball_control_enable_acceleration_feedforward_debug (void)
{
    uint32 interrupt_state;
    ball_control_status_enum parameter_status;

    if (ZF_FALSE == ball_control_snapshot.initialized)
    {
        return BALL_CONTROL_STATUS_NOT_INITIALIZED;
    }

    interrupt_state = interrupt_global_disable();
    /*
     * Q0 ACCEL_FF 固定复用 Q4/Q5 参数表，并在调试模式内强制放行前馈，
     * 便于脱离整车缓启动单独验收。普通 Q0 SOFT_START 仍由题目层关断前馈；
     * Q6 正常控制继续使用独立参数表并受整车缓启动门控。
     */
    parameter_status = ball_control_configure_parameter_set(
        BALL_CONTROL_PARAMETER_SET_Q4_Q5);
    if (BALL_CONTROL_STATUS_OK != parameter_status)
    {
        interrupt_global_enable(interrupt_state);
        return parameter_status;
    }

    (void)pid_reset(&ball_control_position_pid);
    (void)pid_reset(&ball_control_velocity_pid);
    ball_control_last_position_update_count = 0U;
    ball_control_pending_valid = ZF_FALSE;
    ball_control_pending_pulse = ball_control_active_center_pulse;

    ball_control_snapshot.enabled = ZF_TRUE;
    ball_control_snapshot.position_valid = ZF_FALSE;
    ball_control_snapshot.target_x = 0U;
    ball_control_snapshot.actual_x = 0U;
    ball_control_snapshot.error = 0.0F;
    ball_control_snapshot.target_velocity_pixel_per_s = 0.0F;
    ball_control_snapshot.position_integral_velocity_pixel_per_s = 0.0F;
    ball_control_snapshot.velocity_error_pixel_per_s = 0.0F;
    ball_control_snapshot.integral_output_pulse = 0.0F;
    ball_control_snapshot.pid_output_pulse = 0.0F;
    ball_control_reset_cascade_state();
    ball_control_acceleration_feedforward_enabled =
        (0.0F
            < ball_control_active_parameter_table
                ->acceleration_feedforward_output_limit_pulse)
        ? ZF_TRUE
        : ZF_FALSE;
    ball_control_acceleration_feedforward_debug_enabled = ZF_TRUE;
    ball_control_snapshot.requested_absolute_pulse =
        ball_control_active_center_pulse;
    ball_control_snapshot.position_update_count = 0U;
    ball_control_snapshot.control_update_count = 0U;
    ball_control_snapshot.outer_update_count = 0U;
    ball_control_snapshot.status = BALL_CONTROL_STATUS_RUNNING;
    interrupt_global_enable(interrupt_state);

    return BALL_CONTROL_STATUS_OK;
}

ball_control_status_enum ball_control_set_target (uint16 target_x)
{
    uint32 interrupt_state;
    pid_status_enum pid_status;
    ball_control_status_enum status = BALL_CONTROL_STATUS_OK;

    interrupt_state = interrupt_global_disable();
    if (ZF_FALSE == ball_control_snapshot.initialized)
    {
        status = BALL_CONTROL_STATUS_NOT_INITIALIZED;
    }
    else if (ZF_FALSE == ball_control_snapshot.enabled)
    {
        status = BALL_CONTROL_STATUS_DISABLED;
    }
    else
    {
        ball_control_snapshot.target_x = target_x;

        if (ZF_TRUE == ball_control_snapshot.position_valid)
        {
            /*
             * 题目级目标切换必须立即生效，但不能重新启动任一级 PI。使用零
             * 积分时间刷新，保留位置积分，仅更新新目标对应的比例输出。
             */
            pid_status = ball_control_update_position_outer_irq(0.0F);
            if (PID_STATUS_OK == pid_status)
            {
                ball_control_outer_elapsed_s = 0.0F;
                ball_control_outer_updates_until_due =
                    (uint8)(BALL_CONTROL_OUTER_DIVIDER - 1U);
            }
            else
            {
                ball_control_queue_safe_center(
                    BALL_CONTROL_STATUS_PID_ERROR);
                status = BALL_CONTROL_STATUS_PID_ERROR;
            }
        }
    }
    interrupt_global_enable(interrupt_state);

    return status;
}

ball_control_status_enum ball_control_disable (void)
{
    uint32 interrupt_state;

    if (ZF_FALSE == ball_control_snapshot.initialized)
    {
        return BALL_CONTROL_STATUS_NOT_INITIALIZED;
    }

    interrupt_state = interrupt_global_disable();
    ball_control_queue_safe_center(BALL_CONTROL_STATUS_DISABLED);
    interrupt_global_enable(interrupt_state);
    return BALL_CONTROL_STATUS_OK;
}

ball_control_status_enum ball_control_get_q6_runtime_center_pulse (
    int32 *center_pulse)
{
    uint32 interrupt_state;
    ball_control_status_enum status = BALL_CONTROL_STATUS_OK;

    if (0 == center_pulse)
    {
        return BALL_CONTROL_STATUS_INVALID_CONFIG;
    }

    interrupt_state = interrupt_global_disable();
    if (ZF_FALSE == ball_control_snapshot.initialized)
    {
        status = BALL_CONTROL_STATUS_NOT_INITIALIZED;
    }
    else
    {
        *center_pulse = ball_control_q6_runtime_center_pulse;
    }
    interrupt_global_enable(interrupt_state);
    return status;
}

ball_control_status_enum ball_control_set_q6_runtime_center_pulse (
    int32 center_pulse)
{
    uint32 interrupt_state;
    ball_control_status_enum status = BALL_CONTROL_STATUS_OK;

    interrupt_state = interrupt_global_disable();
    if (ZF_FALSE == ball_control_snapshot.initialized)
    {
        status = BALL_CONTROL_STATUS_NOT_INITIALIZED;
    }
    else if (ZF_TRUE == ball_control_snapshot.enabled)
    {
        status = BALL_CONTROL_STATUS_RUNNING;
    }
    else if (ZF_FALSE == ball_control_parameters_are_valid(
            &ball_control_q6_parameter_table,
            center_pulse,
            BALL_CONTROL_Q4_Q5_BREAKAWAY_POSITIVE_MIN_PULSE,
            BALL_CONTROL_Q4_Q5_BREAKAWAY_NEGATIVE_MAX_PULSE))
    {
        status = BALL_CONTROL_STATUS_INVALID_CONFIG;
    }
    else
    {
        ball_control_q6_runtime_center_pulse = center_pulse;
    }
    interrupt_global_enable(interrupt_state);
    return status;
}

uint8 ball_control_is_enabled (void)
{
    return ball_control_snapshot.enabled;
}

uint8 ball_control_is_acceleration_feedforward_debug_enabled (void)
{
    return ball_control_acceleration_feedforward_debug_enabled;
}

ball_control_status_enum ball_control_get_status (void)
{
    return ball_control_snapshot.status;
}

void ball_control_step_irq (uint8 acceleration_feedforward_allowed)
{
    float32 inner_dt_s = 0.0F;
    float32 velocity_error;
    float32 velocity_pid_output;
    float32 velocity_proportional_output;
    uint32 position_update_delta = 0U;
    pid_status_enum pid_status;
    k230_app_snapshot_struct position_snapshot;
    uint8 fresh_position = ZF_FALSE;
    uint8 velocity_ready = ZF_FALSE;

    if (ZF_FALSE == ball_control_snapshot.initialized)
    {
        return;
    }

    if ((ZF_TRUE == ball_control_snapshot.enabled)
        && (ZF_TRUE
            == ball_control_acceleration_feedforward_debug_enabled))
    {
        ball_control_step_acceleration_feedforward_irq(ZF_TRUE);
        ball_control_velocity_pid_output_pulse = 0.0F;
        ball_control_refresh_pending_command_irq();
        ball_control_snapshot.status = BALL_CONTROL_STATUS_RUNNING;
        if (BALL_CONTROL_UPDATE_COUNT_MAX
            != ball_control_snapshot.control_update_count)
        {
            ball_control_snapshot.control_update_count++;
        }
        ball_control_try_send_pending_irq();
        return;
    }

    if (ZF_TRUE == ball_control_snapshot.enabled)
    {
        ball_control_step_acceleration_feedforward_irq(
            acceleration_feedforward_allowed);
        ball_control_advance_velocity_elapsed_irq();

        if ((ZF_TRUE == k230_app_get_snapshot(&position_snapshot))
            && (ZF_TRUE == position_snapshot.position_valid)
            && (ball_control_last_position_update_count
                != position_snapshot.position_update_count))
        {
            fresh_position = ZF_TRUE;
        }

        /*
         * 没有新坐标时保持控制器使能，不推进位置/速度 PI，也不计时回中
         * 或自动关闭。保留两级 PI、速度历史和最新绝对位置请求；加速度
         * 前馈只在调用方允许的缓启动窗口内按 18 ms 路径更新。后续新坐标
         * 到达后按累计间隔继续速度估计和串级闭环。
         */
        if (ZF_TRUE == fresh_position)
        {
            if (position_snapshot.position_update_count
                > ball_control_last_position_update_count)
            {
                position_update_delta =
                    position_snapshot.position_update_count
                    - ball_control_last_position_update_count;
            }
            else
            {
                position_update_delta = 1U;
            }

            velocity_ready = ball_control_update_velocity_irq(
                position_snapshot.ball_x,
                position_update_delta,
                &inner_dt_s);
            ball_control_last_position_update_count =
                position_snapshot.position_update_count;
            ball_control_snapshot.position_update_count =
                position_snapshot.position_update_count;
            ball_control_snapshot.position_valid = ZF_TRUE;
            ball_control_snapshot.actual_x = position_snapshot.ball_x;
            pid_status = ball_control_schedule_position_outer_irq(
                velocity_ready,
                inner_dt_s);
            if (PID_STATUS_OK != pid_status)
            {
                ball_control_queue_safe_center(
                    BALL_CONTROL_STATUS_PID_ERROR);
            }
        }
        if ((ZF_TRUE == ball_control_snapshot.enabled)
            && (ZF_TRUE == velocity_ready))
        {
            velocity_error =
                ball_control_snapshot.target_velocity_pixel_per_s
                - ball_control_filtered_velocity_pixel_per_s;
            pid_status = pid_step(
                &ball_control_velocity_pid,
                ball_control_snapshot.target_velocity_pixel_per_s,
                ball_control_filtered_velocity_pixel_per_s,
                inner_dt_s,
                &velocity_pid_output);

            if (PID_STATUS_OK != pid_status)
            {
                ball_control_queue_safe_center(
                    BALL_CONTROL_STATUS_PID_ERROR);
            }
            else
            {
                velocity_proportional_output =
                    ball_control_active_parameter_table->velocity_inner_kp
                    * velocity_error;

                ball_control_snapshot.velocity_error_pixel_per_s =
                    velocity_error;
                ball_control_snapshot.integral_output_pulse =
                    ball_control_velocity_pid.state.integral_output;
                ball_control_snapshot.pid_output_pulse =
                    velocity_pid_output;
                ball_control_snapshot.velocity_output_pulse =
                    velocity_proportional_output;
                ball_control_velocity_pid_output_pulse =
                    velocity_pid_output;
                ball_control_snapshot.status =
                    BALL_CONTROL_STATUS_RUNNING;

                if (BALL_CONTROL_UPDATE_COUNT_MAX
                    != ball_control_snapshot.control_update_count)
                {
                    ball_control_snapshot.control_update_count++;
                }
            }
        }

        /*
         * 每个 18 ms 滚球控制槽都用当前速度 PI、可选前馈和可选起滚补偿
         * 重新合成最新绝对位置命令；是否产生前馈或起滚补偿由参数表决定。
         */
        if (ZF_TRUE == ball_control_snapshot.enabled)
        {
            ball_control_refresh_pending_command_irq();
        }
    }

    /*
     * 只重试最新绝对位置目标。TX_BUSY 时保留该目标，不建立旧命令队列。
     */
    ball_control_try_send_pending_irq();
}

uint8 ball_control_get_snapshot (ball_control_snapshot_struct *snapshot)
{
    uint32 interrupt_state;

    if (0 == snapshot)
    {
        return ZF_FALSE;
    }

    interrupt_state = interrupt_global_disable();
    *snapshot = ball_control_snapshot;
    interrupt_global_enable(interrupt_state);
    return ZF_TRUE;
}
