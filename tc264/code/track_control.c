#include "track_control.h"

#include "zf_common_interrupt.h"

typedef enum
{
    TRACK_CONTROL_GYRO_AXIS_X = 0,
    TRACK_CONTROL_GYRO_AXIS_Y,
    TRACK_CONTROL_GYRO_AXIS_Z
} track_control_gyro_axis_enum;

typedef struct
{
    float32 position_kp;
    float32 position_kd;
    float32 gyro_kp;
    float32 gyro_kd;
    track_control_gyro_axis_enum gyro_axis;
} track_control_parameter_struct;

/*
 * 循迹运行参数表。
 *
 * 结构和填写方式参考 myProjectV5.0 的 ControlPidConfig_t 参数表。PD 方向直接由
 * 对应 kp/kd 的正负决定，不另设方向参数。两个环的输出限幅见 track_control.h。
 * 普通循迹基础 PWM 由 question_app 按题号传入；Q4/Q5 使用显式软启动入口。
 */
static const track_control_parameter_struct track_control_parameter_table =
{
    -420.0F,                       /* position_kp：12 ms 灰度位置外环比例。 */
    0.0F,                       /* position_kd：12 ms 灰度位置外环微分。 */
    5.0F,                       /* gyro_kp：6 ms 角速度内环比例。 */
    0.38F,                       /* gyro_kd：6 ms 角速度内环微分。 */
    TRACK_CONTROL_GYRO_AXIS_Z   /* gyro_axis：偏航角速度轴，待实物确认。 */
};

static const uint32 track_control_update_count_max = 0xFFFFFFFFU;

#pragma section all "cpu0_dsram"
static pid_controller_struct track_control_outer_pid;
static pid_controller_struct track_control_inner_pid;
static volatile track_control_snapshot_struct track_control_snapshot;
static volatile uint8 track_control_initialized = ZF_FALSE;
#pragma section all restore

static int32 track_control_abs_int32 (int32 value)
{
    int32 magnitude = value;

    if (0 > magnitude)
    {
        magnitude = -magnitude;
    }

    return magnitude;
}

static int32 track_control_average_int32 (
    int32 first,
    int32 second)
{
    return (first + second) / 2;
}

static int32 track_control_calculate_ramp_pwm (
    int32 start_pwm,
    int32 target_pwm,
    uint16 completed_cycles,
    uint16 total_cycles)
{
    int64 denominator;
    int64 scaled_delta_numerator;

    if ((2U > total_cycles) || (completed_cycles >= total_cycles))
    {
        return target_pwm;
    }

    /*
     * total_cycles 包含起点和终点。completed_cycles 表示已经成功输出的次数，
     * 因此索引 0 使用 start_pwm，索引 total_cycles-1 使用 target_pwm。
     * 使用有符号 64 位运算避免溢出，取整会均匀分配非整数步长。
     */
    denominator = (int64)((uint16)(total_cycles - 1U));
    scaled_delta_numerator =
        ((int64)target_pwm - (int64)start_pwm)
        * (int64)completed_cycles;
    if (0 < scaled_delta_numerator)
    {
        scaled_delta_numerator += denominator / 2;
    }
    else if (0 > scaled_delta_numerator)
    {
        scaled_delta_numerator -= denominator / 2;
    }

    return start_pwm
        + (int32)(scaled_delta_numerator / denominator);
}

static track_control_status_enum track_control_validate_motor_ramp_config (
    int32 motor_a_start,
    int32 motor_b_start,
    int32 motor_a_target,
    int32 motor_b_target)
{
    int32 max_magnitude = 0;
    int32 value_magnitude;

    if (((int32)MOTOR_COMMAND_MIN > motor_a_start)
        || ((int32)MOTOR_COMMAND_MAX < motor_a_start)
        || ((int32)MOTOR_COMMAND_MIN > motor_b_start)
        || ((int32)MOTOR_COMMAND_MAX < motor_b_start)
        || ((int32)MOTOR_COMMAND_MIN > motor_a_target)
        || ((int32)MOTOR_COMMAND_MAX < motor_a_target)
        || ((int32)MOTOR_COMMAND_MIN > motor_b_target)
        || ((int32)MOTOR_COMMAND_MAX < motor_b_target))
    {
        return TRACK_CONTROL_STATUS_INVALID_CONFIG;
    }

    value_magnitude = track_control_abs_int32(motor_a_start);
    if (value_magnitude > max_magnitude)
    {
        max_magnitude = value_magnitude;
    }
    value_magnitude = track_control_abs_int32(motor_b_start);
    if (value_magnitude > max_magnitude)
    {
        max_magnitude = value_magnitude;
    }
    value_magnitude = track_control_abs_int32(motor_a_target);
    if (value_magnitude > max_magnitude)
    {
        max_magnitude = value_magnitude;
    }
    value_magnitude = track_control_abs_int32(motor_b_target);
    if (value_magnitude > max_magnitude)
    {
        max_magnitude = value_magnitude;
    }

    if (TRACK_CONTROL_PWM_CURR_LIMIT
        > (float32)((int32)MOTOR_COMMAND_MAX - max_magnitude))
    {
        return TRACK_CONTROL_STATUS_INVALID_CONFIG;
    }

    return TRACK_CONTROL_STATUS_OK;
}

static int16 track_control_clamp_motor_command (int32 command)
{
    if ((int32)MOTOR_COMMAND_MAX < command)
    {
        command = (int32)MOTOR_COMMAND_MAX;
    }
    else if ((int32)MOTOR_COMMAND_MIN > command)
    {
        command = (int32)MOTOR_COMMAND_MIN;
    }

    return (int16)command;
}

static int16 track_control_round_pwm_curr (float32 pwm_curr)
{
    int32 rounded;

    if (0.0F <= pwm_curr)
    {
        rounded = (int32)(pwm_curr + 0.5F);
    }
    else
    {
        rounded = (int32)(pwm_curr - 0.5F);
    }

    return track_control_clamp_motor_command(rounded);
}

static float32 track_control_select_gyro_lsb (
    const volatile ImuAppData *imu_data)
{
    float32 gyro_lsb;

    if (TRACK_CONTROL_GYRO_AXIS_X
        == track_control_parameter_table.gyro_axis)
    {
        gyro_lsb = imu_data->corrected_x_lsb;
    }
    else if (TRACK_CONTROL_GYRO_AXIS_Y
        == track_control_parameter_table.gyro_axis)
    {
        gyro_lsb = imu_data->corrected_y_lsb;
    }
    else
    {
        gyro_lsb = imu_data->corrected_z_lsb;
    }

    return gyro_lsb;
}

static void track_control_reset_snapshot (
    track_control_status_enum status)
{
    track_control_snapshot.status = status;
    track_control_snapshot.enabled = ZF_FALSE;
    track_control_snapshot.gyro_debug_enabled = ZF_FALSE;
    track_control_snapshot.line_valid = ZF_FALSE;

    track_control_snapshot.line_position = 0.0F;
    track_control_snapshot.line_error = 0.0F;
    track_control_snapshot.target_gyro_lsb = 0.0F;
    track_control_snapshot.actual_gyro_lsb = 0.0F;
    track_control_snapshot.gyro_error = 0.0F;
    track_control_snapshot.pwm_curr = 0.0F;
    track_control_snapshot.pwm_base = 0;
    track_control_snapshot.pwm_base_ramp_start = 0;
    track_control_snapshot.pwm_base_target = 0;
    track_control_snapshot.pwm_base_ramp_cycles = 0U;
    track_control_snapshot.pwm_base_ramp_completed_cycles = 0U;
    track_control_snapshot.motor_a_ramp_start = 0;
    track_control_snapshot.motor_b_ramp_start = 0;
    track_control_snapshot.motor_a_ramp_target = 0;
    track_control_snapshot.motor_b_ramp_target = 0;

    track_control_snapshot.motor_a_request = 0;
    track_control_snapshot.motor_b_request = 0;
    track_control_snapshot.motor_a_applied = 0;
    track_control_snapshot.motor_b_applied = 0;

    track_control_snapshot.outer_update_count = 0U;
    track_control_snapshot.inner_update_count = 0U;
}

static void track_control_stop_motors (void)
{
    (void)motor_stop_all();
    track_control_snapshot.motor_a_request = 0;
    track_control_snapshot.motor_b_request = 0;
    track_control_snapshot.motor_a_applied = motor_get_a_command();
    track_control_snapshot.motor_b_applied = motor_get_b_command();
    track_control_snapshot.pwm_curr = 0.0F;
}

track_control_status_enum track_control_init (void)
{
    pid_config_struct outer_config;
    pid_config_struct inner_config;
    pid_status_enum outer_status;
    pid_status_enum inner_status;
    int32 debug_base_command;
    int32 debug_base_magnitude;

    track_control_initialized = ZF_FALSE;
    track_control_reset_snapshot(TRACK_CONTROL_STATUS_NOT_INITIALIZED);

    if ((TRACK_CONTROL_GYRO_AXIS_X
            != track_control_parameter_table.gyro_axis)
        && (TRACK_CONTROL_GYRO_AXIS_Y
            != track_control_parameter_table.gyro_axis)
        && (TRACK_CONTROL_GYRO_AXIS_Z
            != track_control_parameter_table.gyro_axis))
    {
        track_control_snapshot.status =
            TRACK_CONTROL_STATUS_INVALID_CONFIG;
        return track_control_snapshot.status;
    }

    outer_config.kp = track_control_parameter_table.position_kp;
    outer_config.ki = 0.0F;
    outer_config.kd = track_control_parameter_table.position_kd;
    outer_config.output_min =
        -TRACK_CONTROL_TARGET_GYRO_LIMIT_LSB;
    outer_config.output_max =
        TRACK_CONTROL_TARGET_GYRO_LIMIT_LSB;
    outer_config.integral_min = 0.0F;
    outer_config.integral_max = 0.0F;

    inner_config.kp = track_control_parameter_table.gyro_kp;
    inner_config.ki = 0.0F;
    inner_config.kd = track_control_parameter_table.gyro_kd;
    inner_config.output_min =
        -TRACK_CONTROL_PWM_CURR_LIMIT;
    inner_config.output_max =
        TRACK_CONTROL_PWM_CURR_LIMIT;
    inner_config.integral_min = 0.0F;
    inner_config.integral_max = 0.0F;

    debug_base_command = TRACK_CONTROL_GYRO_DEBUG_PWM_BASE;
    if (((int32)MOTOR_COMMAND_MIN > debug_base_command)
        || ((int32)MOTOR_COMMAND_MAX < debug_base_command))
    {
        track_control_snapshot.status =
            TRACK_CONTROL_STATUS_INVALID_CONFIG;
        return track_control_snapshot.status;
    }

    debug_base_magnitude =
        track_control_abs_int32(debug_base_command);

    if (!(TRACK_CONTROL_TARGET_GYRO_LIMIT_LSB > 0.0F)
        || !(TRACK_CONTROL_PWM_CURR_LIMIT > 0.0F)
        || (TRACK_CONTROL_PWM_CURR_LIMIT
            > (float32)((int32)MOTOR_COMMAND_MAX
                - debug_base_magnitude)))
    {
        track_control_snapshot.status =
            TRACK_CONTROL_STATUS_INVALID_CONFIG;
        return track_control_snapshot.status;
    }

    outer_status = pid_init(&track_control_outer_pid, &outer_config);
    inner_status = pid_init(&track_control_inner_pid, &inner_config);
    if ((PID_STATUS_OK != outer_status)
        || (PID_STATUS_OK != inner_status))
    {
        track_control_snapshot.status =
            TRACK_CONTROL_STATUS_INVALID_CONFIG;
        return track_control_snapshot.status;
    }

    track_control_initialized = ZF_TRUE;
    track_control_snapshot.status = TRACK_CONTROL_STATUS_DISABLED;
    return TRACK_CONTROL_STATUS_OK;
}

static track_control_status_enum track_control_enable_internal (
    int32 motor_a_start,
    int32 motor_b_start,
    int32 motor_a_target,
    int32 motor_b_target,
    uint16 ramp_inner_cycles)
{
    uint32 interrupt_state;
    int32 pwm_base_start;
    int32 pwm_base_target;

    if (ZF_FALSE == track_control_initialized)
    {
        return TRACK_CONTROL_STATUS_NOT_INITIALIZED;
    }
    if (ZF_FALSE == motor_is_ready())
    {
        return TRACK_CONTROL_STATUS_NOT_INITIALIZED;
    }

    if (((int32)MOTOR_COMMAND_MIN > motor_a_start)
        || ((int32)MOTOR_COMMAND_MAX < motor_a_start)
        || ((int32)MOTOR_COMMAND_MIN > motor_b_start)
        || ((int32)MOTOR_COMMAND_MAX < motor_b_start)
        || ((int32)MOTOR_COMMAND_MIN > motor_a_target)
        || ((int32)MOTOR_COMMAND_MAX < motor_a_target)
        || ((int32)MOTOR_COMMAND_MIN > motor_b_target)
        || ((int32)MOTOR_COMMAND_MAX < motor_b_target))
    {
        return TRACK_CONTROL_STATUS_INVALID_CONFIG;
    }

    if (TRACK_CONTROL_STATUS_OK
        != track_control_validate_motor_ramp_config(
            motor_a_start,
            motor_b_start,
            motor_a_target,
            motor_b_target))
    {
        return TRACK_CONTROL_STATUS_INVALID_CONFIG;
    }

    interrupt_state = interrupt_global_disable();
    (void)pid_reset(&track_control_outer_pid);
    (void)pid_reset(&track_control_inner_pid);
    track_control_snapshot.enabled = ZF_TRUE;
    track_control_snapshot.gyro_debug_enabled = ZF_FALSE;
    track_control_snapshot.line_valid = ZF_FALSE;
    track_control_snapshot.status = TRACK_CONTROL_STATUS_WAITING_FOR_LINE;
    track_control_snapshot.line_position = 0.0F;
    track_control_snapshot.line_error = 0.0F;
    track_control_snapshot.target_gyro_lsb = 0.0F;
    track_control_snapshot.actual_gyro_lsb = 0.0F;
    track_control_snapshot.gyro_error = 0.0F;
    track_control_snapshot.pwm_curr = 0.0F;
    pwm_base_start =
        track_control_average_int32(motor_a_start, motor_b_start);
    pwm_base_target =
        track_control_average_int32(motor_a_target, motor_b_target);
    track_control_snapshot.pwm_base =
        (0U == ramp_inner_cycles) ? pwm_base_target : 0;
    track_control_snapshot.pwm_base_ramp_start = pwm_base_start;
    track_control_snapshot.pwm_base_target = pwm_base_target;
    track_control_snapshot.pwm_base_ramp_cycles = ramp_inner_cycles;
    track_control_snapshot.pwm_base_ramp_completed_cycles = 0U;
    track_control_snapshot.motor_a_ramp_start = motor_a_start;
    track_control_snapshot.motor_b_ramp_start = motor_b_start;
    track_control_snapshot.motor_a_ramp_target = motor_a_target;
    track_control_snapshot.motor_b_ramp_target = motor_b_target;
    track_control_snapshot.motor_a_request = 0;
    track_control_snapshot.motor_b_request = 0;
    track_control_snapshot.outer_update_count = 0U;
    track_control_snapshot.inner_update_count = 0U;
    interrupt_global_enable(interrupt_state);

    return TRACK_CONTROL_STATUS_OK;
}

track_control_status_enum track_control_enable (int32 pwm_base)
{
    return track_control_enable_internal(
        pwm_base,
        pwm_base,
        pwm_base,
        pwm_base,
        0U);
}

track_control_status_enum track_control_enable_ramp (
    int32 pwm_base_start,
    int32 pwm_base_target,
    uint16 ramp_inner_cycles)
{
    if (2U > ramp_inner_cycles)
    {
        return TRACK_CONTROL_STATUS_INVALID_CONFIG;
    }

    return track_control_enable_internal(
        pwm_base_start,
        pwm_base_start,
        pwm_base_target,
        pwm_base_target,
        ramp_inner_cycles);
}

track_control_status_enum track_control_enable_motor_ramp (
    int32 motor_a_start,
    int32 motor_b_start,
    int32 motor_a_target,
    int32 motor_b_target,
    uint16 ramp_inner_cycles)
{
    if (2U > ramp_inner_cycles)
    {
        return TRACK_CONTROL_STATUS_INVALID_CONFIG;
    }

    return track_control_enable_internal(
        motor_a_start,
        motor_b_start,
        motor_a_target,
        motor_b_target,
        ramp_inner_cycles);
}

track_control_status_enum track_control_set_motor_ramp_irq (
    int32 motor_a_start,
    int32 motor_b_start,
    int32 motor_a_target,
    int32 motor_b_target,
    uint16 ramp_inner_cycles)
{
    int32 pwm_base_start;
    int32 pwm_base_target;

    if (ZF_FALSE == track_control_initialized)
    {
        return TRACK_CONTROL_STATUS_NOT_INITIALIZED;
    }
    if ((ZF_FALSE == track_control_snapshot.enabled)
        || (ZF_TRUE == track_control_snapshot.gyro_debug_enabled))
    {
        return TRACK_CONTROL_STATUS_DISABLED;
    }
    if (2U > ramp_inner_cycles)
    {
        return TRACK_CONTROL_STATUS_INVALID_CONFIG;
    }
    if (TRACK_CONTROL_STATUS_OK
        != track_control_validate_motor_ramp_config(
            motor_a_start,
            motor_b_start,
            motor_a_target,
            motor_b_target))
    {
        return TRACK_CONTROL_STATUS_INVALID_CONFIG;
    }

    pwm_base_start =
        track_control_average_int32(motor_a_start, motor_b_start);
    pwm_base_target =
        track_control_average_int32(motor_a_target, motor_b_target);
    track_control_snapshot.pwm_base = pwm_base_start;
    track_control_snapshot.pwm_base_ramp_start = pwm_base_start;
    track_control_snapshot.pwm_base_target = pwm_base_target;
    track_control_snapshot.pwm_base_ramp_cycles = ramp_inner_cycles;
    track_control_snapshot.pwm_base_ramp_completed_cycles = 0U;
    track_control_snapshot.motor_a_ramp_start = motor_a_start;
    track_control_snapshot.motor_b_ramp_start = motor_b_start;
    track_control_snapshot.motor_a_ramp_target = motor_a_target;
    track_control_snapshot.motor_b_ramp_target = motor_b_target;
    return TRACK_CONTROL_STATUS_OK;
}

track_control_status_enum track_control_set_pwm_base_irq (int32 pwm_base)
{
    int32 base_magnitude;

    if (ZF_FALSE == track_control_initialized)
    {
        return TRACK_CONTROL_STATUS_NOT_INITIALIZED;
    }
    if ((ZF_FALSE == track_control_snapshot.enabled)
        || (ZF_TRUE == track_control_snapshot.gyro_debug_enabled))
    {
        return TRACK_CONTROL_STATUS_DISABLED;
    }
    if (((int32)MOTOR_COMMAND_MIN > pwm_base)
        || ((int32)MOTOR_COMMAND_MAX < pwm_base))
    {
        return TRACK_CONTROL_STATUS_INVALID_CONFIG;
    }

    base_magnitude = track_control_abs_int32(pwm_base);
    if (TRACK_CONTROL_PWM_CURR_LIMIT
        > (float32)((int32)MOTOR_COMMAND_MAX - base_magnitude))
    {
        return TRACK_CONTROL_STATUS_INVALID_CONFIG;
    }

    track_control_snapshot.pwm_base = pwm_base;
    track_control_snapshot.pwm_base_ramp_start = pwm_base;
    track_control_snapshot.pwm_base_target = pwm_base;
    track_control_snapshot.pwm_base_ramp_cycles = 0U;
    track_control_snapshot.pwm_base_ramp_completed_cycles = 0U;
    track_control_snapshot.motor_a_ramp_start = pwm_base;
    track_control_snapshot.motor_b_ramp_start = pwm_base;
    track_control_snapshot.motor_a_ramp_target = pwm_base;
    track_control_snapshot.motor_b_ramp_target = pwm_base;
    return TRACK_CONTROL_STATUS_OK;
}

track_control_status_enum track_control_enable_gyro_debug (void)
{
    uint32 interrupt_state;
    const volatile ImuAppData *imu_data;

    if (ZF_FALSE == track_control_initialized)
    {
        return TRACK_CONTROL_STATUS_NOT_INITIALIZED;
    }
    if (ZF_FALSE == motor_is_ready())
    {
        return TRACK_CONTROL_STATUS_NOT_INITIALIZED;
    }

    imu_data = imu_app_get_data();
    if ((0 == imu_data)
        || (ZF_FALSE == imu_data->initialized)
        || (ZF_FALSE == imu_data->offset_valid))
    {
        return TRACK_CONTROL_STATUS_IMU_INVALID;
    }

    interrupt_state = interrupt_global_disable();
    (void)pid_reset(&track_control_outer_pid);
    (void)pid_reset(&track_control_inner_pid);
    track_control_snapshot.enabled = ZF_TRUE;
    track_control_snapshot.gyro_debug_enabled = ZF_TRUE;
    track_control_snapshot.line_valid = ZF_FALSE;
    track_control_snapshot.status = TRACK_CONTROL_STATUS_OK;
    track_control_snapshot.line_position = 0.0F;
    track_control_snapshot.line_error = 0.0F;
    track_control_snapshot.target_gyro_lsb = 0.0F;
    track_control_snapshot.actual_gyro_lsb = 0.0F;
    track_control_snapshot.gyro_error = 0.0F;
    track_control_snapshot.pwm_curr = 0.0F;
    track_control_snapshot.pwm_base = 0;
    track_control_snapshot.pwm_base_ramp_start = 0;
    track_control_snapshot.pwm_base_target = 0;
    track_control_snapshot.pwm_base_ramp_cycles = 0U;
    track_control_snapshot.pwm_base_ramp_completed_cycles = 0U;
    track_control_snapshot.motor_a_ramp_start = 0;
    track_control_snapshot.motor_b_ramp_start = 0;
    track_control_snapshot.motor_a_ramp_target = 0;
    track_control_snapshot.motor_b_ramp_target = 0;
    track_control_snapshot.motor_a_request = 0;
    track_control_snapshot.motor_b_request = 0;
    track_control_snapshot.outer_update_count = 0U;
    track_control_snapshot.inner_update_count = 0U;
    interrupt_global_enable(interrupt_state);

    return TRACK_CONTROL_STATUS_OK;
}

track_control_status_enum track_control_disable (void)
{
    uint32 interrupt_state;

    if (ZF_FALSE == track_control_initialized)
    {
        return TRACK_CONTROL_STATUS_NOT_INITIALIZED;
    }

    interrupt_state = interrupt_global_disable();
    track_control_snapshot.enabled = ZF_FALSE;
    track_control_snapshot.gyro_debug_enabled = ZF_FALSE;
    track_control_snapshot.line_valid = ZF_FALSE;
    track_control_snapshot.status = TRACK_CONTROL_STATUS_DISABLED;
    track_control_snapshot.target_gyro_lsb = 0.0F;
    track_control_snapshot.gyro_error = 0.0F;
    track_control_snapshot.pwm_base = 0;
    track_control_snapshot.pwm_base_ramp_start = 0;
    track_control_snapshot.pwm_base_target = 0;
    track_control_snapshot.pwm_base_ramp_cycles = 0U;
    track_control_snapshot.pwm_base_ramp_completed_cycles = 0U;
    track_control_snapshot.motor_a_ramp_start = 0;
    track_control_snapshot.motor_b_ramp_start = 0;
    track_control_snapshot.motor_a_ramp_target = 0;
    track_control_snapshot.motor_b_ramp_target = 0;
    (void)pid_reset(&track_control_outer_pid);
    (void)pid_reset(&track_control_inner_pid);
    interrupt_global_enable(interrupt_state);

    track_control_stop_motors();
    return TRACK_CONTROL_STATUS_OK;
}

uint8 track_control_is_enabled (void)
{
    return track_control_snapshot.enabled;
}

uint8 track_control_is_ramp_active_irq (void)
{
    return ((ZF_TRUE == track_control_initialized)
        && (ZF_TRUE == track_control_snapshot.enabled)
        && (0U < track_control_snapshot.pwm_base_ramp_cycles)
        && (track_control_snapshot.pwm_base_ramp_completed_cycles
            < track_control_snapshot.pwm_base_ramp_cycles))
        ? ZF_TRUE
        : ZF_FALSE;
}

uint8 track_control_is_gyro_debug_enabled (void)
{
    return ((ZF_TRUE == track_control_snapshot.enabled)
        && (ZF_TRUE == track_control_snapshot.gyro_debug_enabled))
        ? ZF_TRUE
        : ZF_FALSE;
}

track_control_status_enum track_control_get_status (void)
{
    return track_control_snapshot.status;
}

void track_control_process_gray_sample_irq (
    gw_gray_status_enum gray_status,
    const gray_app_sample_struct *gray_sample)
{
    if ((ZF_FALSE == track_control_initialized)
        || (ZF_FALSE == track_control_snapshot.enabled)
        || (ZF_TRUE == track_control_snapshot.gyro_debug_enabled))
    {
        return;
    }

    if ((GW_GRAY_STATUS_OK != gray_status) || (0 == gray_sample))
    {
        track_control_snapshot.status = TRACK_CONTROL_STATUS_LINE_INVALID;
        return;
    }

    if (0U == gray_sample->error_num)
    {
        track_control_snapshot.status = TRACK_CONTROL_STATUS_WAITING_FOR_LINE;
        return;
    }
}

void track_control_outer_step_irq (
    gw_gray_status_enum gray_status,
    const gray_app_sample_struct *gray_sample)
{
    pid_status_enum pid_status;
    float32 line_position;
    float32 target_gyro_lsb;

    if ((ZF_FALSE == track_control_initialized)
        || (ZF_FALSE == track_control_snapshot.enabled)
        || (ZF_TRUE == track_control_snapshot.gyro_debug_enabled))
    {
        return;
    }

    if ((GW_GRAY_STATUS_OK != gray_status) || (0 == gray_sample))
    {
        track_control_snapshot.status = TRACK_CONTROL_STATUS_LINE_INVALID;
        return;
    }

    if (0U == gray_sample->error_num)
    {
        track_control_snapshot.status = TRACK_CONTROL_STATUS_WAITING_FOR_LINE;
        return;
    }

    line_position = (float32)gray_sample->error;
    target_gyro_lsb = track_control_snapshot.target_gyro_lsb;

    pid_status = pid_step(
        &track_control_outer_pid,
        0.0F,
        line_position,
        (float32)TRACK_CONTROL_OUTER_PERIOD_MS / 1000.0F,
        &target_gyro_lsb);
    if (PID_STATUS_OK != pid_status)
    {
        track_control_snapshot.status = TRACK_CONTROL_STATUS_PID_ERROR;
        return;
    }

    track_control_snapshot.line_position = line_position;
    track_control_snapshot.line_error = -line_position;
    track_control_snapshot.target_gyro_lsb = target_gyro_lsb;
    track_control_snapshot.line_valid = ZF_TRUE;
    track_control_snapshot.status = TRACK_CONTROL_STATUS_OK;

    if (track_control_update_count_max
        != track_control_snapshot.outer_update_count)
    {
        track_control_snapshot.outer_update_count++;
    }
}

static uint8 track_control_run_inner_pid_irq (
    const volatile ImuAppData *imu_data,
    int32 motor_a_base,
    int32 motor_b_base)
{
    pid_status_enum pid_status;
    float32 actual_gyro_lsb;
    float32 pwm_curr;
    int16 pwm_curr_rounded;
    int32 motor_a_request;
    int32 motor_b_request;
    track_control_status_enum cycle_status = TRACK_CONTROL_STATUS_OK;

    actual_gyro_lsb = track_control_snapshot.actual_gyro_lsb;
    pwm_curr = track_control_snapshot.pwm_curr;
    if ((0 == imu_data)
        || (ZF_FALSE == imu_data->initialized)
        || (ZF_FALSE == imu_data->offset_valid))
    {
        cycle_status = TRACK_CONTROL_STATUS_IMU_INVALID;
    }
    else
    {
        actual_gyro_lsb = track_control_select_gyro_lsb(imu_data);
        pid_status = pid_step(
            &track_control_inner_pid,
            track_control_snapshot.target_gyro_lsb,
            actual_gyro_lsb,
            (float32)TRACK_CONTROL_INNER_PERIOD_MS / 1000.0F,
            &pwm_curr);
        if (PID_STATUS_OK != pid_status)
        {
            cycle_status = TRACK_CONTROL_STATUS_PID_ERROR;
        }
    }

    pwm_curr_rounded = track_control_round_pwm_curr(
        (TRACK_CONTROL_STATUS_OK == cycle_status)
        ? pwm_curr
        : track_control_snapshot.pwm_curr);
    motor_a_request =
        motor_a_base - (int32)pwm_curr_rounded;
    motor_b_request =
        motor_b_base + (int32)pwm_curr_rounded;

    track_control_snapshot.motor_a_request =
        track_control_clamp_motor_command(motor_a_request);
    track_control_snapshot.motor_b_request =
        track_control_clamp_motor_command(motor_b_request);

    if ((MOTOR_STATUS_OK
            != motor_set_A(track_control_snapshot.motor_a_request))
        || (MOTOR_STATUS_OK
            != motor_set_B(track_control_snapshot.motor_b_request)))
    {
        track_control_snapshot.status = TRACK_CONTROL_STATUS_MOTOR_ERROR;
        track_control_snapshot.motor_a_applied = motor_get_a_command();
        track_control_snapshot.motor_b_applied = motor_get_b_command();
        return ZF_FALSE;
    }

    if (TRACK_CONTROL_STATUS_OK == cycle_status)
    {
        track_control_snapshot.actual_gyro_lsb = actual_gyro_lsb;
        track_control_snapshot.gyro_error =
            track_control_snapshot.target_gyro_lsb - actual_gyro_lsb;
        track_control_snapshot.pwm_curr = pwm_curr;
    }
    track_control_snapshot.motor_a_applied = motor_get_a_command();
    track_control_snapshot.motor_b_applied = motor_get_b_command();
    track_control_snapshot.status =
        ((TRACK_CONTROL_STATUS_OK == cycle_status)
            && (ZF_FALSE == track_control_snapshot.line_valid))
        ? TRACK_CONTROL_STATUS_WAITING_FOR_LINE
        : cycle_status;

    if (track_control_update_count_max
        != track_control_snapshot.inner_update_count)
    {
        track_control_snapshot.inner_update_count++;
    }

    return ZF_TRUE;
}

void track_control_inner_step_irq (
    const volatile ImuAppData *imu_data)
{
    int32 motor_a_base;
    int32 motor_b_base;
    uint16 completed_cycles;

    if ((ZF_FALSE == track_control_initialized)
        || (ZF_FALSE == track_control_snapshot.enabled)
        || (ZF_TRUE == track_control_snapshot.gyro_debug_enabled))
    {
        return;
    }

    if (ZF_FALSE == track_control_snapshot.line_valid)
    {
        track_control_snapshot.status =
            TRACK_CONTROL_STATUS_WAITING_FOR_LINE;
    }

    completed_cycles =
        track_control_snapshot.pwm_base_ramp_completed_cycles;

    motor_a_base = track_control_calculate_ramp_pwm(
        track_control_snapshot.motor_a_ramp_start,
        track_control_snapshot.motor_a_ramp_target,
        completed_cycles,
        track_control_snapshot.pwm_base_ramp_cycles);
    motor_b_base = track_control_calculate_ramp_pwm(
        track_control_snapshot.motor_b_ramp_start,
        track_control_snapshot.motor_b_ramp_target,
        completed_cycles,
        track_control_snapshot.pwm_base_ramp_cycles);

    if (ZF_TRUE == track_control_run_inner_pid_irq(
            imu_data,
            motor_a_base,
            motor_b_base))
    {
        if (completed_cycles < track_control_snapshot.pwm_base_ramp_cycles)
        {
            completed_cycles++;
        }
        track_control_snapshot.pwm_base =
            track_control_average_int32(motor_a_base, motor_b_base);
        track_control_snapshot.pwm_base_ramp_completed_cycles =
            completed_cycles;
    }
}

void track_control_gyro_debug_step_irq (
    const volatile ImuAppData *imu_data)
{
    if ((ZF_FALSE == track_control_initialized)
        || (ZF_FALSE == track_control_snapshot.enabled)
        || (ZF_FALSE == track_control_snapshot.gyro_debug_enabled))
    {
        return;
    }

    track_control_snapshot.line_valid = ZF_FALSE;
    track_control_snapshot.line_position = 0.0F;
    track_control_snapshot.line_error = 0.0F;
    track_control_snapshot.target_gyro_lsb = 0.0F;

    (void)track_control_run_inner_pid_irq(
        imu_data,
        TRACK_CONTROL_GYRO_DEBUG_PWM_BASE,
        TRACK_CONTROL_GYRO_DEBUG_PWM_BASE);
}

uint8 track_control_get_snapshot (
    track_control_snapshot_struct *snapshot)
{
    uint32 interrupt_state;

    if (0 == snapshot)
    {
        return ZF_FALSE;
    }

    interrupt_state = interrupt_global_disable();
    *snapshot = track_control_snapshot;
    interrupt_global_enable(interrupt_state);

    return ZF_TRUE;
}
