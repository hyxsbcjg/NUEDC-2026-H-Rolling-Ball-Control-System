#include "pid.h"

#include <float.h>

static uint8 pid_float_is_finite (float32 value);
static float32 pid_clamp (
    float32 value,
    float32 minimum,
    float32 maximum);
static void pid_clear_config (pid_config_struct *config);
static void pid_clear_state (pid_state_struct *state);

static uint8 pid_float_is_finite (float32 value)
{
    uint8 finite = ZF_FALSE;

    /* value == value 排除 NaN，FLT_MAX 边界再排除正、负无穷。 */
    if ((value == value)
        && ((float32)(-FLT_MAX) <= value)
        && ((float32)FLT_MAX >= value))
    {
        finite = ZF_TRUE;
    }

    return finite;
}

static float32 pid_clamp (
    float32 value,
    float32 minimum,
    float32 maximum)
{
    float32 limited = value;

    if (maximum < limited)
    {
        limited = maximum;
    }
    else if (minimum > limited)
    {
        limited = minimum;
    }

    return limited;
}

static void pid_clear_config (pid_config_struct *config)
{
    config->kp = 0.0F;
    config->ki = 0.0F;
    config->kd = 0.0F;
    config->output_min = 0.0F;
    config->output_max = 0.0F;
    config->integral_min = 0.0F;
    config->integral_max = 0.0F;
}

static void pid_clear_state (pid_state_struct *state)
{
    state->error = 0.0F;
    state->previous_error = 0.0F;
    state->integral_output = 0.0F;
    state->derivative = 0.0F;
    state->output = 0.0F;
    state->history_valid = ZF_FALSE;
}

uint8 pid_config_is_valid (const pid_config_struct *config)
{
    if (0 == config)
    {
        return ZF_FALSE;
    }

    if ((ZF_FALSE == pid_float_is_finite(config->kp))
        || (ZF_FALSE == pid_float_is_finite(config->ki))
        || (ZF_FALSE == pid_float_is_finite(config->kd))
        || (ZF_FALSE == pid_float_is_finite(config->output_min))
        || (ZF_FALSE == pid_float_is_finite(config->output_max))
        || (ZF_FALSE == pid_float_is_finite(config->integral_min))
        || (ZF_FALSE == pid_float_is_finite(config->integral_max)))
    {
        return ZF_FALSE;
    }

    if ((config->output_min > config->output_max)
        || (config->output_min > 0.0F)
        || (config->output_max < 0.0F)
        || (config->integral_min > config->integral_max)
        || (config->integral_min > 0.0F)
        || (config->integral_max < 0.0F))
    {
        return ZF_FALSE;
    }

    return ZF_TRUE;
}

pid_status_enum pid_init (
    pid_controller_struct *pid,
    const pid_config_struct *config)
{
    if (0 == pid)
    {
        return PID_STATUS_INVALID_ARGUMENT;
    }

    pid_clear_config(&pid->config);
    pid_clear_state(&pid->state);
    pid->initialized = ZF_FALSE;

    /*
     * 先清空目标实例再校验外部配置：初始化失败时对象明确保持“未初始化”，
     * 不会继续沿用上一次参数或历史积分。
     */
    if (ZF_FALSE == pid_config_is_valid(config))
    {
        return PID_STATUS_INVALID_CONFIG;
    }

    pid->config = *config;
    pid->initialized = ZF_TRUE;
    return PID_STATUS_OK;
}

uint8 pid_is_ready (const pid_controller_struct *pid)
{
    uint8 ready = ZF_FALSE;

    if ((0 != pid) && (ZF_TRUE == pid->initialized))
    {
        ready = ZF_TRUE;
    }

    return ready;
}

pid_status_enum pid_reset (pid_controller_struct *pid)
{
    if (0 == pid)
    {
        return PID_STATUS_INVALID_ARGUMENT;
    }
    if (ZF_FALSE == pid->initialized)
    {
        return PID_STATUS_NOT_INITIALIZED;
    }

    pid_clear_state(&pid->state);
    return PID_STATUS_OK;
}

pid_status_enum pid_step (
    pid_controller_struct *pid,
    float32 target,
    float32 actual,
    float32 dt_s,
    float32 *output)
{
    float32 error;
    float32 proportional_output;
    float32 derivative = 0.0F;
    float32 derivative_output;
    float32 integral_previous;
    float32 integral_delta;
    float32 integral_next;
    float32 integral_allowed;
    float32 output_without_integral;
    float32 output_unlimited;
    float32 output_limited;

    if ((0 == pid) || (0 == output))
    {
        return PID_STATUS_INVALID_ARGUMENT;
    }
    if (ZF_FALSE == pid->initialized)
    {
        return PID_STATUS_NOT_INITIALIZED;
    }
    if ((ZF_FALSE == pid_float_is_finite(target))
        || (ZF_FALSE == pid_float_is_finite(actual))
        || (ZF_FALSE == pid_float_is_finite(dt_s))
        || !(dt_s > 0.0F))
    {
        return PID_STATUS_NUMERIC_ERROR;
    }

    error = target - actual;
    proportional_output = pid->config.kp * error;
    if ((ZF_FALSE == pid_float_is_finite(error))
        || (ZF_FALSE == pid_float_is_finite(proportional_output)))
    {
        return PID_STATUS_NUMERIC_ERROR;
    }

    if (ZF_TRUE == pid->state.history_valid)
    {
        /* 微分使用上一次成功提交的误差；首次成功计算无历史，D 项强制为零。 */
        derivative = (error - pid->state.error) / dt_s;
        if (ZF_FALSE == pid_float_is_finite(derivative))
        {
            return PID_STATUS_NUMERIC_ERROR;
        }
    }

    derivative_output = pid->config.kd * derivative;
    integral_previous = pid->state.integral_output;
    integral_delta = pid->config.ki * error * dt_s;
    if ((ZF_FALSE == pid_float_is_finite(derivative_output))
        || (ZF_FALSE == pid_float_is_finite(integral_previous))
        || (ZF_FALSE == pid_float_is_finite(integral_delta)))
    {
        return PID_STATUS_NUMERIC_ERROR;
    }

    /*
     * integral_output 已经包含 ki，是可直接与 P、D 相加的“积分输出项”，
     * 先形成候选值，再施加独立于总输出限幅的积分硬边界。
     */
    integral_next = integral_previous + integral_delta;
    if (ZF_FALSE == pid_float_is_finite(integral_next))
    {
        return PID_STATUS_NUMERIC_ERROR;
    }
    integral_next = pid_clamp(
        integral_next,
        pid->config.integral_min,
        pid->config.integral_max);

    output_without_integral = proportional_output + derivative_output;
    output_unlimited = output_without_integral + integral_next;
    if ((ZF_FALSE == pid_float_is_finite(output_without_integral))
        || (ZF_FALSE == pid_float_is_finite(output_unlimited)))
    {
        return PID_STATUS_NUMERIC_ERROR;
    }

    if ((output_unlimited > pid->config.output_max)
        && (integral_next > integral_previous))
    {
        /*
         * 上饱和且本步积分仍向上推时，只允许积分增长到恰好触及输出上限；
         * 若 P+D 已独自越界则冻结积分。反向积分仍被允许，以便退出饱和。
         */
        integral_allowed =
            pid->config.output_max - output_without_integral;
        if (ZF_FALSE == pid_float_is_finite(integral_allowed))
        {
            return PID_STATUS_NUMERIC_ERROR;
        }

        if (integral_allowed > integral_previous)
        {
            integral_next = pid_clamp(
                integral_allowed,
                integral_previous,
                integral_next);
        }
        else
        {
            integral_next = integral_previous;
        }
    }
    else if ((output_unlimited < pid->config.output_min)
        && (integral_next < integral_previous))
    {
        /*
         * 下饱和与上支路完全对称：限制继续向负方向累积，但不阻止积分回退。
         */
        integral_allowed =
            pid->config.output_min - output_without_integral;
        if (ZF_FALSE == pid_float_is_finite(integral_allowed))
        {
            return PID_STATUS_NUMERIC_ERROR;
        }

        if (integral_allowed < integral_previous)
        {
            integral_next = pid_clamp(
                integral_allowed,
                integral_next,
                integral_previous);
        }
        else
        {
            integral_next = integral_previous;
        }
    }

    output_unlimited = output_without_integral + integral_next;
    if (ZF_FALSE == pid_float_is_finite(output_unlimited))
    {
        return PID_STATUS_NUMERIC_ERROR;
    }
    output_limited = pid_clamp(
        output_unlimited,
        pid->config.output_min,
        pid->config.output_max);

    /*
     * 所有中间量都通过有限性检查后才一次性提交状态；此前任何错误返回都不会
     * 改写历史、积分或调用者的 output，下一次调用仍从上次成功状态继续。
     */
    if (ZF_TRUE == pid->state.history_valid)
    {
        pid->state.previous_error = pid->state.error;
    }
    else
    {
        pid->state.previous_error = error;
    }
    pid->state.error = error;
    pid->state.integral_output = integral_next;
    pid->state.derivative = derivative;
    pid->state.output = output_limited;
    pid->state.history_valid = ZF_TRUE;
    *output = output_limited;

    return PID_STATUS_OK;
}
