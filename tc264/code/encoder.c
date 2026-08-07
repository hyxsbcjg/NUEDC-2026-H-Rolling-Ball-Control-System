#include "encoder.h"

#pragma section all "cpu0_dsram"
static uint8 encoder_initialized = ZF_FALSE;
#pragma section all restore

static int16 encoder_apply_direction (int16 count, int16 direction_sign)
{
    return (0 < direction_sign) ? count : (int16)(-count);
}

static int16 encoder_read_and_clear_index (
    encoder_index_enum index,
    int16 direction_sign)
{
    /*
     * 逐飞接口没有原子“取值并清零”操作：这里先读后清，返回的是上一个采样窗
     * 的增量。两次寄存器访问之间到达的边沿可能随清零丢失，调用方应固定周期
     * 读取，并把该窗口限制在 16 位硬件计数器不会回绕的范围内。
     */
    int16 count = encoder_get_count(index);

    encoder_clear_count(index);
    return encoder_apply_direction(count, direction_sign);
}

encoder_status_enum encoder_init (void)
{
    encoder_initialized = ZF_FALSE;

    encoder_quad_init(
        ENCODER_A_INDEX,
        ENCODER_A_CH1_PIN,
        ENCODER_A_CH2_PIN);
    encoder_quad_init(
        ENCODER_B_INDEX,
        ENCODER_B_CH1_PIN,
        ENCODER_B_CH2_PIN);

    encoder_clear_count(ENCODER_A_INDEX);
    encoder_clear_count(ENCODER_B_INDEX);
    /*
     * 底层初始化/清零接口均无错误返回；ready 只表示软件配置序列已完成，
     * 不能据此判断编码器供电、接线、方向或脉冲输入是否正常。
     */
    encoder_initialized = ZF_TRUE;

    return ENCODER_STATUS_OK;
}

uint8 encoder_is_ready (void)
{
    return encoder_initialized;
}

encoder_status_enum encoder_read_and_clear_a (int16 *count)
{
    if (0 == count)
    {
        return ENCODER_STATUS_INVALID_ARGUMENT;
    }
    if (ZF_FALSE == encoder_initialized)
    {
        return ENCODER_STATUS_NOT_INITIALIZED;
    }

    *count = encoder_read_and_clear_index(
        ENCODER_A_INDEX,
        ENCODER_A_DIRECTION_SIGN);
    return ENCODER_STATUS_OK;
}

encoder_status_enum encoder_read_and_clear_b (int16 *count)
{
    if (0 == count)
    {
        return ENCODER_STATUS_INVALID_ARGUMENT;
    }
    if (ZF_FALSE == encoder_initialized)
    {
        return ENCODER_STATUS_NOT_INITIALIZED;
    }

    *count = encoder_read_and_clear_index(
        ENCODER_B_INDEX,
        ENCODER_B_DIRECTION_SIGN);
    return ENCODER_STATUS_OK;
}

encoder_status_enum encoder_read_and_clear_both (
    encoder_delta_struct *delta)
{
    if (0 == delta)
    {
        return ENCODER_STATUS_INVALID_ARGUMENT;
    }
    if (ZF_FALSE == encoder_initialized)
    {
        return ENCODER_STATUS_NOT_INITIALIZED;
    }

    delta->motor_a = encoder_read_and_clear_index(
        ENCODER_A_INDEX,
        ENCODER_A_DIRECTION_SIGN);
    delta->motor_b = encoder_read_and_clear_index(
        ENCODER_B_INDEX,
        ENCODER_B_DIRECTION_SIGN);

    return ENCODER_STATUS_OK;
}

encoder_status_enum encoder_clear_all (void)
{
    if (ZF_FALSE == encoder_initialized)
    {
        return ENCODER_STATUS_NOT_INITIALIZED;
    }

    encoder_clear_count(ENCODER_A_INDEX);
    encoder_clear_count(ENCODER_B_INDEX);
    return ENCODER_STATUS_OK;
}
