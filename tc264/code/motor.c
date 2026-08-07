#include "motor.h"

#include "my_battery.h"
#include "zf_driver_delay.h"

#define MOTOR_DIRECTION_CHANGE_DELAY_US \
    ((1000000U + MOTOR_PWM_FREQUENCY_HZ - 1U) \
        / MOTOR_PWM_FREQUENCY_HZ)

#pragma section all "cpu0_dsram"
static uint8 motor_initialized = ZF_FALSE;
static int16 motor_a_command = 0;
static int16 motor_b_command = 0;
#pragma section all restore

static int16 motor_clamp_command (int32 command)
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

static int16 motor_compensate_command (int16 command)
{
    uint32 battery_millivolts;
    int32 compensated_command;

    /*
     * 输入先限到业务量程，再按 12 V 等效输出做电压前馈补偿。电池采样尚未
     * 就绪或读数为零时，非零请求会降为安全零输出，而不是带着未知分母驱动。
     */
    command = motor_clamp_command((int32)command);
    if (0 == command)
    {
        return 0;
    }

    if (ZF_FALSE == my_battery_is_ready())
    {
        return 0;
    }

    battery_millivolts = my_battery_get_millivolts();
    if (0U == battery_millivolts)
    {
        return 0;
    }

    compensated_command =
        (int32)command * (int32)MOTOR_VOLTAGE_REFERENCE_MV;
    /* 除法前加减半个分母，使正、负命令都按绝对值四舍五入。 */
    if (0 < compensated_command)
    {
        compensated_command += (int32)(battery_millivolts / 2U);
    }
    else
    {
        compensated_command -= (int32)(battery_millivolts / 2U);
    }
    compensated_command /= (int32)battery_millivolts;

    return motor_clamp_command(compensated_command);
}

static uint8 motor_requires_zero_period (
    int16 previous_command,
    int16 next_command)
{
    uint8 required = ZF_FALSE;

    /*
     * 仅“非零到零”或“正负号改变”需要插入零输出周期；同方向调幅和从零启动
     * 不插入等待。这里的间隔是功率桥换向死区，不是补偿机械静摩擦的命令死区。
     */
    if ((0 != previous_command)
        && ((0 == next_command)
            || ((0 < previous_command) && (0 > next_command))
            || ((0 > previous_command) && (0 < next_command))))
    {
        required = ZF_TRUE;
    }

    return required;
}

static uint32 motor_command_to_duty (int16 command)
{
    uint32 magnitude;

    if (0 <= command)
    {
        magnitude = (uint32)command;
    }
    else
    {
        magnitude = (uint32)(-command);
    }

    /*
     * 业务命令按 |command| / 8000 映射到逐飞 0..PWM_DUTY_MAX 的量程，
     * 加半个参考周期后取整；因此最大命令 7000 对应 87.5% 占空比。
     */
    return (magnitude * PWM_DUTY_MAX
        + (MOTOR_COMMAND_REFERENCE_PERIOD / 2U))
        / MOTOR_COMMAND_REFERENCE_PERIOD;
}

static void motor_set_direction (
    gpio_pin_enum in1_pin,
    gpio_pin_enum in2_pin,
    int16 command)
{
    if (0 < command)
    {
        gpio_set_level(in1_pin, GPIO_HIGH);
        gpio_set_level(in2_pin, GPIO_LOW);
    }
    else if (0 > command)
    {
        gpio_set_level(in1_pin, GPIO_LOW);
        gpio_set_level(in2_pin, GPIO_HIGH);
    }
    else
    {
        gpio_set_level(in1_pin, GPIO_LOW);
        gpio_set_level(in2_pin, GPIO_LOW);
    }
}

static int16 motor_apply_command (
    pwm_channel_enum pwm_pin,
    gpio_pin_enum in1_pin,
    gpio_pin_enum in2_pin,
    int16 previous_command,
    int16 requested_command)
{
    int16 command = motor_clamp_command(requested_command);
    uint8 zero_period_applied = ZF_FALSE;

    if (ZF_TRUE
        == motor_requires_zero_period(previous_command, command))
    {
        /*
         * 逐飞 pwm_set_duty() 更新影子比较值。至少等待一个完整 PWM 周期，
         * 保证零占空比已越过更新边界，再改变 H 桥方向输入，避免带载硬换向。
         */
        pwm_set_duty(pwm_pin, 0U);
        system_delay_us(MOTOR_DIRECTION_CHANGE_DELAY_US);
        zero_period_applied = ZF_TRUE;
    }

    if ((0 == command) && (ZF_FALSE == zero_period_applied))
    {
        pwm_set_duty(pwm_pin, 0U);
    }

    motor_set_direction(in1_pin, in2_pin, command);

    if (0 != command)
    {
        pwm_set_duty(pwm_pin, motor_command_to_duty(command));
    }

    return command;
}

motor_status_enum motor_init (void)
{
    motor_initialized = ZF_FALSE;
    motor_a_command = 0;
    motor_b_command = 0;

    /*
     * 先把输出锁存器预装为低电平，再切换成推挽输出，避免 GPIO 模式切换瞬间
     * 出现有效方向毛刺。当前板 STBY 硬件常高，因此四个桥臂输入必须从低开始。
     */
    gpio_low(MOTOR_A_IN1_PIN);
    gpio_low(MOTOR_A_IN2_PIN);
    gpio_low(MOTOR_B_IN1_PIN);
    gpio_low(MOTOR_B_IN2_PIN);

    gpio_init(MOTOR_A_IN1_PIN, GPO, GPIO_LOW, GPO_PUSH_PULL);
    gpio_init(MOTOR_A_IN2_PIN, GPO, GPIO_LOW, GPO_PUSH_PULL);
    gpio_init(MOTOR_B_IN1_PIN, GPO, GPIO_LOW, GPO_PUSH_PULL);
    gpio_init(MOTOR_B_IN2_PIN, GPO, GPIO_LOW, GPO_PUSH_PULL);

    pwm_init(MOTOR_A_PWM_PIN, MOTOR_PWM_FREQUENCY_HZ, 0U);
    pwm_init(MOTOR_B_PWM_PIN, MOTOR_PWM_FREQUENCY_HZ, 0U);

    motor_initialized = ZF_TRUE;
    return MOTOR_STATUS_OK;
}

uint8 motor_is_ready (void)
{
    return motor_initialized;
}

motor_status_enum motor_set_A (int16 command)
{
    if (ZF_FALSE == motor_initialized)
    {
        return MOTOR_STATUS_NOT_INITIALIZED;
    }

    motor_a_command = motor_apply_command(
        MOTOR_A_PWM_PIN,
        MOTOR_A_IN1_PIN,
        MOTOR_A_IN2_PIN,
        motor_a_command,
        motor_compensate_command(command));

    return MOTOR_STATUS_OK;
}

motor_status_enum motor_set_B (int16 command)
{
    if (ZF_FALSE == motor_initialized)
    {
        return MOTOR_STATUS_NOT_INITIALIZED;
    }

    motor_b_command = motor_apply_command(
        MOTOR_B_PWM_PIN,
        MOTOR_B_IN1_PIN,
        MOTOR_B_IN2_PIN,
        motor_b_command,
        motor_compensate_command(command));

    return MOTOR_STATUS_OK;
}

motor_status_enum motor_stop_all (void)
{
    if (MOTOR_STATUS_OK != motor_set_A(0))
    {
        return MOTOR_STATUS_NOT_INITIALIZED;
    }

    return motor_set_B(0);
}

int16 motor_get_a_command (void)
{
    return motor_a_command;
}

int16 motor_get_b_command (void)
{
    return motor_b_command;
}
