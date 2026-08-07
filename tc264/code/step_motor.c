#include "step_motor.h"

#include "zf_common_interrupt.h"
#include "zf_driver_delay.h"

#pragma section all "cpu0_dsram"
/*
 * busy 只保护本封装发出的命令；直接调用 stepper_uart_* 会绕过互斥。
 * 这两个状态由 CPU0 前台及其控制 ISR 共享，因此声明为 volatile。
 * 初始化留在前台；固定长度的速度/位置帧可由控制 ISR 发送。工程不提供
 * 写零或回零入口。
 */
static volatile uint8 step_motor_initialized = ZF_FALSE;
static volatile uint8 step_motor_command_busy = ZF_FALSE;
#pragma section all restore

static uint8 step_motor_try_lock (void)
{
    uint8 acquired = ZF_FALSE;
    uint32 interrupt_state;

    /*
     * 只在检查/置位 busy 的极短窗口屏蔽中断，并按保存值恢复调用前状态；
     * 不把 UART 发送或延时放进临界区。前台配置与控制 ISR 发生竞争时，后到
     * 的调用立即得到 TX_BUSY，不会在 ISR 中等待锁。
     */
    interrupt_state = interrupt_global_disable();
    if (ZF_FALSE == step_motor_command_busy)
    {
        step_motor_command_busy = ZF_TRUE;
        acquired = ZF_TRUE;
    }
    interrupt_global_enable(interrupt_state);

    return acquired;
}

static void step_motor_unlock (void)
{
    uint32 interrupt_state;

    interrupt_state = interrupt_global_disable();
    step_motor_command_busy = ZF_FALSE;
    interrupt_global_enable(interrupt_state);
}

stepper_uart_status_enum Step_Motor_Init (void)
{
    stepper_uart_status_enum status;

    step_motor_initialized = ZF_FALSE;
    step_motor_command_busy = ZF_TRUE;

    /*
     * 初始化全程保持命令锁：UART3 配置完成后等待驱动器上电，再依次发送使能
     * 和停止。只有停止帧成功交给本地发送层后才发布 initialized，避免异步
     * 速度命令插入这段安全状态建立序列。
     */
    status = stepper_uart_init();
    if (STEPPER_UART_STATUS_OK != status)
    {
        step_motor_unlock();
        return status;
    }

    system_delay_ms(STEPPER_UART_POWER_ON_DELAY_MS);

    status = stepper_uart_set_enable(
        STEP_MOTOR_ADDRESS,
        ZF_TRUE,
        STEP_MOTOR_SYNCHRONIZED);
    if (STEPPER_UART_STATUS_OK != status)
    {
        step_motor_unlock();
        return status;
    }

    system_delay_ms(STEPPER_UART_COMMAND_GAP_MS);

    status = stepper_uart_stop(
        STEP_MOTOR_ADDRESS,
        STEP_MOTOR_SYNCHRONIZED);
    if (STEPPER_UART_STATUS_OK == status)
    {
        step_motor_initialized = ZF_TRUE;
    }

    step_motor_unlock();
    return status;
}

stepper_uart_status_enum Trigger_Return (void)
{
    stepper_uart_status_enum status;

    if (ZF_FALSE == step_motor_initialized)
    {
        return STEPPER_UART_STATUS_NOT_INITIALIZED;
    }
    if (ZF_FALSE == step_motor_try_lock())
    {
        return STEPPER_UART_STATUS_TX_BUSY;
    }

    /*
     * 仅触发驱动器按既有机械零点执行最近点回零，不发送写当前位置、设置
     * 零点或回零配置命令。OK 只表示 5 字节帧进入本地 FIFO，不证明到位。
     */
    status = stepper_uart_try_start_home(STEP_MOTOR_ADDRESS);

    step_motor_unlock();
    return status;
}

stepper_uart_status_enum Step_Motor_Set_Speed (int16 signed_speed)
{
    stepper_uart_status_enum status;

    if (((int32)STEPPER_UART_MAX_SPEED_RPM < (int32)signed_speed)
        || (-((int32)STEPPER_UART_MAX_SPEED_RPM) > (int32)signed_speed))
    {
        return STEPPER_UART_STATUS_INVALID_ARGUMENT;
    }
    if (ZF_FALSE == step_motor_initialized)
    {
        return STEPPER_UART_STATUS_NOT_INITIALIZED;
    }
    if (ZF_FALSE == step_motor_try_lock())
    {
        return STEPPER_UART_STATUS_TX_BUSY;
    }

    /*
     * 实时速度更新刻意不等待 10 ms 配置命令间隔：底层仅在 UART3 TX FIFO
     * 已空时一次写入完整 8 字节帧，否则立即返回 TX_BUSY。控制 ISR 应在
     * 下一拍直接重试最新速度，不排队过期控制量。
     */
    status = stepper_uart_try_run_velocity_signed(
        STEP_MOTOR_ADDRESS,
        signed_speed,
        STEP_MOTOR_ACCELERATION,
        STEP_MOTOR_SYNCHRONIZED);

    step_motor_unlock();
    return status;
}

stepper_uart_status_enum Step_Motor_Set_Position (int32 signed_pulse_count)
{
    stepper_uart_status_enum status;

    if (ZF_FALSE == step_motor_initialized)
    {
        return STEPPER_UART_STATUS_NOT_INITIALIZED;
    }
    if (ZF_FALSE == step_motor_try_lock())
    {
        return STEPPER_UART_STATUS_TX_BUSY;
    }

    /*
     * 实时位置更新不等待命令间隔：底层仅在 UART3 TX FIFO 为空时一次写入
     * 完整 13 字节 FD 帧，否则立即返回 TX_BUSY。控制 ISR 应在后续拍直接
     * 重试最新绝对目标，不排队过期位置。
     *
     * 有符号脉冲表示驱动器绝对零点下的位置；方向拆分、脉冲绝对值和参数
     * 检查由 stepper_uart 统一完成。
     * signed_pulse_count 必须已是 PID 外层算出的最终绝对目标；本函数不把
     * 摄像头误差换算为脉冲，也不累加 PID 输出或执行机械软限位。
     */
    status = stepper_uart_try_move_signed(
        STEP_MOTOR_ADDRESS,
        signed_pulse_count,
        STEP_MOTOR_POSITION_SPEED_RPM,
        STEP_MOTOR_POSITION_ACCELERATION,
        STEP_MOTOR_POSITION_MODE,
        STEP_MOTOR_SYNCHRONIZED);

    step_motor_unlock();
    return status;
}
