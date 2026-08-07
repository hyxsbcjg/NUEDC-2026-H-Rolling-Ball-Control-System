#include "task.h"

#include "ball_control.h"
#include "gray_app.h"
#include "imu_app.h"
#include "question_app.h"
#include "track_control.h"
#include "zf_common_interrupt.h"

/*
 * 下列状态由 CCU60_CH0 ISR 写、CPU0 前台读，因此既要 volatile，也要固定
 * 放在 CPU0 DSRAM。pending 是“至少发生过一次”的锁存位，不是事件计数器；
 * 前台阻塞时，同类事件会合并而不会排队累积。
 */
#pragma section all "cpu0_dsram"
static volatile uint32 my_task_tick_count = 0U;
static volatile gray_app_sample_struct my_task_gray_sample =
{
    GW_GRAY_CHANNEL_ALL,
    0.0F,
    0U
};
static volatile gw_gray_status_enum my_task_gray_sample_status =
    GW_GRAY_STATUS_NOT_READY;
static volatile uint8 my_task_inner_control_divider =
    (uint8)(MY_TASK_INNER_CONTROL_DIVIDER - 1U);
static volatile uint8 my_task_outer_control_divider =
    (uint8)(MY_TASK_OUTER_CONTROL_DIVIDER - 1U);
static volatile uint8 my_task_ball_control_divider =
    (uint8)(MY_TASK_BALL_CONTROL_DIVIDER - 1U);
static volatile uint8 my_task_acceleration_ff_debug_active = ZF_FALSE;
static volatile uint8 my_task_key_scan_pending = ZF_FALSE;
static volatile uint8 my_task_battery_sample_divider = 0U;
static volatile uint8 my_task_battery_sample_pending = ZF_FALSE;
static volatile uint8 my_task_refresh_divider = 0U;
static volatile uint8 my_task_refresh_pending = ZF_FALSE;
#pragma section all restore

static void my_task_reset_control_schedule_irq (void)
{
    my_task_inner_control_divider =
        (uint8)(MY_TASK_INNER_CONTROL_DIVIDER - 1U);
    my_task_outer_control_divider =
        (uint8)(MY_TASK_OUTER_CONTROL_DIVIDER - 1U);
}

static void my_task_ball_control_update_irq (void)
{
    my_task_ball_control_divider++;

    if (MY_TASK_BALL_CONTROL_DIVIDER
        <= my_task_ball_control_divider)
    {
        my_task_ball_control_divider = 0U;
        ball_control_step_irq(
            question_app_is_acceleration_feedforward_active_irq());
        question_app_process_ball_irq();
    }
}

static void my_task_gyro_debug_update_irq (
    const volatile ImuAppData *imu_data)
{
    /*
     * Q0 调试沿用 6 ms 角速度周期，但固定跳过 12 ms 灰度外环。每次进入
     * Q0 发车态后的第一个 3 ms 拍立即执行一次，随后每 2 拍执行一次。
     */
    my_task_outer_control_divider =
        (uint8)(MY_TASK_OUTER_CONTROL_DIVIDER - 1U);
    my_task_inner_control_divider++;

    if (MY_TASK_INNER_CONTROL_DIVIDER
        <= my_task_inner_control_divider)
    {
        my_task_inner_control_divider = 0U;
        track_control_gyro_debug_step_irq(imu_data);
    }
}

static void my_task_control_update_irq (
    gw_gray_status_enum gray_status,
    const gray_app_sample_struct *gray_sample,
    const volatile ImuAppData *imu_data)
{
    uint8 inner_due = ZF_FALSE;
    uint8 outer_due = ZF_FALSE;

    /*
     * 本函数不访问传感器。每个 3 ms 拍先检查本拍灰度快照；无效快照或无
     * 黑线只保留最后一次有效外环目标，不触发停车。随后执行 6/12 ms 控制
     * 分频；共同到期时先执行外环，再执行内环。
     */
    track_control_process_gray_sample_irq(gray_status, gray_sample);
    if (ZF_FALSE == track_control_is_enabled())
    {
        return;
    }

    my_task_inner_control_divider++;
    my_task_outer_control_divider++;

    if (MY_TASK_INNER_CONTROL_DIVIDER
        <= my_task_inner_control_divider)
    {
        my_task_inner_control_divider = 0U;
        inner_due = ZF_TRUE;
    }

    if (MY_TASK_OUTER_CONTROL_DIVIDER
        <= my_task_outer_control_divider)
    {
        my_task_outer_control_divider = 0U;
        outer_due = ZF_TRUE;
    }

    if (ZF_TRUE == outer_due)
    {
        track_control_outer_step_irq(gray_status, gray_sample);
    }

    if (ZF_TRUE == inner_due)
    {
        track_control_inner_step_irq(imu_data);
    }
}

void my_task_init (void)
{
    my_task_tick_count = 0U;
    my_task_gray_sample.digital_value = GW_GRAY_CHANNEL_ALL;
    my_task_gray_sample.error = 0.0F;
    my_task_gray_sample.error_num = 0U;
    my_task_gray_sample_status = GW_GRAY_STATUS_NOT_READY;
    my_task_reset_control_schedule_irq();
    my_task_ball_control_divider =
        (uint8)(MY_TASK_BALL_CONTROL_DIVIDER - 1U);
    my_task_acceleration_ff_debug_active = ZF_FALSE;
    my_task_key_scan_pending = ZF_FALSE;
    my_task_battery_sample_divider = 0U;
    my_task_battery_sample_pending = ZF_FALSE;
    my_task_refresh_divider = 0U;
    my_task_refresh_pending = ZF_FALSE;

    (void)track_control_init();
    pit_ms_init(MY_TASK_PIT_INDEX, MY_TASK_PERIOD_MS);
}

void my_task_service_irq (void)
{
    gray_app_sample_struct gray_sample;
    gw_gray_status_enum gray_status;
    const volatile ImuAppData *imu_data;

    my_task_tick_count++;

    /*
     * 采样层固定每 3 ms 执行一次，不受 6/12 ms 控制分频、题号或控制使能
     * 状态影响。控制层只消费本拍采样结果，不再发起任何传感器事务。
    */
    imu_app_update();
    imu_data = imu_app_get_data();

    if (ZF_TRUE == imu_data->acceleration_valid)
    {
        ball_control_update_acceleration_y_irq(
            imu_data->corrected_acc_y_lsb);
    }

    gray_status = gray_app_read(&gray_sample);

    /*
     * 失败时同样发布快照：gray_app_read() 已将输出清成安全默认值，
     * 消费者必须先检查状态，不能把默认值当作有效测量。
     */
    my_task_gray_sample = gray_sample;
    my_task_gray_sample_status = gray_status;

    /*
     * Q0 调试在采样后按模式分流：GYRO_LOOP 只运行 6 ms 零目标角速度环；
     * ACCEL_FF 只按固定 18 ms 滚球槽推进加速度前馈。SOFT_START 走普通分支，
     * 每拍检查 CH2，滚球始终按 18 ms 推进，CH2 触发后再开放循迹分频。
     * Q2 从按键发车后的首个 3 ms 运行拍累计时间和 Z 轴航向，停车线检测仍
     * 独立确认两次；Q3 在滚球步骤后推进 546→244 单向状态机，Q4/Q5/Q6
     * 则在同一运行态继续推进滚球和循迹两个分频。
     */
    if (ZF_TRUE == question_app_is_gyro_debug_running())
    {
        my_task_acceleration_ff_debug_active = ZF_FALSE;
        my_task_gyro_debug_update_irq(imu_data);
    }
    else if (ZF_TRUE
        == question_app_is_acceleration_feedforward_debug_running())
    {
        if (ZF_FALSE == my_task_acceleration_ff_debug_active)
        {
            my_task_acceleration_ff_debug_active = ZF_TRUE;
            my_task_ball_control_divider =
                (uint8)(MY_TASK_BALL_CONTROL_DIVIDER - 1U);
        }
        my_task_reset_control_schedule_irq();
        my_task_ball_control_update_irq();
    }
    else
    {
        my_task_acceleration_ff_debug_active = ZF_FALSE;
        question_app_process_imu_irq(
            imu_data,
            MY_TASK_PERIOD_MS);

        question_app_process_gray_irq(
            gray_status,
            &gray_sample,
            MY_TASK_PERIOD_MS);

        my_task_ball_control_update_irq();

        if (ZF_TRUE == question_app_is_track_running())
        {
            my_task_control_update_irq(
                gray_status,
                &gray_sample,
                imu_data);
        }
        else
        {
            my_task_reset_control_schedule_irq();
        }
    }

    question_app_sync_control_irq();

    /* 电池是电机补偿输入，题目态仍在前台采样。 */
    my_task_battery_sample_divider++;

    if (MY_TASK_BATTERY_SAMPLE_DIVIDER
        <= my_task_battery_sample_divider)
    {
        my_task_battery_sample_divider = 0U;
        my_task_battery_sample_pending = ZF_TRUE;
    }

    /*
     * 题目态不发布按键和屏幕事件，并清掉进入题目态前可能遗留的请求。
     * Q2 自动停车或显式复位退出题目态后，从当前拍重新开始 150 ms
     * 刷新分频；Q2 的冻结总时间由前台下一次刷新以两位小数显示。Q4/Q5
     * 不在这里执行比赛计时或正常完赛停车。
     */
    if (ZF_TRUE == question_app_is_running())
    {
        my_task_key_scan_pending = ZF_FALSE;
        my_task_refresh_divider = 0U;
        my_task_refresh_pending = ZF_FALSE;
    }
    else
    {
        my_task_key_scan_pending = ZF_TRUE;
        my_task_refresh_divider++;

        if (MY_TASK_REFRESH_DIVIDER <= my_task_refresh_divider)
        {
            my_task_refresh_divider = 0U;
            my_task_refresh_pending = ZF_TRUE;
        }
    }
}

uint32 my_task_get_tick_count (void)
{
    uint32 interrupt_state;
    uint32 tick_count;

    /* 保存并恢复原中断状态，避免读取 32 位共享快照时观察到中间状态。 */
    interrupt_state = interrupt_global_disable();
    tick_count = my_task_tick_count;
    interrupt_global_enable(interrupt_state);

    return tick_count;
}

gw_gray_status_enum my_task_get_gray_sample (
    gray_app_sample_struct *sample)
{
    uint32 interrupt_state;
    gw_gray_status_enum status;

    if (0 == sample)
    {
        return GW_GRAY_STATUS_INVALID_ARGUMENT;
    }

    /* 结构体与对应状态必须在同一个临界区复制，二者才属于同一次外环更新。 */
    interrupt_state = interrupt_global_disable();
    *sample = my_task_gray_sample;
    status = my_task_gray_sample_status;
    interrupt_global_enable(interrupt_state);

    return status;
}

uint8 my_task_take_key_scan_event (void)
{
    uint32 interrupt_state;
    uint8 scan_pending;

    /*
     * check-and-clear 必须原子完成；否则 ISR 可能在读取与清零之间置位，造成
     * 一次事件永久丢失。其余 take_* 函数使用相同的单槽合并语义。
     */
    interrupt_state = interrupt_global_disable();
    scan_pending = my_task_key_scan_pending;
    my_task_key_scan_pending = ZF_FALSE;
    interrupt_global_enable(interrupt_state);

    return scan_pending;
}

uint8 my_task_take_battery_sample_event (void)
{
    uint32 interrupt_state;
    uint8 sample_pending;

    interrupt_state = interrupt_global_disable();
    sample_pending = my_task_battery_sample_pending;
    my_task_battery_sample_pending = ZF_FALSE;
    interrupt_global_enable(interrupt_state);

    return sample_pending;
}

uint8 my_task_take_refresh_event (void)
{
    uint32 interrupt_state;
    uint8 refresh_pending;

    interrupt_state = interrupt_global_disable();
    refresh_pending = my_task_refresh_pending;
    my_task_refresh_pending = ZF_FALSE;
    interrupt_global_enable(interrupt_state);

    return refresh_pending;
}
