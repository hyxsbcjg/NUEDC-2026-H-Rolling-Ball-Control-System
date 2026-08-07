#ifndef TRACK_CONTROL_H_
#define TRACK_CONTROL_H_

#include "gray_app.h"
#include "imu_app.h"
#include "motor.h"
#include "pid.h"
#include "zf_common_typedef.h"

/*
 * 3 ms PIT 仅作为调度时基：
 *   - Q0 零目标角速度调试 PD：6 ms
 *   - Q2/Q4/Q5/Q6 角速度内环 PD：6 ms
 *   - Q2/Q4/Q5/Q6 灰度循迹外环 PD：12 ms
 *
 * 外环 PD 输出目标角速度，单位为校零后的陀螺仪原始 LSB；
 * 内环 PD 输出带符号的差速电机修正量 pwm_curr。
 */
#define TRACK_CONTROL_INNER_PERIOD_MS             (6U)
#define TRACK_CONTROL_OUTER_PERIOD_MS             (12U)

/*
 * 12 ms 外环 PD 输出 target_gyro_lsb 的绝对值上限。
 * 单位为校零后的陀螺仪原始 LSB，必须大于 0。
 */
#define TRACK_CONTROL_TARGET_GYRO_LIMIT_LSB        (4096.0F)

/*
 * 6 ms 内环 PD 输出 pwm_curr 的绝对值上限，单位为电机命令。
 * 必须大于 0，并且普通循迹使能时要求：
 * |pwm_base| + TRACK_CONTROL_PWM_CURR_LIMIT <= MOTOR_COMMAND_MAX。
 */
#define TRACK_CONTROL_PWM_CURR_LIMIT               (2000.0F)

/*
 * Q0 零目标角速度调试不叠加普通循迹基础命令，只把角速度 PD 输出按
 * A=-pwm_curr、B=+pwm_curr 直接施加到两路主驱。
 */
#define TRACK_CONTROL_GYRO_DEBUG_PWM_BASE            (0)

/*
 * PD 增益和陀螺仪轴统一在 track_control.c 的参数表中调整；普通循迹
 * 基础 PWM 由 question_app 按题号传入，Q0 调试基础 PWM 固定为 0。
 * 反馈与电机混控公式的符号固定；某一级方向错误时，同时修改该级 kp、kd 的符号。
 * 仅初始化不会使车辆运动。
 */

typedef enum
{
    TRACK_CONTROL_STATUS_OK = 0,
    TRACK_CONTROL_STATUS_NOT_INITIALIZED,
    TRACK_CONTROL_STATUS_INVALID_CONFIG,
    TRACK_CONTROL_STATUS_DISABLED,
    TRACK_CONTROL_STATUS_WAITING_FOR_LINE,
    TRACK_CONTROL_STATUS_LINE_INVALID,
    TRACK_CONTROL_STATUS_IMU_INVALID,
    TRACK_CONTROL_STATUS_PID_ERROR,
    TRACK_CONTROL_STATUS_MOTOR_ERROR
} track_control_status_enum;

typedef struct
{
    track_control_status_enum status;
    uint8 enabled;
    uint8 gyro_debug_enabled;
    uint8 line_valid;

    /* line_position 已由 gray_app 统一低通；line_error = target(0)-position。 */
    float32 line_position;
    float32 line_error;
    float32 target_gyro_lsb;
    float32 actual_gyro_lsb;
    float32 gyro_error;
    float32 pwm_curr;
    /*
     * 当前等效基础 PWM、对称显示起点/目标及进度，周期单位为有效 6 ms
     * 内环次数。左右轮独立斜坡的精确起点/目标见下列 motor_* 字段。
     */
    int32 pwm_base;
    int32 pwm_base_ramp_start;
    int32 pwm_base_target;
    uint16 pwm_base_ramp_cycles;
    uint16 pwm_base_ramp_completed_cycles;
    int32 motor_a_ramp_start;
    int32 motor_b_ramp_start;
    int32 motor_a_ramp_target;
    int32 motor_b_ramp_target;

    int16 motor_a_request;
    int16 motor_b_request;
    int16 motor_a_applied;
    int16 motor_b_applied;

    uint32 outer_update_count;
    uint32 inner_update_count;
} track_control_snapshot_struct;

/*
 * 初始化两个历史状态互相独立的位置式 PID 实例，两者的 ki 均固定为 0。
 * 初始化不会使能闭环，也不会驱动车辆。
 */
track_control_status_enum track_control_init (void);

/*
 * 后续启动状态机必须显式调用 enable，并传入本题基础 PWM。普通 enable
 * 立即采用目标基础 PWM；enable_ramp 显式传入起点和目标，第一个有效 6 ms
 * 内环输出精确采用起点，最后一个软启动周期精确达到目标。只有内环 PID 和
 * 两路电机输出都成功后才推进一个周期。三种入口都会校验整个基础 PWM 区间
 * 加差速限幅后不超过电机命令范围，并清除两级 PD 历史。发车后即按基础
 * PWM 运行；尚无有效灰度时目标角速度保持 0，取得有效灰度后再进入外环。
 * 灰度/IMU/PID/电机接口运行异常只记录诊断状态，不关闭控制或写零命令。
 * 只有显式 disable（当前用于 Q2 第二次停车线）才立即将两路电机命令置 0。
 */
track_control_status_enum track_control_enable (int32 pwm_base);
track_control_status_enum track_control_enable_ramp (
    int32 pwm_base_start,
    int32 pwm_base_target,
    uint16 ramp_inner_cycles);
/*
 * 左右轮使用独立起点和目标的软启动入口。外环/内环产生的差速修正仍会
 * 叠加在两路各自的基础 PWM 上；当四个参数相同时，行为等价于
 * track_control_enable_ramp()。
 */
track_control_status_enum track_control_enable_motor_ramp (
    int32 motor_a_start,
    int32 motor_b_start,
    int32 motor_a_target,
    int32 motor_b_target,
    uint16 ramp_inner_cycles);
/*
 * 仅供运行态 ISR 调用：替换左右轮软启动参数但保留当前两级 PID 历史。
 * 新参数从下一次有效 6 ms 内环输出开始生效。
 */
track_control_status_enum track_control_set_motor_ramp_irq (
    int32 motor_a_start,
    int32 motor_b_start,
    int32 motor_a_target,
    int32 motor_b_target,
    uint16 ramp_inner_cycles);
/*
 * 仅供运行态 ISR 调用：立即替换普通循迹基础 PWM，但保留灰度外环、角速度
 * 内环的 PID 历史。该接口不会立即写电机，新的基础 PWM 从下一次有效 6 ms
 * 内环输出开始生效。
 */
track_control_status_enum track_control_set_pwm_base_irq (int32 pwm_base);
track_control_status_enum track_control_enable_gyro_debug (void);
track_control_status_enum track_control_disable (void);
uint8                    track_control_is_enabled (void);
/* ISR 内查询：仅在斜坡尚有成功内环周期未完成时返回 ZF_TRUE。 */
uint8                    track_control_is_ramp_active_irq (void);
uint8                    track_control_is_gyro_debug_enabled (void);
track_control_status_enum track_control_get_status (void);

/*
 * 仅供 ISR 调用：3 ms 调度器每拍先调用 process_gray_sample。灰度状态异常、
 * 空样本或 error_num==0 均只记录/保持诊断状态和最后一次有效外环目标，不会
 * 停车。随后每 12 ms 执行 outer_step，每 6 ms 执行 inner_step；共同到期时
 * 外环在前。IMU/PID/电机接口异常同样只采用当前基础值或上次有效修正继续重试。
 */
void track_control_process_gray_sample_irq (
    gw_gray_status_enum gray_status,
    const gray_app_sample_struct *gray_sample);
void track_control_outer_step_irq (
    gw_gray_status_enum gray_status,
    const gray_app_sample_struct *gray_sample);
void track_control_inner_step_irq (
    const volatile ImuAppData *imu_data);

/*
 * 仅供 Q0 调试 ISR 分支调用：目标角速度和基础 PWM 固定为 0，不要求灰度
 * 外环先发布 line_valid，也不会执行灰度 PID。
 */
void track_control_gyro_debug_step_irq (
    const volatile ImuAppData *imu_data);

/* 前台诊断快照复制；本函数不会访问任何传感器硬件。 */
uint8 track_control_get_snapshot (
    track_control_snapshot_struct *snapshot);

#endif
