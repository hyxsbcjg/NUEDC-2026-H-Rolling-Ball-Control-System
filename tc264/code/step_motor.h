#ifndef STEP_MOTOR_H_
#define STEP_MOTOR_H_

#include "stepper_uart.h"

#define STEP_MOTOR_ADDRESS                    (0x01U)
#define STEP_MOTOR_ACCELERATION               (0U)
#define STEP_MOTOR_SYNCHRONIZED                (ZF_FALSE)
/*
 * 当前 X42S 使用 Emm 固件且 S_Vel_IS=Enable，速度字段 500 实际为
 * 50.0 RPM。加速度档 0 表示不启用加减速曲线。
 */
#define STEP_MOTOR_POSITION_SPEED_RPM          (500U)
#define STEP_MOTOR_POSITION_ACCELERATION       (0)
#define STEP_MOTOR_POSITION_MODE               (STEPPER_UART_POSITION_ABSOLUTE)

#if (STEP_MOTOR_POSITION_SPEED_RPM > STEPPER_UART_MAX_SPEED_RPM)
#error "STEP_MOTOR_POSITION_SPEED_RPM exceeds STEPPER_UART_MAX_SPEED_RPM"
#endif

/*
 * 单电机应用接口，固定占用 stepper_uart 所配置的 UART3。本模块是步进电机
 * 的唯一上层所有者，与 K230 UART1 完全独立。所有电机命令必须经本封装
 * 串行化；直接调用 stepper_uart_* 会绕过 command_busy，不再满足互斥契约。
 *
 * Step_Motor_Init() 含毫秒等待和阻塞发送，只能在前台调用。
 *
 * Trigger_Return()、Step_Motor_Set_Speed() 和 Step_Motor_Set_Position()
 * 使用同一非阻塞空 FIFO 机制，并用短临界区防止前台/ISR 状态竞争。它们不等待 10 ms 配置命令
 * 间隔；若本层已有命令，或上一帧仍在 UART3 TX FIFO 中，立即返回 TX_BUSY，
 * 不排队或内部重试。速度和位置接口可由 3 ms 控制 ISR 调用，忙时由后续
 * 控制拍重试最新目标。Trigger_Return() 只供前台触发固定最近点回零；
 * 115200 bit/s 下，5 字节回零帧约需 0.43 ms，13 字节位置帧约需 1.13 ms。ISR
 * 不得调用初始化或任何阻塞式 stepper_uart 接口。
 *
 * Step_Motor_Set_Position() 接收相对驱动器绝对零点的有符号脉冲位置；正值
 * 映射 CW，负值映射 CCW，内部固定使用绝对位置模式、位置速度和加速度宏。
 * 调用方负责按题号选择中位和机械软限位；本封装不钳位脉冲位置。工程只
 * 允许引用驱动器既有绝对零点；Trigger_Return() 只触发回到该既有零点。
 * 工程不提供当前位置清零、设置零点或修改回零配置的接口。
 *
 * PID 接入约束（必须遵守）：signed_pulse_count 是经过单位换算、固定绝对
 * 中心叠加和软限位后的“最终绝对目标脉冲”，不是摄像头距离误差、PID 原始输出或本次
 * 相对增量。位置式 PID 应使用“绝对目标 = 中位脉冲 + PID 输出脉冲”；禁止
 * 再把位置式 PID 输出累加到旧目标，否则等价于额外积分，容易持续跑向限位。
 * 只有明确采用增量式 PID 时，才可把其增量输出累加到保存的绝对目标。
 *
 * 所有 OK 都只确认参数及本地发送动作成功；当前单向配置不接收驱动器 ACK，
 * 因而不能据此判定电机在线、已使能或已达到目标速度/位置。
 */
stepper_uart_status_enum Step_Motor_Init (void);
stepper_uart_status_enum Trigger_Return (void);
stepper_uart_status_enum Step_Motor_Set_Speed (int16 signed_speed);
stepper_uart_status_enum Step_Motor_Set_Position (int32 signed_pulse_count);

#endif
