#ifndef PID_H_
#define PID_H_

#include "zf_common_typedef.h"

typedef enum
{
    PID_STATUS_OK = 0,
    PID_STATUS_INVALID_ARGUMENT,
    PID_STATUS_INVALID_CONFIG,
    PID_STATUS_NOT_INITIALIZED,
    PID_STATUS_NUMERIC_ERROR
} pid_status_enum;

/*
 * 位置式 PID 参数：
 *
 *   error = target - actual
 *   P = kp * error
 *   I = I + ki * error * dt
 *   D = kd * (error - previous_error) / dt
 *
 * integral_min/max 限制的是已经乘过 ki、可直接参与求和的积分输出项，而不是
 * 原始误差累计值。积分与总输出两个区间都必须有序且包含零，所有参数必须有限。
 */
typedef struct
{
    float32 kp;
    float32 ki;
    float32 kd;
    float32 output_min;
    float32 output_max;
    float32 integral_min;
    float32 integral_max;
} pid_config_struct;

typedef struct
{
    /* 当前及上一次成功步的误差，供状态观测；D 项使用这两个采样间的差分。 */
    float32 error;
    float32 previous_error;
    /* 已乘 ki 的积分输出、未乘 kd 的误差变化率，以及最终限幅输出。 */
    float32 integral_output;
    float32 derivative;
    float32 output;
    uint8 history_valid;
} pid_state_struct;

/*
 * 每个控制环独占一个控制器实例；模块内部不加锁。严格固定周期控制由 3 ms
 * 调度 ISR 的固定分频分支在采样完成后调用 pid_step()；同一实例不得再由
 * 前台并发访问，并且 dt_s 必须传入该分支的真实周期。
 * 初始化成功后把 config 视为只读，改参数时应先停用控制 ISR，再重新调用
 * pid_init()，以免新参数与旧积分/微分历史混用。
 *
 * 若实例定义为 CPU0 的全局或静态对象，应在定义它的源文件中放入 cpu0_dsram。
 */
typedef struct
{
    pid_config_struct config;
    pid_state_struct state;
    uint8 initialized;
} pid_controller_struct;

/*
 * 仅当全部参数为有限值、两个限幅区间有序且都包含零时返回 ZF_TRUE。
 */
uint8 pid_config_is_valid (const pid_config_struct *config);

/*
 * 复制配置并清空全部运行状态。若配置无效，目标实例仍会被清空并保持未初始化，
 * 不会保留旧控制参数。
 */
pid_status_enum pid_init (
    pid_controller_struct *pid,
    const pid_config_struct *config);

uint8 pid_is_ready (const pid_controller_struct *pid);

/*
 * 保留当前配置，只清除误差历史、积分项、微分项和输出；下一步会重新按首样本
 * 处理，因此不会产生基于复位前误差的微分突跳。
 */
pid_status_enum pid_reset (pid_controller_struct *pid);

/*
 * 执行一步位置式 PID。
 *
 * 误差固定为 target - actual；dt_s 是距上一次成功步骤的实际秒数。首次成功
 * 步没有历史样本，微分项为零。抗饱和采用条件积分：只有候选积分把已饱和输出
 * 继续推向饱和方向时才冻结或截断，反向积分可继续释放饱和。
 *
 * 成功时 output 获得总输出限幅后的值；失败时控制器状态和调用者 output 均不变。
 */
pid_status_enum pid_step (
    pid_controller_struct *pid,
    float32 target,
    float32 actual,
    float32 dt_s,
    float32 *output);

#endif
