#ifndef BALL_CONTROL_H_
#define BALL_CONTROL_H_

#include "zf_common_typedef.h"

/*
 * 钢球位置 PI 外环与速度位置式 PI 内环的串级控制契约。
 *
 * 3 ms 系统时基每 18 ms 调用一次 ball_control_step_irq()，K230 成帧仍在
 * 前台执行。只有 position_update_count 标识新坐标时，才更新速度估计并推进
 * 速度 PI；重复快照保持目标速度、两级积分状态和电机请求。位置外环每消费
 * 三个新坐标更新一次，名义周期为 54 ms。位置积分仅在参数表指定的小误差
 * 窗口内累计，窗口外冻结，并受目标速度单位的积分上下限约束。
 * Q3、Q4/Q5 和 Q6 都在最终绝对脉冲形成前使用实测的非对称起滚补偿：
 * 位置误差超出保持窗口且钢球近似静止时，直接按位置误差方向把控制量
 * 提升到最小起滚脉冲；不要求速度 PI 先给出同方向输出，运动中的制动命令
 * 不被覆盖。近似静止定义为 |filtered_velocity| <= 30 pixel/s。该补偿
 * 不写入任一级积分状态。
 *
 * 使能后的第一个新坐标建立速度历史并更新位置外环，但不积分；第二个新坐标
 * 产生第一份实测速度和首个速度 PI 输出。位置/速度增益、滤波权重和限幅位于
 * ball_control.c 中按题号拆分的三张参数表。所有位置均直接引用驱动器既有
 * 机械零点。Q6 允许在停车态用按键调整 RAM 中的绝对中心脉冲，发车后只按
 * “运行中心 + 控制器输出”形成最终绝对位置；该运行中心不是机械零点，不会
 * 写入驱动器零点或回零配置。绝对软限位由下列宏给出。所有数值仍需实机标定。
 */
#define BALL_CONTROL_CAMERA_FRAME_RATE_HZ       (56U)
#define BALL_CONTROL_INNER_PERIOD_MS             (18U)
#define BALL_CONTROL_OUTER_PERIOD_MS             (54U)
#define BALL_CONTROL_PERIOD_MS                   BALL_CONTROL_INNER_PERIOD_MS
#define BALL_CONTROL_OUTER_DIVIDER               \
    (BALL_CONTROL_OUTER_PERIOD_MS / BALL_CONTROL_INNER_PERIOD_MS)
#define BALL_CONTROL_CENTER_PULSE              (0)
#define BALL_CONTROL_MIN_PULSE                 (-400)
#define BALL_CONTROL_MAX_PULSE                 (400)

/* Q4/Q5 与 Q6 共用的实测起滚补偿；正脉冲使 Ball_X 减小。 */
#define BALL_CONTROL_Q4_Q5_BREAKAWAY_POSITIVE_MIN_PULSE (75.0F)
#define BALL_CONTROL_Q4_Q5_BREAKAWAY_NEGATIVE_MAX_PULSE (-75.0F)

/* Q6 停车态每次按键对 RAM 运行中心的调整量。 */
#define BALL_CONTROL_Q6_CENTER_ADJUST_STEP_PULSE (20)

/*
 * 可选加速度前馈参数直接写在 ball_control.c 的题号参数表中。Q3 前馈输出
 * 限幅为 0；Q4/Q5 与 Q6 前馈输出限幅为 100，并启用各自的实测起滚补偿。
 * Q0 ACCEL_FF 调试复用 Q4/Q5 表，因此可以输出非零前馈。
 */

#if (BALL_CONTROL_PERIOD_MS == 0U)
#error "BALL_CONTROL_PERIOD_MS must be greater than zero"
#endif

#if (BALL_CONTROL_CAMERA_FRAME_RATE_HZ == 0U)
#error "BALL_CONTROL_CAMERA_FRAME_RATE_HZ must be greater than zero"
#endif

#if ((BALL_CONTROL_OUTER_PERIOD_MS % BALL_CONTROL_INNER_PERIOD_MS) != 0U)
#error "BALL_CONTROL_OUTER_PERIOD_MS must be a multiple of inner period"
#endif

#if ((BALL_CONTROL_OUTER_DIVIDER == 0U) \
    || (BALL_CONTROL_OUTER_DIVIDER > 255U))
#error "BALL_CONTROL_OUTER_DIVIDER must fit in uint8"
#endif

#if (BALL_CONTROL_MIN_PULSE > BALL_CONTROL_CENTER_PULSE)
#error "BALL_CONTROL_MIN_PULSE must not exceed BALL_CONTROL_CENTER_PULSE"
#endif

#if (BALL_CONTROL_CENTER_PULSE > BALL_CONTROL_MAX_PULSE)
#error "BALL_CONTROL_CENTER_PULSE must not exceed BALL_CONTROL_MAX_PULSE"
#endif

#if (BALL_CONTROL_Q6_CENTER_ADJUST_STEP_PULSE <= 0)
#error "Q6 center adjustment step must be greater than zero"
#endif

typedef enum
{
    BALL_CONTROL_PARAMETER_SET_Q3 = 0,
    BALL_CONTROL_PARAMETER_SET_Q4_Q5,
    BALL_CONTROL_PARAMETER_SET_Q6,
    BALL_CONTROL_PARAMETER_SET_COUNT
} ball_control_parameter_set_enum;


typedef enum
{
    BALL_CONTROL_STATUS_OK = 0,
    BALL_CONTROL_STATUS_NOT_INITIALIZED,
    BALL_CONTROL_STATUS_INVALID_CONFIG,
    BALL_CONTROL_STATUS_DISABLED,
    BALL_CONTROL_STATUS_WAITING_FOR_POSITION,
    BALL_CONTROL_STATUS_RUNNING,
    BALL_CONTROL_STATUS_PID_ERROR,
    BALL_CONTROL_STATUS_MOTOR_ERROR
} ball_control_status_enum;

/*
 * ADS 1.6 内置 CDT 7.3。其语义索引器可能保留 typedef，却丢失本头文件中的
 * 枚举项，从而在 ball_control.c 中产生错误的“符号未解析”标记。真实编译器
 * 不定义 __CDT_PARSER__，所以下列带类型常量只影响 CDT 代码分析。
 */
#ifdef __CDT_PARSER__
#define BALL_CONTROL_PARAMETER_SET_Q3           \
    ((ball_control_parameter_set_enum)0)
#define BALL_CONTROL_PARAMETER_SET_Q4_Q5        \
    ((ball_control_parameter_set_enum)1)
#define BALL_CONTROL_PARAMETER_SET_Q6           \
    ((ball_control_parameter_set_enum)2)
#define BALL_CONTROL_PARAMETER_SET_COUNT        \
    ((ball_control_parameter_set_enum)3)

#define BALL_CONTROL_STATUS_OK                  ((ball_control_status_enum)0)
#define BALL_CONTROL_STATUS_NOT_INITIALIZED     ((ball_control_status_enum)1)
#define BALL_CONTROL_STATUS_INVALID_CONFIG      ((ball_control_status_enum)2)
#define BALL_CONTROL_STATUS_DISABLED            ((ball_control_status_enum)3)
#define BALL_CONTROL_STATUS_WAITING_FOR_POSITION \
    ((ball_control_status_enum)4)
#define BALL_CONTROL_STATUS_RUNNING             ((ball_control_status_enum)5)
#define BALL_CONTROL_STATUS_PID_ERROR           ((ball_control_status_enum)6)
#define BALL_CONTROL_STATUS_MOTOR_ERROR         ((ball_control_status_enum)7)
#endif

typedef struct
{
    ball_control_status_enum status;
    uint8 initialized;
    uint8 enabled;
    uint8 position_valid;
    uint16 target_x;
    uint16 actual_x;
    float32 error;
    float32 target_velocity_pixel_per_s;
    float32 position_integral_velocity_pixel_per_s;
    float32 velocity_error_pixel_per_s;
    float32 integral_output_pulse;
    float32 pid_output_pulse;
    float32 filtered_velocity_pixel_per_s;
    /* 保留原字段名；当前记录速度 PI 未限幅的比例项。 */
    float32 velocity_output_pulse;
    float32 controller_output_pulse;
    int32 requested_absolute_pulse;
    int32 applied_absolute_pulse;
    uint32 position_update_count;
    uint32 control_update_count;
    uint32 outer_update_count;
    uint8 motor_status;
} ball_control_snapshot_struct;

/*
 * 仅限前台调用的初始化。通过 Step_Motor_Init() 初始化并使能步进驱动器。
 * 工程不提供写零或回零命令，初始化不会建立或修改机械零点。
 */
ball_control_status_enum ball_control_init (void);

/*
 * 题目控制入口。enable() 使能后，必须先收到一帧晚于使能时快照的 K230 坐标
 * 才建立速度历史；再收到第二个新坐标后，才执行第一次速度 PI。
 *
 * enable() 必须显式指定题号参数集：Q3、Q4/Q5 和 Q6 互不串用增益、滤波
 * 与反馈限幅。Q4/Q5 共用一套参数，Q3 和 Q6 各自独立；Q3 关闭前馈，
 * Q4/Q5 与 Q6 配置前馈，但正常题目运行只在对应车体缓启动阶段允许前馈；
 * 三题继续使用非对称起滚补偿。
 *
 * set_target() 修改已使能控制器的目标，但不复位任一级 PI、速度历史或待发送
 * 电机请求。若实际位置已经有效，它会以零积分时间立即刷新位置外环比例输出，
 * 并重新开始 54 ms 外环分频。该函数不发送、不等待，执行时间有界，可在
 * ISR 中调用。
 */
ball_control_status_enum ball_control_enable (
    uint16 target_x,
    ball_control_parameter_set_enum parameter_set);
ball_control_status_enum
    ball_control_enable_acceleration_feedforward_debug (void);
ball_control_status_enum ball_control_set_target (uint16 target_x);
ball_control_status_enum ball_control_disable (void);

/*
 * Q6 停车态运行中心接口。中心以驱动器既有机械零点为基准，保存在 RAM 中；
 * setter 只校验并保存数值，不发送电机命令，也不写机械零点或回零配置。
 * Q6 enable() 装载该中心，并继续复用 Q4/Q5 的 +80/-80 pulse 起滚补偿。
 * 控制使能时 setter 返回 RUNNING，越界或无法保留起滚余量时返回 INVALID_CONFIG。
 */
ball_control_status_enum ball_control_get_q6_runtime_center_pulse (
    int32 *center_pulse);
ball_control_status_enum ball_control_set_q6_runtime_center_pulse (
    int32 center_pulse);

uint8                   ball_control_is_enabled (void);
uint8 ball_control_is_acceleration_feedforward_debug_enabled (void);
ball_control_status_enum ball_control_get_status (void);

/*
 * 仅供 ISR 调用的有界控制步骤。调度器每 18 ms 调用一次；新坐标更新实测
 * 速度并执行一次位置式速度 PI，重复快照不推进任何积分。每三个已消费的新
 * 坐标更新一次位置 PI 外环；仅在小误差窗口内按实际外环间隔累计位置积分。
 * 速度观察 dt 取实际控制间隔与 56 Hz 帧计数间隔中的较大值，速度 PI dt
 * 使用两次实际内环执行间隔。只有当前参数表配置前馈且调用方允许时，18 ms
 * 控制槽才消费最新的已去零漂 Y 轴 raw LSB 并独立计算前馈。Q3、Q4/Q5 和 Q6
 * 随后都可在近静止、误差超出保持窗口且 PI 方向正确时施加起滚补偿。没有
 * 新坐标时保持使能、两级 PI 状态和最新绝对位置请求，不计时回中或自动关闭；
 * 后续新坐标到达后按实际累计间隔继续闭环。
 */
void ball_control_step_irq (uint8 acceleration_feedforward_allowed);

/*
 * 每 3 ms 由采样层调用一次。传入 imu_app 已去零漂的 Y 轴 raw LSB；函数
 * 只发布最新输入，不访问传感器、不计算输出、不发送串口。固定 18 ms 控制槽
 * 才推进前馈输出。Q3 前馈输出限幅为 0；Q4/Q5 与 Q6 的限幅为 100，但
 * 正常题目运行只在对应车体缓启动尚未完成时由调度器允许前馈。
 */
void ball_control_update_acceleration_y_irq (
    float32 acceleration_y_lsb);

uint8 ball_control_get_snapshot (ball_control_snapshot_struct *snapshot);

#endif
