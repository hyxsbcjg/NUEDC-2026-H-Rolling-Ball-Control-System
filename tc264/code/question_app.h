#ifndef QUESTION_APP_H_
#define QUESTION_APP_H_

#include "ball_control.h"
#include "gray_app.h"
#include "track_control.h"
#include "zf_common_typedef.h"

/*
 * 上电默认进入题号 0 的 K230 坐标监视页。停车态短按 KEY_1 按
 * 0->1->2->3->4->5->6->0 循环切换；KEY_4 可启动 Q0 调试模式、
 * Q2 或 Q3..Q6。
 */
#define QUESTION_APP_QUESTION_K230_MONITOR          (0U)
#define QUESTION_APP_QUESTION_FIRST                 \
    (QUESTION_APP_QUESTION_K230_MONITOR)
#define QUESTION_APP_QUESTION_DEFAULT               \
    (QUESTION_APP_QUESTION_K230_MONITOR)
#define QUESTION_APP_QUESTION_LAST                  (6U)
#define QUESTION_APP_QUESTION_TRACK_AND_STOP        (2U)
#define QUESTION_APP_QUESTION_BALL_Q3               (3U)
#define QUESTION_APP_QUESTION_BALL_Q4               (4U)
#define QUESTION_APP_QUESTION_BALL_Q5               (5U)
#define QUESTION_APP_QUESTION_BALL_Q6               (6U)
#define QUESTION_APP_QUESTION_BALL_FIRST             \
    (QUESTION_APP_QUESTION_BALL_Q3)
#define QUESTION_APP_QUESTION_BALL_LAST              \
    (QUESTION_APP_QUESTION_BALL_Q6)

/*
 * 各题车体循迹基础 PWM 独立配置。Q2 发车后立即使用 3200，校零后的 Z 轴
 * 角速度从按键发车开始积分；绝对累计航向达到 300 deg 后，只把基础 PWM
 * 切到 800，保留两级 PID 历史。Q4/Q5/Q6 共用稳态目标 1700，并在发车后的
 * 前 350 个成功 6 ms 电机输出周期内从 400 线性提升到 1700。Q3 不启动车体
 * 循迹。
 */
#define QUESTION_APP_Q2_TRACK_PWM_HIGH              (3400)
#define QUESTION_APP_Q2_TRACK_PWM_LOW               (800)
#define QUESTION_APP_Q2_SLOWDOWN_YAW_DEG            (300.0F)
#define QUESTION_APP_Q4_Q5_Q6_TRACK_PWM_BASE        (1700)
#define QUESTION_APP_Q4_Q5_Q6_TRACK_PWM_START       (400)
#define QUESTION_APP_Q4_Q5_Q6_TRACK_RAMP_INNER_CYCLES (350)

/*
 * Q0 自定义调试模式使用单一编译期选择。ACCEL_FF 模式正常执行 3 ms IMU
 * 采样，并按 18 ms 滚球周期只下发加速度前馈；不读取 K230、不运行循迹。
 * GYRO_LOOP 保留零目标角速度环；SOFT_START 用于钢球控制与车体缓启动联调；
 * OFF 恢复 Q0 只监视 K230 的旧行为。
 */
#define QUESTION_APP_Q0_DEBUG_MODE_OFF               (0U)
#define QUESTION_APP_Q0_DEBUG_MODE_GYRO_LOOP         (1U)
#define QUESTION_APP_Q0_DEBUG_MODE_ACCEL_FF          (2U)
#define QUESTION_APP_Q0_DEBUG_MODE_SOFT_START        (3U)
#define QUESTION_APP_Q0_DEBUG_MODE                    \
    (QUESTION_APP_Q0_DEBUG_MODE_SOFT_START)

/*
 * Q0 SOFT_START 时序：
 *   1. KEY_4 发车后，钢球使用 Q4/Q5 参数组保持在 X=396；
 *   2. 只给物理左轮 500 命令，使车体向左偏移，等待 CH2 首次识别到黑线；
 *   3. CH2 为低电平黑线后，左右轮基础 PWM 分别从 500/0 线性提升到
 *      1000/1000；完成这段 2 s 车体缓启动后，循迹基础 PWM 再继续提升到
 *      1700。
 *
 * 源码只能确认电机 A/B，不能证明物理左右轮。默认按 A=左轮；若实车左轮接
 * 电机 B，只把 LEFT_MOTOR_USE_A 改为 0。每段斜坡有 334 个 6 ms 采样点：
 * 第一段从 500/0 到 1000/1000，第二段从 1000/1000 到 1700/1700；
 * 每段均在 333*6=1998 ms 后输出终点值。
 */
#define QUESTION_APP_Q0_SOFT_START_BALL_TARGET_X     (396U)
#define QUESTION_APP_Q0_SOFT_START_LEFT_MOTOR_PWM    (500)
#define QUESTION_APP_Q0_SOFT_START_LEFT_MOTOR_USE_A  (1U)
#define QUESTION_APP_Q0_SOFT_START_TRIGGER_MASK      \
    (GW_GRAY_CHANNEL_2)
#define QUESTION_APP_Q0_SOFT_START_TRACK_PWM_START   (1000)
#define QUESTION_APP_Q0_SOFT_START_TRACK_PWM_TARGET  (1700)
#define QUESTION_APP_Q0_SOFT_START_TRACK_RAMP_MS     (2000U)
#define QUESTION_APP_Q0_SOFT_START_TRACK_RAMP_CYCLES \
    ((QUESTION_APP_Q0_SOFT_START_TRACK_RAMP_MS \
        / TRACK_CONTROL_INNER_PERIOD_MS) + 1U)

/*
 * Q3..Q6 的本地目标像素统一在
 * question_app.c:question_app_ball_parameter_table 中调整。Q3 启动后先到
 * 546，实际位置进入闭区间 [526, 546] 后单向切换到 234 并持续稳球；
 * Q4/Q5 共用中心目标 396。Q6 没有本地固定目标：停车态合法 Touch_X 优先；
 * 未设置 Touch_X 时，KEY_4 发车入口锁存当时最新的有效 Ball_X。两者都无效时
 * 拒绝启动。Q6 使用停车态按 KEY_2/KEY_3 调整好的 RAM 绝对中心；中心与
 * Target 数值无关。发车后目标冻结，后续 Ball_X 仅作反馈，触摸更新只消费。
 */

/*
 * 仅 Q2 使用内部比赛计时、航向积分和停车线判定；Q4/Q5/Q6 不计时、不判完成：
 *   - KEY_4 成功启动 Q2 后，从首个 3 ms 运行拍开始累计时间与 Z 轴航向；
 *   - |yaw| 达到 300 deg 时只把基础 PWM 从 3200 切到 800；
 *   - 第一次停车线只确认为起点，第二次停车线确认当拍计入总时间并停车；
 *   - 进入 FINISHED 或 FAULT 后冻结，Q2 再次启动时重新清零；
 *   - error_num >= 4，即至少 4 路同时识别到黑线；
 *   - digital_value 的 bit3/bit4 必须同时为 0，即 CH4、CH5 都在黑线上；
 *   - 上述两个条件必须连续满足 2 个 3 ms 样本；任一不合格样本重新计数；
 *   - 第一次确认后冷却 1000 ms；
 *   - 第二次确认当拍立即关闭循迹并停车。
 */
#define QUESTION_APP_STOP_LINE_ERROR_NUM_THRESHOLD  (4U)
#define QUESTION_APP_STOP_LINE_REQUIRED_BLACK_MASK  \
    (GW_GRAY_CHANNEL_4 | GW_GRAY_CHANNEL_5)
#define QUESTION_APP_STOP_LINE_CONFIRM_FRAMES        (2U)
#define QUESTION_APP_STOP_LINE_COOLDOWN_MS           (1000U)

#if (QUESTION_APP_QUESTION_FIRST > QUESTION_APP_QUESTION_LAST)
#error "QUESTION_APP question range is invalid"
#endif

#if ((QUESTION_APP_QUESTION_DEFAULT < QUESTION_APP_QUESTION_FIRST) \
    || (QUESTION_APP_QUESTION_DEFAULT > QUESTION_APP_QUESTION_LAST))
#error "QUESTION_APP_QUESTION_DEFAULT is outside the question range"
#endif

#if ((QUESTION_APP_QUESTION_TRACK_AND_STOP < QUESTION_APP_QUESTION_FIRST) \
    || (QUESTION_APP_QUESTION_TRACK_AND_STOP > QUESTION_APP_QUESTION_LAST))
#error "QUESTION_APP_QUESTION_TRACK_AND_STOP is outside the question range"
#endif

#if (2U > QUESTION_APP_Q4_Q5_Q6_TRACK_RAMP_INNER_CYCLES)
#error "Q4/Q5/Q6 track ramp cycles must be at least two"
#endif

#if ((QUESTION_APP_QUESTION_BALL_FIRST > QUESTION_APP_QUESTION_BALL_LAST) \
    || (QUESTION_APP_QUESTION_BALL_FIRST <= QUESTION_APP_QUESTION_TRACK_AND_STOP) \
    || (QUESTION_APP_QUESTION_BALL_LAST > QUESTION_APP_QUESTION_LAST))
#error "QUESTION_APP ball question range is invalid"
#endif

#if ((QUESTION_APP_Q0_DEBUG_MODE != QUESTION_APP_Q0_DEBUG_MODE_OFF) \
    && (QUESTION_APP_Q0_DEBUG_MODE \
        != QUESTION_APP_Q0_DEBUG_MODE_GYRO_LOOP) \
    && (QUESTION_APP_Q0_DEBUG_MODE \
        != QUESTION_APP_Q0_DEBUG_MODE_ACCEL_FF) \
    && (QUESTION_APP_Q0_DEBUG_MODE \
        != QUESTION_APP_Q0_DEBUG_MODE_SOFT_START))
#error "QUESTION_APP_Q0_DEBUG_MODE is invalid"
#endif

#if (QUESTION_APP_Q0_SOFT_START_LEFT_MOTOR_USE_A > 1U)
#error "Q0 soft-start left motor mapping must be 0 or 1"
#endif

#if ((QUESTION_APP_Q0_SOFT_START_LEFT_MOTOR_PWM < MOTOR_COMMAND_MIN) \
    || (QUESTION_APP_Q0_SOFT_START_LEFT_MOTOR_PWM > MOTOR_COMMAND_MAX))
#error "Q0 soft-start left motor PWM is outside the motor range"
#endif

#if ((QUESTION_APP_Q0_SOFT_START_TRACK_PWM_START < MOTOR_COMMAND_MIN) \
    || (QUESTION_APP_Q0_SOFT_START_TRACK_PWM_START > MOTOR_COMMAND_MAX) \
    || (QUESTION_APP_Q0_SOFT_START_TRACK_PWM_TARGET < MOTOR_COMMAND_MIN) \
    || (QUESTION_APP_Q0_SOFT_START_TRACK_PWM_TARGET > MOTOR_COMMAND_MAX))
#error "Q0 soft-start track PWM is outside the motor range"
#endif

#if ((2U > QUESTION_APP_Q0_SOFT_START_TRACK_RAMP_CYCLES) \
    || (65535U < QUESTION_APP_Q0_SOFT_START_TRACK_RAMP_CYCLES))
#error "Q0 soft-start ramp cycles must fit in uint16"
#endif

#if (QUESTION_APP_STOP_LINE_COOLDOWN_MS == 0U)
#error "QUESTION_APP_STOP_LINE_COOLDOWN_MS must be greater than zero"
#endif

#if ((QUESTION_APP_STOP_LINE_CONFIRM_FRAMES == 0U) \
    || (QUESTION_APP_STOP_LINE_CONFIRM_FRAMES > 255U))
#error "QUESTION_APP_STOP_LINE_CONFIRM_FRAMES must fit in uint8"
#endif

typedef enum
{
    QUESTION_APP_STATE_IDLE = 0,
    QUESTION_APP_STATE_RUNNING,
    QUESTION_APP_STATE_FINISHED,
    QUESTION_APP_STATE_FAULT
} question_app_state_enum;

typedef enum
{
    QUESTION_APP_Q3_STATE_INACTIVE = 0,
    QUESTION_APP_Q3_STATE_MOVE_TO_546,
    QUESTION_APP_Q3_STATE_HOLD_244
} question_app_q3_state_enum;

typedef enum
{
    QUESTION_APP_STOP_LINE_WAIT_FIRST = 0,
    QUESTION_APP_STOP_LINE_COOLDOWN,
    QUESTION_APP_STOP_LINE_WAIT_SECOND,
    QUESTION_APP_STOP_LINE_COMPLETE
} question_app_stop_line_state_enum;

typedef enum
{
    QUESTION_APP_STATUS_OK = 0,
    QUESTION_APP_STATUS_NOT_INITIALIZED,
    QUESTION_APP_STATUS_ALREADY_RUNNING,
    QUESTION_APP_STATUS_UNSUPPORTED_QUESTION,
    QUESTION_APP_STATUS_CONTROL_REJECTED,
    QUESTION_APP_STATUS_TARGET_NOT_SET
} question_app_status_enum;

typedef struct
{
    uint8 selected_question;
    question_app_state_enum state;
    question_app_q3_state_enum q3_state;
    question_app_stop_line_state_enum stop_line_state;
    uint8 stop_line_count;
    uint8 stop_line_consecutive_frames;
    uint16 stop_line_cooldown_remaining_ms;
    uint32 q2_elapsed_ms;
    float32 q2_yaw_deg;
    uint8 q2_slowdown_active;
    uint16 ball_target_x;
    uint8 q6_target_valid;
    track_control_status_enum control_status;
    ball_control_status_enum ball_control_status;
} question_app_snapshot_struct;

/* 初始化只清业务状态，不使能循迹或滚球 PID。 */
void question_app_init (void);

/*
 * 停车态题号与发车入口。select_next() 在运行态保持当前题号不变；
 * start_selected() 对 Q0 启动编译期选中的自定义调试模式，对 Q2 准备计时
 * 并使能 track_control，对 Q3 仅使能 ball_control，对 Q4/Q5/Q6 同时使能
 * track_control 与 ball_control。Q0 SOFT_START 使用 Q4/Q5 滚球参数组，并
 * 在 CH2 触发前保持单左轮驱动。Q6 优先采用停车态 Touch_X；未设置时锁存
 * 发车前最新有效 Ball_X，两者均无效才拒绝启动。Q1 不可启动。
 * 每次使能滚球时都显式选择参数集：Q3 独占一套，Q4/Q5 共用一套，Q6 独占一套。
 */
uint8                    question_app_select_next (void);
question_app_status_enum question_app_start_selected (void);

/*
 * CPU0 前台在 k230_app_service() 后调用。每个 Touch_X 更新都会被消费，
 * 但只有当前处于 Q6 的 IDLE 停车态时才锁存为本次目标。其它题号、运行态、
 * 完成态和故障态都不改变目标。返回 ZF_TRUE 表示 Q6 停车态目标已变化。
 */
uint8 question_app_process_k230_touch (void);

/*
 * 仅供 3 ms 调度 ISR 调用。process_imu_irq() 从发车后首拍累计 Q2 时间和
 * Z 轴航向，并在 300 deg 切换基础 PWM；SOFT_START 的 process_gray_irq()
 * 在 CH2 首次有效黑线样本上开启 500/0→1000/1000 的第一段斜坡；
 * process_imu_irq() 约 2 s 后切换到 1000/1000→1700/1700 的第二段斜坡。
 * Q2 的 process_gray_irq() 独立处理两次停车线，第二次确认后计时与控制同时冻结。
 * process_ball_irq() 紧跟 18 ms 滚球步骤，处理 Q3 的 546→244 单向切换；
 * sync_control_irq() 只同步循迹/滚球诊断状态，不再因任一闭环故障退出运行态
 * 或关闭另一闭环。当前唯一自动停车是 Q2 第二次停车线确认。
 */
void question_app_process_gray_irq (
    gw_gray_status_enum gray_status,
    const gray_app_sample_struct *gray_sample,
    uint16 elapsed_ms);
void question_app_process_imu_irq (
    const volatile ImuAppData *imu_data,
    uint16 elapsed_ms);
void question_app_process_ball_irq (void);
void question_app_sync_control_irq (void);

uint8 question_app_is_running (void);
uint8 question_app_is_gyro_debug_running (void);
uint8 question_app_is_acceleration_feedforward_debug_running (void);
/* ISR 内查询：Q4/Q5/Q6 运行且车体缓启动尚未完成时允许滚球前馈。 */
uint8 question_app_is_acceleration_feedforward_active_irq (void);
uint8 question_app_is_track_running (void);
uint8 question_app_is_ball_running (void);
uint8 question_app_get_snapshot (
    question_app_snapshot_struct *snapshot);

#endif
