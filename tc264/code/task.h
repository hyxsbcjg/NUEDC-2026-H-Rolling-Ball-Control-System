#ifndef TASK_H_
#define TASK_H_

#include "ball_control.h"
#include "gray_app.h"
#include "track_control.h"
#include "zf_driver_pit.h"

/* 使用 CCU60_CH0 的固定 3 ms CPU0 系统节拍。 */
#define MY_TASK_PERIOD_MS                      (3U)
#define MY_TASK_PIT_INDEX                      (CCU60_CH0)
#define MY_TASK_INNER_CONTROL_DIVIDER          \
    (TRACK_CONTROL_INNER_PERIOD_MS / MY_TASK_PERIOD_MS)
#define MY_TASK_OUTER_CONTROL_DIVIDER          \
    (TRACK_CONTROL_OUTER_PERIOD_MS / MY_TASK_PERIOD_MS)
#define MY_TASK_BALL_CONTROL_DIVIDER           \
    (BALL_CONTROL_PERIOD_MS / MY_TASK_PERIOD_MS)
#define MY_TASK_BATTERY_SAMPLE_DIVIDER         (3U)
#define MY_TASK_BATTERY_SAMPLE_PERIOD_MS       \
    (MY_TASK_PERIOD_MS * MY_TASK_BATTERY_SAMPLE_DIVIDER)
#define MY_TASK_REFRESH_DIVIDER                (50U)
#define MY_TASK_REFRESH_PERIOD_MS              (MY_TASK_PERIOD_MS * MY_TASK_REFRESH_DIVIDER)

#if ((TRACK_CONTROL_INNER_PERIOD_MS % MY_TASK_PERIOD_MS) != 0U)
#error "TRACK_CONTROL_INNER_PERIOD_MS must be a multiple of MY_TASK_PERIOD_MS"
#endif

#if ((TRACK_CONTROL_OUTER_PERIOD_MS % MY_TASK_PERIOD_MS) != 0U)
#error "TRACK_CONTROL_OUTER_PERIOD_MS must be a multiple of MY_TASK_PERIOD_MS"
#endif

#if ((TRACK_CONTROL_OUTER_PERIOD_MS % TRACK_CONTROL_INNER_PERIOD_MS) != 0U)
#error "TRACK_CONTROL_OUTER_PERIOD_MS must be a multiple of inner period"
#endif

#if ((BALL_CONTROL_PERIOD_MS % MY_TASK_PERIOD_MS) != 0U)
#error "BALL_CONTROL_PERIOD_MS must be a multiple of MY_TASK_PERIOD_MS"
#endif

#if ((MY_TASK_INNER_CONTROL_DIVIDER == 0U) \
    || (MY_TASK_INNER_CONTROL_DIVIDER > 255U))
#error "MY_TASK_INNER_CONTROL_DIVIDER must fit in uint8"
#endif

#if ((MY_TASK_OUTER_CONTROL_DIVIDER == 0U) \
    || (MY_TASK_OUTER_CONTROL_DIVIDER > 255U))
#error "MY_TASK_OUTER_CONTROL_DIVIDER must fit in uint8"
#endif

#if ((MY_TASK_BALL_CONTROL_DIVIDER == 0U) \
    || (MY_TASK_BALL_CONTROL_DIVIDER > 255U))
#error "MY_TASK_BALL_CONTROL_DIVIDER must fit in uint8"
#endif

#if (MY_TASK_BATTERY_SAMPLE_DIVIDER == 0U)
#error "MY_TASK_BATTERY_SAMPLE_DIVIDER must be greater than zero"
#endif

#if (MY_TASK_BATTERY_SAMPLE_DIVIDER > 255U)
#error "MY_TASK_BATTERY_SAMPLE_DIVIDER must fit in uint8"
#endif

#if (MY_TASK_REFRESH_DIVIDER == 0U)
#error "MY_TASK_REFRESH_DIVIDER must be greater than zero"
#endif

#if (MY_TASK_REFRESH_DIVIDER > 255U)
#error "MY_TASK_REFRESH_DIVIDER must fit in uint8"
#endif

/*
 * 初始化并启动固定 3 ms PIT；底层 PIT 初始化接口没有返回值。
 */
void my_task_init (void);

/*
 * 固定 3 ms 调度中断回调。每拍先采样一次 IMU 和灰度并发布最新数据；
 * 普通循迹遇到无黑线或灰度通信异常时保留最后一次有效外环目标并继续行驶，
 * 不再因丢线进入故障或停车。Q0 调试发车后只按 6 ms 执行零目标角速度环。题目 2
 * 发车后处理停车线；第一次停车线确认时从零开始计时，第二次确认后冻结
 * 时间并停车。循迹仍按 6 ms 执行角速度内环、按 12 ms 执行灰度外环，
 * 共同到期时外环先执行。
 * 题目 3..6 每 18 ms 进入一次滚球控制时隙，其中 Q4/Q5
 * 同时执行与 Q2 相同的 6/12 ms 循迹控制，但不处理停车线、不计时且不
 * 自动停车；滚球故障也不关闭车体循迹。只有 position_update_count 标识新 K230 坐标时才推进速度
 * PI，位置 PI 外环每三个有效内环时隙更新一次，名义周期为 54 ms。
 *
 * 采样周期与控制处理周期相互独立，循迹两级 PD 只复用本拍采样结果。
 * 滚球控制在重复快照上保持速度目标、积分和电机请求；速度观察时间同时
 * 考虑实际控制间隔和 56 Hz 帧计数间隔，速度 PI 使用两次实际执行间隔。
 * 每次扩展都要测量最坏执行时间并保证小于 3 ms；显示、日志、Flash、
 * 显式延时和协议解析仍留在前台。
 *
 * 电池 ADC 始终按 9 ms 投递给前台，供电机电压补偿使用。按键扫描和
 * 150 ms 屏幕刷新只在非题目态投递，题目态不做人机交互；Q2 停车后由
 * 前台以两位小数显示冻结的总时间。
 */
void my_task_service_irq (void);

/* 返回当前 3 ms 系统节拍计数。 */
uint32 my_task_get_tick_count (void);

/*
 * 原子复制 3 ms 采样层发布的最新灰度数据；本函数不会访问 IIC 总线。
 */
gw_gray_status_enum my_task_get_gray_sample (
    gray_app_sample_struct *sample);

/*
 * 兼容旧 CPU0 入口：IMU 已由 my_task_service_irq() 每 3 ms 采样一次，
 * 因此旧前台 IMU 更新分支在编译期固定为不可达。
 */
#define my_task_take_imu_update_event()          (ZF_FALSE)

/*
 * 原子消费一个 3 ms 前台按键扫描请求；前台延迟时同类请求合并。
 */
uint8 my_task_take_key_scan_event (void);

/*
 * 原子消费一个 9 ms 前台电池采样请求；前台延迟时同类请求合并。
 */
uint8 my_task_take_battery_sample_event (void);

/*
 * 原子消费一个 150 ms 前台刷新请求；前台延迟时同类请求合并。
 */
uint8 my_task_take_refresh_event (void);

#endif
