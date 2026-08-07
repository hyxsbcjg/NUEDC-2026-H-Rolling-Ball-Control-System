#ifndef K230_APP_H_
#define K230_APP_H_

#include "k230_uart.h"
#include "zf_common_typedef.h"

/*
 * 每次前台服务最多处理 127 字节 UART 环形缓冲区能够容纳的完整 5 字节位置
 * 帧，再加上服务期间可能新到达的两帧，保证每次主循环调用的执行量有界。
 */
#define K230_APP_MAX_FRAMES_PER_SERVICE          (27U)
/*
 * 当前 K230 显示坐标范围为 0..799。超过该范围的数据视为可能的传输损坏：
 * Ball_X 不允许进入滚球控制器，Touch_X 不允许成为 Q6 目标。
 */
#define K230_APP_BALL_X_MAX                      (799U)
#define K230_APP_TOUCH_X_MAX                     (799U)

#if ((K230_APP_MAX_FRAMES_PER_SERVICE == 0U) \
    || (K230_APP_MAX_FRAMES_PER_SERVICE > 255U))
#error "K230_APP_MAX_FRAMES_PER_SERVICE must fit in uint8"
#endif

#if (K230_APP_BALL_X_MAX > 0xFFFFU)
#error "K230_APP_BALL_X_MAX must fit in uint16"
#endif

#if (K230_APP_TOUCH_X_MAX > 0xFFFFU)
#error "K230_APP_TOUCH_X_MAX must fit in uint16"
#endif

typedef struct
{
    uint8 initialized;
    uint8 position_valid;
    uint8 touch_valid;
    uint16 ball_x;
    uint16 touch_x;
    uint32 position_update_count;
    uint32 touch_update_count;
} k230_app_snapshot_struct;

/*
 * Ball_X 与 Touch_X 是两条独立数据流。k230_app 只发布范围合法的最新值和
 * 更新计数；是否允许当前题目把 Touch_X 作为控制目标，仅由 question_app
 * 决定。
 */

/*
 * 清空应用快照并初始化 UART1。k230_uart_init() 只有在软件环形缓冲区就绪后
 * 才使能接收中断。
 */
k230_uart_status_enum k230_app_init (void);

/*
 * CPU0 前台服务。负责消费并解析完整 UART 帧，不写屏幕，也不在 ISR 中运行。
 */
void k230_app_service (void);

/*
 * 原子复制最新的 Ball_X/Touch_X 快照。尚未成功初始化或输出指针为空时，
 * 返回 ZF_FALSE。
 */
uint8 k230_app_get_snapshot (k230_app_snapshot_struct *snapshot);

#endif
