#ifndef MY_KEY_H_
#define MY_KEY_H_

#include "zf_device_key.h"

/*
 * 逐飞按键扫描器必须使用固定周期。本模块复用 CPU0 的 3 ms 任务节拍，
 * 并在前台执行 GPIO 扫描。
 */
#define MY_KEY_SCAN_PERIOD_MS                 (3U)
#define MY_KEY_CLICK_QUEUE_CAPACITY           (8U)

#if (MY_KEY_SCAN_PERIOD_MS == 0U)
#error "MY_KEY_SCAN_PERIOD_MS must be greater than zero"
#endif

#if ((MY_KEY_CLICK_QUEUE_CAPACITY == 0U) \
    || (MY_KEY_CLICK_QUEUE_CAPACITY > 255U))
#error "MY_KEY_CLICK_QUEUE_CAPACITY must be in the range 1..255"
#endif

/*
 * 点击事件保留底层 KEY_1 到 KEY_4 的编号。当前 zf_device_key.h 中的
 * KEY_LIST 将它们映射到 P20.6、P20.7、P11.2 和 P11.3。menu/question_app
 * 负责分配业务含义：KEY_1 在 Q0 到 Q6 间切换；KEY_2 在 Q0 触发回到
 * 驱动器既有机械零点，在 Q6 停车态将 RAM 运行中心减 20 pulse；KEY_3 在 Q0
 * 停车态推进固定绝对脉冲表，在 Q6 停车态将 RAM 运行中心加 20 pulse；KEY_4
 * 启动 Q0 自定义调试模式、Q2 或 Q3 到 Q6。Q6 调整只发送绝对位置并修改 RAM
 * 中心；任何按键都不能修改机械零点或回零配置。
 */

/*
 * 初始化四个逐飞按键，清空点击队列，并将各按键累计点击次数清零。应在
 * my_task_init() 启动 3 ms 节拍前，由 CPU0 调用一次。
 */
void  my_key_init (void);
uint8 my_key_is_ready (void);

/*
 * 消费一个合并后的 3 ms 扫描请求，并在 CPU0 前台调用逐飞按键扫描器。
 * 应由主循环持续调用，绝不能在 ISR 中调用。
 *
 * 只有松手后底层报告 KEY_SHORT_PRESS，才会加入一次点击事件。持续按住按键
 * 不会生成业务层点击。
 */
void my_key_proc (void);

/*
 * 取出并消费队列中最早的一次点击事件。成功返回 ZF_TRUE；输出指针为空、
 * 模块未初始化或队列为空时返回 ZF_FALSE，并且不消费任何事件。
 * 累计计数与事件队列相互独立：队列满时新事件会丢弃，但对应按键的累计次数
 * 仍会增加（直到 uint32 饱和）。
 */
uint8 my_key_fetch_click (key_index_enum *key);

/*
 * 返回指定按键的累计点击次数。按键编号无效时返回零。
 */
uint32 my_key_get_click_count (key_index_enum key);

/*
 * 丢弃所有已排队点击事件，但不改变累计点击次数。
 */
void my_key_clear_clicks (void);

#endif
