#include "my_key.h"

#include "task.h"

#if (MY_KEY_SCAN_PERIOD_MS != MY_TASK_PERIOD_MS)
#error "MY_KEY_SCAN_PERIOD_MS must match MY_TASK_PERIOD_MS"
#endif

#define MY_KEY_CLICK_COUNT_MAX                ((uint32)0xFFFFFFFFU)

/* 本模块只有 CPU0 前台一个读写者，因此队列本身不需要关中断保护。 */
#pragma section all "cpu0_dsram"
static key_index_enum my_key_click_queue[MY_KEY_CLICK_QUEUE_CAPACITY];
static uint32 my_key_click_total[KEY_NUMBER] = {0U, 0U, 0U, 0U};
static uint8 my_key_click_head = 0U;
static uint8 my_key_click_tail = 0U;
static uint8 my_key_click_count = 0U;
static uint8 my_key_initialized = ZF_FALSE;
#pragma section all restore

static void my_key_record_click (key_index_enum key)
{
    uint8 key_index = (uint8)key;

    if ((uint8)KEY_NUMBER <= key_index)
    {
        return;
    }

    /* 总计数先饱和递增；即使下面队列已满，诊断计数仍保留这次点击。 */
    if (MY_KEY_CLICK_COUNT_MAX > my_key_click_total[key_index])
    {
        my_key_click_total[key_index]++;
    }

    if (MY_KEY_CLICK_QUEUE_CAPACITY <= my_key_click_count)
    {
        return;
    }

    /* 固定容量环形 FIFO：满时保留旧事件，丢弃最新事件。 */
    my_key_click_queue[my_key_click_tail] = key;

    my_key_click_tail++;
    if (MY_KEY_CLICK_QUEUE_CAPACITY <= my_key_click_tail)
    {
        my_key_click_tail = 0U;
    }
    my_key_click_count++;
}

void my_key_init (void)
{
    uint8 key_index;

    my_key_initialized = ZF_FALSE;
    my_key_click_head = 0U;
    my_key_click_tail = 0U;
    my_key_click_count = 0U;

    for (key_index = 0U;
         (uint8)KEY_NUMBER > key_index;
         key_index++)
    {
        my_key_click_total[key_index] = 0U;
    }

    key_init(MY_KEY_SCAN_PERIOD_MS);
    key_clear_all_state();
    my_key_initialized = ZF_TRUE;
}

uint8 my_key_is_ready (void)
{
    return my_key_initialized;
}

void my_key_proc (void)
{
    uint8 key_index;
    key_index_enum key;
    key_state_enum state;

    /*
     * 3 ms pending 位只提供节拍，不在 ISR 内读 GPIO。若前台曾阻塞，多次节拍
     * 会合并为一次扫描，因此消抖时间也会随实际扫描间隔拉长。
     */
    if ((ZF_FALSE == my_key_initialized)
        || (ZF_FALSE == my_task_take_key_scan_event()))
    {
        return;
    }

    key_scanner();

    for (key_index = 0U;
         (uint8)KEY_NUMBER > key_index;
         key_index++)
    {
        key = (key_index_enum)key_index;
        state = key_get_state(key);

        /* 只把“释放后确认的短按”提升为业务事件，长按状态不在此转换。 */
        if (KEY_SHORT_PRESS == state)
        {
            my_key_record_click(key);
            key_clear_state(key);
        }
    }
}

uint8 my_key_fetch_click (key_index_enum *key)
{
    if ((0 == key)
        || (ZF_FALSE == my_key_initialized)
        || (0U == my_key_click_count))
    {
        return ZF_FALSE;
    }

    /* 先取 head 再推进，保持与 record_click() 的 tail 写入相对称。 */
    *key = my_key_click_queue[my_key_click_head];

    my_key_click_head++;
    if (MY_KEY_CLICK_QUEUE_CAPACITY <= my_key_click_head)
    {
        my_key_click_head = 0U;
    }
    my_key_click_count--;

    return ZF_TRUE;
}

uint32 my_key_get_click_count (key_index_enum key)
{
    uint8 key_index = (uint8)key;

    if ((ZF_FALSE == my_key_initialized)
        || ((uint8)KEY_NUMBER <= key_index))
    {
        return 0U;
    }

    return my_key_click_total[key_index];
}

void my_key_clear_clicks (void)
{
    /* 这里只丢弃未消费事件；my_key_click_total[] 故意保留。 */
    my_key_click_head = 0U;
    my_key_click_tail = 0U;
    my_key_click_count = 0U;
}
