#include "my_battery.h"

#pragma section all "cpu0_dsram"
static uint16 my_battery_raw = 0U;
/*
 * CPU0 前台每 9 ms 更新，题目态电机控制 ISR 通过 motor 层读取。TC264 的
 * 对齐 32 位访问是单次访问，volatile 保证每次控制调用都重新读取最新值。
 */
static volatile uint32 my_battery_millivolts = 0U;
static uint8 my_battery_initialized = ZF_FALSE;
static uint16 my_battery_filter_window[MY_BATTERY_FILTER_WINDOW_SIZE];
static uint32 my_battery_filter_sum = 0U;
static uint8 my_battery_filter_index = 0U;
static uint8 my_battery_filter_count = 0U;
#pragma section all restore

static void my_battery_reset_filter (void)
{
    uint8 index;

    for (index = 0U;
        index < MY_BATTERY_FILTER_WINDOW_SIZE;
        index++)
    {
        my_battery_filter_window[index] = 0U;
    }

    my_battery_filter_sum = 0U;
    my_battery_filter_index = 0U;
    my_battery_filter_count = 0U;
}

static uint16 my_battery_filter_sample (uint16 sample)
{
    /*
     * 启动阶段只除以实际样本数，不把尚未填充的零槽纳入平均；窗口填满后，
     * 每次减去最旧样本再加入新样本，形成 O(1) 滑动平均。
     */
    if (MY_BATTERY_FILTER_WINDOW_SIZE
        <= my_battery_filter_count)
    {
        my_battery_filter_sum -=
            my_battery_filter_window[my_battery_filter_index];
    }
    else
    {
        my_battery_filter_count++;
    }

    my_battery_filter_window[my_battery_filter_index] = sample;
    my_battery_filter_sum += sample;

    my_battery_filter_index++;
    if (MY_BATTERY_FILTER_WINDOW_SIZE
        <= my_battery_filter_index)
    {
        my_battery_filter_index = 0U;
    }

    /* 加 count/2 后再做整数除法，实现最接近值舍入。 */
    return (uint16)(
        (my_battery_filter_sum
            + ((uint32)my_battery_filter_count / 2U))
        / (uint32)my_battery_filter_count);
}

static uint32 my_battery_raw_to_millivolts (uint16 raw)
{
    uint64 numerator;
    uint64 denominator;

    /*
     * ADC码值 × VREF × 分压反比；中间乘积超过 32 位，必须用 uint64。
     * 加 denominator/2 后取整，最终结果仍以电池端毫伏表示。
     */
    numerator =
        (uint64)raw
        * (uint64)MY_BATTERY_ADC_REFERENCE_MV
        * (uint64)(
            MY_BATTERY_DIVIDER_TOP_OHM
            + MY_BATTERY_DIVIDER_BOTTOM_OHM);
    denominator =
        (uint64)MY_BATTERY_ADC_MAX_COUNT
        * (uint64)MY_BATTERY_DIVIDER_BOTTOM_OHM;

    numerator += denominator / 2U;
    return (uint32)(numerator / denominator);
}

void my_battery_init (void)
{
    my_battery_initialized = ZF_FALSE;
    my_battery_raw = 0U;
    my_battery_millivolts = 0U;
    my_battery_reset_filter();

    /*
     * 逐飞 ADC 初始化接口无错误返回；initialized 只表示调用序列完成，不
     * 证明 A11、电阻分压或 VAREF 在实板上正常。
     */
    adc_init(MY_BATTERY_ADC_CHANNEL, MY_BATTERY_ADC_RESOLUTION);
    my_battery_initialized = ZF_TRUE;
    (void)my_battery_update();
}

uint8 my_battery_is_ready (void)
{
    return my_battery_initialized;
}

uint8 my_battery_update (void)
{
    uint16 filtered_raw;

    if (ZF_FALSE == my_battery_initialized)
    {
        return ZF_FALSE;
    }

    filtered_raw = my_battery_filter_sample(
        adc_convert(MY_BATTERY_ADC_CHANNEL));
    my_battery_raw = filtered_raw;
    my_battery_millivolts =
        my_battery_raw_to_millivolts(filtered_raw);

    return ZF_TRUE;
}

uint16 my_battery_get_raw (void)
{
    return (ZF_TRUE == my_battery_initialized) ? my_battery_raw : 0U;
}

uint32 my_battery_get_millivolts (void)
{
    return (ZF_TRUE == my_battery_initialized)
        ? my_battery_millivolts
        : 0U;
}
