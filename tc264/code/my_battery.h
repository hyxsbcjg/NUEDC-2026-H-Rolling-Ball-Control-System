#ifndef MY_BATTERY_H_
#define MY_BATTERY_H_

#include "zf_driver_adc.h"

/*
 * 逐飞 TC264 母板 V2.6.2.1 的采样链路为：
 * VCCBAT -> 200 kOhm -> A11 -> 20 kOhm -> GND。
 * 核心板 V3.5 将 VAREF 接到 3.3 V。
 * 因而 A11 电压约为电池电压的 1/11；该换算只反映原理图比例，电阻误差、
 * VAREF 误差和 ADC 校准误差仍需上板标定。
 */
#define MY_BATTERY_ADC_CHANNEL                 (ADC0_CH11_A11)
#define MY_BATTERY_ADC_RESOLUTION              (ADC_12BIT)
#define MY_BATTERY_ADC_MAX_COUNT               (4095U)
#define MY_BATTERY_ADC_REFERENCE_MV            (3300U)
#define MY_BATTERY_DIVIDER_TOP_OHM             (200000U)
#define MY_BATTERY_DIVIDER_BOTTOM_OHM          (20000U)
#define MY_BATTERY_FILTER_WINDOW_SIZE          (8U)

#if (MY_BATTERY_ADC_MAX_COUNT == 0U)
#error "MY_BATTERY_ADC_MAX_COUNT must be greater than zero"
#endif

#if (MY_BATTERY_ADC_REFERENCE_MV == 0U)
#error "MY_BATTERY_ADC_REFERENCE_MV must be greater than zero"
#endif

#if (MY_BATTERY_DIVIDER_BOTTOM_OHM == 0U)
#error "MY_BATTERY_DIVIDER_BOTTOM_OHM must be greater than zero"
#endif

#if ((MY_BATTERY_FILTER_WINDOW_SIZE == 0U) \
    || (MY_BATTERY_FILTER_WINDOW_SIZE > 255U))
#error "MY_BATTERY_FILTER_WINDOW_SIZE must be in the range 1..255"
#endif

/*
 * 通过逐飞 ADC 驱动初始化 A11，并用第一次转换结果填充滑动平均初值。
 * 底层驱动不提供硬件错误返回值。
 */
void  my_battery_init (void);
uint8 my_battery_is_ready (void);

/*
 * 在 CPU0 前台采集一个新样本并更新 8 点滑动平均。初始化前返回 ZF_FALSE。
 */
uint8 my_battery_update (void);

/* 返回滤波后的 12 位 ADC 数值和换算后的电池毫伏值。 */
uint16 my_battery_get_raw (void);
uint32 my_battery_get_millivolts (void);

#endif
