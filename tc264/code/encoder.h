#ifndef ENCODER_H_
#define ENCODER_H_

#include "zf_common_typedef.h"
#include "zf_driver_encoder.h"

#define ENCODER_A_INDEX                         (TIM2_ENCODER)
#define ENCODER_A_CH1_PIN                       (TIM2_ENCODER_CH1_P33_7)
#define ENCODER_A_CH2_PIN                       (TIM2_ENCODER_CH2_P33_6)

#define ENCODER_B_INDEX                         (TIM4_ENCODER)
#define ENCODER_B_CH1_PIN                       (TIM4_ENCODER_CH1_P02_8)
#define ENCODER_B_CH2_PIN                       (TIM4_ENCODER_CH2_P00_9)

/*
 * 首次单轮低速正转测试后，若编码器符号与约定电机正方向相反，只把对应值改为
 * -1；不要通过交换 GPT12 通道来修正软件方向。
 */
#define ENCODER_A_DIRECTION_SIGN                (1)
#define ENCODER_B_DIRECTION_SIGN                (1)

#if ((1 != ENCODER_A_DIRECTION_SIGN) \
    && (-1 != ENCODER_A_DIRECTION_SIGN))
#error "ENCODER_A_DIRECTION_SIGN must be 1 or -1"
#endif

#if ((1 != ENCODER_B_DIRECTION_SIGN) \
    && (-1 != ENCODER_B_DIRECTION_SIGN))
#error "ENCODER_B_DIRECTION_SIGN must be 1 or -1"
#endif

typedef enum
{
    ENCODER_STATUS_OK = 0,
    ENCODER_STATUS_INVALID_ARGUMENT,
    ENCODER_STATUS_NOT_INITIALIZED
} encoder_status_enum;

typedef struct
{
    int16 motor_a;
    int16 motor_b;
} encoder_delta_struct;

/*
 * E1 使用 GPT12 TIM2、E2 使用 GPT12 TIM4，均由硬件正交接口计数，初始化后
 * 清零，无需 GPIO 中断。OK/ready 只证明无返回值的底层配置调用已经执行，
 * 不代表真实信号存在或方向正确。
 */
encoder_status_enum encoder_init (void);
uint8               encoder_is_ready (void);

/*
 * 每次调用返回“上次清零至本次读取”的带符号增量，随后立即清零对应硬件计数器，
 * 因而结果不是上电累计里程。逐飞正交驱动会把 x4 边沿计数除以 4，应用层一个
 * 计数代表一个完整 AB 周期。
 *
 * 读取与清零是两次寄存器操作，并非原子快照；both 版本也按 A 后 B 顺序执行，
 * 两路采样边界并不完全同时。实时速度环可在固定控制 ISR 中调用，并保证只有
 * 该 ISR 读取/清零计数器；同时保证周期内计数不越过 int16 范围。
 */
encoder_status_enum encoder_read_and_clear_a (int16 *count);
encoder_status_enum encoder_read_and_clear_b (int16 *count);
encoder_status_enum encoder_read_and_clear_both (
    encoder_delta_struct *delta);
encoder_status_enum encoder_clear_all (void);

#endif
