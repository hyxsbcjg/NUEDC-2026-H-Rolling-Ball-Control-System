#ifndef MOTOR_H_
#define MOTOR_H_

#include "zf_common_typedef.h"
#include "zf_driver_gpio.h"
#include "zf_driver_pwm.h"

#define MOTOR_A_IN1_PIN                         (P21_2)
#define MOTOR_A_IN2_PIN                         (P21_3)
#define MOTOR_B_IN1_PIN                         (P21_4)
#define MOTOR_B_IN2_PIN                         (P21_5)

#define MOTOR_A_PWM_PIN                         (ATOM0_CH6_P02_6)
#define MOTOR_B_PWM_PIN                         (ATOM0_CH7_P02_7)
#define MOTOR_PWM_FREQUENCY_HZ                  (10000U)

/*
 * 沿用 MSPM0 参考工程的有符号命令量程。命令绝对值按 8000 映射到逐飞
 * PWM_DUTY_MAX，因此 7000 表示 7000/8000 = 87.5% 占空比，而不是 70%。
 */
#define MOTOR_COMMAND_REFERENCE_PERIOD          (8000U)
#define MOTOR_COMMAND_MAX                       (7000)
#define MOTOR_COMMAND_MIN                       (-MOTOR_COMMAND_MAX)
#define MOTOR_VOLTAGE_REFERENCE_MV              (12000U)

/*
 * 当前板 STBY 直接接 5 V，不受 TC264 软件控制；软件停止只会撤销 PWM 并把
 * 两个方向输入拉低，不等于驱动器硬件断能。
 */

#if (0U == MOTOR_PWM_FREQUENCY_HZ)
#error "MOTOR_PWM_FREQUENCY_HZ must be greater than zero"
#endif

#if (MOTOR_COMMAND_MAX > MOTOR_COMMAND_REFERENCE_PERIOD)
#error "MOTOR_COMMAND_MAX must not exceed MOTOR_COMMAND_REFERENCE_PERIOD"
#endif

#if (0U == MOTOR_VOLTAGE_REFERENCE_MV)
#error "MOTOR_VOLTAGE_REFERENCE_MV must be greater than zero"
#endif

typedef enum
{
    MOTOR_STATUS_OK = 0,
    MOTOR_STATUS_NOT_INITIALIZED
} motor_status_enum;

/*
 * 初始化契约：先把方向 GPIO 置为零驱动状态，再以零占空比启动两路 PWM。
 * 正命令对应 IN1 高、IN2 低，负命令相反。返回 OK 仅表示无返回值的底层配置
 * 调用已经执行，不证明驱动板、电机或 PWM 波形正常。
 */
motor_status_enum motor_init (void);
uint8             motor_is_ready (void);

/*
 * 命令表示 MOTOR_VOLTAGE_REFERENCE_MV 下的期望等效输出。每次调用都会按
 * 参考电压/滤波电池电压补偿并再次限幅；电池采样未就绪时，非零请求安全降为
 * 零输出，但接口仍返回 OK。
 *
 * 从非零转为零或反向时，会先撤销 PWM，阻塞等待至少一个 PWM 周期，再修改
 * 方向 GPIO；同方向调幅不等待。零命令令 PWM 为零且 IN1/IN2 均低，实际是
 * 滑行还是制动仍取决于驱动芯片真值表。实时 PID 可在固定控制 ISR 中调用；
 * 换向路径最多忙等一个 10 kHz PWM 周期，必须计入 ISR 最坏执行时间，且不得
 * 与前台并发调用。连续调用 A、B 接口时，两路命令按调用顺序应用。
 */
motor_status_enum motor_set_A (int16 command);
motor_status_enum motor_set_B (int16 command);
motor_status_enum motor_stop_all (void);

/* 返回最近一次实际应用的“已电压补偿且已限幅”命令，不是调用者原始请求。 */
int16 motor_get_a_command (void);
int16 motor_get_b_command (void);

#endif
