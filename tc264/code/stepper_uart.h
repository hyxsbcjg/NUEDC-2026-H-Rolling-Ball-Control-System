#ifndef STEPPER_UART_H_
#define STEPPER_UART_H_

#include "zf_common_typedef.h"
#include "zf_driver_uart.h"

/*
 * Emm_V5 专用 UART3：115200 bit/s，8N1，无流控，只允许 TC264 -> 驱动器。
 * MCU P15.6(TX) -> 驱动器 RX。逐飞 uart_init() 必须同时接收一组 TX/RX
 * 枚举，因此 P15.7 仍作为初始化占位，但外部不要连接驱动器 TX。
 * UART3 RX/TX 中断均关闭；UART3 不能再与 GNSS 共用。
 */
#define STEPPER_UART_INDEX                      (UART_3)
#define STEPPER_UART_BAUD_RATE                  (115200U)
#define STEPPER_UART_TX_PIN                     (UART3_TX_P15_6)
#define STEPPER_UART_RX_PIN                     (UART3_RX_P15_7)

#define STEPPER_UART_FRAME_CHECK_BYTE           (0x6BU)
/* 地址 0 是广播地址，可用于不需要驱动器回包的控制命令。 */
#define STEPPER_UART_BROADCAST_ADDRESS           (0x00U)
/*
 * X42S V1.0.5 的 Emm F6/FD 速度字段范围为 0..3000。接口沿用 rpm 命名，
 * 但数值原样写入字段；S_Vel_IS=Enable 时 1 LSB 实际对应 0.1 RPM。
 */
#define STEPPER_UART_MAX_SPEED_RPM               (3000U)
#define STEPPER_UART_POWER_ON_DELAY_MS           (500U)
/* 前台配置命令之间的保护间隔，不是周期速度命令的固定限频。 */
#define STEPPER_UART_COMMAND_GAP_MS              (10U)
#define STEPPER_UART_MULTI_VELOCITY_MAX_COUNT    (63U)
#define STEPPER_UART_MULTI_MOVE_MAX_COUNT        (39U)

typedef enum
{
    STEPPER_UART_STATUS_OK = 0,
    STEPPER_UART_STATUS_INVALID_ARGUMENT,
    STEPPER_UART_STATUS_NOT_INITIALIZED,
    STEPPER_UART_STATUS_TX_BUSY
} stepper_uart_status_enum;

typedef enum
{
    STEPPER_UART_DIRECTION_CW = 0,
    STEPPER_UART_DIRECTION_CCW = 1
} stepper_uart_direction_enum;

typedef enum
{
    STEPPER_UART_POSITION_RELATIVE = 0,
    STEPPER_UART_POSITION_ABSOLUTE = 1,
    STEPPER_UART_POSITION_RELATIVE_CURRENT = 2
} stepper_uart_position_mode_enum;

/*
 * 所选 Y42 官方例程把 0x46 的值解释为开环/闭环；旧 Emm42_V5 固件把
 * 同一命令解释为下方 P_Pul 模式。两套枚举不可混用。
 */
typedef enum
{
    STEPPER_UART_CONTROL_OPEN_LOOP = 0,
    STEPPER_UART_CONTROL_CLOSED_LOOP = 1
} stepper_uart_control_mode_enum;

typedef enum
{
    STEPPER_UART_LEGACY_PULSE_DISABLED = 0,
    STEPPER_UART_LEGACY_PULSE_OPEN_LOOP = 1,
    STEPPER_UART_LEGACY_PULSE_CLOSED_LOOP = 2
} stepper_uart_legacy_pulse_mode_enum;

/* 多机速度子帧：[地址,F6,方向,速度BE16,加速度,同步标志,6B]。 */
typedef struct
{
    uint8 address;
    stepper_uart_direction_enum direction;
    uint16 speed_rpm;
    uint8 acceleration;
    uint8 synchronized;
} stepper_uart_velocity_command_struct;

/* 多机位置子帧：[地址,FD,方向,速度BE16,加速度,脉冲BE32,模式,同步,6B]。 */
typedef struct
{
    uint8 address;
    stepper_uart_direction_enum direction;
    uint16 speed_rpm;
    uint8 acceleration;
    uint32 pulse_count;
    stepper_uart_position_mode_enum position_mode;
    uint8 synchronized;
} stepper_uart_move_command_struct;

/*
 * 仅配置 UART3 的发送方向，收发中断均保持关闭。P15.7 只是 uart_init()
 * 的占位 RX 引脚，外部保持不连接。函数不等待驱动器上电，不发送使能/停止
 * 命令，也不验证驱动器在线；这些业务步骤由上层完成。
 */
stepper_uart_status_enum stepper_uart_init (void);
uint8                   stepper_uart_is_ready (void);

/*
 * Emm_V5 命令族；除名称中含 try 的两个固定帧接口外，其余发送接口均为
 * 阻塞式，只能在前台调用。通用格式为
 * [地址, 命令, 参数..., 0x6B]；多字节字段高字节在前，布尔参数只接受
 * ZF_FALSE/ZF_TRUE。
 *
 * 速度参数沿用参考 API 的 rpm 命名，但实现只把数值原样写入 16 位字段，
 * 不做 x10 或 /10 换算；真实物理单位须结合驱动器 S_Vel 配置上板确认。
 *
 * 地址 0 可用于控制广播。synchronized=ZF_TRUE 只把动作加入同步队列，
 * 仍需发送 0xFF 同步触发。返回 OK 只表示本地校验和发送调用完成；当前
 * 单向方案不接收 ACK。
 */
stepper_uart_status_enum stepper_uart_clear_stall (uint8 address);
stepper_uart_status_enum stepper_uart_set_control_mode (
    uint8 address,
    uint8 save_to_flash,
    stepper_uart_control_mode_enum mode);
stepper_uart_status_enum stepper_uart_set_legacy_pulse_mode (
    uint8 address,
    uint8 save_to_flash,
    stepper_uart_legacy_pulse_mode_enum mode);
stepper_uart_status_enum stepper_uart_set_enable (
    uint8 address,
    uint8 enabled,
    uint8 synchronized);
stepper_uart_status_enum stepper_uart_run_velocity (
    uint8 address,
    stepper_uart_direction_enum direction,
    uint16 speed_rpm,
    uint8 acceleration,
    uint8 synchronized);
stepper_uart_status_enum stepper_uart_run_velocity_signed (
    uint8 address,
    int16 signed_speed_rpm,
    uint8 acceleration,
    uint8 synchronized);

/*
 * 技术上的非阻塞固定帧发送：速度帧为 8 字节、位置帧为 13 字节，均可一次
 * 放入 16 字节 UART3 TX FIFO。只有 FIFO 为空时才写入
 * 完整帧，否则立即返回 TX_BUSY；函数不等待线路发送完成，也不保存或排队
 * 旧命令。
 *
 * 速度/位置 try 接口可在 3 ms 控制 ISR 中调用。调用方必须保证单一所有者，
 * 并与所有前台 UART3 配置命令
 * 串行化。115200 bit/s 下，13 字节位置帧在线约 1.13 ms，理论上可在下一次
 * 3 ms 控制拍前发送完成；但发生前台竞争或 FIFO 仍非空时依然会返回
 * TX_BUSY。ISR 中不得调用阻塞式配置接口。
 */
stepper_uart_status_enum stepper_uart_try_run_velocity_signed (
    uint8 address,
    int16 signed_speed_rpm,
    uint8 acceleration,
    uint8 synchronized);
stepper_uart_status_enum stepper_uart_send_multi_velocity (
    uint8 address,
    const stepper_uart_velocity_command_struct *commands,
    uint8 command_count);
stepper_uart_status_enum stepper_uart_move (
    uint8 address,
    stepper_uart_direction_enum direction,
    uint16 speed_rpm,
    uint8 acceleration,
    uint32 pulse_count,
    stepper_uart_position_mode_enum position_mode,
    uint8 synchronized);
stepper_uart_status_enum stepper_uart_move_signed (
    uint8 address,
    int32 signed_pulse_count,
    uint16 speed_rpm,
    uint8 acceleration,
    stepper_uart_position_mode_enum position_mode,
    uint8 synchronized);
stepper_uart_status_enum stepper_uart_try_move_signed (
    uint8 address,
    int32 signed_pulse_count,
    uint16 speed_rpm,
    uint8 acceleration,
    stepper_uart_position_mode_enum position_mode,
    uint8 synchronized);
stepper_uart_status_enum stepper_uart_send_multi_move (
    uint8 address,
    const stepper_uart_move_command_struct *commands,
    uint8 command_count);
stepper_uart_status_enum stepper_uart_set_quick_move (
    uint8 address,
    uint16 speed_rpm,
    uint8 acceleration,
    stepper_uart_position_mode_enum position_mode,
    uint8 synchronized);
stepper_uart_status_enum stepper_uart_quick_move (
    uint8 address,
    int32 signed_pulse_count);
stepper_uart_status_enum stepper_uart_stop (
    uint8 address,
    uint8 synchronized);
stepper_uart_status_enum stepper_uart_start_synchronous_motion (
    uint8 address);
/*
 * 非阻塞发送固定最近点回零帧 [地址,9A,00,00,6B]。该接口只触发驱动器
 * 回到外部既有机械零点，不写当前位置、不设置零点，也不修改回零配置。
 */
stepper_uart_status_enum stepper_uart_try_start_home (uint8 address);
/*
 * 兼容现有 user/isr.c 声明的空回调。UART3 RX 中断始终关闭，函数不读
 * UART、不缓存数据；保留它只是为了不修改逐飞 user 中断文件。
 */
void stepper_uart_rx_irq_handler (void);

#endif
