#ifndef MY_UART_H_
#define MY_UART_H_

#include "zf_common_typedef.h"

/*
 * 本层独占逐飞无线串口设备：UART2，115200 bit/s，8N1。
 * MCU P10.5(TX) -> 模块 RX，MCU P10.6(RX) <- 模块 TX，
 * P10.2 <- 模块 RTS。引脚和自动波特率流程由 zf_device_wireless_uart
 * 统一配置，本层不得再次调用 uart_init() 改写 UART2。
 */

/*
 * 格式化缓冲区包含结尾 '\0'；超长文本只发送前
 * MY_PRINTF_BUFFER_SIZE - 1 字节，并返回 TRUNCATED。
 */
#ifndef MY_PRINTF_BUFFER_SIZE
#define MY_PRINTF_BUFFER_SIZE                  (128U)
#endif

#if (MY_PRINTF_BUFFER_SIZE < 2U)
#error "MY_PRINTF_BUFFER_SIZE must be at least 2"
#endif

typedef enum
{
    MY_UART_STATUS_OK = 0,
    MY_UART_STATUS_INVALID_ARGUMENT,
    MY_UART_STATUS_NOT_INITIALIZED,
    MY_UART_STATUS_INIT_FAILED,
    MY_UART_STATUS_FORMAT_ERROR,
    MY_UART_STATUS_TRUNCATED,
    MY_UART_STATUS_SEND_TIMEOUT
} my_uart_status_enum;

/*
 * 初始化无线串口、RTS 和底层 RX FIFO。底层开启自动波特率时会执行毫秒级
 * 延时并等待模块应答，因此只能在前台启动阶段调用。OK 只表示底层初始化流程
 * 返回成功，不证明模块在线或物理链路可用。
 */
my_uart_status_enum my_uart_init (void);
uint8              my_uart_is_ready (void);

/*
 * 前台格式化并阻塞发送文本，不自动追加 "\r\n"。底层每 30 字节检查一次
 * 模块 RTS；RTS 持续为高会等待至超时，所以禁止在 ISR 或硬实时路径调用。
 * TRUNCATED 表示截短后的文本已发送；若发送又超时，则最终返回 SEND_TIMEOUT。
 */
my_uart_status_enum my_printf (const char *format, ...);

/*
 * 返回最近一次 my_printf() 尚未发送的字节数；非零值与 SEND_TIMEOUT 对应。
 */
uint32 my_uart_get_last_unsent_length (void);

#endif
