#ifndef K230_UART_H_
#define K230_UART_H_

#include "zf_common_typedef.h"
#include "zf_driver_uart.h"

/*
 * K230 专用双向 UART1：115200 bit/s，8N1，无流控。
 * MCU P02.2(TX) -> K230 RX；MCU P02.3(RX) <- K230 TX。
 * TC264 发送使用前台阻塞接口；接收使用 RX 中断，ISR 只搬运一个字节。
 * TX 中断不需要开启。
 */
#define K230_UART_INDEX                         (UART_1)
#define K230_UART_BAUD_RATE                     (115200U)
#define K230_UART_TX_PIN                        (UART1_TX_P02_2)
#define K230_UART_RX_PIN                        (UART1_RX_P02_3)

#define K230_UART_RX_BUFFER_SIZE                (128U)
#define K230_UART_COMMAND_FRAME_HEAD            (0x20U)
#define K230_UART_COMMAND_FRAME_TAIL            (0x30U)
#define K230_UART_DATA_FRAME_HEAD               (0x40U)
#define K230_UART_DATA_FRAME_TAIL               (0x50U)
#define K230_UART_DATA_FRAME_SIZE               (5U)
#define K230_UART_DATA_SIZE                     (2U)
#define K230_UART_FLAG_BALL_POSITION            (0x00U)
#define K230_UART_FLAG_TOUCH_X                  (0x0AU)

/*
 * 双向协议均没有长度域和 CRC：
 * TC264 -> K230：[0x20, Data1, Data2, 0x30]
 * K230 -> TC264：[0x40, Flag, Data1, Data2, 0x50]
 * Flag=0x00 时，Data1、Data2 是 Ball_X 的高、低字节。控制目标不在协议中。
 * Flag=0x0A 时，Data1、Data2 是 Touch_X 的高、低字节；只允许 Q6 采用。
 * 触摸标志是 0x0A，不是 0xA0；0xA0 按其它保留 Flag 处理。
 * 其它 Flag 值保留；接收端消费该帧，但不更新任何坐标快照。
 */

#if ((K230_UART_RX_BUFFER_SIZE < 2U) \
    || (0U != (K230_UART_RX_BUFFER_SIZE & (K230_UART_RX_BUFFER_SIZE - 1U))))
#error "K230_UART_RX_BUFFER_SIZE must be a power of two and at least 2"
#endif

typedef enum
{
    K230_UART_STATUS_OK = 0,
    K230_UART_STATUS_INVALID_ARGUMENT,
    K230_UART_STATUS_NOT_INITIALIZED,
    K230_UART_STATUS_RX_BUFFER_EMPTY
} k230_uart_status_enum;

typedef struct
{
    uint8 flag;
    uint8 data[K230_UART_DATA_SIZE];
} k230_uart_frame_struct;

/*
 * 先清空软件状态，再配置 UART1，最后开启 RX 中断；TX 中断始终关闭。
 * OK 只确认本地初始化调用完成，不确认 K230 在线。
 */
k230_uart_status_enum k230_uart_init (void);
uint8                k230_uart_is_ready (void);

/*
 * 阻塞式前台发送接口。逐飞 uart_write_buffer() 会等待硬件 FIFO 排空，
 * 禁止在 ISR 或硬实时控制路径调用。
 */
k230_uart_status_enum k230_uart_write (
    const uint8 *data,
    uint32 length);
k230_uart_status_enum k230_uart_send_command (
    uint8 data_1,
    uint8 data_2);

/*
 * 环形缓冲区由 UART1 RX ISR 生产、CPU0 前台消费。为区分空和满，128 字节
 * 数组最多保存 127 字节；满时丢弃新字节并累计 overflow_count。
 */
k230_uart_status_enum k230_uart_read_byte (uint8 *data);
uint16                k230_uart_available (void);
void                  k230_uart_clear_rx_buffer (void);
uint32                k230_uart_get_rx_overflow_count (void);

/*
 * 在前台消费环形缓冲区并拼装一个固定 5 字节帧。解析器只验证包头、固定
 * 长度和包尾，不验证载荷；RX_BUFFER_EMPTY 同时表示“当前无数据”和
 * “只有半帧”，半帧状态会保留到下次调用。
 */
k230_uart_status_enum k230_uart_read_frame (
    k230_uart_frame_struct *frame);
uint8 k230_uart_decode_ball_x (
    const k230_uart_frame_struct *frame,
    uint16 *ball_x);
uint8 k230_uart_decode_touch_x (
    const k230_uart_frame_struct *frame,
    uint16 *touch_x);
uint32 k230_uart_get_frame_error_count (void);

/*
 * UART1_RX_IRQHandler 的短回调：非阻塞读取一个硬件字节并尝试入环形缓冲，
 * 不在 ISR 中成帧、解码或发送。
 */
void k230_uart_rx_irq_handler (void);

#endif
