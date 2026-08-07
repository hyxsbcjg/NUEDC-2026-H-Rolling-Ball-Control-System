#include "k230_uart.h"

/*
 * 单生产者/单消费者环形缓冲：
 * - UART1 RX ISR 只推进 head；
 * - CPU0 前台只推进 tail；
 * - next(head) == tail 表示满，容量因此是 127 字节。
 * 满时保留旧数据、丢弃新字节，两个错误计数均饱和在 UINT32_MAX。
 * 帧缓存和帧下标只由前台解析器访问。
 */
#pragma section all "cpu0_dsram"
static volatile uint8 k230_uart_rx_buffer[K230_UART_RX_BUFFER_SIZE];
static volatile uint16 k230_uart_rx_head = 0U;
static volatile uint16 k230_uart_rx_tail = 0U;
static volatile uint32 k230_uart_rx_overflow_count = 0U;
static volatile uint8 k230_uart_initialized = ZF_FALSE;
static uint8 k230_uart_frame_buffer[K230_UART_DATA_FRAME_SIZE];
static uint8 k230_uart_frame_index = 0U;
static uint32 k230_uart_frame_error_count = 0U;
#pragma section all restore

static uint16 k230_uart_next_rx_index (uint16 index)
{
    return (uint16)((index + 1U) & (K230_UART_RX_BUFFER_SIZE - 1U));
}

static uint8 k230_uart_decode_x (
    const k230_uart_frame_struct *frame,
    uint8 expected_flag,
    uint16 *position_x)
{
    if ((0 == frame)
        || (0 == position_x)
        || (expected_flag != frame->flag))
    {
        return ZF_FALSE;
    }

    /* 两种 X 坐标都采用网络序：先拼高字节，再拼低字节。 */
    *position_x =
        (uint16)(((uint16)frame->data[0] << 8U)
            | (uint16)frame->data[1]);

    return ZF_TRUE;
}

k230_uart_status_enum k230_uart_init (void)
{
    k230_uart_initialized = ZF_FALSE;
    k230_uart_rx_head = 0U;
    k230_uart_rx_tail = 0U;
    k230_uart_rx_overflow_count = 0U;
    k230_uart_frame_index = 0U;
    k230_uart_frame_error_count = 0U;

    /* uart_init() 配置 8N1，并在内部先关闭 UART1 的收发中断。 */
    uart_init(
        K230_UART_INDEX,
        K230_UART_BAUD_RATE,
        K230_UART_TX_PIN,
        K230_UART_RX_PIN);

    /* TX 走前台轮询/阻塞发送，不需要 TX 中断。RX 中断负责异步收字节。 */
    uart_tx_interrupt(K230_UART_INDEX, 0U);
    /*
     * 先发布 initialized，再打开 RX 中断；这样中断一旦到达，回调不会把
     * 已完成 UART 配置后的首字节误判为“尚未初始化”。
     */
    k230_uart_initialized = ZF_TRUE;
    uart_rx_interrupt(K230_UART_INDEX, 1U);

    return K230_UART_STATUS_OK;
}

uint8 k230_uart_is_ready (void)
{
    return k230_uart_initialized;
}

k230_uart_status_enum k230_uart_write (
    const uint8 *data,
    uint32 length)
{
    if (0 == data)
    {
        return K230_UART_STATUS_INVALID_ARGUMENT;
    }
    if (ZF_FALSE == k230_uart_initialized)
    {
        return K230_UART_STATUS_NOT_INITIALIZED;
    }

    if (0U != length)
    {
        /* 逐飞实现逐字节等待 TX FIFO 清空，所以只能在 CPU0 前台调用。 */
        uart_write_buffer(K230_UART_INDEX, data, length);
    }

    return K230_UART_STATUS_OK;
}

k230_uart_status_enum k230_uart_send_command (
    uint8 data_1,
    uint8 data_2)
{
    const uint8 frame[4] =
    {
        K230_UART_COMMAND_FRAME_HEAD,
        data_1,
        data_2,
        K230_UART_COMMAND_FRAME_TAIL
    };

    return k230_uart_write(frame, (uint32)sizeof(frame));
}

k230_uart_status_enum k230_uart_read_byte (uint8 *data)
{
    uint16 tail;

    if (0 == data)
    {
        return K230_UART_STATUS_INVALID_ARGUMENT;
    }
    if (ZF_FALSE == k230_uart_initialized)
    {
        return K230_UART_STATUS_NOT_INITIALIZED;
    }

    tail = k230_uart_rx_tail;
    if (tail == k230_uart_rx_head)
    {
        return K230_UART_STATUS_RX_BUFFER_EMPTY;
    }

    *data = k230_uart_rx_buffer[tail];
    k230_uart_rx_tail = k230_uart_next_rx_index(tail);

    return K230_UART_STATUS_OK;
}

uint16 k230_uart_available (void)
{
    uint16 head = k230_uart_rx_head;
    uint16 tail = k230_uart_rx_tail;

    if (head >= tail)
    {
        return (uint16)(head - tail);
    }

    return (uint16)(
        K230_UART_RX_BUFFER_SIZE
        - (uint32)((uint16)(tail - head)));
}

void k230_uart_clear_rx_buffer (void)
{
    k230_uart_rx_tail = k230_uart_rx_head;
    k230_uart_frame_index = 0U;
}

uint32 k230_uart_get_rx_overflow_count (void)
{
    return k230_uart_rx_overflow_count;
}

k230_uart_status_enum k230_uart_read_frame (
    k230_uart_frame_struct *frame)
{
    uint8 data;
    uint8 index;

    if (0 == frame)
    {
        return K230_UART_STATUS_INVALID_ARGUMENT;
    }
    if (ZF_FALSE == k230_uart_initialized)
    {
        return K230_UART_STATUS_NOT_INITIALIZED;
    }

    /*
     * 状态 0：扫描 0x40 包头；状态 1..4：无条件收满固定长度。
     * 因此载荷中的 0x40/0x50 都按普通数据处理，避免提前截断合法载荷。
     */
    while (K230_UART_STATUS_OK == k230_uart_read_byte(&data))
    {
        if (0U == k230_uart_frame_index)
        {
            if (K230_UART_DATA_FRAME_HEAD == data)
            {
                k230_uart_frame_buffer[0] = data;
                k230_uart_frame_index = 1U;
            }
            continue;
        }

        k230_uart_frame_buffer[k230_uart_frame_index] = data;
        k230_uart_frame_index++;

        if (K230_UART_DATA_FRAME_SIZE > k230_uart_frame_index)
        {
            continue;
        }

        k230_uart_frame_index = 0U;
        if (K230_UART_DATA_FRAME_TAIL
            != k230_uart_frame_buffer[K230_UART_DATA_FRAME_SIZE - 1U])
        {
            /*
             * 协议无 CRC，包尾是唯一完整性检查。坏帧末字节若恰为新包头，
             * 立即把它复用为下一帧起点；否则回到扫描状态。
             */
            if (0xFFFFFFFFU != k230_uart_frame_error_count)
            {
                k230_uart_frame_error_count++;
            }

            if (K230_UART_DATA_FRAME_HEAD == data)
            {
                k230_uart_frame_buffer[0] = data;
                k230_uart_frame_index = 1U;
            }
            continue;
        }

        frame->flag = k230_uart_frame_buffer[1];
        for (index = 0U; index < K230_UART_DATA_SIZE; index++)
        {
            frame->data[index] = k230_uart_frame_buffer[index + 2U];
        }
        return K230_UART_STATUS_OK;
    }

    return K230_UART_STATUS_RX_BUFFER_EMPTY;
}

uint8 k230_uart_decode_ball_x (
    const k230_uart_frame_struct *frame,
    uint16 *ball_x)
{
    return k230_uart_decode_x(
        frame,
        K230_UART_FLAG_BALL_POSITION,
        ball_x);
}

uint8 k230_uart_decode_touch_x (
    const k230_uart_frame_struct *frame,
    uint16 *touch_x)
{
    return k230_uart_decode_x(
        frame,
        K230_UART_FLAG_TOUCH_X,
        touch_x);
}

uint32 k230_uart_get_frame_error_count (void)
{
    return k230_uart_frame_error_count;
}

void k230_uart_rx_irq_handler (void)
{
    uint8 data;
    uint16 head;
    uint16 next_head;

    if ((ZF_FALSE == k230_uart_initialized)
        || (0U == uart_query_byte(K230_UART_INDEX, &data)))
    {
        return;
    }

    head = k230_uart_rx_head;
    next_head = k230_uart_next_rx_index(head);
    if (next_head == k230_uart_rx_tail)
    {
        /* 缓冲区满时丢弃本次新字节，避免覆盖前台尚未消费的旧数据。 */
        if (0xFFFFFFFFU != k230_uart_rx_overflow_count)
        {
            k230_uart_rx_overflow_count++;
        }
        return;
    }

    k230_uart_rx_buffer[head] = data;
    k230_uart_rx_head = next_head;
}
