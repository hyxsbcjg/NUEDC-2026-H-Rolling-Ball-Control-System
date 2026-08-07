#include "my_uart.h"

#include <stdarg.h>
#include <stdio.h>

#include "zf_device_wireless_uart.h"

/*
 * 两个状态只由 CPU0 前台访问，显式放入 cpu0_dsram。ready 表示最近一次
 * wireless_uart_init() 的返回值，不是远端在线状态。
 */
#pragma section all "cpu0_dsram"
static uint8 my_uart_initialized = ZF_FALSE;
static uint32 my_uart_last_unsent_length = 0U;
#pragma section all restore

my_uart_status_enum my_uart_init (void)
{
    my_uart_status_enum status = MY_UART_STATUS_INIT_FAILED;

    my_uart_initialized = ZF_FALSE;
    my_uart_last_unsent_length = 0U;

    /* 底层同时配置 UART2、RTS、RX FIFO、RX 中断和自动波特率握手。 */
    if (0U == wireless_uart_init())
    {
        my_uart_initialized = ZF_TRUE;
        status = MY_UART_STATUS_OK;
    }

    return status;
}

uint8 my_uart_is_ready (void)
{
    return my_uart_initialized;
}

my_uart_status_enum my_printf (const char *format, ...)
{
    char tx_buffer[MY_PRINTF_BUFFER_SIZE];
    va_list args;
    int formatted_length;
    uint32 send_length;
    my_uart_status_enum status = MY_UART_STATUS_OK;

    my_uart_last_unsent_length = 0U;
    if (0 == format)
    {
        return MY_UART_STATUS_INVALID_ARGUMENT;
    }
    if (ZF_FALSE == my_uart_initialized)
    {
        return MY_UART_STATUS_NOT_INITIALIZED;
    }

    /*
     * vsnprintf() 返回“完整文本本应占用的长度”。返回值超过本地数组时，
     * 数组内仍是安全的截短字符串，发送长度必须限制为末尾 '\0' 之前。
     */
    va_start(args, format);
    formatted_length = vsnprintf(
        tx_buffer,
        sizeof(tx_buffer),
        format,
        args);
    va_end(args);

    if (0 > formatted_length)
    {
        return MY_UART_STATUS_FORMAT_ERROR;
    }

    send_length = (uint32)formatted_length;
    if (send_length >= (uint32)sizeof(tx_buffer))
    {
        send_length = (uint32)sizeof(tx_buffer) - 1U;
        status = MY_UART_STATUS_TRUNCATED;
    }

    if (0U != send_length)
    {
        /*
         * 底层按 RTS 分块阻塞发送并返回未发送余量。发送超时的优先级高于
         * TRUNCATED；调用者可再读 last_unsent_length 区分实际余量。
         */
        my_uart_last_unsent_length = wireless_uart_send_buffer(
            (const uint8 *)tx_buffer,
            send_length);
        if (0U != my_uart_last_unsent_length)
        {
            status = MY_UART_STATUS_SEND_TIMEOUT;
        }
    }

    return status;
}

uint32 my_uart_get_last_unsent_length (void)
{
    return my_uart_last_unsent_length;
}
