#ifndef MY_SCREEN_H_
#define MY_SCREEN_H_

#include "zf_common_font.h"
#include "zf_common_typedef.h"

/*
 * 固定显示配置：IPS200、硬件 SPI、横屏 320 x 240、8 x 16 ASCII 字体。
 * 文本坐标从 1 开始，与参考工程的应用层约定一致。
 */
#define MY_SCREEN_WIDTH_PIXELS                 (320U)
#define MY_SCREEN_HEIGHT_PIXELS                (240U)
#define MY_SCREEN_FONT_WIDTH_PIXELS            (8U)
#define MY_SCREEN_FONT_HEIGHT_PIXELS           (16U)
#define MY_SCREEN_COLUMN_COUNT                 (40U)
#define MY_SCREEN_ROW_COUNT                    (15U)
#define MY_SCREEN_FORMAT_BUFFER_SIZE           (128U)
#define MY_SCREEN_DEFAULT_PEN_COLOR            ((uint16)RGB565_RED)
#define MY_SCREEN_DEFAULT_BACKGROUND_COLOR     ((uint16)RGB565_BLACK)

#if (MY_SCREEN_COLUMN_COUNT != (MY_SCREEN_WIDTH_PIXELS / MY_SCREEN_FONT_WIDTH_PIXELS))
#error "MY_SCREEN_COLUMN_COUNT does not match the fixed IPS200 layout"
#endif

#if (MY_SCREEN_ROW_COUNT != (MY_SCREEN_HEIGHT_PIXELS / MY_SCREEN_FONT_HEIGHT_PIXELS))
#error "MY_SCREEN_ROW_COUNT does not match the fixed IPS200 layout"
#endif

#if (MY_SCREEN_FORMAT_BUFFER_SIZE < (MY_SCREEN_COLUMN_COUNT + 1U))
#error "MY_SCREEN_FORMAT_BUFFER_SIZE is too small"
#endif

typedef enum
{
    MY_SCREEN_STATUS_OK = 0,
    MY_SCREEN_STATUS_INVALID_ARGUMENT,
    MY_SCREEN_STATUS_NOT_INITIALIZED,
    MY_SCREEN_STATUS_OUT_OF_RANGE,
    MY_SCREEN_STATUS_FORMAT_ERROR,
    MY_SCREEN_STATUS_TRUNCATED
} my_screen_status_enum;

/*
 * 以硬件 SPI 横屏模式初始化固定的 IPS200。底层驱动没有身份或回读结果，
 * 因此 OK 仅表示初始化序列执行完成，不能证明屏幕真实在线。
 */
my_screen_status_enum my_screen_init (void);
uint8                my_screen_is_ready (void);

/*
 * 所有绘图调用都会阻塞。只能由一个 CPU0 前台/服务所有者调用，不能在 ISR
 * 或高频控制环中调用。
 */
my_screen_status_enum my_screen_clear (void);
my_screen_status_enum my_screen_clear_line (uint8 row);
my_screen_status_enum my_screen_set_color (uint16 pen_color, uint16 background_color);

/*
 * 列范围 1..40，行范围 1..15。超出右边界的文本会被截断，并返回
 * MY_SCREEN_STATUS_TRUNCATED，而不会触发底层驱动的断言。
 */
my_screen_status_enum my_screen_show_string (
    uint8 column,
    uint8 row,
    const char *text);

/*
 * 安全格式化到有界缓冲区，再绘制到从 1 开始的文本单元格；不追加换行符。
 */
my_screen_status_enum my_screen_printf (
    uint8 column,
    uint8 row,
    const char *format,
    ...);

/*
 * 从第 1 列开始格式化，并用空格填满该行剩余区域。更新动态数值时优先使用
 * 此接口，较短的新内容不会在屏幕上留下旧字符。
 */
my_screen_status_enum my_screen_show_line (
    uint8 row,
    const char *format,
    ...);

/*
 * 将经过的毫秒数显示为 "TIME:<秒>.<百分之一秒>s"。显示时舍弃不足百分之一
 * 秒的余数并清空所选行；计时数据仍由调用者负责维护。
 */
my_screen_status_enum my_screen_show_elapsed_time (
    uint8 row,
    uint32 elapsed_ms);

#endif
