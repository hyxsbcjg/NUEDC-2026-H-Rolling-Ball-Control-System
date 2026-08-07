#include "my_screen.h"

#include <stdarg.h>
#include <stdio.h>
#include <string.h>

#include "zf_device_ips200.h"

#pragma section all "cpu0_dsram"
static uint8 my_screen_initialized = ZF_FALSE;
#pragma section all restore

static my_screen_status_enum my_screen_validate_cell (uint8 column, uint8 row);
static my_screen_status_enum my_screen_draw_text (
    uint8 column,
    uint8 row,
    const char *text);
static my_screen_status_enum my_screen_vprintf (
    uint8 column,
    uint8 row,
    const char *format,
    va_list args);
static my_screen_status_enum my_screen_vshow_line (
    uint8 row,
    const char *format,
    va_list args);

my_screen_status_enum my_screen_init (void)
{
    my_screen_initialized = ZF_FALSE;

    /*
     * TC264 的 IPS200 驱动会在 ips200_init() 中读取显示方向，因此必须先
     * 选择方向。屏幕与传输方式固定为 IPS200 和硬件 SPI2。
     */
    ips200_set_dir(IPS200_CROSSWISE);
    ips200_set_font(IPS200_8X16_FONT);
    ips200_set_color(
        MY_SCREEN_DEFAULT_PEN_COLOR,
        MY_SCREEN_DEFAULT_BACKGROUND_COLOR);
    ips200_init(IPS200_TYPE_SPI);

    my_screen_initialized = ZF_TRUE;
    return MY_SCREEN_STATUS_OK;
}

uint8 my_screen_is_ready (void)
{
    return my_screen_initialized;
}

my_screen_status_enum my_screen_clear (void)
{
    if (ZF_FALSE == my_screen_initialized)
    {
        return MY_SCREEN_STATUS_NOT_INITIALIZED;
    }

    ips200_clear();
    return MY_SCREEN_STATUS_OK;
}

my_screen_status_enum my_screen_clear_line (uint8 row)
{
    char line_buffer[MY_SCREEN_COLUMN_COUNT + 1U];

    if (ZF_FALSE == my_screen_initialized)
    {
        return MY_SCREEN_STATUS_NOT_INITIALIZED;
    }
    if ((0U == row) || (MY_SCREEN_ROW_COUNT < row))
    {
        return MY_SCREEN_STATUS_OUT_OF_RANGE;
    }

    /* IPS200 没有“清文本行”原语，用一整行空格覆盖旧像素。 */
    memset(line_buffer, ' ', MY_SCREEN_COLUMN_COUNT);
    line_buffer[MY_SCREEN_COLUMN_COUNT] = '\0';
    ips200_show_string(
        0U,
        (uint16)((uint16)(row - 1U) * MY_SCREEN_FONT_HEIGHT_PIXELS),
        line_buffer);

    return MY_SCREEN_STATUS_OK;
}

my_screen_status_enum my_screen_set_color (
    uint16 pen_color,
    uint16 background_color)
{
    if (ZF_FALSE == my_screen_initialized)
    {
        return MY_SCREEN_STATUS_NOT_INITIALIZED;
    }

    ips200_set_color(pen_color, background_color);
    return MY_SCREEN_STATUS_OK;
}

my_screen_status_enum my_screen_show_string (
    uint8 column,
    uint8 row,
    const char *text)
{
    if (0 == text)
    {
        return MY_SCREEN_STATUS_INVALID_ARGUMENT;
    }

    return my_screen_draw_text(column, row, text);
}

my_screen_status_enum my_screen_printf (
    uint8 column,
    uint8 row,
    const char *format,
    ...)
{
    my_screen_status_enum status;
    va_list args;

    if (0 == format)
    {
        return MY_SCREEN_STATUS_INVALID_ARGUMENT;
    }

    va_start(args, format);
    status = my_screen_vprintf(column, row, format, args);
    va_end(args);

    return status;
}

my_screen_status_enum my_screen_show_line (
    uint8 row,
    const char *format,
    ...)
{
    my_screen_status_enum status;
    va_list args;

    if (0 == format)
    {
        return MY_SCREEN_STATUS_INVALID_ARGUMENT;
    }

    va_start(args, format);
    status = my_screen_vshow_line(row, format, args);
    va_end(args);

    return status;
}

my_screen_status_enum my_screen_show_elapsed_time (
    uint8 row,
    uint32 elapsed_ms)
{
    uint32 total_centiseconds = elapsed_ms / 10U;
    uint32 seconds = total_centiseconds / 100U;
    uint32 centiseconds = total_centiseconds % 100U;

    return my_screen_show_line(
        row,
        "TIME:%lu.%02lus",
        (unsigned long)seconds,
        (unsigned long)centiseconds);
}

static my_screen_status_enum my_screen_validate_cell (
    uint8 column,
    uint8 row)
{
    if (ZF_FALSE == my_screen_initialized)
    {
        return MY_SCREEN_STATUS_NOT_INITIALIZED;
    }
    if ((0U == column) || (MY_SCREEN_COLUMN_COUNT < column)
        || (0U == row) || (MY_SCREEN_ROW_COUNT < row))
    {
        return MY_SCREEN_STATUS_OUT_OF_RANGE;
    }

    return MY_SCREEN_STATUS_OK;
}

static my_screen_status_enum my_screen_draw_text (
    uint8 column,
    uint8 row,
    const char *text)
{
    char display_buffer[MY_SCREEN_COLUMN_COUNT + 1U];
    size_t text_length;
    size_t display_length;
    size_t remaining_columns;
    my_screen_status_enum status;

    status = my_screen_validate_cell(column, row);
    if (MY_SCREEN_STATUS_OK != status)
    {
        return status;
    }
    if (0 == text)
    {
        return MY_SCREEN_STATUS_INVALID_ARGUMENT;
    }

    text_length = strlen(text);
    /*
     * 应用坐标从 1 开始，底层像素坐标从 0 开始。先按剩余列裁剪，既防止
     * 越过 320 像素，也避免把超长字符串交给底层断言路径。
     */
    remaining_columns =
        (size_t)MY_SCREEN_COLUMN_COUNT - (size_t)column + 1U;
    display_length = text_length;
    status = MY_SCREEN_STATUS_OK;

    if (remaining_columns < display_length)
    {
        display_length = remaining_columns;
        status = MY_SCREEN_STATUS_TRUNCATED;
    }

    if (0U != display_length)
    {
        memcpy(display_buffer, text, display_length);
        display_buffer[display_length] = '\0';
        /* 该底层调用通过硬件 SPI2 同步绘制，只允许 CPU0 前台单一所有者。 */
        ips200_show_string(
            (uint16)((uint16)(column - 1U) * MY_SCREEN_FONT_WIDTH_PIXELS),
            (uint16)((uint16)(row - 1U) * MY_SCREEN_FONT_HEIGHT_PIXELS),
            display_buffer);
    }

    return status;
}

static my_screen_status_enum my_screen_vprintf (
    uint8 column,
    uint8 row,
    const char *format,
    va_list args)
{
    char format_buffer[MY_SCREEN_FORMAT_BUFFER_SIZE];
    int formatted_length;
    my_screen_status_enum status;
    my_screen_status_enum draw_status;

    status = my_screen_validate_cell(column, row);
    if (MY_SCREEN_STATUS_OK != status)
    {
        return status;
    }

    /* 第一层边界保护格式化缓冲，第二层由 draw_text 按屏幕剩余列裁剪。 */
    formatted_length = vsnprintf(
        format_buffer,
        sizeof(format_buffer),
        format,
        args);
    if (0 > formatted_length)
    {
        return MY_SCREEN_STATUS_FORMAT_ERROR;
    }

    status = MY_SCREEN_STATUS_OK;
    if ((size_t)formatted_length >= sizeof(format_buffer))
    {
        status = MY_SCREEN_STATUS_TRUNCATED;
    }

    draw_status = my_screen_draw_text(column, row, format_buffer);
    if (MY_SCREEN_STATUS_OK != draw_status)
    {
        status = draw_status;
    }

    return status;
}

static my_screen_status_enum my_screen_vshow_line (
    uint8 row,
    const char *format,
    va_list args)
{
    char format_buffer[MY_SCREEN_FORMAT_BUFFER_SIZE];
    char line_buffer[MY_SCREEN_COLUMN_COUNT + 1U];
    int formatted_length;
    size_t copy_length;
    my_screen_status_enum status;

    status = my_screen_validate_cell(1U, row);
    if (MY_SCREEN_STATUS_OK != status)
    {
        return status;
    }

    formatted_length = vsnprintf(
        format_buffer,
        sizeof(format_buffer),
        format,
        args);
    if (0 > formatted_length)
    {
        return MY_SCREEN_STATUS_FORMAT_ERROR;
    }

    /*
     * 最多复制 40 个字符，其余位置补空格；数值从多位变少时不会残留旧字。
     * MY_SCREEN_FORMAT_BUFFER_SIZE 的编译期约束保证这 40 字节始终已格式化。
     */
    copy_length = (size_t)formatted_length;
    status = MY_SCREEN_STATUS_OK;
    if (MY_SCREEN_COLUMN_COUNT < copy_length)
    {
        copy_length = MY_SCREEN_COLUMN_COUNT;
        status = MY_SCREEN_STATUS_TRUNCATED;
    }
    if ((size_t)formatted_length >= sizeof(format_buffer))
    {
        status = MY_SCREEN_STATUS_TRUNCATED;
    }

    memcpy(line_buffer, format_buffer, copy_length);
    memset(
        &line_buffer[copy_length],
        ' ',
        (size_t)MY_SCREEN_COLUMN_COUNT - copy_length);
    line_buffer[MY_SCREEN_COLUMN_COUNT] = '\0';
    ips200_show_string(
        0U,
        (uint16)((uint16)(row - 1U) * MY_SCREEN_FONT_HEIGHT_PIXELS),
        line_buffer);

    return status;
}
