#include "gw_gray_sensor.h"

#include "zf_driver_delay.h"
#include "zf_driver_soft_iic.h"

#define GW_GRAY_COMMAND_PING                    (0xAAU)
#define GW_GRAY_PING_RESPONSE                   (0x66U)
#define GW_GRAY_COMMAND_ANALOG_ALL              (0xB0U)
#define GW_GRAY_COMMAND_ANALOG_FIRST            (0xB1U)
#define GW_GRAY_COMMAND_REBOOT                  (0xC0U)
#define GW_GRAY_COMMAND_FIRMWARE                (0xC1U)
#define GW_GRAY_COMMAND_CHANNEL_ENABLE          (0xCEU)
#define GW_GRAY_COMMAND_NORMALIZE               (0xCFU)
#define GW_GRAY_COMMAND_DIGITAL                 (0xDDU)
#define GW_GRAY_COMMAND_ERROR                   (0xDEU)

#define GW_GRAY_NORMALIZE_MIN_MAJOR             (3U)
#define GW_GRAY_NORMALIZE_MIN_MINOR             (6U)

#pragma section all "cpu0_dsram"
/*
 * bus_initialized 只表示 GPIO/Soft IIC 对象已配置；device_ready 还要求设备
 * 完成 PING 握手。firmware 是握手成功后读取的版本缓存，供归一化门槛判断。
 */
static soft_iic_info_struct gw_gray_iic;
static uint8 gw_gray_bus_initialized = ZF_FALSE;
static uint8 gw_gray_device_ready = ZF_FALSE;
static uint8 gw_gray_firmware = 0U;
#pragma section all restore

static gw_gray_status_enum gw_gray_require_ready (void)
{
    gw_gray_status_enum status = GW_GRAY_STATUS_OK;

    if (ZF_FALSE == gw_gray_bus_initialized)
    {
        status = GW_GRAY_STATUS_NOT_INITIALIZED;
    }
    else if (ZF_FALSE == gw_gray_device_ready)
    {
        status = GW_GRAY_STATUS_NOT_READY;
    }

    return status;
}

static uint8 gw_gray_firmware_supports_normalize (uint8 firmware)
{
    uint8 major = GW_GRAY_FIRMWARE_MAJOR(firmware);
    uint8 minor = GW_GRAY_FIRMWARE_MINOR(firmware);
    uint8 supported = ZF_FALSE;

    /* 版本比较按主版本优先，避免把 0x40 等后续版本误判为低于 V3.6。 */
    if ((major > GW_GRAY_NORMALIZE_MIN_MAJOR)
        || ((major == GW_GRAY_NORMALIZE_MIN_MAJOR)
            && (minor >= GW_GRAY_NORMALIZE_MIN_MINOR)))
    {
        supported = ZF_TRUE;
    }

    return supported;
}

static uint8 gw_gray_count_enabled_channels (uint8 channel_mask)
{
    uint8 channel;
    uint8 count = 0U;

    for (channel = 0U; channel < GW_GRAY_CHANNEL_COUNT; channel++)
    {
        if (0U != (channel_mask & (uint8)(1U << channel)))
        {
            count++;
        }
    }

    return count;
}

static uint8 gw_gray_ping_address (uint8 address)
{
    uint8 response;

    /*
     * 0xAA 在逐飞“读 8 位寄存器”事务中充当命令字；只有随后读回 0x66
     * 才证明该 7 位地址上是目标设备，不能用普通读写的状态代替。
     */
    gw_gray_iic.addr = address;
    response = soft_iic_read_8bit_register(&gw_gray_iic, GW_GRAY_COMMAND_PING);

    return (GW_GRAY_PING_RESPONSE == response) ? ZF_TRUE : ZF_FALSE;
}

static gw_gray_status_enum gw_gray_read_register (uint8 command, uint8 *data)
{
    gw_gray_status_enum status;

    if (0 == data)
    {
        return GW_GRAY_STATUS_INVALID_ARGUMENT;
    }

    status = gw_gray_require_ready();
    if (GW_GRAY_STATUS_OK == status)
    {
        /* 下层不上传 ACK；此处的 OK 不等价于本次物理传输成功。 */
        *data = soft_iic_read_8bit_register(&gw_gray_iic, command);
    }

    return status;
}

gw_gray_status_enum gw_gray_init (void)
{
    gw_gray_status_enum status;

    soft_iic_init(
        &gw_gray_iic,
        GW_GRAY_IIC_ADDRESS,
        GW_GRAY_SOFT_IIC_DELAY,
        GW_GRAY_SOFT_IIC_SCL_PIN,
        GW_GRAY_SOFT_IIC_SDA_PIN);

    /*
     * soft_iic_init 默认把 SCL 配为推挽，随即改为开漏；逻辑高表示释放总线，
     * 由模块上拉决定高电平。模块上拉为 5 V 时，核心板 VEXT 也必须允许 5 V。
     */
    gpio_set_dir(GW_GRAY_SOFT_IIC_SCL_PIN, GPO, GPO_OPEN_DTAIN);

    gw_gray_bus_initialized = ZF_TRUE;
    gw_gray_device_ready = ZF_FALSE;
    gw_gray_firmware = 0U;

    status = gw_gray_wait_ready();

    if (GW_GRAY_STATUS_OK == status)
    {
        status = gw_gray_read_firmware(&gw_gray_firmware);
    }

    return status;
}

uint8 gw_gray_is_ready (void)
{
    return gw_gray_device_ready;
}

uint8 gw_gray_get_iic_address (void)
{
    uint8 address = 0U;

    if (ZF_TRUE == gw_gray_device_ready)
    {
        address = gw_gray_iic.addr;
    }

    return address;
}

uint8 gw_gray_ping (void)
{
    uint8 current_address;
    uint8 candidate_address;

    if (ZF_FALSE == gw_gray_bus_initialized)
    {
        return ZF_FALSE;
    }

    current_address = gw_gray_iic.addr;
    if (ZF_TRUE == gw_gray_ping_address(current_address))
    {
        gw_gray_device_ready = ZF_TRUE;
        return ZF_TRUE;
    }

    for (candidate_address = GW_GRAY_IIC_ADDRESS_FIRST;
         candidate_address <= GW_GRAY_IIC_ADDRESS_LAST;
         candidate_address++)
    {
        if ((candidate_address != current_address)
            && (ZF_TRUE == gw_gray_ping_address(candidate_address)))
        {
            gw_gray_device_ready = ZF_TRUE;
            return ZF_TRUE;
        }
    }

    /*
     * gw_gray_ping_address 会临时改写对象地址。全范围均失败时恢复原地址；
     * 命中时则保留候选地址，后续所有事务自动使用该地址。
     */
    gw_gray_iic.addr = current_address;
    gw_gray_device_ready = ZF_FALSE;
    return gw_gray_device_ready;
}

gw_gray_status_enum gw_gray_wait_ready (void)
{
    uint16 attempt;

    if (ZF_FALSE == gw_gray_bus_initialized)
    {
        return GW_GRAY_STATUS_NOT_INITIALIZED;
    }

    /*
     * 参考例程会无限等待。本循环限定重试次数，并且最后一次失败后不再额外
     * 延时；system_delay_ms 是忙等待，只适合 PIT 启动前初始化或显式维护
     * 窗口，不能进入 ISR。
     */
    for (attempt = 0U; attempt < GW_GRAY_PING_RETRY_COUNT; attempt++)
    {
        if (ZF_TRUE == gw_gray_ping())
        {
            return GW_GRAY_STATUS_OK;
        }

        if ((0U != GW_GRAY_PING_RETRY_INTERVAL_MS)
            && ((attempt + 1U) < GW_GRAY_PING_RETRY_COUNT))
        {
            system_delay_ms(GW_GRAY_PING_RETRY_INTERVAL_MS);
        }
    }

    return GW_GRAY_STATUS_PING_TIMEOUT;
}

gw_gray_status_enum gw_gray_read_digital (uint8 *digital_value)
{
    /* 返回一个通道位图：bit0=CH1；0=黑线，1=白底。 */
    return gw_gray_read_register(GW_GRAY_COMMAND_DIGITAL, digital_value);
}

gw_gray_status_enum gw_gray_read_analog_channel (uint8 channel, uint8 *analog_value)
{
    if ((0U == channel) || (channel > GW_GRAY_CHANNEL_COUNT))
    {
        return GW_GRAY_STATUS_INVALID_ARGUMENT;
    }

    return gw_gray_read_register(
        (uint8)(GW_GRAY_COMMAND_ANALOG_FIRST + channel - 1U),
        analog_value);
}

gw_gray_status_enum gw_gray_read_analog (
    uint8 analog_values[GW_GRAY_CHANNEL_COUNT],
    uint8 *valid_mask)
{
    gw_gray_status_enum status;
    uint8 packed_values[GW_GRAY_CHANNEL_COUNT];
    uint8 channel_mask;
    uint8 enabled_count;
    uint8 channel;
    uint8 packed_index = 0U;

    if ((0 == analog_values) || (0 == valid_mask))
    {
        return GW_GRAY_STATUS_INVALID_ARGUMENT;
    }

    *valid_mask = 0U;
    for (channel = 0U; channel < GW_GRAY_CHANNEL_COUNT; channel++)
    {
        analog_values[channel] = 0U;
        packed_values[channel] = 0U;
    }

    status = gw_gray_get_channel_enable(&channel_mask);
    if (GW_GRAY_STATUS_OK != status)
    {
        return status;
    }

    enabled_count = gw_gray_count_enabled_channels(channel_mask);
    if (0U == enabled_count)
    {
        return GW_GRAY_STATUS_NO_CHANNEL_ENABLED;
    }

    soft_iic_read_8bit_registers(
        &gw_gray_iic,
        GW_GRAY_COMMAND_ANALOG_ALL,
        packed_values,
        enabled_count);

    /*
     * 模块不会为禁用通道占用返回字节；packed_index 只在掩码命中时递增，
     * 从而把紧凑序列恢复到 CH1～CH8 的固定下标。
     */
    for (channel = 0U; channel < GW_GRAY_CHANNEL_COUNT; channel++)
    {
        if (0U != (channel_mask & (uint8)(1U << channel)))
        {
            analog_values[channel] = packed_values[packed_index];
            packed_index++;
        }
    }

    *valid_mask = channel_mask;
    return GW_GRAY_STATUS_OK;
}

gw_gray_status_enum gw_gray_set_channel_enable (uint8 channel_mask)
{
    gw_gray_status_enum status;
    uint8 readback = 0U;

    status = gw_gray_require_ready();
    if (GW_GRAY_STATUS_OK != status)
    {
        return status;
    }

    soft_iic_write_8bit_register(
        &gw_gray_iic,
        GW_GRAY_COMMAND_CHANNEL_ENABLE,
        channel_mask);

    /* 下层无 ACK 状态，只能立即回读配置；值不一致时明确报告校验失败。 */
    status = gw_gray_get_channel_enable(&readback);
    if ((GW_GRAY_STATUS_OK == status) && (readback != channel_mask))
    {
        status = GW_GRAY_STATUS_VERIFY_FAILED;
    }

    return status;
}

gw_gray_status_enum gw_gray_get_channel_enable (uint8 *channel_mask)
{
    return gw_gray_read_register(GW_GRAY_COMMAND_CHANNEL_ENABLE, channel_mask);
}

gw_gray_status_enum gw_gray_set_normalize (uint8 channel_mask)
{
    gw_gray_status_enum status;
    uint8 readback = 0U;

    status = gw_gray_require_ready();
    if (GW_GRAY_STATUS_OK != status)
    {
        return status;
    }

    if (ZF_FALSE == gw_gray_firmware_supports_normalize(gw_gray_firmware))
    {
        return GW_GRAY_STATUS_UNSUPPORTED_FIRMWARE;
    }

    soft_iic_write_8bit_register(
        &gw_gray_iic,
        GW_GRAY_COMMAND_NORMALIZE,
        channel_mask);

    /* 归一化掩码同样采用写后回读，不能仅凭写函数返回判断配置已生效。 */
    status = gw_gray_get_normalize(&readback);
    if ((GW_GRAY_STATUS_OK == status) && (readback != channel_mask))
    {
        status = GW_GRAY_STATUS_VERIFY_FAILED;
    }

    return status;
}

gw_gray_status_enum gw_gray_get_normalize (uint8 *channel_mask)
{
    gw_gray_status_enum status;

    if (0 == channel_mask)
    {
        return GW_GRAY_STATUS_INVALID_ARGUMENT;
    }

    status = gw_gray_require_ready();
    if (GW_GRAY_STATUS_OK != status)
    {
        return status;
    }

    if (ZF_FALSE == gw_gray_firmware_supports_normalize(gw_gray_firmware))
    {
        return GW_GRAY_STATUS_UNSUPPORTED_FIRMWARE;
    }

    *channel_mask = soft_iic_read_8bit_register(
        &gw_gray_iic,
        GW_GRAY_COMMAND_NORMALIZE);

    return GW_GRAY_STATUS_OK;
}

gw_gray_status_enum gw_gray_read_firmware (uint8 *firmware)
{
    gw_gray_status_enum status;

    status = gw_gray_read_register(GW_GRAY_COMMAND_FIRMWARE, firmware);
    if (GW_GRAY_STATUS_OK == status)
    {
        gw_gray_firmware = *firmware;
    }

    return status;
}

gw_gray_status_enum gw_gray_read_and_clear_error (uint8 *error_value)
{
    return gw_gray_read_register(GW_GRAY_COMMAND_ERROR, error_value);
}

gw_gray_status_enum gw_gray_reboot (void)
{
    gw_gray_status_enum status;

    status = gw_gray_require_ready();
    if (GW_GRAY_STATUS_OK != status)
    {
        return status;
    }

    soft_iic_write_8bit(&gw_gray_iic, GW_GRAY_COMMAND_REBOOT);
    gw_gray_device_ready = ZF_FALSE;
    gw_gray_firmware = 0U;

    /* 重启后重新执行有限 PING 窗口，并刷新固件缓存。整个过程为阻塞调用。 */
    status = gw_gray_wait_ready();

    if (GW_GRAY_STATUS_OK == status)
    {
        status = gw_gray_read_firmware(&gw_gray_firmware);
    }

    return status;
}
