#ifndef GW_GRAY_SENSOR_H_
#define GW_GRAY_SENSOR_H_

#include "zf_common_typedef.h"
#include "zf_driver_gpio.h"

/*
 * 地址宏均为 7 位地址。逐飞 Soft IIC 在发出地址字节时才左移一位并拼接
 * R/W 位，因此这里不能填写参考例程中常见的 8 位地址（0x4C << 1）。
 * 模块拨码可能产生 0x4C～0x4F，初始化会逐个用 PING 握手确认。
 */
#define GW_GRAY_IIC_ADDRESS                     (0x4CU)
#define GW_GRAY_IIC_ADDRESS_FIRST               (0x4CU)
#define GW_GRAY_IIC_ADDRESS_LAST                (0x4FU)
#define GW_GRAY_SOFT_IIC_SCL_PIN                (P10_1)
#define GW_GRAY_SOFT_IIC_SDA_PIN                (P10_3)
#define GW_GRAY_SOFT_IIC_DELAY                  (100U)

/*
 * wait_ready 最多执行 100 轮探测，仅在相邻轮次间忙等待 10 ms，共 99 个
 * 间隔（约 990 ms）；该时长尚未计入每轮最多四个地址的位操作时间。
 */
#define GW_GRAY_PING_RETRY_COUNT                (100U)
#define GW_GRAY_PING_RETRY_INTERVAL_MS          (10U)

/*
 * 通道掩码 bit0～bit7 对应 CH1～CH8。数字量中 0 表示黑线、1 表示白底；
 * 模拟量接口返回传感器的原始字节，不沿用数字量的 0/1 含义。
 */
#define GW_GRAY_CHANNEL_COUNT                   (8U)
#define GW_GRAY_CHANNEL_1                       (0x01U)
#define GW_GRAY_CHANNEL_2                       (0x02U)
#define GW_GRAY_CHANNEL_3                       (0x04U)
#define GW_GRAY_CHANNEL_4                       (0x08U)
#define GW_GRAY_CHANNEL_5                       (0x10U)
#define GW_GRAY_CHANNEL_6                       (0x20U)
#define GW_GRAY_CHANNEL_7                       (0x40U)
#define GW_GRAY_CHANNEL_8                       (0x80U)
#define GW_GRAY_CHANNEL_ALL                     (0xFFU)

#define GW_GRAY_ERROR_OVEREXPOSURE              (0x01U)
#define GW_GRAY_ERROR_BUTTON                    (0x02U)

/* 固件字节的高、低半字节分别表示主、次版本，例如 0x36 表示 V3.6。 */
#define GW_GRAY_FIRMWARE_MAJOR(firmware)        ((uint8)(((firmware) >> 4U) & 0x0FU))
#define GW_GRAY_FIRMWARE_MINOR(firmware)        ((uint8)((firmware) & 0x0FU))

typedef enum
{
    GW_GRAY_STATUS_OK = 0,
    GW_GRAY_STATUS_INVALID_ARGUMENT,
    GW_GRAY_STATUS_NOT_INITIALIZED,
    GW_GRAY_STATUS_NOT_READY,
    GW_GRAY_STATUS_PING_TIMEOUT,
    GW_GRAY_STATUS_NO_CHANNEL_ENABLED,
    GW_GRAY_STATUS_UNSUPPORTED_FIRMWARE,
    GW_GRAY_STATUS_VERIFY_FAILED
} gw_gray_status_enum;

/*
 * 固定使用 P10.1/P10.3 和 Soft IIC 延时 100。两线均按开漏使用，必须依赖
 * 外部上拉；仅当核心板 VEXT 域允许 5 V 时，才可接模块板载的 5 V 上拉。
 * 扫描 0x4C～0x4F 后，只保留对命令 0xAA 返回 0x66 的地址。
 */
gw_gray_status_enum gw_gray_init (void);

uint8               gw_gray_is_ready (void);
uint8               gw_gray_get_iic_address (void);
uint8               gw_gray_ping (void);
gw_gray_status_enum gw_gray_wait_ready (void);

gw_gray_status_enum gw_gray_read_digital (uint8 *digital_value);
gw_gray_status_enum gw_gray_read_analog_channel (uint8 channel, uint8 *analog_value);

/*
 * 0xB0 只按通道号顺序返回“已使能通道”的紧凑数据。本接口将其展开为固定
 * 8 项：禁用通道填 0，valid_mask 标记数据映射到哪些通道，但不代表 ACK
 * 或在线校验通过；物理通信有效性仍受下述 Soft IIC 返回能力限制。
 */
gw_gray_status_enum gw_gray_read_analog (
    uint8 analog_values[GW_GRAY_CHANNEL_COUNT],
    uint8 *valid_mask);

gw_gray_status_enum gw_gray_set_channel_enable (uint8 channel_mask);
gw_gray_status_enum gw_gray_get_channel_enable (uint8 *channel_mask);

/* 归一化命令仅对 V3.6 及以上固件开放，判断依据是初始化时缓存的版本。 */
gw_gray_status_enum gw_gray_set_normalize (uint8 channel_mask);
gw_gray_status_enum gw_gray_get_normalize (uint8 *channel_mask);

gw_gray_status_enum gw_gray_read_firmware (uint8 *firmware);
/* 0xDE 是读后清除寄存器；读取会消费此前保存的错误位。 */
gw_gray_status_enum gw_gray_read_and_clear_error (uint8 *error_value);
gw_gray_status_enum gw_gray_reboot (void);

/*
 * 逐飞 Soft IIC 虽在内部采样 ACK，却不通过这些公开读写接口返回 ACK。
 * GW_GRAY_STATUS_OK 只表示参数、驱动状态合法且事务已发出；需要确认设备在线
 * 时必须调用 gw_gray_ping()，并以 0x66 响应为准。
 *
 * 所有接口都是同步 Soft IIC 事务。运行期固定长度的数字量读取由 3 ms 调度
 * ISR 每拍唯一调用；PING、地址扫描、配置、模拟量读取和重启等非周期操作
 * 只允许在 PIT 启动前或显式停用调度 ISR 后执行。CPU0 前台不得与该 ISR
 * 并发访问同一个 Soft IIC 对象。
 */

#endif
