#include "stepper_uart.h"

/*
 * Emm_V5 命令统一以地址开头、0x6B 结尾。部分配置命令还要求固定 key；
 * key 是协议字段，不是本地计算的校验和。下方 wrapper 按命令族集中组帧。
 */
#define STEPPER_UART_CMD_CLEAR_STALL              (0x0EU)
#define STEPPER_UART_CMD_SET_CONTROL_MODE         (0x46U)
#define STEPPER_UART_CMD_SET_ENABLE               (0xF3U)
#define STEPPER_UART_CMD_RUN_VELOCITY             (0xF6U)
#define STEPPER_UART_CMD_MOVE                     (0xFDU)
#define STEPPER_UART_CMD_SET_QUICK_MOVE           (0xF1U)
#define STEPPER_UART_CMD_QUICK_MOVE               (0xFCU)
#define STEPPER_UART_CMD_STOP                     (0xFEU)
#define STEPPER_UART_CMD_SYNCHRONOUS_MOTION       (0xFFU)
#define STEPPER_UART_CMD_START_HOME               (0x9AU)
#define STEPPER_UART_CMD_MULTI_MOTOR               (0xAAU)

/* 驱动器用这些固定 key 防止配置/状态命令被普通数据误触发。 */
#define STEPPER_UART_KEY_CLEAR_STALL              (0x52U)
#define STEPPER_UART_KEY_SET_CONTROL_MODE         (0x69U)
#define STEPPER_UART_KEY_SET_ENABLE               (0xABU)
#define STEPPER_UART_KEY_STOP                     (0x98U)
#define STEPPER_UART_KEY_SYNCHRONOUS_MOTION       (0x66U)

#define STEPPER_UART_VELOCITY_COMMAND_FRAME_SIZE  (8U)
#define STEPPER_UART_MOVE_COMMAND_FRAME_SIZE      (13U)
#define STEPPER_UART_HOME_COMMAND_FRAME_SIZE      (5U)
#define STEPPER_UART_MULTI_FRAME_HEADER_SIZE      (4U)
#define STEPPER_UART_MULTI_FRAME_OVERHEAD         (5U)
#define STEPPER_UART_MULTI_FRAME_MAX_SIZE         (512U)
#define STEPPER_UART_TX_FIFO_CAPACITY              (16U)

#if (STEPPER_UART_VELOCITY_COMMAND_FRAME_SIZE > STEPPER_UART_TX_FIFO_CAPACITY)
#error "velocity frame does not fit in UART3 TX FIFO"
#endif

#if (STEPPER_UART_MOVE_COMMAND_FRAME_SIZE > STEPPER_UART_TX_FIFO_CAPACITY)
#error "move frame does not fit in UART3 TX FIFO"
#endif

#if (STEPPER_UART_HOME_COMMAND_FRAME_SIZE > STEPPER_UART_TX_FIFO_CAPACITY)
#error "home frame does not fit in UART3 TX FIFO"
#endif

/* 初始化状态和 512 字节多机发送工作区只允许一个 CPU0 所有者使用。 */
#pragma section all "cpu0_dsram"
static volatile uint8 stepper_uart_initialized = ZF_FALSE;
static uint8 stepper_uart_multi_frame[STEPPER_UART_MULTI_FRAME_MAX_SIZE];
#pragma section all restore

static uint8 stepper_uart_is_boolean (uint8 value)
{
    return (uint8)(((ZF_FALSE == value) || (ZF_TRUE == value))
        ? ZF_TRUE
        : ZF_FALSE);
}

static uint8 stepper_uart_is_direction (
    stepper_uart_direction_enum direction)
{
    return (uint8)(((STEPPER_UART_DIRECTION_CW == direction)
        || (STEPPER_UART_DIRECTION_CCW == direction))
        ? ZF_TRUE
        : ZF_FALSE);
}

static uint8 stepper_uart_is_position_mode (
    stepper_uart_position_mode_enum position_mode)
{
    return (uint8)(((STEPPER_UART_POSITION_RELATIVE == position_mode)
        || (STEPPER_UART_POSITION_ABSOLUTE == position_mode)
        || (STEPPER_UART_POSITION_RELATIVE_CURRENT == position_mode))
        ? ZF_TRUE
        : ZF_FALSE);
}

static uint8 stepper_uart_is_control_mode (
    stepper_uart_control_mode_enum mode)
{
    return (uint8)(((STEPPER_UART_CONTROL_OPEN_LOOP == mode)
        || (STEPPER_UART_CONTROL_CLOSED_LOOP == mode))
        ? ZF_TRUE
        : ZF_FALSE);
}

static uint8 stepper_uart_is_legacy_pulse_mode (
    stepper_uart_legacy_pulse_mode_enum mode)
{
    return (uint8)(((STEPPER_UART_LEGACY_PULSE_DISABLED <= mode)
        && (STEPPER_UART_LEGACY_PULSE_CLOSED_LOOP >= mode))
        ? ZF_TRUE
        : ZF_FALSE);
}

/* Emm_V5 的所有多字节命令字段均按高字节在前写入。 */
static void stepper_uart_store_u16_be (
    uint8 *destination,
    uint16 value)
{
    destination[0] = (uint8)(value >> 8U);
    destination[1] = (uint8)value;
}

static void stepper_uart_store_u32_be (
    uint8 *destination,
    uint32 value)
{
    destination[0] = (uint8)(value >> 24U);
    destination[1] = (uint8)(value >> 16U);
    destination[2] = (uint8)(value >> 8U);
    destination[3] = (uint8)value;
}

static stepper_uart_status_enum stepper_uart_build_move_frame (
    uint8 *frame,
    uint8 address,
    stepper_uart_direction_enum direction,
    uint16 speed_rpm,
    uint8 acceleration,
    uint32 pulse_count,
    stepper_uart_position_mode_enum position_mode,
    uint8 synchronized)
{
    if ((0 == frame)
        || (ZF_FALSE == stepper_uart_is_direction(direction))
        || (STEPPER_UART_MAX_SPEED_RPM < speed_rpm)
        || (ZF_FALSE == stepper_uart_is_position_mode(position_mode))
        || (ZF_FALSE == stepper_uart_is_boolean(synchronized)))
    {
        return STEPPER_UART_STATUS_INVALID_ARGUMENT;
    }

    frame[0] = address;
    frame[1] = STEPPER_UART_CMD_MOVE;
    frame[2] = (uint8)direction;
    stepper_uart_store_u16_be(&frame[3], speed_rpm);
    frame[5] = acceleration;
    stepper_uart_store_u32_be(&frame[6], pulse_count);
    frame[10] = (uint8)position_mode;
    frame[11] = synchronized;
    frame[12] = STEPPER_UART_FRAME_CHECK_BYTE;

    return STEPPER_UART_STATUS_OK;
}

static stepper_uart_status_enum stepper_uart_try_write (
    const uint8 *data,
    uint32 length)
{
    uint32 index;
    uint8 fill_level;
    Ifx_ASCLIN *asclin;

    if ((0 == data)
        || (0U == length)
        || (STEPPER_UART_TX_FIFO_CAPACITY < length))
    {
        return STEPPER_UART_STATUS_INVALID_ARGUMENT;
    }
    if (ZF_FALSE == stepper_uart_initialized)
    {
        return STEPPER_UART_STATUS_NOT_INITIALIZED;
    }

    asclin = uart3_handle.asclin;
    if (0 == asclin)
    {
        return STEPPER_UART_STATUS_NOT_INITIALIZED;
    }

    fill_level = IfxAsclin_getTxFifoFillLevel(asclin);
    if (0U != fill_level)
    {
        return STEPPER_UART_STATUS_TX_BUSY;
    }

    /*
     * 空 FIFO 可容纳完整固定帧时直接写 TXDATA，不调用逐飞的阻塞式
     * uart_write_buffer()。当前 5 字节回零帧、8 字节速度帧和 13 字节位置帧
     * 都小于 16 字节 FIFO 容量。返回 OK 时数据可能仍在线路上发送；若 FIFO
     * 非空立即返回，不等待线路发送完成。
     */
    for (index = 0U; index < length; index++)
    {
        asclin->TXDATA.U = data[index];
    }

    return STEPPER_UART_STATUS_OK;
}

stepper_uart_status_enum stepper_uart_init (void)
{
    stepper_uart_initialized = ZF_FALSE;

    uart_init(
        STEPPER_UART_INDEX,
        STEPPER_UART_BAUD_RATE,
        STEPPER_UART_TX_PIN,
        STEPPER_UART_RX_PIN);

    uart_tx_interrupt(STEPPER_UART_INDEX, 0U);
    uart_rx_interrupt(STEPPER_UART_INDEX, 0U);
    /*
     * 当前项目只向驱动器发命令：P15.7 不接线，UART3 RX/TX 中断均关闭。
     * 本模块没有接收状态、回包解析器或遥测读取接口。
     */
    stepper_uart_initialized = ZF_TRUE;

    return STEPPER_UART_STATUS_OK;
}

uint8 stepper_uart_is_ready (void)
{
    return stepper_uart_initialized;
}

static stepper_uart_status_enum stepper_uart_write (
    const uint8 *data,
    uint32 length)
{
    if (0 == data)
    {
        return STEPPER_UART_STATUS_INVALID_ARGUMENT;
    }
    if (ZF_FALSE == stepper_uart_initialized)
    {
        return STEPPER_UART_STATUS_NOT_INITIALIZED;
    }

    if (0U != length)
    {
        /* 逐飞实现每写一个字节都等待 TX FIFO 清空，因此该路径会阻塞。 */
        uart_write_buffer(STEPPER_UART_INDEX, data, length);
    }

    return STEPPER_UART_STATUS_OK;
}

/*
 * 状态/配置命令族不提供任何写零或回零帧。set_control_mode 与
 * legacy_pulse_mode 的线格式相同：
 * [地址,46,69,是否写 Flash,模式,6B]，但不同固件对“模式”的语义不同。
 * save_to_flash=ZF_TRUE 会改变驱动器掉电保存配置。
 */
stepper_uart_status_enum stepper_uart_clear_stall (uint8 address)
{
    const uint8 frame[4] =
    {
        address,
        STEPPER_UART_CMD_CLEAR_STALL,
        STEPPER_UART_KEY_CLEAR_STALL,
        STEPPER_UART_FRAME_CHECK_BYTE
    };

    return stepper_uart_write(frame, (uint32)sizeof(frame));
}

stepper_uart_status_enum stepper_uart_set_control_mode (
    uint8 address,
    uint8 save_to_flash,
    stepper_uart_control_mode_enum mode)
{
    uint8 frame[6];

    if ((ZF_FALSE == stepper_uart_is_boolean(save_to_flash))
        || (ZF_FALSE == stepper_uart_is_control_mode(mode)))
    {
        return STEPPER_UART_STATUS_INVALID_ARGUMENT;
    }

    frame[0] = address;
    frame[1] = STEPPER_UART_CMD_SET_CONTROL_MODE;
    frame[2] = STEPPER_UART_KEY_SET_CONTROL_MODE;
    frame[3] = save_to_flash;
    frame[4] = (uint8)mode;
    frame[5] = STEPPER_UART_FRAME_CHECK_BYTE;

    return stepper_uart_write(frame, (uint32)sizeof(frame));
}

stepper_uart_status_enum stepper_uart_set_legacy_pulse_mode (
    uint8 address,
    uint8 save_to_flash,
    stepper_uart_legacy_pulse_mode_enum mode)
{
    uint8 frame[6];

    if ((ZF_FALSE == stepper_uart_is_boolean(save_to_flash))
        || (ZF_FALSE == stepper_uart_is_legacy_pulse_mode(mode)))
    {
        return STEPPER_UART_STATUS_INVALID_ARGUMENT;
    }

    frame[0] = address;
    frame[1] = STEPPER_UART_CMD_SET_CONTROL_MODE;
    frame[2] = STEPPER_UART_KEY_SET_CONTROL_MODE;
    frame[3] = save_to_flash;
    frame[4] = (uint8)mode;
    frame[5] = STEPPER_UART_FRAME_CHECK_BYTE;

    return stepper_uart_write(frame, (uint32)sizeof(frame));
}

stepper_uart_status_enum stepper_uart_set_enable (
    uint8 address,
    uint8 enabled,
    uint8 synchronized)
{
    uint8 frame[6];

    if ((ZF_FALSE == stepper_uart_is_boolean(enabled))
        || (ZF_FALSE == stepper_uart_is_boolean(synchronized)))
    {
        return STEPPER_UART_STATUS_INVALID_ARGUMENT;
    }

    frame[0] = address;
    frame[1] = STEPPER_UART_CMD_SET_ENABLE;
    frame[2] = STEPPER_UART_KEY_SET_ENABLE;
    frame[3] = enabled;
    frame[4] = synchronized;
    frame[5] = STEPPER_UART_FRAME_CHECK_BYTE;

    return stepper_uart_write(frame, (uint32)sizeof(frame));
}

/*
 * 速度命令：[地址,F6,方向,速度BE16,加速度,同步标志,6B]。
 * API 虽以 rpm 命名，signed wrapper 只把正负号映射到 CW/CCW，并原样
 * 发送 16 位绝对值；真实单位/缩放取决于驱动器 S_Vel，仍待上板确认。
 * synchronized=1 时驱动器等待后续 0xFF 同步触发。
 */
stepper_uart_status_enum stepper_uart_run_velocity (
    uint8 address,
    stepper_uart_direction_enum direction,
    uint16 speed_rpm,
    uint8 acceleration,
    uint8 synchronized)
{
    uint8 frame[8];

    if ((ZF_FALSE == stepper_uart_is_direction(direction))
        || (STEPPER_UART_MAX_SPEED_RPM < speed_rpm)
        || (ZF_FALSE == stepper_uart_is_boolean(synchronized)))
    {
        return STEPPER_UART_STATUS_INVALID_ARGUMENT;
    }

    frame[0] = address;
    frame[1] = STEPPER_UART_CMD_RUN_VELOCITY;
    frame[2] = (uint8)direction;
    stepper_uart_store_u16_be(&frame[3], speed_rpm);
    frame[5] = acceleration;
    frame[6] = synchronized;
    frame[7] = STEPPER_UART_FRAME_CHECK_BYTE;

    return stepper_uart_write(frame, (uint32)sizeof(frame));
}

stepper_uart_status_enum stepper_uart_run_velocity_signed (
    uint8 address,
    int16 signed_speed_rpm,
    uint8 acceleration,
    uint8 synchronized)
{
    stepper_uart_direction_enum direction;
    uint16 speed_rpm;

    if (((int16)(-((int16)STEPPER_UART_MAX_SPEED_RPM))
            > signed_speed_rpm)
        || ((int16)STEPPER_UART_MAX_SPEED_RPM < signed_speed_rpm))
    {
        return STEPPER_UART_STATUS_INVALID_ARGUMENT;
    }

    if (0 <= signed_speed_rpm)
    {
        direction = STEPPER_UART_DIRECTION_CW;
        speed_rpm = (uint16)signed_speed_rpm;
    }
    else
    {
        direction = STEPPER_UART_DIRECTION_CCW;
        speed_rpm = (uint16)(-signed_speed_rpm);
    }

    return stepper_uart_run_velocity(
        address,
        direction,
        speed_rpm,
        acceleration,
        synchronized);
}

stepper_uart_status_enum stepper_uart_try_run_velocity_signed (
    uint8 address,
    int16 signed_speed_rpm,
    uint8 acceleration,
    uint8 synchronized)
{
    uint8 frame[STEPPER_UART_VELOCITY_COMMAND_FRAME_SIZE];
    uint32 speed_magnitude;
    stepper_uart_direction_enum direction;

    if (ZF_FALSE == stepper_uart_is_boolean(synchronized))
    {
        return STEPPER_UART_STATUS_INVALID_ARGUMENT;
    }

    if (0 <= signed_speed_rpm)
    {
        direction = STEPPER_UART_DIRECTION_CW;
        speed_magnitude = (uint32)signed_speed_rpm;
    }
    else
    {
        direction = STEPPER_UART_DIRECTION_CCW;
        speed_magnitude = (uint32)(-((int32)signed_speed_rpm));
    }

    if (STEPPER_UART_MAX_SPEED_RPM < speed_magnitude)
    {
        return STEPPER_UART_STATUS_INVALID_ARGUMENT;
    }

    frame[0] = address;
    frame[1] = STEPPER_UART_CMD_RUN_VELOCITY;
    frame[2] = (uint8)direction;
    stepper_uart_store_u16_be(&frame[3], (uint16)speed_magnitude);
    frame[5] = acceleration;
    frame[6] = synchronized;
    frame[7] = STEPPER_UART_FRAME_CHECK_BYTE;

    return stepper_uart_try_write(
        frame,
        (uint32)sizeof(frame));
}

/*
 * 多机速度外层帧：
 * [地址,AA,总长度BE16,8字节速度子帧...,外层6B]。
 * 子帧地址不允许为广播地址；静态 512 字节工作区要求调用者串行调用。
 */
stepper_uart_status_enum stepper_uart_send_multi_velocity (
    uint8 address,
    const stepper_uart_velocity_command_struct *commands,
    uint8 command_count)
{
    uint8 command_index;
    uint16 offset;
    uint16 total_length;
    const stepper_uart_velocity_command_struct *command;

    if ((0 == commands)
        || (0U == command_count)
        || (STEPPER_UART_MULTI_VELOCITY_MAX_COUNT < command_count))
    {
        return STEPPER_UART_STATUS_INVALID_ARGUMENT;
    }

    for (command_index = 0U;
         command_index < command_count;
         command_index++)
    {
        command = &commands[command_index];
        if ((STEPPER_UART_BROADCAST_ADDRESS == command->address)
            || (ZF_FALSE
                == stepper_uart_is_direction(command->direction))
            || (STEPPER_UART_MAX_SPEED_RPM < command->speed_rpm)
            || (ZF_FALSE
                == stepper_uart_is_boolean(command->synchronized)))
        {
            return STEPPER_UART_STATUS_INVALID_ARGUMENT;
        }
    }

    total_length = (uint16)(
        STEPPER_UART_MULTI_FRAME_OVERHEAD
        + ((uint16)command_count
            * STEPPER_UART_VELOCITY_COMMAND_FRAME_SIZE));

    stepper_uart_multi_frame[0] = address;
    stepper_uart_multi_frame[1] = STEPPER_UART_CMD_MULTI_MOTOR;
    stepper_uart_store_u16_be(
        &stepper_uart_multi_frame[2],
        total_length);

    for (command_index = 0U;
         command_index < command_count;
         command_index++)
    {
        command = &commands[command_index];
        offset = (uint16)(
            STEPPER_UART_MULTI_FRAME_HEADER_SIZE
            + ((uint16)command_index
                * STEPPER_UART_VELOCITY_COMMAND_FRAME_SIZE));

        stepper_uart_multi_frame[offset] = command->address;
        stepper_uart_multi_frame[offset + 1U] =
            STEPPER_UART_CMD_RUN_VELOCITY;
        stepper_uart_multi_frame[offset + 2U] =
            (uint8)command->direction;
        stepper_uart_store_u16_be(
            &stepper_uart_multi_frame[offset + 3U],
            command->speed_rpm);
        stepper_uart_multi_frame[offset + 5U] =
            command->acceleration;
        stepper_uart_multi_frame[offset + 6U] =
            command->synchronized;
        stepper_uart_multi_frame[offset + 7U] =
            STEPPER_UART_FRAME_CHECK_BYTE;
    }

    stepper_uart_multi_frame[total_length - 1U] =
        STEPPER_UART_FRAME_CHECK_BYTE;

    return stepper_uart_write(
        stepper_uart_multi_frame,
        (uint32)total_length);
}

/*
 * 位置命令：[地址,FD,方向,速度BE16,加速度,脉冲BE32,模式,同步,6B]。
 * signed wrapper 把脉冲正负号映射为方向，线上脉冲字段始终发送绝对值。
 */
stepper_uart_status_enum stepper_uart_move (
    uint8 address,
    stepper_uart_direction_enum direction,
    uint16 speed_rpm,
    uint8 acceleration,
    uint32 pulse_count,
    stepper_uart_position_mode_enum position_mode,
    uint8 synchronized)
{
    uint8 frame[STEPPER_UART_MOVE_COMMAND_FRAME_SIZE];
    stepper_uart_status_enum status;

    status = stepper_uart_build_move_frame(
        frame,
        address,
        direction,
        speed_rpm,
        acceleration,
        pulse_count,
        position_mode,
        synchronized);
    if (STEPPER_UART_STATUS_OK != status)
    {
        return status;
    }

    return stepper_uart_write(frame, (uint32)sizeof(frame));
}

stepper_uart_status_enum stepper_uart_move_signed (
    uint8 address,
    int32 signed_pulse_count,
    uint16 speed_rpm,
    uint8 acceleration,
    stepper_uart_position_mode_enum position_mode,
    uint8 synchronized)
{
    stepper_uart_direction_enum direction;
    uint32 pulse_count;

    if (0 <= signed_pulse_count)
    {
        direction = STEPPER_UART_DIRECTION_CW;
        pulse_count = (uint32)signed_pulse_count;
    }
    else
    {
        direction = STEPPER_UART_DIRECTION_CCW;
        pulse_count = 0U - (uint32)signed_pulse_count;
    }

    return stepper_uart_move(
        address,
        direction,
        speed_rpm,
        acceleration,
        pulse_count,
        position_mode,
        synchronized);
}

stepper_uart_status_enum stepper_uart_try_move_signed (
    uint8 address,
    int32 signed_pulse_count,
    uint16 speed_rpm,
    uint8 acceleration,
    stepper_uart_position_mode_enum position_mode,
    uint8 synchronized)
{
    uint8 frame[STEPPER_UART_MOVE_COMMAND_FRAME_SIZE];
    stepper_uart_direction_enum direction;
    stepper_uart_status_enum status;
    uint32 pulse_count;

    if (0 <= signed_pulse_count)
    {
        direction = STEPPER_UART_DIRECTION_CW;
        pulse_count = (uint32)signed_pulse_count;
    }
    else
    {
        direction = STEPPER_UART_DIRECTION_CCW;
        pulse_count = 0U - (uint32)signed_pulse_count;
    }

    status = stepper_uart_build_move_frame(
        frame,
        address,
        direction,
        speed_rpm,
        acceleration,
        pulse_count,
        position_mode,
        synchronized);
    if (STEPPER_UART_STATUS_OK != status)
    {
        return status;
    }

    return stepper_uart_try_write(frame, (uint32)sizeof(frame));
}

/*
 * 多机位置帧沿用 AA 外层格式，每个子帧 13 字节；39 个子帧受
 * 512 字节静态工作区上限约束。
 */
stepper_uart_status_enum stepper_uart_send_multi_move (
    uint8 address,
    const stepper_uart_move_command_struct *commands,
    uint8 command_count)
{
    uint8 command_index;
    uint16 offset;
    uint16 total_length;
    const stepper_uart_move_command_struct *command;

    if ((0 == commands)
        || (0U == command_count)
        || (STEPPER_UART_MULTI_MOVE_MAX_COUNT < command_count))
    {
        return STEPPER_UART_STATUS_INVALID_ARGUMENT;
    }

    for (command_index = 0U;
         command_index < command_count;
         command_index++)
    {
        command = &commands[command_index];
        if ((STEPPER_UART_BROADCAST_ADDRESS == command->address)
            || (ZF_FALSE
                == stepper_uart_is_direction(command->direction))
            || (STEPPER_UART_MAX_SPEED_RPM < command->speed_rpm)
            || (ZF_FALSE
                == stepper_uart_is_position_mode(command->position_mode))
            || (ZF_FALSE
                == stepper_uart_is_boolean(command->synchronized)))
        {
            return STEPPER_UART_STATUS_INVALID_ARGUMENT;
        }
    }

    total_length = (uint16)(
        STEPPER_UART_MULTI_FRAME_OVERHEAD
        + ((uint16)command_count
            * STEPPER_UART_MOVE_COMMAND_FRAME_SIZE));

    stepper_uart_multi_frame[0] = address;
    stepper_uart_multi_frame[1] = STEPPER_UART_CMD_MULTI_MOTOR;
    stepper_uart_store_u16_be(
        &stepper_uart_multi_frame[2],
        total_length);

    for (command_index = 0U;
         command_index < command_count;
         command_index++)
    {
        command = &commands[command_index];
        offset = (uint16)(
            STEPPER_UART_MULTI_FRAME_HEADER_SIZE
            + ((uint16)command_index
                * STEPPER_UART_MOVE_COMMAND_FRAME_SIZE));

        stepper_uart_multi_frame[offset] = command->address;
        stepper_uart_multi_frame[offset + 1U] = STEPPER_UART_CMD_MOVE;
        stepper_uart_multi_frame[offset + 2U] =
            (uint8)command->direction;
        stepper_uart_store_u16_be(
            &stepper_uart_multi_frame[offset + 3U],
            command->speed_rpm);
        stepper_uart_multi_frame[offset + 5U] =
            command->acceleration;
        stepper_uart_store_u32_be(
            &stepper_uart_multi_frame[offset + 6U],
            command->pulse_count);
        stepper_uart_multi_frame[offset + 10U] =
            (uint8)command->position_mode;
        stepper_uart_multi_frame[offset + 11U] =
            command->synchronized;
        stepper_uart_multi_frame[offset + 12U] =
            STEPPER_UART_FRAME_CHECK_BYTE;
    }

    stepper_uart_multi_frame[total_length - 1U] =
        STEPPER_UART_FRAME_CHECK_BYTE;

    return stepper_uart_write(
        stepper_uart_multi_frame,
        (uint32)total_length);
}

/*
 * 快速位置模式分两步：F1 保存速度/加速度/模式，FC 再下发有符号脉冲。
 * FC 没有独立方向字节，int32 直接按二进制补码写入 4 字节字段。
 */
stepper_uart_status_enum stepper_uart_set_quick_move (
    uint8 address,
    uint16 speed_rpm,
    uint8 acceleration,
    stepper_uart_position_mode_enum position_mode,
    uint8 synchronized)
{
    uint8 frame[8];

    if ((STEPPER_UART_MAX_SPEED_RPM < speed_rpm)
        || (ZF_FALSE == stepper_uart_is_position_mode(position_mode))
        || (ZF_FALSE == stepper_uart_is_boolean(synchronized)))
    {
        return STEPPER_UART_STATUS_INVALID_ARGUMENT;
    }

    frame[0] = address;
    frame[1] = STEPPER_UART_CMD_SET_QUICK_MOVE;
    stepper_uart_store_u16_be(&frame[2], speed_rpm);
    frame[4] = acceleration;
    frame[5] = (uint8)position_mode;
    frame[6] = synchronized;
    frame[7] = STEPPER_UART_FRAME_CHECK_BYTE;

    return stepper_uart_write(frame, (uint32)sizeof(frame));
}

stepper_uart_status_enum stepper_uart_quick_move (
    uint8 address,
    int32 signed_pulse_count)
{
    uint8 frame[7];

    frame[0] = address;
    frame[1] = STEPPER_UART_CMD_QUICK_MOVE;
    stepper_uart_store_u32_be(&frame[2], (uint32)signed_pulse_count);
    frame[6] = STEPPER_UART_FRAME_CHECK_BYTE;

    return stepper_uart_write(frame, (uint32)sizeof(frame));
}

/*
 * FE 立即/同步停止；FF 让此前 synchronized=1 的多轴命令同时开始。
 * 同步标志只负责排队，FF 才触发动作。
 */
stepper_uart_status_enum stepper_uart_stop (
    uint8 address,
    uint8 synchronized)
{
    uint8 frame[5];

    if (ZF_FALSE == stepper_uart_is_boolean(synchronized))
    {
        return STEPPER_UART_STATUS_INVALID_ARGUMENT;
    }

    frame[0] = address;
    frame[1] = STEPPER_UART_CMD_STOP;
    frame[2] = STEPPER_UART_KEY_STOP;
    frame[3] = synchronized;
    frame[4] = STEPPER_UART_FRAME_CHECK_BYTE;

    return stepper_uart_write(frame, (uint32)sizeof(frame));
}

stepper_uart_status_enum stepper_uart_start_synchronous_motion (
    uint8 address)
{
    const uint8 frame[4] =
    {
        address,
        STEPPER_UART_CMD_SYNCHRONOUS_MOTION,
        STEPPER_UART_KEY_SYNCHRONOUS_MOTION,
        STEPPER_UART_FRAME_CHECK_BYTE
    };

    return stepper_uart_write(frame, (uint32)sizeof(frame));
}

stepper_uart_status_enum stepper_uart_try_start_home (uint8 address)
{
    const uint8 frame[STEPPER_UART_HOME_COMMAND_FRAME_SIZE] =
    {
        address,
        STEPPER_UART_CMD_START_HOME,
        0x00U, /* 最近点回零模式。 */
        0x00U, /* 不使用多机同步。 */
        STEPPER_UART_FRAME_CHECK_BYTE
    };

    /*
     * 只触发驱动器按既有配置回到机械零点。该固定帧不会写当前位置、设置
     * 零点或修改回零配置；FIFO 非空时立即返回 TX_BUSY，不排队或重试。
     */
    return stepper_uart_try_write(frame, (uint32)sizeof(frame));
}

void stepper_uart_rx_irq_handler (void)
{
    /*
     * 为兼容现有 user/isr.c 保留空符号。stepper_uart_init() 不开启 UART3
     * RX 中断，P15.7 也不接驱动器 TX，因此这里不会参与运行时数据路径。
     */
}
