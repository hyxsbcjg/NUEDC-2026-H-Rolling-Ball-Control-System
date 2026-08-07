#include "imu_app.h"

#include "zf_common_interrupt.h"

#if (IMU_APP_OFFSET_METHOD == IMU_APP_OFFSET_METHOD_AUTO_CALIBRATION)
#include "my_uart.h"
#include "zf_driver_delay.h"
#endif

#if (IMU_APP_DEVICE == IMU_APP_DEVICE_IMU660RA)
#include "zf_device_imu660ra.h"
#define IMU_APP_DEVICE_NAME                     "RA"
#else
#include "zf_device_imu660rc.h"
#define IMU_APP_DEVICE_NAME                     "RC"
#endif

#pragma section all "cpu0_dsram"
/*
 * 3 ms 调度 ISR 的采样层是运行期唯一写者，CPU0 前台通过关中断快照读取。
 * volatile 保证每个字段实际访存，跨字段一致性由 imu_app_get_snapshot 保证。
 */
static volatile ImuAppData imu_app_data =
{
    .init_status = IMU_APP_STATUS_NOT_INITIALIZED,
    .initialized = ZF_FALSE,
    .offset_valid = ZF_FALSE
};
#pragma section all restore

static void imu_app_clear_data (void)
{
    imu_app_data.raw_x = 0;
    imu_app_data.raw_y = 0;
    imu_app_data.raw_z = 0;
    imu_app_data.raw_acc_x = 0;
    imu_app_data.raw_acc_y = 0;

    imu_app_data.gyro_offset_x_lsb = 0.0F;
    imu_app_data.gyro_offset_y_lsb = 0.0F;
    imu_app_data.gyro_offset_z_lsb = 0.0F;
    imu_app_data.acc_offset_x_lsb = 0.0F;
    imu_app_data.acc_offset_y_lsb = 0.0F;

    imu_app_data.corrected_x_lsb = 0.0F;
    imu_app_data.corrected_y_lsb = 0.0F;
    imu_app_data.corrected_z_lsb = 0.0F;
    imu_app_data.corrected_acc_x_lsb = 0.0F;
    imu_app_data.corrected_acc_y_lsb = 0.0F;

    imu_app_data.init_status = IMU_APP_STATUS_NOT_INITIALIZED;
    imu_app_data.initialized = ZF_FALSE;
    imu_app_data.offset_valid = ZF_FALSE;
    imu_app_data.acceleration_valid = ZF_FALSE;
}

#if (IMU_APP_OFFSET_METHOD == IMU_APP_OFFSET_METHOD_AUTO_CALIBRATION)
static void imu_app_print_auto_offsets (void)
{
    /*
     * binary32 最多需要 9 位十进制有效数字才能无损往返；变参调用必须把
     * float32 显式提升为 double。该阻塞日志仅在启动阶段尝试一次，其发送
     * 状态被有意忽略，不影响已经算出的零偏有效标志。
     */
    (void)my_printf(
        "IMU %s gyro offset LSB: X=%+.9g Y=%+.9g Z=%+.9g\r\n",
        IMU_APP_DEVICE_NAME,
        (double)imu_app_data.gyro_offset_x_lsb,
        (double)imu_app_data.gyro_offset_y_lsb,
        (double)imu_app_data.gyro_offset_z_lsb);
    (void)my_printf(
        "IMU %s acc offset LSB: X=%+.9g Y=%+.9g\r\n",
        IMU_APP_DEVICE_NAME,
        (double)imu_app_data.acc_offset_x_lsb,
        (double)imu_app_data.acc_offset_y_lsb);
}
#endif

static void imu_app_update_corrected (void)
{
    /* 保持 raw LSB 域：这里只减零偏，不调用底层的 dps 转换宏。 */
    imu_app_data.corrected_x_lsb =
        (float32)imu_app_data.raw_x
        - imu_app_data.gyro_offset_x_lsb;
    imu_app_data.corrected_y_lsb =
        (float32)imu_app_data.raw_y
        - imu_app_data.gyro_offset_y_lsb;
    imu_app_data.corrected_z_lsb =
        (float32)imu_app_data.raw_z
        - imu_app_data.gyro_offset_z_lsb;
    imu_app_data.corrected_acc_x_lsb =
        (float32)imu_app_data.raw_acc_x
        - imu_app_data.acc_offset_x_lsb;
    imu_app_data.corrected_acc_y_lsb =
        (float32)imu_app_data.raw_acc_y
        - imu_app_data.acc_offset_y_lsb;
}

static void imu_app_capture_gyro (void)
{
    /*
     * 底层先把一次 6 字节读取拆入三个驱动全局量，再立即复制到模块状态。
     * get_gyro() 无返回值，故这些赋值不能证明本次 SPI 事务成功。
     */
#if (IMU_APP_DEVICE == IMU_APP_DEVICE_IMU660RA)
    imu660ra_get_gyro();
    imu_app_data.raw_x = imu660ra_gyro_x;
    imu_app_data.raw_y = imu660ra_gyro_y;
    imu_app_data.raw_z = imu660ra_gyro_z;
#else
    imu660rc_get_gyro();
    imu_app_data.raw_x = imu660rc_gyro_x;
    imu_app_data.raw_y = imu660rc_gyro_y;
    imu_app_data.raw_z = imu660rc_gyro_z;
#endif

    imu_app_update_corrected();
}

static void imu_app_capture_acc (void)
{
#if (IMU_APP_DEVICE == IMU_APP_DEVICE_IMU660RA)
    imu660ra_get_acc();
    imu_app_data.raw_acc_x = imu660ra_acc_x;
    imu_app_data.raw_acc_y = imu660ra_acc_y;
#else
    imu660rc_get_acc();
    imu_app_data.raw_acc_x = imu660rc_acc_x;
    imu_app_data.raw_acc_y = imu660rc_acc_y;
#endif

    imu_app_update_corrected();
    imu_app_data.acceleration_valid = ZF_TRUE;
}

uint8 imu_app_init (void)
{
    uint8 init_status;

    imu_app_clear_data();

#if (IMU_APP_DEVICE == IMU_APP_DEVICE_IMU660RA)
    init_status = imu660ra_init();
#else
    /*
     * 关闭四元数输出后，IMU660RC 的 get_gyro() 才执行普通寄存器轮询；
     * 底层也不会配置 INT2 及 ERU/EXTI，避免占用 P15.4。
     */
    init_status = imu660rc_init(IMU660RC_QUARTERNION_DISABLE);
#endif

    imu_app_data.init_status = init_status;
    imu_app_data.initialized =
        (IMU_APP_STATUS_OK == init_status) ? ZF_TRUE : ZF_FALSE;

    return init_status;
}

uint8 imu_app_calibrate_offset (void)
{
#if (IMU_APP_OFFSET_METHOD == IMU_APP_OFFSET_METHOD_AUTO_CALIBRATION)
    uint16 sample_index;
    int64 sum_x = 0;
    int64 sum_y = 0;
    int64 sum_z = 0;
    int64 sum_acc_x = 0;
    int64 sum_acc_y = 0;
#endif

    if (ZF_FALSE == imu_app_data.initialized)
    {
        return IMU_APP_STATUS_NOT_INITIALIZED;
    }

#if (IMU_APP_OFFSET_METHOD == IMU_APP_OFFSET_METHOD_FIXED)
    /* 固定模式无采样等待；首次 imu_app_update 前 raw 仍是清零后的占位值。 */
    imu_app_data.gyro_offset_x_lsb =
        (float32)IMU_APP_FIXED_OFFSET_X_LSB;
    imu_app_data.gyro_offset_y_lsb =
        (float32)IMU_APP_FIXED_OFFSET_Y_LSB;
    imu_app_data.gyro_offset_z_lsb =
        (float32)IMU_APP_FIXED_OFFSET_Z_LSB;
    imu_app_data.acc_offset_x_lsb =
        (float32)IMU_APP_FIXED_ACC_OFFSET_X_LSB;
    imu_app_data.acc_offset_y_lsb =
        (float32)IMU_APP_FIXED_ACC_OFFSET_Y_LSB;
    imu_app_data.offset_valid = ZF_TRUE;
    imu_app_update_corrected();
#else
    imu_app_data.gyro_offset_x_lsb = 0.0F;
    imu_app_data.gyro_offset_y_lsb = 0.0F;
    imu_app_data.gyro_offset_z_lsb = 0.0F;
    imu_app_data.acc_offset_x_lsb = 0.0F;
    imu_app_data.acc_offset_y_lsb = 0.0F;
    imu_app_data.offset_valid = ZF_FALSE;

    /* 丢弃启动瞬态样本；每次读取后忙等待一个完整标定采样间隔。 */
    for (sample_index = 0U;
         sample_index < IMU_APP_CALIBRATION_DISCARD_COUNT;
         sample_index++)
    {
        imu_app_capture_gyro();
        imu_app_capture_acc();
        system_delay_ms(IMU_APP_CALIBRATION_INTERVAL_MS);
    }

    /* int64 累加覆盖 200 个 int16 样本，求均值前不会发生范围溢出。 */
    for (sample_index = 0U;
         sample_index < IMU_APP_CALIBRATION_SAMPLE_COUNT;
         sample_index++)
    {
        imu_app_capture_gyro();
        imu_app_capture_acc();
        sum_x += (int64)imu_app_data.raw_x;
        sum_y += (int64)imu_app_data.raw_y;
        sum_z += (int64)imu_app_data.raw_z;
        sum_acc_x += (int64)imu_app_data.raw_acc_x;
        sum_acc_y += (int64)imu_app_data.raw_acc_y;
        system_delay_ms(IMU_APP_CALIBRATION_INTERVAL_MS);
    }

    imu_app_data.gyro_offset_x_lsb =
        (float32)sum_x / (float32)IMU_APP_CALIBRATION_SAMPLE_COUNT;
    imu_app_data.gyro_offset_y_lsb =
        (float32)sum_y / (float32)IMU_APP_CALIBRATION_SAMPLE_COUNT;
    imu_app_data.gyro_offset_z_lsb =
        (float32)sum_z / (float32)IMU_APP_CALIBRATION_SAMPLE_COUNT;
    imu_app_data.acc_offset_x_lsb =
        (float32)sum_acc_x / (float32)IMU_APP_CALIBRATION_SAMPLE_COUNT;
    imu_app_data.acc_offset_y_lsb =
        (float32)sum_acc_y / (float32)IMU_APP_CALIBRATION_SAMPLE_COUNT;
    imu_app_data.offset_valid = ZF_TRUE;

    /* 用最后一个原始样本和新均值重算去偏值，再输出可复制的固定零偏。 */
    imu_app_update_corrected();
    imu_app_print_auto_offsets();
#endif

    return IMU_APP_STATUS_OK;
}

void imu_app_update (void)
{
    /*
     * 运行期由 3 ms 调度 ISR 每拍单一调用。CPU0 前台只复制快照，
     * 不得并发发起 SPI 采样；6 ms 角速度内环只读取采样层发布的数据。
     */
    if (ZF_TRUE == imu_app_data.initialized)
    {
        imu_app_capture_gyro();
        imu_app_capture_acc();
    }
}

float32 imu_app_gyro_lsb_to_dps (float32 gyro_lsb)
{
    float32 transition_factor;

#if (IMU_APP_DEVICE == IMU_APP_DEVICE_IMU660RA)
    transition_factor = (float32)imu660ra_transition_factor[1];
#else
    transition_factor = (float32)imu660rc_transition_factor[1];
#endif

    return (0.0F < transition_factor)
        ? gyro_lsb / transition_factor
        : 0.0F;
}

uint8 imu_app_get_snapshot (ImuAppData *snapshot)
{
    uint32 interrupt_state;

    if (0 == snapshot)
    {
        return ZF_FALSE;
    }

    /*
     * 保存原中断状态，复制完成后按原状态恢复；临界区只包围结构体赋值，
     * 不执行 SPI、延时或格式化。
     */
    interrupt_state = interrupt_global_disable();
    *snapshot = imu_app_data;
    interrupt_global_enable(interrupt_state);

    return ZF_TRUE;
}

const volatile ImuAppData *imu_app_get_data (void)
{
    return &imu_app_data;
}
