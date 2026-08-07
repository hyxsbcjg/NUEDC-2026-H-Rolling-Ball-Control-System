#ifndef IMU_APP_H_
#define IMU_APP_H_

#include "zf_common_typedef.h"

/* 编译期二选一，默认 RA；运行时不会自动探测或切换器件。 */
#define IMU_APP_DEVICE_IMU660RA                 (1U)
#define IMU_APP_DEVICE_IMU660RC                 (2U)

#ifndef IMU_APP_DEVICE
#define IMU_APP_DEVICE                          (IMU_APP_DEVICE_IMU660RA)
#endif

#define IMU_APP_OFFSET_METHOD_FIXED             (1U)
#define IMU_APP_OFFSET_METHOD_AUTO_CALIBRATION  (2U)

#ifndef IMU_APP_OFFSET_METHOD
#define IMU_APP_OFFSET_METHOD                   (IMU_APP_OFFSET_METHOD_FIXED)
#endif

/*
 * 固定零偏与驱动读出的 int16 原始 LSB 同域，不是 dps。若更换器件型号、
 * 量程、安装方向或传感器，必须重新标定，不能沿用旧值。
 */
#ifndef IMU_APP_FIXED_OFFSET_X_LSB
#define IMU_APP_FIXED_OFFSET_X_LSB              (2.762F)
#endif

#ifndef IMU_APP_FIXED_OFFSET_Y_LSB
#define IMU_APP_FIXED_OFFSET_Y_LSB              (7.037F)
#endif

#ifndef IMU_APP_FIXED_OFFSET_Z_LSB
#define IMU_APP_FIXED_OFFSET_Z_LSB              (2.831F)
#endif

#ifndef IMU_APP_FIXED_ACC_OFFSET_X_LSB
#define IMU_APP_FIXED_ACC_OFFSET_X_LSB          (27.169F)
#endif

#ifndef IMU_APP_FIXED_ACC_OFFSET_Y_LSB
#define IMU_APP_FIXED_ACC_OFFSET_Y_LSB          (-15.894F)
#endif

#define IMU_APP_CALIBRATION_DISCARD_COUNT       (20U)
#define IMU_APP_CALIBRATION_SAMPLE_COUNT        (200U)
#define IMU_APP_CALIBRATION_INTERVAL_MS         (5U)

/*
 * 自动标定先丢弃 20 组，再同时平均 200 组三轴角速度与 X/Y 轴加速度；
 * 每组间隔 5 ms，阻塞时间约 1.1 s（另加 SPI 时间）。标定期间车辆和滚球
 * 机构必须完全静止，且保持随后运行时的机械姿态。
 */
#define IMU_APP_STATUS_OK                       (0U)
#define IMU_APP_STATUS_NOT_INITIALIZED          (1U)

#if ((IMU_APP_DEVICE != IMU_APP_DEVICE_IMU660RA) \
    && (IMU_APP_DEVICE != IMU_APP_DEVICE_IMU660RC))
#error "IMU_APP_DEVICE must select IMU660RA or IMU660RC"
#endif

#if ((IMU_APP_OFFSET_METHOD != IMU_APP_OFFSET_METHOD_FIXED) \
    && (IMU_APP_OFFSET_METHOD != IMU_APP_OFFSET_METHOD_AUTO_CALIBRATION))
#error "IMU_APP_OFFSET_METHOD must select fixed or auto calibration"
#endif

#if (IMU_APP_CALIBRATION_DISCARD_COUNT == 0U)
#error "IMU_APP_CALIBRATION_DISCARD_COUNT must be greater than zero"
#endif

#if (IMU_APP_CALIBRATION_DISCARD_COUNT > 65535U)
#error "IMU_APP_CALIBRATION_DISCARD_COUNT must fit in uint16"
#endif

#if (IMU_APP_CALIBRATION_SAMPLE_COUNT == 0U)
#error "IMU_APP_CALIBRATION_SAMPLE_COUNT must be greater than zero"
#endif

#if (IMU_APP_CALIBRATION_SAMPLE_COUNT > 65535U)
#error "IMU_APP_CALIBRATION_SAMPLE_COUNT must fit in uint16"
#endif

#if (IMU_APP_CALIBRATION_INTERVAL_MS == 0U)
#error "IMU_APP_CALIBRATION_INTERVAL_MS must be greater than zero"
#endif

#if ((IMU_APP_DEVICE == IMU_APP_DEVICE_IMU660RA) \
    && (IMU_APP_CALIBRATION_INTERVAL_MS < 5U))
#error "IMU660RA calibration interval must be at least 5 ms"
#endif

typedef struct
{
    /* 一次驱动读取后立即复制的三轴原始 LSB。 */
    int16 raw_x;
    int16 raw_y;
    int16 raw_z;

    /* 一次驱动读取后立即复制的 X/Y 轴加速度原始 LSB。 */
    int16 raw_acc_x;
    int16 raw_acc_y;

    /* 固定宏或静止均值产生的零偏，单位均为 raw LSB。 */
    float32 gyro_offset_x_lsb;
    float32 gyro_offset_y_lsb;
    float32 gyro_offset_z_lsb;
    float32 acc_offset_x_lsb;
    float32 acc_offset_y_lsb;

    /* corrected = raw - offset，不做物理单位换算、滤波或积分。 */
    float32 corrected_x_lsb;
    float32 corrected_y_lsb;
    float32 corrected_z_lsb;
    float32 corrected_acc_x_lsb;
    float32 corrected_acc_y_lsb;

    /* init_status 保留底层 0/1；另外两项分别表示驱动与零偏是否可用。 */
    uint8 init_status;
    uint8 initialized;
    uint8 offset_valid;
    uint8 acceleration_valid;
} ImuAppData;

/*
 * 初始化所选底层并原样返回其状态。默认选择 IMU660RA；选择 IMU660RC 时，
 * 实现会关闭四元数模式，保留普通轮询并避免占用 INT2/ERU。
 */
uint8 imu_app_init (void);

/*
 * 固定模式加载角速度与 X/Y 加速度宏；自动模式计算五轴静止均值并打印。
 * 自动标定全程必须保持静止，且包含忙等待和 UART 格式化，只能在 PIT
 * 启动前由前台调用。
 */
uint8 imu_app_calibrate_offset (void);

/*
 * 轮询一组三轴陀螺仪和 X/Y 轴加速度计原始值。底层读取函数返回 void，无法
 * 报告本次 SPI 成败，因而本函数也不伪造通信状态。本函数由 3 ms 调度 ISR
 * 每拍调用；
 * 初始化、自动标定、打印和显示仍必须在 PIT 启动前或 CPU0 前台完成。
 */
void imu_app_update (void);

/*
 * 将已校零的陀螺仪原始 LSB 换算为 deg/s。换算因子沿用当前已初始化驱动的
 * 量程配置，不额外访问 IMU 硬件，可在调度 ISR 内调用。
 */
float32 imu_app_gyro_lsb_to_dps (float32 gyro_lsb);

/*
 * CPU0 前台通过短暂关闭全局中断，原子复制控制 ISR 发布的完整结构体，
 * 防止 raw/offset/corrected 字段来自不同更新时刻。
 */
uint8 imu_app_get_snapshot (ImuAppData *snapshot);

/*
 * 返回共享状态的只读 volatile 指针，仅供已处于调度 ISR、不会与写者并发
 * 的代码使用。前台解引用期间可能遇到半更新样本，必须改用 snapshot。
 */
const volatile ImuAppData *imu_app_get_data (void);

#endif
