#ifndef GRAY_APP_H_
#define GRAY_APP_H_

#include "gw_gray_sensor.h"

/*
 * error 输出的一阶低通系数。0.3 表示本拍原始误差权重为 30%，上一拍滤波
 * 结果权重为 70%。滤波由 gray_app_read() 统一执行，后续消费者只见滤波值。
 */
#define GRAY_APP_ERROR_FILTER_ALPHA              (0.3F)

typedef struct
{
    uint8 digital_value;       /* CH1～CH8 对应 bit0～bit7，0=黑线，1=白底。 */
    float error;               /* 黑线通道权重平均值经过 0.3 一阶低通后的结果。 */
    uint16 error_num;          /* 黑线通道数，不是通信错误码或有效标志。 */
} gray_app_sample_struct;

/* 初始化底层、自动锁定地址，并写入后回读校验八路通道使能掩码。 */
gw_gray_status_enum gray_app_init (void);

/*
 * 读取一次数字量，并在同一个 sample 中保留原始位图和滤波后的加权误差。
 * 非 OK 状态下 digital_value 置为 0xFF、error/error_num 置零，调用方不能
 * 把该占位值当成真实“全白”读数。即使返回 OK，逐飞 Soft IIC 也没有上传
 * ACK；OK 表示本层状态允许并已发出事务，不单独构成运行期在线证明。
 * 通信失败或本拍无黑线时会清除滤波历史；恢复后的首个有效误差直接装载。
 */
gw_gray_status_enum gray_app_read (
    gray_app_sample_struct *sample);

/*
 * 读取一次数字量并计算黑线位置误差。
 *
 * 传感器以 0 表示黑线、1 表示白底，CH1 位于 bit0。通道权重为：
 *   CH1  CH2  CH3  CH4  CH5  CH6  CH7  CH8
 *   +3.5 +2.5 +1.5 +0.5 -0.5 -1.5 -2.5 -3.5
 *
 * error 是全部黑线通道权重算术平均的一阶低通结果，error_num 是本拍参与
 * 平均的通道数。gray_app_get_error()、get_error() 与所有 sample 消费者
 * 获得的都是同一滤波口径，不另外公开原始误差。
 * error=0 既可能表示左右对称命中，也可能表示未检测到黑线；返回状态和
 * error_num 共同区分这些情况。有效但无黑线时返回 0.0F、0。
 *
 * 驱动失败时两个输出都清零并返回原状态；空指针在访问传感器前即被拒绝。
 */
gw_gray_status_enum gray_app_get_error (
    float *error,
    uint16 *error_num);

/*
 * 兼容接口沿用相同输出规则，却丢弃驱动状态；调用者无法区分通信失败、
 * 无黑线和权重恰好居中。需要可靠判断时应调用 gray_app_get_error()。
 */
void get_error (float *error, uint16 *error_num);

/*
 * gray_app_read() 由 3 ms 调度 ISR 每拍单一调用；CPU0 前台只读取 task
 * 发布的快照，不得并发访问灰度总线。每次修改 Soft IIC 时序后，都要重新
 * 测量 3 ms ISR 的最坏执行时间。
 */

#endif
