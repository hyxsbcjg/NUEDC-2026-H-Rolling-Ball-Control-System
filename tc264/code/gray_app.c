#include "gray_app.h"

#if (GW_GRAY_CHANNEL_COUNT != 8U)
#error "gray_app requires exactly eight gray-sensor channels"
#endif

/*
 * 正权重位于 CH1 侧，负权重位于 CH8 侧；符号只表达安装方向约定，
 * 不代表传感器强度。权重关于中心对称，黑线居中时平均值为 0。
 */
static const float gray_app_weights[GW_GRAY_CHANNEL_COUNT] =
{
    3.5F,
    2.5F,
    1.5F,
    0.5F,
    -0.5F,
    -1.5F,
    -2.5F,
    -3.5F
};

#pragma section all "cpu0_dsram"
static volatile float gray_app_error_filtered = 0.0F;
static volatile uint8 gray_app_error_filter_valid = ZF_FALSE;
#pragma section all restore

static void gray_app_reset_error_filter (void)
{
    gray_app_error_filtered = 0.0F;
    gray_app_error_filter_valid = ZF_FALSE;
}

static void gray_app_clear_sample (gray_app_sample_struct *sample)
{
    /*
     * 先写入“全白、零误差”占位值，保证失败路径不泄漏旧样本。
     * 该位图与真实全白相同，因此有效性必须由 gray_app_read 返回状态判断。
     */
    sample->digital_value = GW_GRAY_CHANNEL_ALL;
    sample->error = 0.0F;
    sample->error_num = 0U;
}

static void gray_app_filter_error (gray_app_sample_struct *sample)
{
    if (0U == sample->error_num)
    {
        gray_app_reset_error_filter();
        return;
    }

    if (ZF_FALSE == gray_app_error_filter_valid)
    {
        gray_app_error_filtered = sample->error;
        gray_app_error_filter_valid = ZF_TRUE;
    }
    else
    {
        gray_app_error_filtered +=
            (sample->error - gray_app_error_filtered)
            * GRAY_APP_ERROR_FILTER_ALPHA;
    }

    sample->error = gray_app_error_filtered;
}

static void gray_app_calculate_error (gray_app_sample_struct *sample)
{
    uint8 channel;
    float weight_sum = 0.0F;

    for (channel = 0U; channel < GW_GRAY_CHANNEL_COUNT; channel++)
    {
        /* 数字量为低有效：只有 bit=0 的黑线通道参与权重平均。 */
        if (0U == (sample->digital_value & (uint8)(1U << channel)))
        {
            weight_sum += gray_app_weights[channel];
            sample->error_num++;
        }
    }

    if (0U != sample->error_num)
    {
        /* error_num 是除数和黑线宽度信息；零通道时保持 0，避免除零。 */
        sample->error = weight_sum / (float)sample->error_num;
    }
}

gw_gray_status_enum gray_app_init (void)
{
    gw_gray_status_enum status;

    gray_app_reset_error_filter();
    if (!(GRAY_APP_ERROR_FILTER_ALPHA > 0.0F)
        || !(GRAY_APP_ERROR_FILTER_ALPHA <= 1.0F))
    {
        return GW_GRAY_STATUS_INVALID_ARGUMENT;
    }

    status = gw_gray_init();
    if (GW_GRAY_STATUS_OK == status)
    {
        /* 配置函数会回读 0xCE 做值校验；在线证明仍以 PING 握手为准。 */
        status = gw_gray_set_channel_enable(GW_GRAY_CHANNEL_ALL);
    }

    return status;
}

gw_gray_status_enum gray_app_read (
    gray_app_sample_struct *sample)
{
    gw_gray_status_enum status;

    if (0 == sample)
    {
        return GW_GRAY_STATUS_INVALID_ARGUMENT;
    }

    gray_app_clear_sample(sample);

    status = gw_gray_read_digital(&sample->digital_value);
    if (GW_GRAY_STATUS_OK == status)
    {
        gray_app_calculate_error(sample);
        gray_app_filter_error(sample);
    }
    else
    {
        gray_app_reset_error_filter();
    }

    return status;
}

gw_gray_status_enum gray_app_get_error (
    float *error,
    uint16 *error_num)
{
    gw_gray_status_enum status;
    gray_app_sample_struct sample;

    if ((0 == error) || (0 == error_num))
    {
        return GW_GRAY_STATUS_INVALID_ARGUMENT;
    }

    *error = 0.0F;
    *error_num = 0U;

    /* 先清零输出；只有完整采样成功后才发布同一次读取产生的两个结果。 */
    status = gray_app_read(&sample);
    if (GW_GRAY_STATUS_OK == status)
    {
        *error = sample.error;
        *error_num = sample.error_num;
    }

    return status;
}

void get_error (float *error, uint16 *error_num)
{
    (void)gray_app_get_error(error, error_num);
}
