/**
 ********************************************************
 * @file      biquad_filter.h
 * @brief     二阶巴特沃斯低通滤波器 (IIR Biquad Filter)
 ********************************************************
 */
#ifndef BIQUAD_FILTER_H
#define BIQUAD_FILTER_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>

/* 二阶滤波器结构体 */
typedef struct {
    float b0, b1, b2; // 前馈系数
    float a1, a2;     // 反馈系数
    float x1, x2;     // 历史输入状态
    float y1, y2;     // 历史输出状态
} BiquadFilter_t;

/**
 * @brief  动态计算二阶低通滤波器系数 (巴特沃斯响应 Q=0.707)
 * @param  filter: 滤波器结构体指针
 * @param  sample_freq: 系统的采样频率 (Hz)
 * @param  cutoff_freq: 期望的截止频率 (Hz)
 */
void BiquadFilter_Setup(BiquadFilter_t *filter, float sample_freq, float cutoff_freq);

/**
 * @brief  执行二阶滤波运算
 * @param  filter: 滤波器结构体指针
 * @param  input:  当前时刻的原始输入数据
 * @return 滤波后的平滑输出数据
 */
float BiquadFilter_Apply(BiquadFilter_t *filter, float input);

#ifdef __cplusplus
}
#endif

#endif /* BIQUAD_FILTER_H */