/**
 ********************************************************
 * @file      biquad_filter.c
 * @brief     二阶巴特沃斯低通滤波器实现
 ********************************************************
 */
#include "biquad_filter.h"
#include <math.h>

#ifndef M_PI
#define M_PI 3.14159265358979323846f
#endif

void BiquadFilter_Setup(BiquadFilter_t *filter, float sample_freq, float cutoff_freq) {
    // 1. 安全保护：如果给定的截止频率非法，则设定为一个安全的默认值
    if (cutoff_freq <= 0.0f || cutoff_freq >= (sample_freq / 2.0f)) {
        cutoff_freq = sample_freq * 0.1f; // 默认设为采样率的十分之一
    }

    // 2. 设定 Q 值 (0.70710678f 为标准巴特沃斯响应，最平坦无震荡)
    float q_factor = 0.70710678f; 

    // 3. 计算归一化角频率等中间变量
    float omega = 2.0f * M_PI * cutoff_freq / sample_freq;
    float sn = sinf(omega);
    float cs = cosf(omega);
    float alpha = sn / (2.0f * q_factor);

    // 4. 计算原始系数 (a0 是分母，用于归一化)
    float a0 = 1.0f + alpha;
    float a1_raw = -2.0f * cs;
    float a2_raw = 1.0f - alpha;
    
    float b1_raw = 1.0f - cs;
    float b0_raw = b1_raw / 2.0f;
    float b2_raw = b0_raw;

    // 5. 归一化系数并赋值给结构体
    filter->b0 = b0_raw / a0;
    filter->b1 = b1_raw / a0;
    filter->b2 = b2_raw / a0;
    filter->a1 = a1_raw / a0;
    filter->a2 = a2_raw / a0;

    // 6. 清除所有历史状态，防止启动时的阶跃数据乱窜
    filter->x1 = 0.0f;
    filter->x2 = 0.0f;
    filter->y1 = 0.0f;
    filter->y2 = 0.0f;
}

float BiquadFilter_Apply(BiquadFilter_t *filter, float input) {
    // 执行差分方程： y[n] = b0*x[n] + b1*x[n-1] + b2*x[n-2] - a1*y[n-1] - a2*y[n-2]
    float output = filter->b0 * input + filter->b1 * filter->x1 + filter->b2 * filter->x2
                 - filter->a1 * filter->y1 - filter->a2 * filter->y2;
                 
    // 更新历史状态，为下一次计算做准备
    filter->x2 = filter->x1;
    filter->x1 = input;
    filter->y2 = filter->y1;
    filter->y1 = output;

    return output;
}