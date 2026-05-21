#ifndef ALT_EKF_H
#define ALT_EKF_H

#include <stdint.h>

// EKF 结构体
typedef struct {
    // 状态向量: [0]=高度(m), [1]=垂直速度(m/s), [2]=加速度计Z轴零偏(m/s^2)
    float x[3];       
    
    // 协方差矩阵 3x3
    float P[3][3];    

    // 过程噪声协方差 (Q) - 决定对加速度计的信任程度
    float q_accel;    // 加速度计噪声方差
    float q_bias;     // 零偏漂移方差 (随机游走)

    // 观测噪声协方差 (R) - 决定对气压计的信任程度
    float r_baro;     // 气压计噪声方差
	
    
} Alt_EKF_t;

// 函数声明
void EKF_Init(Alt_EKF_t *ekf, float initial_alt);
void EKF_Predict(Alt_EKF_t *ekf, float acc_z_earth, float dt);
void EKF_Update(Alt_EKF_t *ekf, float baro_alt);
float LowPassFilter_1st(float raw_data) ;
#endif