#include "ekf.h"
#include <string.h>
// 滤波系数 alpha (范围 0.0 ~ 1.0)
// 值越小：滤波越平滑，抗噪越好，但延迟越大
// 值越大：跟随越快，延迟越小，但抗噪变差
#define LPF_ALPHA  0.1f 

   float q_accel_1=0.02;    // 过程噪声：适当调大可加快系统响应
   float q_bias_1=0.001;   // 零偏漂移噪声：通常很小
   float r_baro_1=1.0;

// 一阶低通滤波函数
float LowPassFilter_1st(float raw_data) {
    // 使用静态变量保存上一次的滤波结果
    static float prev_filtered = 0.0f;
    static uint8_t is_first_time = 1;

    // 第一次运行，直接赋值，防止数据从 0 开始缓慢爬升
    if (is_first_time) {
        prev_filtered = raw_data;
        is_first_time = 0;
        return prev_filtered;
    }

    // 核心滤波公式
    prev_filtered = LPF_ALPHA * raw_data + (1.0f - LPF_ALPHA) * prev_filtered;
    
    return prev_filtered;
}
// 初始化 EKF 
void EKF_Init(Alt_EKF_t *ekf, float initial_alt) {
    // 初始状态
    ekf->x[0] = initial_alt; // 初始高度
    ekf->x[1] = 0.0f;        // 初始速度
    ekf->x[2] = 0.0f;        // 初始零偏
	

    // 初始协方差 P (对角矩阵，表示初始不确定度)
    memset(ekf->P, 0, sizeof(ekf->P));
    ekf->P[0][0] = 0.5f;     // 高度不确定度
    ekf->P[1][1] = 0.5f;     // 速度不确定度
    ekf->P[2][2] = 0.1f;     // 零偏不确定度

//    // 调参关键：噪声协方差
//    ekf->q_accel = 0.05f;    // 过程噪声：适当调大可加快系统响应
//    ekf->q_bias  = 0.001f;   // 零偏漂移噪声：通常很小
//    ekf->r_baro  = 2.0f;     // 气压计测量噪声：气压计易受气流干扰，值较大
	//    // 调参关键：噪声协方差
    ekf->q_accel = q_accel_1;    // 过程噪声：适当调大可加快系统响应
    ekf->q_bias  = q_bias_1;   // 零偏漂移噪声：通常很小
    ekf->r_baro  = r_baro_1;     // 气压计测量噪声：气压计易受气流干扰，值较大
}

// 预测步 (使用加速度计数据推进系统状态)
// dt: 距离上一次 Predict 的时间间隔(秒)
// acc_z_earth: 去除重力后的地球系垂直加速度 (向上为正)
void EKF_Predict(Alt_EKF_t *ekf, float acc_z_earth, float dt) {
    float dt2 = dt * dt;
    
    // 1. 状态预测 (State Prediction)
    // u = 实际加速度 - 估算的零偏
    float u = acc_z_earth - ekf->x[2]; 
    
    // x = F * x + B * u
    ekf->x[0] = ekf->x[0] + ekf->x[1] * dt + 0.5f * u * dt2;
    ekf->x[1] = ekf->x[1] + u * dt;
    // ekf->x[2] (零偏) 保持不变

    // 2. 协方差预测 P = F * P * F^T + Q
    // 状态转移矩阵 F:
    // [1, dt, -0.5*dt^2]
    // [0,  1, -dt      ]
    // [0,  0,  1       ]
    float F[3][3] = {
        {1.0f, dt,   -0.5f * dt2},
        {0.0f, 1.0f, -dt},
        {0.0f, 0.0f, 1.0f}
    };

    float P_pred[3][3];
    float temp[3][3];

    // 计算 temp = P * F^T
    for (int i = 0; i < 3; i++) {
        for (int j = 0; j < 3; j++) {
            temp[i][j] = ekf->P[i][0] * F[j][0] + 
                         ekf->P[i][1] * F[j][1] + 
                         ekf->P[i][2] * F[j][2];
        }
    }

    // 计算 P_pred = F * temp + Q
    for (int i = 0; i < 3; i++) {
        for (int j = 0; j < 3; j++) {
            P_pred[i][j] = F[i][0] * temp[0][j] + 
                           F[i][1] * temp[1][j] + 
                           F[i][2] * temp[2][j];
        }
    }

    // 加上过程噪声 Q
    P_pred[1][1] += ekf->q_accel * dt2; // 速度的过程噪声
    P_pred[2][2] += ekf->q_bias * dt;   // 零偏的过程噪声

    // 更新 P
    memcpy(ekf->P, P_pred, sizeof(ekf->P));
}

// 更新步 (使用气压计高度修正状态)
// baro_alt: 气压计解算出的高度 (米)
void EKF_Update(Alt_EKF_t *ekf, float baro_alt) {
    // 观测矩阵 H = [1, 0, 0] (因为我们只直接测量高度)
    
    // 1. 计算卡尔曼增益 K = P * H^T * (H * P * H^T + R)^-1
    // 由于 H = [1, 0, 0]，(H * P * H^T) 直接简化为 P[0][0]
    float S = ekf->P[0][0] + ekf->r_baro; 
    float K[3];
    K[0] = ekf->P[0][0] / S;
    K[1] = ekf->P[1][0] / S;
    K[2] = ekf->P[2][0] / S;

    // 2. 状态更新 x = x + K * (Z - H * x)
    // Z 为气压计测量值，H * x 为预测高度 x[0]
    float innovation = baro_alt - ekf->x[0]; 
    
    ekf->x[0] += K[0] * innovation;
    ekf->x[1] += K[1] * innovation;
    ekf->x[2] += K[2] * innovation;

    // 3. 协方差更新 P = (I - K * H) * P
    // 代数化简：因为 H=[1,0,0]，所以只需要用 K 乘以 P 的第一行即可
    float P_new[3][3];
    for (int i = 0; i < 3; i++) {
        for (int j = 0; j < 3; j++) {
            P_new[i][j] = ekf->P[i][j] - K[i] * ekf->P[0][j];
        }
    }

    memcpy(ekf->P, P_new, sizeof(ekf->P));
}