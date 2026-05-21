#ifndef SPL06_H
#define SPL06_H

#include "main.h" // 包含 HAL 库

// --- SPL06 设备结构体 ---
typedef struct {
    // I2C 句柄
    I2C_HandleTypeDef *hi2c;

    // DMA 接收缓冲区 (必须 32 字节对齐以适配 STM32H7 D-Cache)
    uint8_t rx_buf[6] __attribute__((aligned(32)));

    // 状态标志位
    volatile uint8_t dma_busy;    // 1: DMA正在搬运中，不要发起新请求
    volatile uint8_t data_ready;  // 1: 数据已搬运到RAM，等待解算

    // 校准系数 (从寄存器读取)
    int16_t c0, c1, c01, c11, c20, c21, c30;
    int32_t c00, c10;

    // 缩放因子
    float kP; 
    float kT;

    // 最终数据
    int32_t raw_p;      // 原始气压 (调试用)
    float temperature;  // 摄氏度
    float pressure;     // 气压 (Pa)
    float altitude;     // 绝对高度 (m)
		float altitude_filtered;
		float test_altitude;     // 绝对高度 (m)
		float test_altitude_filtered;
    
} SPL06_t;

// --- API 函数声明 ---
uint8_t SPL06_Init(SPL06_t *dev, I2C_HandleTypeDef *hi2c);
void SPL06_Start_Read_IT(SPL06_t *dev);
void SPL06_Process_Data(SPL06_t *dev);
void SPL06_On_IT_Complete(SPL06_t *dev); // 放在中断回调中

#endif