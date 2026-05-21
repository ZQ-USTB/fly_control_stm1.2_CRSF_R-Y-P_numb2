#include "spl06.h"
#include <math.h>
#include "main.h"
#include "ekf.h"

// I2C 地址 (SDO=GND -> 0x76, 左移一位 -> 0xEC)
#define SPL06_I2C_ADDR (0x76 << 1)

// 寄存器地址
#define SPL06_REG_PRESS_DATA    0x00
#define SPL06_REG_PRS_CFG       0x06
#define SPL06_REG_TMP_CFG       0x07
#define SPL06_REG_MEAS_CFG      0x08
#define SPL06_REG_CFG_REG       0x09
#define SPL06_REG_RESET         0x0C
#define SPL06_REG_ID            0x0D
#define SPL06_REG_COEF          0x10

// 内部辅助：阻塞式读写 (仅用于初始化)
static void write_reg_block(SPL06_t *dev, uint8_t reg, uint8_t val) {
    HAL_I2C_Mem_Write(dev->hi2c, SPL06_I2C_ADDR, reg, I2C_MEMADD_SIZE_8BIT, &val, 1, 100);
}

static void read_regs_block(SPL06_t *dev, uint8_t reg, uint8_t *buf, uint16_t len) {
    HAL_I2C_Mem_Read(dev->hi2c, SPL06_I2C_ADDR, reg, I2C_MEMADD_SIZE_8BIT, buf, len, 100);
}

// --- 初始化函数 ---
uint8_t SPL06_Init(SPL06_t *dev, I2C_HandleTypeDef *hi2c) {
    dev->hi2c = hi2c;
    dev->dma_busy = 0;
    dev->data_ready = 0;

    uint8_t id = 0;
    uint8_t coef_buf[18];

    // 1. 复位并检查 ID
    read_regs_block(dev, SPL06_REG_ID, &id, 1);
    if ((id & 0xF0) != 0x10) return 1; // ID 错误

    // 2. 读取校准系数
    read_regs_block(dev, SPL06_REG_COEF, coef_buf, 18);

    // 3. 拼接系数 (标准逻辑)
    dev->c0 = (int16_t)((coef_buf[0] << 4) | (coef_buf[1] >> 4));
    if(dev->c0 & 0x0800) dev->c0 |= 0xF000;

    dev->c1 = (int16_t)(((coef_buf[1] & 0x0F) << 8) | coef_buf[2]);
    if(dev->c1 & 0x0800) dev->c1 |= 0xF000;

    dev->c00 = (int32_t)((coef_buf[3] << 12) | (coef_buf[4] << 4) | (coef_buf[5] >> 4));
    if(dev->c00 & 0x080000) dev->c00 |= 0xFFF00000;

    dev->c10 = (int32_t)(((coef_buf[5] & 0x0F) << 16) | (coef_buf[6] << 8) | coef_buf[7]);
    if(dev->c10 & 0x080000) dev->c10 |= 0xFFF00000;

    dev->c01 = (int16_t)((coef_buf[8] << 8) | coef_buf[9]);
    dev->c11 = (int16_t)((coef_buf[10] << 8) | coef_buf[11]);
    dev->c20 = (int16_t)((coef_buf[12] << 8) | coef_buf[13]);
    dev->c21 = (int16_t)((coef_buf[14] << 8) | coef_buf[15]);
    dev->c30 = (int16_t)((coef_buf[16] << 8) | coef_buf[17]);

    // 4. 配置传感器 (16x 过采样)
    // 压力: 16x -> Scale Factor = 253952
    dev->kP = 253952.0f;
    write_reg_block(dev, SPL06_REG_PRS_CFG, 0x54); // 16x oversampling

    // 温度: 1x -> Scale Factor = 524288
    dev->kT = 524288.0f;
    write_reg_block(dev, SPL06_REG_TMP_CFG, 0x80); // Ext sensor, 1x

    // 关键修正: 16x 过采样必须开启 Shift (bit 2 = 1)
    // 0x04 = P_SHIFT Enable
    write_reg_block(dev, SPL06_REG_CFG_REG, 0x04);

    // 连续测量模式 (Background Mode): Pressure + Temperature
    write_reg_block(dev, SPL06_REG_MEAS_CFG, 0x07);

    return 0; // 成功
}

// --- 步骤1: 启动 DMA 读取 (非阻塞) ---
void SPL06_Start_Read_IT(SPL06_t *dev) {
    if (dev->dma_busy) return; // 上一次还在传，跳过

    dev->dma_busy = 1;
    // 使用 DMA 读取 6 字节到结构体内的 rx_buf
   HAL_StatusTypeDef status = HAL_I2C_Mem_Read_IT(dev->hi2c, SPL06_I2C_ADDR, SPL06_REG_PRESS_DATA, 
                         I2C_MEMADD_SIZE_8BIT, dev->rx_buf, 6);
     if (status != HAL_OK) {
        dev->dma_busy = 0; // 出错复位
    }
}

// --- 步骤2: 中断回调 (极简，只置位) ---
void SPL06_On_IT_Complete(SPL06_t *dev) {
    dev->dma_busy = 0;
    dev->data_ready = 1; // 告诉任务数据好了
}

// --- 步骤3: 任务级解算 (含 Cache 维护和浮点运算) ---
void SPL06_Process_Data(SPL06_t *dev) {
    if (!dev->data_ready) return;

    // H7 必须操作: DMA 写了 RAM，CPU 读之前要清除 Cache
   // SCB_InvalidateDCache_by_Addr((uint32_t*)dev->rx_buf, 6);

    uint8_t *buf = dev->rx_buf;
    int32_t raw_p, raw_t;
    float P_scaled, T_scaled;

    // 1. 原始数据拼接 (修正后的 << 16 逻辑)
    raw_p = (int32_t)((buf[0] << 16) | (buf[1] << 8) | buf[2]);
    if (raw_p & 0x800000) raw_p |= 0xFF000000; // 符号扩展
    dev->raw_p = raw_p;

    raw_t = (int32_t)((buf[3] << 16) | (buf[4] << 8) | buf[5]);
    if (raw_t & 0x800000) raw_t |= 0xFF000000;

    // 2. 归一化
    P_scaled = (float)raw_p / dev->kP;
    T_scaled = (float)raw_t / dev->kT;

    // 3. 补偿计算
    float fTsc = T_scaled;
    float fPsc = P_scaled;
    
    dev->temperature = 0.5f * (float)dev->c0 + fTsc * (float)dev->c1;

    float qua2 = (float)dev->c10 + fPsc * ((float)dev->c20 + fPsc * (float)dev->c30);
    float qua3 = fTsc * fPsc * ((float)dev->c11 + fPsc * (float)dev->c21);
    
    dev->pressure = (float)dev->c00 + fPsc * qua2 + fTsc * (float)dev->c01 + qua3;

    // 4. 高度计算
    dev->altitude = 44330.0f * (1.0f - powf(dev->pressure / 101325.0f, 0.1903f));
		
		dev->test_altitude=dev->altitude*100;     // 绝对高度 (m)

		
    // 在这里调用一阶低通滤波
    dev->altitude_filtered = LowPassFilter_1st(dev->altitude);
		
		dev->test_altitude_filtered=dev->altitude_filtered*100; 
		
    dev->data_ready = 0; // 清除标志
}