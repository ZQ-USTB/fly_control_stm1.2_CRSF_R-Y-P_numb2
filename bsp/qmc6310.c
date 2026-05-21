#include "qmc6310.h"
#include "i2c.h" // 确保能调用 hi2c2 句柄
#include <math.h>
#include "main.h"
#include <string.h>
#include <stdio.h>

uint8_t chip_id = 0;
// 内部封装一个写寄存器函数
static void QMC_Write_Reg(uint8_t reg, uint8_t value)
{
    HAL_I2C_Mem_Write(&hi2c2, QMC6310U_ADDR, reg, I2C_MEMADD_SIZE_8BIT, &value, 1, 100);
}

// 内部封装一个读寄存器函数
static void QMC_Read_Regs(uint8_t reg, uint8_t *buf, uint16_t len)
{
    HAL_I2C_Mem_Read(&hi2c2, QMC6310U_ADDR, reg, I2C_MEMADD_SIZE_8BIT, buf, len, 100);
}

/**
 * @brief 初始化 QMC6310 磁力计
 * @return 0:成功, 1:失败(找不到芯片)
 */
uint8_t QMC6310_Init(void)
{
    // 1. 检查芯片是否在线并读取 CHIP ID
    QMC_Read_Regs(QMC_REG_CHIPID, &chip_id, 1);
    if(chip_id != 0x80) {
        return 1; // ID 不对或通信失败
    }

  // 软复位后，给传感器充足的苏醒时间 (向 SPL06 那种复杂的芯片学习)
    QMC_Write_Reg(QMC_REG_CTRL2, 0x80);

    QMC_Write_Reg(QMC_REG_SIGN, 0x06);
    QMC_Write_Reg(QMC_REG_CTRL2, 0x08);
//    QMC_Write_Reg(QMC_REG_CTRL1, 0xCF); // 建议先用 Continuous 模式排查
    QMC_Write_Reg(QMC_REG_CTRL1, 0xCD); 

    
    // 强制读回验证
    uint8_t check_val = 0;
    QMC_Read_Regs(QMC_REG_CTRL1, &check_val, 1);
    if (check_val != 0xCD) {
        // 如果这里没进，说明写入彻底失败了！
        return 2; 
    }
    return 0;
}

/**
 * @brief 读取 16位 原始磁场数据
 * @param mag_raw 长度为3的 int16 数组 [X, Y, Z]
 * @return 0:有新数据, 1:数据未准备好
 */
uint8_t QMC6310_Read_Raw(int16_t *mag_raw)
{
    uint8_t status = 0;
    uint8_t buf[6];
    
    // 检查状态寄存器的 DRDY (Data Ready) 位 (Bit 0)
    QMC_Read_Regs(QMC_REG_STATUS, &status, 1);
    if((status & 0x01) == 0) {
        return 1; // 数据还没准备好
    }
    
    // 连续读取 6 个字节的数据 (从 0x01 开始)
    QMC_Read_Regs(QMC_REG_XOUT_LSB, buf, 6);
    
    // 数据拼接 (由于是 2的补码，直接赋给 int16_t 即可处理正负号)
    mag_raw[0] = (int16_t)((buf[1] << 8) | buf[0]); // X轴
    mag_raw[1] = (int16_t)((buf[3] << 8) | buf[2]); // Y轴
    mag_raw[2] = (int16_t)((buf[5] << 8) | buf[4]); // Z轴
    
    return 0;
}

/**
 * @brief 读取转换为物理量的高斯值 (Gauss)
 * @param mag_gauss 长度为3的 float 数组 [X, Y, Z]
 * @return 0:读取成功, 1:无新数据
 */
uint8_t QMC6310_Read_Gauss(float *mag_gauss)
{
    int16_t raw[3];
    
    if(QMC6310_Read_Raw(raw) == 0) {
        // 根据手册 Table 2，±8G量程下的 Sensitivity 为 3750 LSB/G
        mag_gauss[0] = (float)raw[0] / QMC_SENSITIVITY_8G;
        mag_gauss[1] = (float)raw[1] / QMC_SENSITIVITY_8G;
        mag_gauss[2] = (float)raw[2] / QMC_SENSITIVITY_8G;
        return 0;
    }
    return 1;
}