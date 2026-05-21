#ifndef __QMC6310_H
#define __QMC6310_H

#include "main.h"


#define QMC6310U_ADDR       0x38

#define QMC_REG_CHIPID      0x00 
#define QMC_REG_XOUT_LSB    0x01
#define QMC_REG_XOUT_MSB    0x02
#define QMC_REG_YOUT_LSB    0x03
#define QMC_REG_YOUT_MSB    0x04
#define QMC_REG_ZOUT_LSB    0x05
#define QMC_REG_ZOUT_MSB    0x06
#define QMC_REG_STATUS      0x09
#define QMC_REG_CTRL1       0x0A
#define QMC_REG_CTRL2       0x0B


#define QMC_REG_SIGN        0x29 


#define QMC_SENSITIVITY_8G  3750.0f

uint8_t QMC6310_Init(void);
uint8_t QMC6310_Read_Raw(int16_t *mag_raw);
uint8_t QMC6310_Read_Gauss(float *mag_gauss);

#endif