/**
 * Copyright (C) 2023 Bosch Sensortec GmbH. All rights reserved.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

#include <stdio.h>
#include <stdint.h>
#include <stdlib.h>

#include "common.h"
//#include "delay.h"
#include "spi.h"

#define	SPI_IMU_CS(x) PAout(x)  //选中IMU	

void SPI_IMU_CS_H(uint8_t dev_add)
{
    if (dev_add == 3)
    {
        HAL_GPIO_WritePin(CS1_GPIO_Port, CS1_Pin, GPIO_PIN_SET);
    }
    else if (dev_add == 4)
    {
        HAL_GPIO_WritePin(CS2_GPIO_Port, CS2_Pin, GPIO_PIN_SET);
    }
}
void SPI_IMU_CS_L(uint8_t dev_add)
{
    if (dev_add == 3)
    { 
        HAL_GPIO_WritePin(CS1_GPIO_Port, CS1_Pin, GPIO_PIN_RESET);
    }
    else if (dev_add == 4)
    {
        HAL_GPIO_WritePin(CS2_GPIO_Port, CS2_Pin, GPIO_PIN_RESET);
    }
}
/*
Systick功能实现us延时，参数SYSCLK为系统时钟
*/
uint32_t fac_us;
 
void HAL_Delay_us_init(uint16_t SYSCLK)
{
     fac_us=SYSCLK; 
}
 
void HAL_Delay_us(uint32_t nus)
{
    uint32_t ticks;
    uint32_t told,tnow,tcnt=0;
    uint32_t reload=SysTick->LOAD;
    ticks=nus*fac_us; 
    told=SysTick->VAL; 
    while(1)
    {
        tnow=SysTick->VAL;
        if(tnow!=told)
        {
            if(tnow<told)tcnt+=told-tnow;
            else tcnt+=reload-tnow+told;
            told=tnow;
            if(tcnt>=ticks)break; 
        }
    };
}
/******************************************************************************/
/*!                 Macro definitions                                         */
//#define BMI2XY_SHUTTLE_ID  UINT16_C(0x1B8)

/*! Macro that defines read write length */
//#define READ_WRITE_LEN     UINT8_C(46)

/******************************************************************************/
/*!                Static variable definition                                 */

/*! Variable that holds the I2C device address or SPI chip selection */
//static uint8_t dev_addr;

/*! Variable that holds the I2C or SPI bus instance */
//static uint8_t bus_inst;

/*! Structure to hold interface configurations */
//static struct coines_intf_config intf_conf;

/******************************************************************************/
/*!                User interface functions                                   */

/*! BMI08 shuttle id */
#define BMI08_SHUTTLE_ID_1  UINT16_C(0x86)
#define BMI08_SHUTTLE_ID_2  UINT16_C(0x66)

uint8_t acc_dev_add;
uint8_t gyro_dev_add;
/*!
 * I2C read function map to COINES platform
 */
BMI08_INTF_RET_TYPE bmi08_i2c_read(uint8_t reg_addr, uint8_t *reg_data, uint32_t len, void *intf_ptr)
{
//	uint8_t device_addr = *(uint8_t*)intf_ptr;
//    if (1 == IIC_Read_nByte(device_addr, reg_addr, len, reg_data))
//	{
//		return 1;
//	}
	return 0;
}

/*!
 * I2C write function map to COINES platform
 */
BMI08_INTF_RET_TYPE bmi08_i2c_write(uint8_t reg_addr, const uint8_t *reg_data, uint32_t len, void *intf_ptr)
{
//	uint8_t device_addr = *(uint8_t*)intf_ptr;
//    if ( 1 == IIC_Write_nByte(device_addr, reg_addr, len, reg_data) )
//	{
//		return 1;
//	}
	return 0;
}

/*!
 * SPI read function map to COINES platform
 */
BMI08_INTF_RET_TYPE bmi08_spi_read(uint8_t reg_addr, uint8_t *reg_data, uint32_t len, void *intf_ptr)
{
    //struct coines_intf_config intf_info = *(struct coines_intf_config *)intf_ptr;
    uint8_t device_addr = *(uint8_t*)intf_ptr;
    
	reg_addr |= 0x80;
    SPI_IMU_CS_L(device_addr);
    /* 写入要读的寄存器地址 */
    SPI1_ReadWriteByte(reg_addr);
    /* 读取寄存器数据 */
    while(len)
	{
		*reg_data = SPI1_ReadWriteByte(0x00);
		len--;
		reg_data++;
	}
    SPI_IMU_CS_H(device_addr);

    //return coines_read_spi(intf_info.bus, intf_info.dev_addr, reg_addr, reg_data, (uint16_t)len);
	return 0;
}

/*!
 * SPI write function map to COINES platform
 */
BMI08_INTF_RET_TYPE bmi08_spi_write(uint8_t reg_addr, const uint8_t *reg_data, uint32_t len, void *intf_ptr)
{
    //struct coines_intf_config intf_info = *(struct coines_intf_config *)intf_ptr;

    uint8_t device_addr = *(uint8_t*)intf_ptr;
    //return coines_write_spi(intf_info.bus, intf_info.dev_addr, reg_addr, (uint8_t *)reg_data, (uint16_t)len);
	SPI_IMU_CS_L(device_addr);
    /* 写入要读的寄存器地址 */
    /* 写入要读的寄存器地址 */
    SPI1_ReadWriteByte(reg_addr);
    /* 写寄存器数据 */
	for(int i = 0; i < len; i++)
	{
		SPI1_ReadWriteByte(reg_data[i]);
	}
    SPI_IMU_CS_H(device_addr);
	return 0;
}

/*!
 * Delay function map to COINES platform
 */
void bmi08_delay_us(uint32_t period, void *intf_ptr)
{
    //coines_delay_usec(period);
	HAL_Delay_us(period);
}



void bmi08_check_rslt(const char api_name[], int8_t rslt)
{
    switch (rslt)
    {
        case BMI08_OK:

            /* Do nothing */
            break;
        case BMI08_E_NULL_PTR:
            //printf("API [%s] Error [%d] : Null pointer\r\n", api_name, rslt);
            break;
        case BMI08_E_COM_FAIL:
           // printf("API [%s] Error [%d] : Communication failure\r\n", api_name, rslt);
            break;
        case BMI08_E_INVALID_CONFIG:
           // printf("API [%s] Error [%d] : Invalid configuration\r\n", api_name, rslt);
            break;
        case BMI08_E_DEV_NOT_FOUND:
           // printf("API [%s] Error [%d] : Device not found\r\n", api_name, rslt);
            break;
        case BMI08_E_OUT_OF_RANGE:
           // printf("API [%s] Error [%d] : Out of Range\r\n", api_name, rslt);
            break;
        case BMI08_E_INVALID_INPUT:
           // printf("API [%s] Error [%d] : Invalid Input\r\n", api_name, rslt);
            break;
        case BMI08_E_CONFIG_STREAM_ERROR:
           // printf("API [%s] Error [%d] : Config Stream error\r\n", api_name, rslt);
            break;
        case BMI08_E_RD_WR_LENGTH_INVALID:
           // printf("API [%s] Error [%d] : Invalid Read-write length\r\n", api_name, rslt);
            break;
        case BMI08_E_FEATURE_NOT_SUPPORTED:
           // printf("API [%s] Error [%d] : Feature not supported\r\n", api_name, rslt);
            break;
        case BMI08_W_FIFO_EMPTY:
          //  printf("API [%s] Warning [%d] : FIFO empty\r\n", api_name, rslt);
            break;
        default:
           // printf("API [%s] Error [%d] : Unknown error code\r\n", api_name, rslt);
            break;
    }
}

int8_t bmi08_interface_init(struct bmi08_dev *bmi08dev, uint8_t intf)
{
    int8_t rslt = BMI08_OK;

    if (bmi08dev != NULL)
    {
        /* Bus configuration : I2C */
        if (intf == BMI08_I2C_INTF)
        {
        //    printf("I2C Interface \n");

            bmi08dev->write = bmi08_i2c_write;
            bmi08dev->read = bmi08_i2c_read;

            acc_dev_add = (unsigned char) BMI08_ACCEL_I2C_ADDR_SECONDARY;
            gyro_dev_add = (unsigned char) BMI08_GYRO_I2C_ADDR_SECONDARY;
            bmi08dev->intf = BMI08_I2C_INTF;

            HAL_Delay(100);

            //result = coines_config_i2c_bus(COINES_I2C_BUS_0, COINES_I2C_STANDARD_MODE);
			//IIC_Init();

        }
        /* Bus configuration : SPI */
        else if (intf == BMI08_SPI_INTF)
        {
        //    printf("SPI Interface \n");

            bmi08dev->write = bmi08_spi_write;//spi写入api
            bmi08dev->read = bmi08_spi_read;//spi读取api

            bmi08dev->intf = BMI08_SPI_INTF;
            //acc_dev_add = COINES_SHUTTLE_PIN_8;
            
			
			     SPI_IMU_CS_H(3);			                //SPI 不选中
			     SPI_IMU_CS_H(4);			                //SPI 不选中

            acc_dev_add = 3;//3
            gyro_dev_add = 4;//4

        }

        bmi08dev->intf_ptr_accel = &acc_dev_add;
        bmi08dev->intf_ptr_gyro = &gyro_dev_add;
        bmi08dev->delay_us = bmi08_delay_us;
        bmi08dev->read_write_len = 1024;

        HAL_Delay(200);

    }
    else
    {
        rslt = BMI08_E_NULL_PTR;
    }

    return rslt;
}

void bmi08_coines_deinit(void)
{
//    (void)fflush(stdout);

//    (void)coines_set_shuttleboard_vdd_vddio_config(0, 0);
//    coines_delay_msec(100);

//    /* Coines interface reset */
//    coines_soft_reset();
//    coines_delay_msec(100);

//    (void)coines_close_comm_intf(COINES_COMM_INTF_USB, NULL);
}
