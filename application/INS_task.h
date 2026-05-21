#ifndef __INS_TASK_H__
#define __INS_TASK_H__
#include "bmi08_defs.h"
#define PI 3.14159265358979323846
#define DEG_TO_RAD 0.01745329 
#define RAD_TO_DEG 57.29578

typedef struct
{
	/*
	 raw data variable
	*/
	 struct bmi08_sensor_data bmi08_accel_raw;//raw accel data
   struct bmi08_sensor_data bmi08_gyro_raw;//raw gyro data
   float  INS_gyro[3];//after calibrated real gyro data
	 float  INS_accel[3];//after calibrated real accel data
   float  gyro_offset[3];//gyro calibrated data zero shift
   float  accel_offset[3];//accel calibrated data zero shift
	 float  INS_quat[4];//quaternion array
	 float  INS_angle_gyro[3];// gyro integral get euler angle
	 float  INS_angle_accel[3];// accel resolving get euler angle
	 float  INS_angle[3];//final output euler angle
	 float  alpha ;//scale factor
	 float altitude_ekf; 
   float velocity_z_ekf;
	
	 float test_altitude_ekf; 
   float test_velocity_z_ekf;
	
	/*
	Auxiliary initialization variable
	*/
  bool IMU_init_flag;  //accel Auxiliary initialization flag
	float pitch_setup_data;// pitch start angle
	float roll_setup_data;  //roll start angle
} IMU_data_t;
void INS_task(void const *pvParameters);
const IMU_data_t*get_imu_data_point(void);
#define INS_YAW_ADDRESS_OFFSET    0
#define INS_PITCH_ADDRESS_OFFSET  1
#define INS_ROLL_ADDRESS_OFFSET   2

#define BMI088_ACCEL_3G_SEN     0.0008974358974f
#define BMI088_ACCEL_6G_SEN     0.00179443359375f
#define BMI088_ACCEL_12G_SEN    0.0035888671875f
#define BMI088_ACCEL_24G_SEN    0.007177734375f


#define BMI088_GYRO_2000_SEN    0.00106526443603169529841533860381f
#define BMI088_GYRO_1000_SEN    0.00053263221801584764920766930190693f
#define BMI088_GYRO_500_SEN     0.00026631610900792382460383465095346f
#define BMI088_GYRO_250_SEN     0.00013315805450396191230191732547673f
#define BMI088_GYRO_125_SEN     0.000066579027251980956150958662738366f
#endif