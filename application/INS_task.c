
#include "INS_task.h"
#include "cmsis_os.h"
#include <math.h>
#include "bmi088_mm.h"
#include "common.h"
#include "string.h"
#include "main.h"
#include "pid.h"
#include "ahrs.h"
#include "spl06.h"
#include "ekf.h"
#include "qmc6310.h"
#include "biquad_filter.h"
#define GRAVITY_EARTH  (9.80665f)
struct bmi08_sensor_data bmi08_accel;
struct bmi08_sensor_data bmi08_gyro;
struct bmi08_sensor_data bmi08_accel_offset;
struct bmi08_sensor_data bmi08_gyro_offset;
    int8_t rslt;
struct bmi08_dev bmi08;
IMU_data_t imu_data;
SPL06_t spl06_inst;
extern I2C_HandleTypeDef hi2c2;


Alt_EKF_t alt_ekf;
uint8_t ekf_initialized = 0;
uint32_t last_ekf_time = 0;
uint32_t baro_read_timer = 0;

	float accgyroval[6];
	int test_speed=0;
	uint32_t current_time=0;
	 uint32_t prev_time =0;
	float dt =0;
 float	timing_time =0.001;
static fp32 INS_gyro[3] = {0.0f, 0.0f, 0.0f};
static fp32 INS_accel[3] = {0.0f, 0.0f, 0.0f};
static fp32 INS_mag[3] = {0.0f, 0.0f, 0.0f};
static fp32 INS_quat[4] = {0.0f, 0.0f, 0.0f, 0.0f};
fp32 INS_angle[3] = {0.0f, 0.0f, 0.0f};
fp32 angle[3] = {0.0f, 0.0f, 0.0f};
#define SAMPLE_FREQ 500.0f
// #define M_PI 3.14159265358979323846f
volatile float beta = 0.2f;
volatile float q0 = 1.0f, q1 = 0.0f, q2 = 0.0f, q3 = 0.0f;
 float g_roll = 0.0f;  //  (X, )
 float g_pitch = 0.0f;
 float g_yaw = 0.0f;

float simple_yaw = 0.0f;
#define GZ_DEADBAND 0.3f

#define LPF_ALPHA 0.01f
static float filtered_gz_dps = 0.0f;
float Mag_Data[3];

int test1=5;
int test2=2;

// ====================================================================

// ====================================================================
BiquadFilter_t gyro_filter[3];
BiquadFilter_t accel_filter[3];

// uint32_t sign_0=0;
// int FLAG_T1=0;
// uint32_t sign_1=0;
// int FLAG_T2=0;
// float t1=0;
// float aaa=0;


void Yaw_Update_Integration(float gz_dps)
{


//    if (gz_dps > -GZ_DEADBAND && gz_dps < GZ_DEADBAND) {
//        gz_dps = 0.0f;
//    }
	  filtered_gz_dps = (LPF_ALPHA * gz_dps) + ((1.0f - LPF_ALPHA) * filtered_gz_dps);


    float deltat = 1.0f / SAMPLE_FREQ;


    simple_yaw += (filtered_gz_dps * deltat);


    imu_data.INS_angle[0] = simple_yaw;
}

/*********************************************************************/
/*                       Function Declarations                       */
/*********************************************************************/

float InvSqrt(float x) {
    float halfx = 0.5f * x;
    float y = x;
    long i = *(long*)&y;
    i = 0x5f3759df - (i>>1);
    y = *(float*)&i;
    y = y * (1.5f - (halfx * y * y));
    y = y * (1.5f - (halfx * y * y));
    return y;
}
void MadgwickAHRSupdateIMU(float gx, float gy, float gz, float ax, float ay, float az) {
    float recipNorm;
    float s0, s1, s2, s3;
    float qDot1, qDot2, qDot3, qDot4;
    float _2q0, _2q1, _2q2, _2q3, _4q0, _4q1, _4q2 ,_8q1, _8q2, q0q0, q1q1, q2q2, q3q3;


    float deltat = 1.0f / SAMPLE_FREQ;


    qDot1 = 0.5f * (-q1 * gx - q2 * gy - q3 * gz);
    qDot2 = 0.5f * (q0 * gx + q2 * gz - q3 * gy);
    qDot3 = 0.5f * (q0 * gy - q1 * gz + q3 * gx);
    qDot4 = 0.5f * (q0 * gz + q1 * gy - q2 * gx);


    if(!((ax == 0.0f) && (ay == 0.0f) && (az == 0.0f))) {


        recipNorm = InvSqrt(ax * ax + ay * ay + az * az);
        ax *= recipNorm;
        ay *= recipNorm;
        az *= recipNorm;

        //
        _2q0 = 2.0f * q0;
        _2q1 = 2.0f * q1;
        _2q2 = 2.0f * q2;
        _2q3 = 2.0f * q3;
        _4q0 = 4.0f * q0;
        _4q1 = 4.0f * q1;
        _4q2 = 4.0f * q2;
        _8q1 = 8.0f * q1;
        _8q2 = 8.0f * q2;
        q0q0 = q0 * q0;
        q1q1 = q1 * q1;
        q2q2 = q2 * q2;
        q3q3 = q3 * q3;


        s0 = _4q0 * q2q2 + _2q2 * ax + _4q0 * q1q1 - _2q1 * ay;
        s1 = _4q1 * q3q3 - _2q3 * ax + 4.0f * q0q0 * q1 - _2q0 * ay - _4q1 + _8q1 * q1q1 + _8q1 * q2q2 + _4q1 * az;
        s2 = 4.0f * q0q0 * q2 + _2q0 * ax + _4q2 * q3q3 - _2q3 * ay - _4q2 + _8q2 * q1q1 + _8q2 * q2q2 + _4q2 * az;
        s3 = 4.0f * q1q1 * q3 - _2q1 * ax + 4.0f * q2q2 * q3 - _2q2 * ay;


        recipNorm = InvSqrt(s0 * s0 + s1 * s1 + s2 * s2 + s3 * s3);
        s0 *= recipNorm;
        s1 *= recipNorm;
        s2 *= recipNorm;
        s3 *= recipNorm;


        qDot1 -= beta * s0;
        qDot2 -= beta * s1;
        qDot3 -= beta * s2;
        qDot4 -= beta * s3;
    }


    q0 += qDot1 * deltat;
    q1 += qDot2 * deltat;
    q2 += qDot3 * deltat;
    q3 += qDot4 * deltat;


    recipNorm = InvSqrt(q0 * q0 + q1 * q1 + q2 * q2 + q3 * q3);
    q0 *= recipNorm;
    q1 *= recipNorm;
    q2 *= recipNorm;
    q3 *= recipNorm;
}
void attitude_update_madgwick(float* INS_gyro, float* INS_accel)
{
    float gx_algo_rad = INS_gyro[1] * (M_PI / 180.0f); // Algo X = User Y
    float gy_algo_rad = -INS_gyro[0] * (M_PI / 180.0f); // Algo Y = -User X
    float gz_algo_rad = INS_gyro[2] * (M_PI / 180.0f); // Algo Z = User Z

    float ax_algo = INS_accel[1]; // Algo X = User Y
    float ay_algo = -INS_accel[0]; // Algo Y = -User X
    float az_algo = INS_accel[2]; // Algo Z = User Z

    MadgwickAHRSupdateIMU(gx_algo_rad, gy_algo_rad, gz_algo_rad,
                          ax_algo, ay_algo, az_algo);

//    float siny_cosp = 2.0f * (q0 * q3 + q1 * q2);
//    float cosy_cosp = 1.0f - 2.0f * (q2 * q2 + q3 * q3);
//    g_yaw = atan2f(siny_cosp, cosy_cosp) * (180.0f / M_PI);

    float sinp = 2.0f * (q0 * q1 + q2 * q3);
    float cosp = 1.0f - 2.0f * (q1 * q1 + q2 * q2);
    g_pitch = atan2f(sinp, cosp) * (180.0f / M_PI);

    sinp = 2.0f * (q0 * q2 - q3 * q1);
    if (fabsf(sinp) >= 1.0f) {
        g_roll = copysignf(M_PI / 2.0f, sinp) * (180.0f / M_PI);
    } else {
        g_roll = asinf(sinp) * (180.0f / M_PI);
    }
    g_roll = -g_roll; //  (Algo Y , User X )

// 		imu_data.INS_angle[0]=g_yaw;
		imu_data.INS_angle[1]=g_pitch-9.5;
		imu_data.INS_angle[2]=g_roll+6.5;
}
const IMU_data_t*get_imu_data_point(void)
{
	return &imu_data;
}
/*!
 * @brief This internal API is used to initialize the bmi08 sensor
 */
static void init_bmi08(struct bmi08_dev *bmi08dev);

/*!
 * @brief This internal API is used to configure accel and gyro data ready interrupts
 */
static void configure_accel_gyro_data_ready_interrupts(struct bmi08_dev *bmi08dev);

/*!
 *  @brief This function converts lsb to meter per second squared for 16 bit accelerometer at
 *  range 2G, 4G, 8G or 16G.
 *
 *  @param[in] val       : LSB from each axis.
 *  @param[in] g_range   : Gravity range.
 *  @param[in] bit_width : Resolution for accel.
 *
 *  @return Value in Meter Per second squared.
 */
static float lsb_to_mps2(int16_t val, float g_range, uint8_t bit_width);

/*!
 *  @brief This function converts lsb to degree per second for 16 bit gyro at
 *  range 125, 250, 500, 1000 or 2000dps.
 *
 *  @param[in] val       : LSB from each axis.
 *  @param[in] dps       : Degree per second.
 *  @param[in] bit_width : Resolution for gyro.
 *
 *  @return Value in Degree Per Second.
 */
static float lsb_to_dps(int16_t val, float dps, uint8_t bit_width);
/*********************************************************************/
/*                          Functions                                */
/*********************************************************************/

/*!
 *  @brief This internal API is used to initializes the bmi08 sensor
 *
 *  @param[in] void
 *
 *  @return void
 *
 */
static void init_bmi08(struct bmi08_dev *bmi08dev)
{
    int8_t rslt;

    /* Initialize bmi08 sensors (accel & gyro) */
    if (bmi088_mma_init(bmi08dev) == BMI08_OK && bmi08g_init(bmi08dev) == BMI08_OK)
    {
//        printf("BMI08 initialization success!\n");
//        printf("Accel chip ID - 0x%x\n", bmi08dev->accel_chip_id);
//        printf("Gyro chip ID - 0x%x\n", bmi08dev->gyro_chip_id);

        /* Reset the accelerometer */
        rslt = bmi08a_soft_reset(bmi08dev);

    }
    else
    {

//        printf("BMI08 initialization failure!\n");
        return;
    }

    if (rslt == BMI08_OK)
    {
        /* Set accel power mode */
        bmi08dev->accel_cfg.power = BMI08_ACCEL_PM_ACTIVE;
        rslt = bmi08a_set_power_mode(bmi08dev);
    }

    if (rslt == BMI08_OK)
    {
        bmi08dev->gyro_cfg.power = BMI08_GYRO_PM_NORMAL;
        rslt = bmi08g_set_power_mode(bmi08dev);
    }

    if (rslt == BMI08_OK)
    {
        bmi08dev->accel_cfg.odr = BMI08_ACCEL_ODR_1600_HZ;//100
        bmi08dev->accel_cfg.range = BMI088_MM_ACCEL_RANGE_12G;
        bmi08dev->accel_cfg.power = BMI08_ACCEL_PM_ACTIVE;
        bmi08dev->accel_cfg.bw = BMI08_ACCEL_BW_NORMAL; /* Bandwidth and OSR are same */

        rslt = bmi08a_set_power_mode(bmi08dev);

        if (rslt == BMI08_OK)
        {
            rslt = bmi088_mma_set_meas_conf(bmi08dev);
            bmi08_check_rslt("bmi088_mma_set_meas_conf", rslt);

            bmi08dev->gyro_cfg.odr = BMI08_GYRO_BW_230_ODR_2000_HZ;//32/100
            bmi08dev->gyro_cfg.range = BMI08_GYRO_RANGE_2000_DPS;//1000
            bmi08dev->gyro_cfg.bw = BMI08_GYRO_BW_230_ODR_2000_HZ;//32/100
            bmi08dev->gyro_cfg.power = BMI08_GYRO_PM_NORMAL;

            rslt = bmi08g_set_power_mode(bmi08dev);

            if (rslt == BMI08_OK)
            {
                rslt = bmi08g_set_meas_conf(bmi08dev);
                bmi08_check_rslt("bmi08g_set_meas_conf", rslt);
            }
        }
    }
}
static void configure_accel_gyro_data_ready_interrupts(struct bmi08_dev *bmi08dev)
{
    int8_t rslt;
    struct bmi08_accel_int_channel_cfg accel_int_config;
    struct bmi08_gyro_int_channel_cfg gyro_int_config;

    /* Configure the Interrupt configurations for accel */
    accel_int_config.int_channel = BMI08_INT_CHANNEL_1;
    accel_int_config.int_type = BMI08_ACCEL_INT_DATA_RDY;
    accel_int_config.int_pin_cfg.lvl = BMI08_INT_ACTIVE_HIGH;
    accel_int_config.int_pin_cfg.output_mode = BMI08_INT_MODE_PUSH_PULL;
    accel_int_config.int_pin_cfg.enable_int_pin = BMI08_ENABLE;

    /* Set the interrupt configuration */
    rslt = bmi088_mma_set_int_config(&accel_int_config, BMI088_MM_ACCEL_DATA_RDY_INT, bmi08dev);

    if (rslt == BMI08_OK)
    {
        /* Configure the Interrupt configurations for gyro */
        gyro_int_config.int_channel = BMI08_INT_CHANNEL_3;
        gyro_int_config.int_type = BMI08_GYRO_INT_DATA_RDY;
        gyro_int_config.int_pin_cfg.enable_int_pin = BMI08_ENABLE;
        gyro_int_config.int_pin_cfg.lvl = BMI08_INT_ACTIVE_HIGH;
        gyro_int_config.int_pin_cfg.output_mode = BMI08_INT_MODE_PUSH_PULL;

        rslt = bmi08g_set_int_config(&gyro_int_config, bmi08dev);
    }

    if (rslt != BMI08_OK)
    {
//        printf("Failure in interrupt configurations \n");
        return;
    }
}
/*!
 * @brief This function converts lsb to meter per second squared for 16 bit accelerometer at
 * range 2G, 4G, 8G or 16G.
 */
static float lsb_to_mps2(int16_t val, float g_range, uint8_t bit_width)
{
    double power = 2;

    float half_scale = (float)((pow((double)power, (double)bit_width) / 2.0f));

    return (GRAVITY_EARTH * val * g_range) / half_scale;
}

/*!
 * @brief This function converts lsb to degree per second for 16 bit gyro at
 * range 125, 250, 500, 1000 or 2000dps.
 */
static float lsb_to_dps(int16_t val, float dps, uint8_t bit_width)
{
    double power = 2;

    float half_scale = (float)((pow((double)power, (double)bit_width) / 2.0f));

    return (dps / (half_scale)) * (val);
}
void IMU_data_init(IMU_data_t *imu_data_init)
{

	memset(imu_data_init->accel_offset,0,sizeof(imu_data_init->accel_offset));
	memset(imu_data_init->gyro_offset,0,sizeof(imu_data_init->gyro_offset));
	memset(imu_data_init->INS_gyro,0,sizeof(imu_data_init->INS_gyro));
	memset(imu_data_init->INS_accel,0,sizeof(imu_data_init->INS_accel));
	memset(imu_data_init->accel_offset,0,sizeof(imu_data_init->accel_offset));
	memset(imu_data_init->INS_angle_gyro,0,sizeof(imu_data_init->INS_angle_gyro));
	memset(imu_data_init->INS_angle_accel,0,sizeof(imu_data_init->INS_angle_accel));
	imu_data_init->alpha=0.8;
	imu_data_init->IMU_init_flag=false;
	imu_data_init->altitude_ekf=95;

	test2=QMC6310_Init();
}
void offset_input(IMU_data_t *imu_data_offset)
{

	imu_data_offset->accel_offset[0]=484.75;//-301/61.51
	imu_data_offset->accel_offset[1]=279.4;//-207/-201.32
	imu_data_offset->accel_offset[2]=-30.6;//-59/-7.411

	imu_data_offset->gyro_offset[0]=1.3;//10.76/-1.531
	imu_data_offset->gyro_offset[1]=-5.675;//3.83/-0.549
	imu_data_offset->gyro_offset[2]=2.2;//1.47/-3.509
}

void autooffset(IMU_data_t *imu_data_cail)
{


	for(int i=0;i<1000;i++)
	{
  bmi088_mma_get_data(&bmi08_accel_offset, &bmi08);
	bmi08g_get_data(&bmi08_gyro_offset, &bmi08);
	imu_data_cail->accel_offset[0]+=bmi08_accel_offset.x;
	imu_data_cail->accel_offset[1]+=bmi08_accel_offset.y;
	imu_data_cail->accel_offset[2]+=bmi08_accel_offset.z;
	imu_data_cail->gyro_offset[0]+=bmi08_gyro_offset.x;
	imu_data_cail->gyro_offset[1]+=bmi08_gyro_offset.y;
	imu_data_cail->gyro_offset[2]+=bmi08_gyro_offset.z;
	HAL_Delay(10);
	}
	imu_data_cail->accel_offset[0]/=1000;
	imu_data_cail->accel_offset[1]/=1000;
	imu_data_cail->accel_offset[2]/=1000;
	imu_data_cail->gyro_offset[0]/=1000;
	imu_data_cail->gyro_offset[1]/=1000;
	imu_data_cail->gyro_offset[2]/=1000;
	imu_data_cail->accel_offset[2]-=5461.3;
}

float a_z_earth_G=0;
float acc_z_earth_mps2=0;

void INS_task(void const *pvParameters)
{
	  vTaskDelay(1000);
	  bmi08_interface_init(&bmi08, BMI08_SPI_INTF);
	  init_bmi08(&bmi08);
	  SPL06_Init(&spl06_inst, &hi2c2);
	  IMU_data_init(&imu_data);

	  offset_input(&imu_data);
	  uint8_t baro_divider = 0;

	  // ====================================================================


    // ====================================================================
    float filter_cutoff_acc =10.0f;
    float filter_cutoff_gyro_x = 10.0f;
 	  float filter_cutoff_gyro_y = 10.0f;
	  float filter_cutoff_gyro_z = 10.0f;


    BiquadFilter_Setup(&gyro_filter[0], SAMPLE_FREQ, filter_cutoff_gyro_x); //  X
    BiquadFilter_Setup(&gyro_filter[1], SAMPLE_FREQ, filter_cutoff_gyro_y); //  Y
    BiquadFilter_Setup(&gyro_filter[2], SAMPLE_FREQ, filter_cutoff_gyro_z); //  Z

    BiquadFilter_Setup(&accel_filter[0], SAMPLE_FREQ, filter_cutoff_acc);
    BiquadFilter_Setup(&accel_filter[1], SAMPLE_FREQ, filter_cutoff_acc);
    BiquadFilter_Setup(&accel_filter[2], SAMPLE_FREQ, filter_cutoff_acc);


	  while (1)
    {

			 baro_divider++;
        if (baro_divider >= 20)
        {
            baro_divider = 0;


            if (spl06_inst.data_ready) {

                SPL06_Process_Data(&spl06_inst);



                // imu_data.baro_alt = spl06_inst.altitude;
                // imu_data.baro_pressure = spl06_inst.pressure;

							       if (!ekf_initialized) {

                EKF_Init(&alt_ekf, spl06_inst.altitude_filtered);
                ekf_initialized = 1;
            } else {

                EKF_Update(&alt_ekf, spl06_inst.altitude_filtered);
            }




            }


            SPL06_Start_Read_IT(&spl06_inst);
        }

// ==========================================================

        // ==========================================================
        rslt = bmi088_mma_get_data(&bmi08_accel, &bmi08);
        bmi08_check_rslt("bmi088_mma_get_data", rslt);

        imu_data.bmi08_accel_raw.x = bmi08_accel.x;
        imu_data.bmi08_accel_raw.y = bmi08_accel.y;
        imu_data.bmi08_accel_raw.z = bmi08_accel.z;

        float raw_ax = lsb_to_mps2(bmi08_accel.x - imu_data.accel_offset[0], 12.0f, 16) / GRAVITY_EARTH;
        float raw_ay = lsb_to_mps2(bmi08_accel.y - imu_data.accel_offset[1], 12.0f, 16) / GRAVITY_EARTH;
        float raw_az = lsb_to_mps2(bmi08_accel.z - imu_data.accel_offset[2], 12.0f, 16) / GRAVITY_EARTH;


        INS_accel[0] = BiquadFilter_Apply(&accel_filter[0], raw_ax);
        INS_accel[1] = BiquadFilter_Apply(&accel_filter[1], raw_ay);
        INS_accel[2] = BiquadFilter_Apply(&accel_filter[2], raw_az);
     // ==========================================================

        // ==========================================================
        rslt = bmi08g_get_data(&bmi08_gyro, &bmi08);
        bmi08_check_rslt("bmi08g_get_data", rslt);

        imu_data.bmi08_gyro_raw.x = bmi08_gyro.x;
        imu_data.bmi08_gyro_raw.y = bmi08_gyro.y;
        imu_data.bmi08_gyro_raw.z = bmi08_gyro.z;

        float raw_gx = lsb_to_dps((int16_t)(bmi08_gyro.x - imu_data.gyro_offset[0]), 1000.0f, 16);
        float raw_gy = lsb_to_dps((int16_t)(bmi08_gyro.y - imu_data.gyro_offset[1]), 1000.0f, 16);
        float raw_gz = lsb_to_dps((int16_t)(bmi08_gyro.z - imu_data.gyro_offset[2]), 1000.0f, 16);


        INS_gyro[0] = BiquadFilter_Apply(&gyro_filter[0], raw_gx);
        INS_gyro[1] = BiquadFilter_Apply(&gyro_filter[1], raw_gy);
        INS_gyro[2] = BiquadFilter_Apply(&gyro_filter[2], raw_gz);

        imu_data.INS_gyro[0] = INS_gyro[0];
        imu_data.INS_gyro[1] = INS_gyro[1];
        imu_data.INS_gyro[2] = INS_gyro[2];


//			test1=QMC6310_Read_Gauss(Mag_Data);



     attitude_update_madgwick(INS_gyro, INS_accel);

		 //  Yaw ( + )
    Yaw_Update_Integration(INS_gyro[2]);
//			if(FLAG_T1==0)
//			{
//		  	if (g_roll>=45||g_pitch>=45)
//       {
//         sign_0=HAL_GetTick();
//         FLAG_T1=1;
//
//			 }
//		 	}
//			imu_data.INS_angle_accel[1]=atan2(-imu_data.INS_accel[0],sqrt(imu_data.INS_accel[1]*imu_data.INS_accel[1]+imu_data.INS_accel[2]*imu_data.INS_accel[2]))*RAD_TO_DEG;
//			imu_data.INS_angle_accel[2]=atan2(imu_data.INS_accel[1],imu_data.INS_accel[2])*RAD_TO_DEG;
//			aaa=imu_data.INS_angle_accel[1];
//			if(FLAG_T2==0)
//			{
//		  	if (imu_data.INS_angle_accel[1]>=45||imu_data.INS_angle_accel[2]>=45)
//       {
//         sign_1=HAL_GetTick();
//         FLAG_T2=1;
//			 }
//		 	}
//	    if(sign_1!=0&&sign_0!=0)
//			{
//			  t1=sign_0-sign_1;
//			}


			// ==========================================================

        // ==========================================================

        uint32_t current_time_ms = HAL_GetTick();


        float ax_algo = INS_accel[1];
        float ay_algo = -INS_accel[0];
        float az_algo = INS_accel[2];



         a_z_earth_G = 2.0f * (q1 * q3 - q0 * q2) * ax_algo +
                            2.0f * (q2 * q3 + q0 * q1) * ay_algo +
                            (q0 * q0 - q1 * q1 - q2 * q2 + q3 * q3) * az_algo;


         acc_z_earth_mps2 = a_z_earth_G * GRAVITY_EARTH - GRAVITY_EARTH;



        if (acc_z_earth_mps2 > 10.0f) acc_z_earth_mps2 = 10.0f;
        if (acc_z_earth_mps2 < -10.0f) acc_z_earth_mps2 = -10.0f;




        float dt_ekf = (current_time_ms - last_ekf_time) * 0.001f;
        if (dt_ekf <= 0.0f || dt_ekf > 0.05f) dt_ekf = 0.001f;
        last_ekf_time = current_time_ms;

        if (ekf_initialized) {
            EKF_Predict(&alt_ekf, acc_z_earth_mps2, dt_ekf);


            imu_data.altitude_ekf = alt_ekf.x[0];
				  	imu_data.test_altitude_ekf=alt_ekf.x[0]*100;


            imu_data.velocity_z_ekf = alt_ekf.x[1];
				    imu_data.test_velocity_z_ekf=alt_ekf.x[1]*100;
        }




			vTaskDelay(1);
		}

}