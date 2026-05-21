#include "Aircraft_ctrl_task.h"
#include "cmsis_os.h"
#include "remote_control.h"
#include "tim.h"
aircraft_control_t aircraft_control;
#include "pid.h"
#include "INS_task.h"


pid_type_def PID_control_pitch;  
pid_type_def PID_control_roll;  
pid_type_def PID_control_yaw;   

// --- ٶȻPIDṹ ---
pid_type_def PID_rate_pitch;
pid_type_def PID_rate_roll;// ROLLٶȻ

int32_t servo_value=0;
float pitch1_angle,yaw_angle,roll_angle,PITCH,ROLL,YAW = 0.0f;
float pitch1_bias,yaw_bias,roll_bias=0;

fp32 PID_R[3] = {8.0f, 0.0f, 0.0f};
fp32 PID_P1[3] = {6.0f, 0.0f, 0.0f};
fp32 PID_P2[3] = {5.0f, 0.0f, 0.0f};

fp32 PID_Rate_Roll[3] = {4.5f, 0.0f, 0.0f}; 
fp32 PID_Rate_Pitch[3] = {3.5f, 0.0f, 0.0f}; 

float PID_P=1.6;
float Pitch_Rate_Kp = 3.5f; 
float Pitch_Angle_Kp = 6.0f; 
float Roll_Rate_Kp = 4.5f; 
float Roll_Angle_Kp = 8.0f;  
float YAW_Kp = 5.0f;
float Roll_Rate_Set=0;
float Pitch_Rate_Set=0;
float YAW_angle_Set=0;
float INS_G0=0;
float INS_G1=0;

pid_type_def PID_rate_yaw;    

fp32 PID_Rate_Yaw[3] = {5.0f, 0.0f, 0.0f}; 
float Yaw_Rate_Kp = 5.0f;      
float Yaw_Rate_Set = 0;          
float INS_G2 = 0;               

/*

*/
void aircraft_init(aircraft_control_t*init);//ʼ
void aricraft_behaviour_mode_set(aircraft_control_t*aircraft_mode_set);
void aricraft_fly_ctrl(aircraft_control_t*aircraft_fly_ctrl);

/*

*/
void Aircraft_ctrl_task(void const *pvParameters)
{
	PID_init(&PID_control_pitch,PID_POSITION,PID_R,10000,-10000,10000);
	PID_init(&PID_control_roll,PID_POSITION,PID_P1,10000,-10000,10000);
	PID_init(&PID_control_yaw,PID_POSITION,PID_P2,10000,-10000,10000);
	
	PID_init(&PID_rate_roll, PID_POSITION, PID_Rate_Roll, 50000, -50000, 20000.0f);
	PID_init(&PID_rate_pitch, PID_POSITION, PID_Rate_Pitch, 50000, -50000, 20000.0f);
	
	PID_init(&PID_rate_yaw, PID_POSITION, PID_Rate_Yaw, 50000, -50000, 20000.0f); 

	
	aircraft_init(&aircraft_control);
	
	 while (1)
    {
      aricraft_behaviour_mode_set(&aircraft_control);
			aricraft_fly_ctrl(&aircraft_control);
			vTaskDelay(1);
		}
}

void aircraft_init(aircraft_control_t*init)
{
	 init->aircraft_behaviour=Aircraft_ZERO_FORCE;
	init->last_aircraft_behaviour=Aircraft_ZERO_FORCE;
	init->remote_data=get_remote_control_point();
	init->IMU_data=get_imu_data_point();
	/*
	IMUָȡ
	*/
}
void aricraft_behaviour_mode_set(aircraft_control_t*aircraft_mode_set)
{
	
	if(aircraft_mode_set->remote_data->B==0 && aircraft_mode_set->remote_data->C==0) {
    aircraft_mode_set->aircraft_behaviour = Aircraft_ZERO_FORCE;
 } else if(aircraft_mode_set->remote_data->B==2 && aircraft_mode_set->remote_data->C==0) {
    aircraft_mode_set->aircraft_behaviour = Aircraft_RUN;
 } else if(aircraft_mode_set->remote_data->C==1 && aircraft_mode_set->remote_data->B==0) {
    aircraft_mode_set->aircraft_behaviour = Aircraft_InRing_P;
 } else if(aircraft_mode_set->remote_data->B==0 && aircraft_mode_set->remote_data->C==2) {
    aircraft_mode_set->aircraft_behaviour = Aircraft_InRing_R;
 } else if(aircraft_mode_set->remote_data->B==1 && aircraft_mode_set->remote_data->C==2) {
    aircraft_mode_set->aircraft_behaviour = Aircraft_OutRing_P;
 } else if(aircraft_mode_set->remote_data->B==2 && aircraft_mode_set->remote_data->C==2) {
    aircraft_mode_set->aircraft_behaviour = Aircraft_OutRing_R;
 } else if(aircraft_mode_set->remote_data->B==1 && aircraft_mode_set->remote_data->C==0) {
    aircraft_mode_set->aircraft_behaviour = Aircraft_Y;
 }else if(aircraft_mode_set->remote_data->B==1 && aircraft_mode_set->remote_data->C==1) {
    aircraft_mode_set->aircraft_behaviour = Aircraft_InRing_Y;
 } else {
    aircraft_mode_set->aircraft_behaviour = Aircraft_ZERO_FORCE;
 }

 if(aircraft_mode_set->aircraft_behaviour==Aircraft_RUN&&aircraft_mode_set->last_aircraft_behaviour==Aircraft_ZERO_FORCE)
 {
	 YAW_angle_Set=aircraft_mode_set->IMU_data->INS_angle[0];
 }
 aircraft_mode_set->last_aircraft_behaviour=aircraft_mode_set->aircraft_behaviour;
}

//ͨ˲/////////////////////////////////////////////////////////////////////////////
float lowPass(float raw, float prevFiltered, float alpha1) {
  return alpha1 * raw + (1 - alpha1) * prevFiltered;
}

void Limit(aircraft_control_t*aircraft_fly_ctrl)
{
 if (roll_angle > 2100.0f)
        {
            roll_angle = 2100.0f;
        }
        else if (roll_angle < 900.0f)
        {
            roll_angle = 900.0f;
        }
				if (pitch1_angle > 2100.0f)
        {
            pitch1_angle = 2100.0f;
        }
        else if (pitch1_angle < 900.0f)
        {
            pitch1_angle = 900.0f;
        }

				if (yaw_angle > 2100.0f)
        {
            yaw_angle = 2100.0f;
        }
        else if (yaw_angle < 900.0f)
        {
            yaw_angle = 900.0f;
        }
}

static int bias_trigger_state = 0;

void Bias(aircraft_control_t* aircraft_fly_ctrl)
{

    if (aircraft_fly_ctrl->remote_data->F == 2)
    {
        // ֻ׼״̬²ִ߼
        if (bias_trigger_state == 0)
        {
            if (aircraft_fly_ctrl->remote_data->Right_X < -50)
            {
                roll_bias -= 10;
            }
            else if (aircraft_fly_ctrl->remote_data->Right_X > 50)
            {
                roll_bias += 10;
            }
            
            if (aircraft_fly_ctrl->remote_data->Right_Y < -50)
            {
                pitch1_bias -= 10;
            }
            else if (aircraft_fly_ctrl->remote_data->Right_Y > 50)
            {
                pitch1_bias += 10;
            }
            
            if (aircraft_fly_ctrl->remote_data->Left_X < -50)
            {
                yaw_bias -= 10;
            }
            else if (aircraft_fly_ctrl->remote_data->Left_X > 50)
            {
                yaw_bias += 10;
            }

 
            bias_trigger_state = 1; 
        }
    }
  
    else if (aircraft_fly_ctrl->remote_data->F == 0)
    {
        bias_trigger_state = 0; // һ F==2 
    }
}


void pid_ctrl(aircraft_control_t*aircraft_fly_ctrl)
{
        Roll_Rate_Set = PID_calc(&PID_control_roll, aircraft_fly_ctrl->IMU_data->INS_angle[2], 0.0f);
        Pitch_Rate_Set = PID_calc(&PID_control_pitch, aircraft_fly_ctrl->IMU_data->INS_angle[1], 0.0f);


        Yaw_Rate_Set = yaw_PID_calc(&PID_control_yaw, aircraft_fly_ctrl->IMU_data->INS_angle[0], YAW_angle_Set);
	
     
        ROLL = PID_calc(&PID_rate_roll, aircraft_fly_ctrl->IMU_data->INS_gyro[0], Roll_Rate_Set);
        PITCH = PID_calc(&PID_rate_pitch, aircraft_fly_ctrl->IMU_data->INS_gyro[1], Pitch_Rate_Set);
				// --- Yawڻ (INS_gyro[2]  Z ٶ) ---
        YAW = PID_calc(&PID_rate_yaw, aircraft_fly_ctrl->IMU_data->INS_gyro[2], Yaw_Rate_Set);
	
         roll_angle = 1500.0f -PITCH + roll_bias;
         pitch1_angle = 1500.0f - ROLL*0.5 + pitch1_bias;  
         yaw_angle = 1500.0f + YAW*0.5 + yaw_bias;
}

void WritePWM(aircraft_control_t*aircraft_fly_ctrl)
{
     __HAL_TIM_SetCompare(&htim5, TIM_CHANNEL_3, pitch1_angle);///
		 __HAL_TIM_SetCompare(&htim5, TIM_CHANNEL_2, roll_angle);///
		 __HAL_TIM_SetCompare(&htim5, TIM_CHANNEL_4, yaw_angle);///
}

void aricraft_fly_ctrl(aircraft_control_t*aircraft_fly_ctrl)
{
	if(aircraft_fly_ctrl->aircraft_behaviour==Aircraft_ZERO_FORCE)
	{
		__HAL_TIM_SetCompare(&htim3, TIM_CHANNEL_3, 1000);//ͣת
		__HAL_TIM_SetCompare(&htim3, TIM_CHANNEL_4, 1000);//ͣת	
	 __HAL_TIM_SetCompare(&htim5, TIM_CHANNEL_3, 1500+ pitch1_bias);//ʼǶ
	 __HAL_TIM_SetCompare(&htim5, TIM_CHANNEL_2, 1500+ roll_bias);
	 __HAL_TIM_SetCompare(&htim5, TIM_CHANNEL_4, 1500+ yaw_bias+90);
	}
	//////////////////////////////////////////////////////////////////////////////////////////////////YAW-inring
	else if(aircraft_fly_ctrl->aircraft_behaviour==Aircraft_InRing_Y)
	{
		if(aircraft_fly_ctrl->remote_data->F==2)
	{
		PID_P=0.1*aircraft_fly_ctrl->remote_data->Left_Y;
    YAW_Kp =PID_P;
    PID_rate_yaw.Kp = YAW_Kp;
	}
      YAW = PID_calc(&PID_rate_yaw, INS_G2, 0);
     yaw_angle = 1500.0f + YAW*0.5 + yaw_bias+90;
     Limit(&aircraft_control);
		 WritePWM(&aircraft_control);	
	   Bias(&aircraft_control);
	}
	//////////////////////////////////////////////////////////////////////////////////////////////////YAW
	else if(aircraft_fly_ctrl->aircraft_behaviour==Aircraft_Y)
	{
		if(aircraft_fly_ctrl->remote_data->F==2)
	{
		PID_P=0.1*aircraft_fly_ctrl->remote_data->Left_Y;
    YAW_Kp =PID_P;
    PID_control_yaw.Kp = YAW_Kp;
	}
     YAW=yaw_PID_calc(&PID_control_yaw, aircraft_fly_ctrl->IMU_data->INS_angle[0], YAW_angle_Set);
     yaw_angle = 1500.0f + YAW*0.5 + yaw_bias+90;
     Limit(&aircraft_control);
		 WritePWM(&aircraft_control);	
	   Bias(&aircraft_control);
	}
//////////////////////////////////////////////////////////////////////////////////////////////////
	else if(aircraft_fly_ctrl->aircraft_behaviour==Aircraft_InRing_P)
	{
		if(aircraft_fly_ctrl->remote_data->F==2)
	{
		PID_P=0.05*aircraft_fly_ctrl->remote_data->Left_Y;
    Pitch_Rate_Kp =PID_P;
    PID_rate_pitch.Kp = Pitch_Rate_Kp;
	}
  
     ROLL = PID_calc(&PID_rate_roll, aircraft_fly_ctrl->IMU_data->INS_gyro[0], 0);
     PITCH = PID_calc(&PID_rate_pitch, aircraft_fly_ctrl->IMU_data->INS_gyro[1], 0);
		 roll_angle = 1500.0f -PITCH + roll_bias;
     pitch1_angle = 1500.0f - ROLL*0.5 + pitch1_bias;  
     yaw_angle = 1500.0f + YAW*0.5 + yaw_bias+90;
     Limit(&aircraft_control);
		 WritePWM(&aircraft_control);	
	   Bias(&aircraft_control);
	}
//////////////////////////////////////////////////////////////////////////////////////////////////	
  else if(aircraft_fly_ctrl->aircraft_behaviour==Aircraft_InRing_R)
	{
		if(aircraft_fly_ctrl->remote_data->F==2)
	{
		PID_P=0.05*aircraft_fly_ctrl->remote_data->Left_Y;
    Roll_Rate_Kp =PID_P;
    PID_rate_roll.Kp = Roll_Rate_Kp;
	}
		
 
     ROLL = PID_calc(&PID_rate_roll, aircraft_fly_ctrl->IMU_data->INS_gyro[0], 0);
     PITCH = PID_calc(&PID_rate_pitch, aircraft_fly_ctrl->IMU_data->INS_gyro[1], 0);
		 roll_angle = 1500.0f -PITCH + roll_bias;
     pitch1_angle = 1500.0f - ROLL*0.5 + pitch1_bias;  
     yaw_angle = 1500.0f + YAW*0.5 + yaw_bias+90;
     Limit(&aircraft_control);
		 WritePWM(&aircraft_control);	
	   Bias(&aircraft_control);
	}
//////////////////////////////////////////////////////////////////////////////////////////////////	
	else if(aircraft_fly_ctrl->aircraft_behaviour==Aircraft_OutRing_P)
	{
		if(aircraft_fly_ctrl->remote_data->F==2)
	{
		PID_P=0.2*aircraft_fly_ctrl->remote_data->Left_Y;
    Pitch_Angle_Kp =PID_P;
    PID_control_pitch.Kp = Pitch_Angle_Kp;
	}
     pid_ctrl(&aircraft_control);
     Limit(&aircraft_control);
		 WritePWM(&aircraft_control);	
	   Bias(&aircraft_control);
	}
//////////////////////////////////////////////////////////////////////////////////////////////////	
	else if(aircraft_fly_ctrl->aircraft_behaviour==Aircraft_OutRing_R)
	{
		if(aircraft_fly_ctrl->remote_data->F==2)
	{
		PID_P=0.2*aircraft_fly_ctrl->remote_data->Left_Y;
    Roll_Angle_Kp=PID_P;
    PID_control_roll.Kp = Roll_Angle_Kp;
	}
     pid_ctrl(&aircraft_control);
     Limit(&aircraft_control);
		 WritePWM(&aircraft_control);	
	   Bias(&aircraft_control);
	}
//////////////////////////////////////////////////////////////////////////////////////////////////	
	else if(aircraft_fly_ctrl->aircraft_behaviour==Aircraft_RUN)
	{
					 servo_value=1000+(int)(aircraft_fly_ctrl->remote_data->Left_Y)*10;
     __HAL_TIM_SetCompare(&htim3, TIM_CHANNEL_3, servo_value);//
     __HAL_TIM_SetCompare(&htim3, TIM_CHANNEL_4, servo_value);//
        
		YAW_angle_Set=YAW_angle_Set+aircraft_fly_ctrl->remote_data->Left_X*0.0003;
	
		if(YAW_angle_Set>180)
		{
			YAW_angle_Set=YAW_angle_Set-360;
		}
		if(YAW_angle_Set<-180)
		{
			YAW_angle_Set=YAW_angle_Set+360;
		}
	
        Roll_Rate_Set = PID_calc(&PID_control_roll, aircraft_fly_ctrl->IMU_data->INS_angle[2], aircraft_fly_ctrl->remote_data->Right_X*0.3);
        Pitch_Rate_Set = PID_calc(&PID_control_pitch, aircraft_fly_ctrl->IMU_data->INS_angle[1], aircraft_fly_ctrl->remote_data->Right_Y*0.3);
        Yaw_Rate_Set = yaw_PID_calc(&PID_control_yaw, aircraft_fly_ctrl->IMU_data->INS_angle[0], YAW_angle_Set+aircraft_fly_ctrl->remote_data->Left_X*0.3);
		
		
		    INS_G0=lowPass(aircraft_fly_ctrl->IMU_data->INS_gyro[0],INS_G0, 1);
		    INS_G1=lowPass(aircraft_fly_ctrl->IMU_data->INS_gyro[1],INS_G1, 1);
	    	INS_G2=lowPass(aircraft_fly_ctrl->IMU_data->INS_gyro[2],INS_G2, 1); // --- Yawٶ˲ ---

        ROLL = PID_calc(&PID_rate_roll, INS_G0, Roll_Rate_Set);
        PITCH = PID_calc(&PID_rate_pitch, INS_G1, Pitch_Rate_Set);
		
		    YAW = PID_calc(&PID_rate_yaw, INS_G2, Yaw_Rate_Set);
				
        roll_angle = 1500.0f -PITCH + roll_bias;
        pitch1_angle = 1500.0f -ROLL*0.5 + pitch1_bias;  
        yaw_angle = 1500.0f +YAW*0.5 + yaw_bias+90;
		
//		    roll_angle = 1500.0f + roll_bias;
//        pitch1_angle = 1500.0f  + pitch1_bias;  
//        yaw_angle = 1500.0f +YAW*0.5 + yaw_bias;
		 // yaw_angle = 1500.0f + yaw_bias;
     Limit(&aircraft_control);
		 WritePWM(&aircraft_control);	
	}
}
