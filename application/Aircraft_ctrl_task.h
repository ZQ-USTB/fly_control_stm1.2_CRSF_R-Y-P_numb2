#ifndef __AIRCRAFT_CTRL_TASK_H__
#define __AIRCRAFT_CTRL_TASK_H__
#include "remote_control.h"
#include "INS_task.h"
typedef enum
{
	
  Aircraft_ZERO_FORCE = 0, 
  Aircraft_InRing_P,    
  Aircraft_InRing_R,	
	Aircraft_OutRing_P,    
  Aircraft_OutRing_R,
  Aircraft_Y,
	Aircraft_InRing_Y,

  Aircraft_RUN,        
} aircraft_behaviour_e;

typedef struct
{
	const ELRS_Data *remote_data;
	const IMU_data_t *IMU_data;
	aircraft_behaviour_e aircraft_behaviour; 
  aircraft_behaviour_e last_aircraft_behaviour; 

} aircraft_control_t;
void Aircraft_ctrl_task(void const *pvParameters);



















#endif