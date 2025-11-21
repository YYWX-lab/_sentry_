#include "RV_task.h"

#include "comm_task.h"
#include "cmsis_os.h"
#include "infantry_def.h"

#include "gimbal_app.h"
#include "RV_protocol.h"
#include "bsp_uart.h"
#include "bsp_init.h"
#include "AHRS_MiddleWare.h"
#include "referee_system.h"




osThreadId RVTaskHandle;
#if INCLUDE_uxTaskGetStackHighWaterMark
static uint32_t RV_task_stack = 0;
#endif







extern GimbalHandle_t gimbal_handle;
extern ext_game_robot_state_t robot_state;

RV_RX_STR RV_RXS;


RV_GB_MOVE_STR rv_gb_s;


RV_TX_STR RV_TXS;

//KF_Struct RV_pitch_kalman;



#define Px 0.0
#define Py 0.0
#define Pz 0.8f




static uint8_t RV_TX_buff[RV_TX_PACK_SIZE];



static void RV_gb_move(RV_RX_STR* RV_RX,RV_GB_MOVE_STR* move_str)
{
	float x0 = RV_RX->x,y0 = RV_RX->y,z0 = RV_RX->z;
	float pitch_out;
	
	x0 = RV_RX->x*(1 - Px) + move_str->last_x * Px;
	y0 = RV_RX->y*(1 - Py) + move_str->last_y * Py;
	z0 = RV_RX->z*(1 - Pz) + move_str->last_z * Pz;
	
	
	move_str->s = AHRS_invSqrt(x0*x0 + y0*y0 + z0*z0);
	move_str->t = move_str->s / 22;
	
	z0 += RV_RX->vz * move_str->t;
	
//	z_out = KalmanUpdate(&RV_pitch_kalman,z0);
	
	move_str->yaw_e = AHRS_atan2f(y0,x0);
	move_str->pitch_e = AHRS_atan2f(z0,AHRS_invSqrt( (x0*x0 + y0*y0) ));
	

	
	move_str->last_x 	= RV_RX->x;
	move_str->last_y  = RV_RX->y;
	move_str->last_z  = RV_RX->z;
	
}



void RV_Task(void *argument)
{
    for(;;)
    {
		 

		 
		 
		//  if(robot_state.robot_id >= 11)
		//  {
		// 	 RV_TXS.detect_color = 0; //NOW ENEMY ROBO IS BLUE
		//  }
		//  else
		//  {
		// 	 RV_TXS.detect_color = 1;//NOW ENEMYq ROBO IS RED
		//  }
		 RV_TXS.detect_color = 0;//²âÊÔÓÃ
		 
		 RV_TXS.pitch = (gimbal_handle.imu->euler.pitch)*3.1415926/180;
		 RV_TXS.roll =  gimbal_handle.imu->euler.roll;
		 RV_TXS.yaw = gimbal_handle.imu->euler.yaw;
		 
		 
		 
		 
		 RV_MCU_TO_PC_PackMake(RV_TX_buff,&RV_TXS);
		 BSP_UART_TransmitData(&com2_obj, RV_TX_buff,RV_TX_PACK_SIZE);
		 
		 
		 RV_gb_move(&RV_RXS,&rv_gb_s);
		 
		 
		 osDelay(5);
		 
		 
		 
       

//#if INCLUDE_uxTaskGetStackHighWaterMark
//        RV_task_stack = uxTaskGetStackHighWaterMark(NULL);
//#endif
    }
}

void RV_TaskInit(void)
{
	
    osThreadDef(rv_task, RV_Task, osPriorityNormal, 0, 256);
    RVTaskHandle = osThreadCreate(osThread(rv_task), NULL);
}

