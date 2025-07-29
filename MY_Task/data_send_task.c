#include "data_send_task.h"

#include "bsp_uart.h"

#include "chassis_app.h"

#include "bsp_init.h"
#include "stdio.h"
#include "cmsis_os.h"
#include "MY_protocol.h"
#include "user_protocol.h"
#include "referee_system.h"


osThreadId SendDataTaskHandle;
#if INCLUDE_uxTaskGetStackHighWaterMark
static uint32_t send_data_task_stack = 0;
#endif



extern ist8310_real_data_t ist8310_handle;
extern ChassisHandle_t chassis_handle;
extern Comm_GimbalInfo_t gimbal_info;
extern ext_game_state_t game_state;
ext_game_robot_HP_t hp;
ext_game_robot_state_t robot_stste;
static uint8_t send_buff[40];
static u16 robot_hp;
static u8 robot_id;
static fp32 vx;
static fp32 vy;
static fp32 x;
static fp32 y;
static fp32 now_time;
static fp32 last_time;
static fp32 time_err;
static fp32 yaw;

// static uint8_t receive_buff[50] = "";

void data_send_task_init()
{
    osThreadDef(data_sendTask, data_send_task, osPriorityNormal, 0, 256);
    SendDataTaskHandle = osThreadCreate(osThread(data_sendTask), NULL);
}


void PC_ReceiveCallback(uint8_t* data, uint16_t len)
{
    PACK_ANALYSIS_T PACK_ANALYSIS_T_1;
    // ChassisHandle_t chassis_handle;
    
    pack_analysis(data, &PACK_ANALYSIS_T_1);
    // chassis_handle . vx = PACK_ANALYSIS_T_1 . f1;
    // chassis_handle . vy = PACK_ANALYSIS_T_1 . f2;
    // chassis_handle . vw = PACK_ANALYSIS_T_1 . f3;

    chassis_handle.vx_pc = PACK_ANALYSIS_T_1 . f1*1000;
    chassis_handle.vy_pc = -PACK_ANALYSIS_T_1 . f2*1000;
    chassis_handle . vw_pc = PACK_ANALYSIS_T_1 . f3;

    // BSP_UART_TransmitData(&com2_obj,"MCU REED\r\n",12);

}





void data_send_task(void *argument)
{



    for (;;)
    {
    //     chassis_handle.imu->euler.pitch
    //    sprintf(send_buff,"ECD: %d\r\nPITCH:%.2f\r\nROL:%.2f\r\nYAW:%.2f\r\n\r\n",
    //                                            chassis_handle.chassis_motor[1].motor_info->ecd,
    //                                            chassis_handle.imu->euler.pitch,
    //                                            chassis_handle.imu->euler.roll,
    //                                            chassis_handle.imu->euler.yaw
    //                                            );
    //    BSP_UART_TransmitData(&com2_obj,send_buff,sizeof(send_buff));
        now_time = HAL_GetTick();
        if (robot_stste.robot_id > 100)     //ID大于100是蓝方  
        {
            robot_hp = hp.blue_7_robot_HP;
            robot_id = 1;
        }
        else if (robot_stste.robot_id > 1) //红色置1
        {
            robot_hp = hp.red_7_robot_HP;
            robot_id = 0;
        }
        if (gimbal_info.yaw_gyro_angle<0)
        {
            yaw = gimbal_info.yaw_gyro_angle + 360;       
            yaw = gimbal_info.yaw_gyro_angle/57.295780490;
        }
        else
        {
            yaw = gimbal_info.yaw_gyro_angle/57.295780490;
        }
        vx = (chassis_handle.motor_speed[0]+chassis_handle.motor_speed[1]-chassis_handle.motor_speed[2]-chassis_handle.motor_speed[3])/2/1.414213562373095048801688f/1000;
        vy = (chassis_handle.motor_speed[0]-chassis_handle.motor_speed[1]-chassis_handle.motor_speed[2]+chassis_handle.motor_speed[3])/2/1.414213562373095048801688f/1000;
        time_err = (now_time - last_time)/100000000;
        x += ((arm_sin_f32(yaw)*vy + arm_cos_f32(yaw)*vx)*time_err);
        y += ((arm_sin_f32(yaw)*vx - arm_cos_f32(yaw)*vy)*time_err);
        tx_pack_make(send_buff,
        NAVIGATION_HEAD,
        MCU_TO_PC_SEND_CMD,
        // chassis_handle.imu->euler.pitch,
        // chassis_handle.imu->euler.roll,
        // chassis_handle.imu->euler.yaw,
        // chassis_handle.chassis_motor[0].motor_info->speed_rpm/60.f/19.f*0.47752208334564857224632179425848443839796f, //rpm -> m/s  （rpm/60秒/减速比19）*轮子周长0.473m
        // chassis_handle.chassis_motor[1].motor_info->speed_rpm/60.f/19.f*0.47752208334564857224632179425848443839796f, 
        // chassis_handle.chassis_motor[2].motor_info->speed_rpm/60.f/19.f*0.47752208334564857224632179425848443839796f,
        // chassis_handle.chassis_motor[3].motor_info->speed_rpm/60.f/19.f*0.47752208334564857224632179425848443839796f,
        
        x,
        y,
        vx,
        vy,
        // chassis_handle.motor_speed[2],
        // chassis_handle.motor_speed[3],
        // chassis_handle.imu->euler.true_yaw
        yaw,
        robot_hp,
        // robot_id,
        game_state.game_progress


        // chassis_handle.chassis_motor[0].motor_info->ecd,
        // chassis_handle.chassis_motor[1].motor_info->ecd,
        // chassis_handle.chassis_motor[2].motor_info->ecd,
        // chassis_handle.chassis_motor[3].motor_info->ecd
        );

        BSP_UART_TransmitData(&com2_obj,send_buff,sizeof(send_buff));
        
        osDelay(10);
        last_time = now_time;
    }
    
}




