#include "data_send_task.h"

#include "bsp_uart.h"

#include "chassis_app.h"
#include "app_init.h"
#include "bsp_init.h"
#include "stdio.h"
#include "cmsis_os.h"
#include "MY_protocol.h"
#include "user_protocol.h"
#include "referee_system.h"
#include "RV_protocol.h"
#include "dm_motor_drv.h"
#include "dm_motor_ctrl.h"
#include "detect_task.h"

#include "math.h"

osThreadId SendDataTaskHandle;
osThreadId VofaSendDataTaskHandle;
osTimerId timeStampTimerHandle;
#if INCLUDE_uxTaskGetStackHighWaterMark
static uint32_t send_data_task_stack = 0;
#endif



extern ist8310_real_data_t ist8310_handle;
extern ChassisHandle_t chassis_handle;
extern GimbalHandle_t gimbal_handle;
// extern Comm_GimbalInfo_t gimbal_info;
extern Comm_ChassisInfo_t chassis_info;
// extern ext_game_state_t game_state;
extern Comm_VisionInfo_t vision_info;
extern RV_GB_MOVE_STR rv_gb_s;
extern RV_RX_STR RV_RXS;


// ext_game_robot_HP_t hp;
// ext_game_robot_state_t robot_stste;
extern AppType_e app_type;
static uint8_t send_buff[60];
static uint8_t vofa_buff[100];
static u16 robot_hp;
static u8 enemy_colcor;//红色是1，蓝色是0
static fp32 vx;//x方向上的速度
static fp32 vy;//y方向上的速度
static fp32 x = 0;//走过的距离x
static fp32 y = 0;//走过的距离y
static fp32 now_time = 0;
static fp32 last_time = 0;
static fp32 time_err = 0;
static float yaw = 0;//弧度
static int32_t time_stamp = 0;
static u8 first_communicat = 0;
static fp32 k = 0;
static fp32 b = 0;



void data_send_task_init()
{
    osThreadDef(data_sendTask, data_send_task, osPriorityNormal, 0, 256);
    SendDataTaskHandle = osThreadCreate(osThread(data_sendTask), NULL);
}

void vofa_send_task_init()
{
    osThreadDef(vofa_sengTask, vofa_send_task, osPriorityNormal, 0, 256);
    VofaSendDataTaskHandle = osThreadCreate(osThread(vofa_sengTask), NULL);
}
//=============================================尚未使用==================================================
void time_stamp_timer_init(void)
{
    // 1. 定义定时器参数：周期1ms，循环执行
    osTimerDef(timeStampTimer, timeStampTimerCallback);
    // 2. 创建定时器
    timeStampTimerHandle = osTimerCreate(osTimer(timeStampTimer), osTimerPeriodic, NULL);
    // 3. 启动定时器，周期1ms（FreeRTOS tick需配置为1ms，即configTICK_RATE_HZ=1000）
    if(timeStampTimerHandle != NULL)
    {
        osTimerStart(timeStampTimerHandle, 1);
    }
}
//=============================================尚未使用===================================================

void PC_ReceiveCallback(uint8_t* data, uint16_t len)
{
    PACK_ANALYSIS_T PACK_ANALYSIS_T_1;
    // ChassisHandle_t chassis_handle;
    
    pack_analysis(data, &PACK_ANALYSIS_T_1);
    OfflineHandle_TimeUpdate(OFFLINE_VISION_INFO);
    // chassis_handle . vx = PACK_ANALYSIS_T_1 . f1;
    // chassis_handle . vy = PACK_ANALYSIS_T_1 . f2;
    // chassis_handle . vw = PACK_ANALYSIS_T_1 . f3;
    // if(app_type == CHASSIS_APP)
    // {
    //     chassis_handle.vx_pc = PACK_ANALYSIS_T_1 . f1*1000;
    //     chassis_handle.vy_pc = -PACK_ANALYSIS_T_1 . f2*1000;
    //     chassis_handle . vw_pc = PACK_ANALYSIS_T_1 . f3;
    // }
    if (app_type == GIMBAL_APP)
    {   

        first_communicat = 1;
        
        if(PACK_ANALYSIS_T_1.state == 1)
        {
            vision_info.yaw_angle = PACK_ANALYSIS_T_1.f1;
            vision_info.pitch_angle = PACK_ANALYSIS_T_1.f2;
            vision_info.distance = PACK_ANALYSIS_T_1.f3;
            vision_info.is_track = PACK_ANALYSIS_T_1.d5;
            vision_info.up_date = 1;
            vision_info.is_shoot = PACK_ANALYSIS_T_1.is_shoot;
            

            // if(vision_info.is_track == 0)
            // {
            //     vision_info.pitch_angle = 0;
            //     vision_info.yaw_angle = 0;
            // }

        }
        else if (PACK_ANALYSIS_T_1.state == 0)
        {
            Comm_GimbalInfo_t* gimbal_info =  GimbalInfo_Pointer();
            gimbal_info->vx_pc = PACK_ANALYSIS_T_1.f1 * 1000;
            gimbal_info->vy_pc = -PACK_ANALYSIS_T_1.f2 * 1000;
            gimbal_info->vw_pc = PACK_ANALYSIS_T_1.f3;
        }
        

    }
    
}


void timeStampTimerCallback(void const *argument)
{
    if (first_communicat == 1)
    {
        time_stamp++;
    }
    
}


void data_send_task(void *argument)
{



    for (;;)
    {

        ext_game_robot_state_t* robot_state = RefereeSystem_RobotState_Pointer();
        ext_game_state_t* game_state = Game_State_Pointer();

        now_time = HAL_GetTick();
        if (robot_state->robot_id > 60)     //ID大于100是蓝方  
        {
            robot_hp = robot_state->remain_HP;
            enemy_colcor = 1;
        }
        else if (robot_state->robot_id > 1) //红色置1
        {
            robot_hp = robot_state->remain_HP;
            enemy_colcor = 0;
        }
        else 
        {
            enemy_colcor = 0;
        }

        // if (app_type == CHASSIS_APP)
        // {
        //      if (gimbal_info.yaw_gyro_angle<0)
        //     {
        //         yaw = gimbal_info.yaw_gyro_angle + 360;       
        //         yaw = gimbal_info.yaw_gyro_angle/57.295780490;
        //     }
        //     else
        //     {
        //         yaw = gimbal_info.yaw_gyro_angle/57.295780490;
        //     }
        //     vx = (chassis_handle.motor_speed[0]+chassis_handle.motor_speed[1]-chassis_handle.motor_speed[2]-chassis_handle.motor_speed[3])/2/1.414213562373095048801688f/1000;
        //     vy = (chassis_handle.motor_speed[0]-chassis_handle.motor_speed[1]-chassis_handle.motor_speed[2]+chassis_handle.motor_speed[3])/2/1.414213562373095048801688f/1000;
        //     time_err = (now_time - last_time)/100000000;
        //     x += ((arm_sin_f32(yaw)*vy + arm_cos_f32(yaw)*vx)*time_err);
        //     y += ((arm_sin_f32(yaw)*vx - arm_cos_f32(yaw)*vy)*time_err);


        //     tx_pack_make(send_buff,
        //     NAVIGATION_HEAD,
        //     MCU_TO_PC_SEND_CMD,
        //     // chassis_handle.imu->euler.pitch,
        //     // chassis_handle.imu->euler.roll,
        //     // chassis_handle.imu->euler.yaw,
        //     // chassis_handle.chassis_motor[0].motor_info->speed_rpm/60.f/19.f*0.47752208334564857224632179425848443839796f, //rpm -> m/s  （rpm/60秒/减速比19）*轮子周长0.473m
        //     // chassis_handle.chassis_motor[1].motor_info->speed_rpm/60.f/19.f*0.47752208334564857224632179425848443839796f, 
        //     // chassis_handle.chassis_motor[2].motor_info->speed_rpm/60.f/19.f*0.47752208334564857224632179425848443839796f,
        //     // chassis_handle.chassis_motor[3].motor_info->speed_rpm/60.f/19.f*0.47752208334564857224632179425848443839796f,
            
        //     x,
        //     y,
        //     vx,
        //     vy,
        //     // chassis_handle.motor_speed[2],
        //     // chassis_handle.motor_speed[3],
        //     // chassis_handle.imu->euler.true_yaw
        //     yaw,
        //     robot_hp,
        //     // robot_id,
        //     game_state.game_progress,
        //     0


        //     // chassis_handle.chassis_motor[0].motor_info->ecd,
        //     // chassis_handle.chassis_motor[1].motor_info->ecd,
        //     // chassis_handle.chassis_motor[2].motor_info->ecd,
        //     // chassis_handle.chassis_motor[3].motor_info->ecd
        //     );
        // }
        if (app_type == GIMBAL_APP)
        {

            if (gimbal_handle.yaw_motor.sensor.relative_angle<0)
            {
                yaw = gimbal_handle.yaw_motor.sensor.relative_angle + 360;       
                yaw = gimbal_handle.yaw_motor.sensor.relative_angle/57.295780490;//转化为弧度制
            }
            else
            {
                yaw = gimbal_handle.yaw_motor.sensor.relative_angle/57.295780490;
            }
            vy = chassis_info.y_speed;
            vx = chassis_info.x_speed;
            time_err = (now_time - last_time)/1000;

            // k = sinf(yaw);
            // b = cosf(yaw);
            x += ((sinf(yaw)*vy + cosf(yaw)*vx)*time_err);
            y += -((-sinf(yaw)*vx + cosf(yaw)*vy)*time_err);
            // float yaw_speed = ((float)gimbal_handle.yaw_motor.motor_info->speed_rpm)*0.1046666f;
            float yaw_speed = motor[Motor1].para.vel;
            float pitch_speed = ((float)gimbal_handle.pitch_motor.motor_info->speed_rpm)*0.1046666f;
            tx_pack_make(send_buff,
            VISION_HEAD,
            PC_TO_MCU_RECEIVE,
            gimbal_handle.pitch_motor.sensor.relative_angle,
            gimbal_handle.yaw_motor.sensor.gyro_angle,
            yaw_speed,
            pitch_speed,
            x,
            y,
            vx,
            vy,
            robot_hp,
            game_state->game_progress,
            enemy_colcor
            );
        }
       

        BSP_UART_TransmitData(&com2_obj,send_buff,sizeof(send_buff));
        
        osDelay(10);
        last_time = now_time;
    }
    
}


void vofa_send_task(void *argumen)
{
    for (;;)
    {
        Comm_VisionInfo_t* info = VisionInfo_Pointer();
        fp32 pitch_given_current = (fp32)gimbal_handle.pitch_motor.motor_info->given_current;
        fp32 pitch_current_set = (fp32)gimbal_handle.pitch_motor.current_set;
        
        sprintf(vofa_buff, "samples: %.2f, %.2f, %.2f, %.2f,%.2f, %.2f\r\n",info->pitch_angle,
                                                                        gimbal_handle.pitch_motor.sensor.relative_angle,
                                                                        info->yaw_angle,
                                                                        gimbal_handle.yaw_motor.sensor.gyro_angle,
                                                                        pitch_given_current,
                                                                        gimbal_handle.yaw_motor.j4310_info->rad_s);

        BSP_UART_TransmitData(&com1_obj,vofa_buff,sizeof(vofa_buff));     
        
        osDelay(10);
    }
}



