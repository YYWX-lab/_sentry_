/* 包含头文件 ----------------------------------------------------------------*/
#include "gimbal_task.h"
#include "infantry_def.h"
#include "cmsis_os.h"
#include "gimbal_function.h"
#include "user_protocol.h"
#include "RV_protocol.h"
#include "RV_task.h"
#include "dm_motor_ctrl.h"
#include "dm_motor_drv.h"
#include "detect_task.h"
/* 私有类型定义 --------------------------------------------------------------*/

/* 私有宏定义 ----------------------------------------------------------------*/
/* gimbal back center time (ms) */
#define BACK_CENTER_TIME 2500

/* 私有变量 ------------------------------------------------------------------*/
/* 任务 */
osThreadId GimbalTaskHandle;
osTimerId mstimeStampTimerHandle;
#if INCLUDE_uxTaskGetStackHighWaterMark
static uint32_t gimbal_task_stack = 0;
#endif

/* control ramp parameter */
static ramp_v0_t yaw_ramp = RAMP_GEN_DAFAULT;
static ramp_v0_t pitch_ramp = RAMP_GEN_DAFAULT;
static ramp_v0_t vision_pitch_ramp = RAMP_GEN_DAFAULT;
static ramp_v0_t pitch_vision_ramp = RAMP_GEN_DAFAULT;
static uint8_t update_flag = 0;


/* 扩展变量 ------------------------------------------------------------------*/
extern GimbalHandle_t gimbal_handle;

extern RV_GB_MOVE_STR rv_gb_s;
extern RV_RX_STR RV_RXS;

/* 私有函数原形 --------------------------------------------------------------*/
static void GimbalSensorUpdata(void);
static void GimbalCtrlModeSwitch(void);
static void TimerCallback(void);
static void GimbalMotorSendCurrent(int16_t yaw, int16_t pitch);

static void GimbalInitMode(void);
static void GimbalGyroAngleMode(void);
static void GimbalRelativeAngleMode(void);
static void GimbalNormalMode(void);
static void GimbalVisionMode(void);

/* 函数体 --------------------------------------------------------------------*/
void GimbalTask(void *argument)
{
    for(;;)
    {
        GimbalSensorUpdata();
        GimbalCtrlModeSwitch();
        switch (gimbal_handle.ctrl_mode)
        {
            case GIMBAL_INIT:
            {
                GimbalInitMode();
            }break;

            case GIMBAL_GYRO:
            {
                GimbalGyroAngleMode();
            }break;

            case GIMBAL_RELATIVE:
            {
                GimbalRelativeAngleMode();
            }break;

            case GIMBAL_NORMAL:
            {
                GimbalNormalMode();
            }break;

            case GIMBAL_VISION:
            {
                GimbalVisionMode();
            }break;
            default:
                break;
        }

        update_flag = 1;

        GimbalMotorControl(&gimbal_handle.yaw_motor);
        GimbalMotorControl(&gimbal_handle.pitch_motor);

        if (gimbal_handle.ctrl_mode == GIMBAL_RELAX)
        {
            pid_clear(&gimbal_handle.yaw_motor.pid.outer_pid);
            pid_clear(&gimbal_handle.yaw_motor.pid.inter_pid);
            pid_clear(&gimbal_handle.pitch_motor.pid.outer_pid);
            pid_clear(&gimbal_handle.pitch_motor.pid.inter_pid);
            pid_clear(&gimbal_handle.yaw_motor.j4310_pid);
            gimbal_handle.yaw_motor.current_set = 0;
            gimbal_handle.pitch_motor.current_set = 0;
            dm_motor_disable(gimbal_handle.gimbal_can,&motor[Motor1]);
            if (!HAL_GPIO_ReadPin(KEY_GPIO_Port,KEY_Pin))
            {
                HAL_GPIO_EXTI_Callback(KEY_Pin);
            }
            // HAL_GPIO_EXTI_Callback(KEY_Pin);
        }


        GimbalMotorSendCurrent((int16_t)YAW_MOTO_POSITIVE_DIR * gimbal_handle.yaw_motor.current_set,
                               (int16_t)PITCH_MOTO_POSITIVE_DIR * gimbal_handle.pitch_motor.current_set);

        // Motor_SendMessage(gimbal_handle.gimbal_can, 0x3fe, 5555, 5555, 5555, 5555);
        osDelay(GIMBAL_TASK_PERIOD);

#if INCLUDE_uxTaskGetStackHighWaterMark
        gimbal_task_stack = uxTaskGetStackHighWaterMark(NULL);
#endif
    }
}

void GimbalTaskInit(void)
{
    ramp_v0_init(&yaw_ramp, BACK_CENTER_TIME/GIMBAL_TASK_PERIOD);
    ramp_v0_init(&pitch_ramp, BACK_CENTER_TIME/GIMBAL_TASK_PERIOD);
    osThreadDef(gimbal_task, GimbalTask, osPriorityNormal, 0, 256);
    GimbalTaskHandle = osThreadCreate(osThread(gimbal_task), NULL);
}

void ms_timer_init(void)
{
    // 1. 定义定时器参数：周期1ms，循环执行
    osTimerDef(mstimeStampTimer, TimerCallback);
    // 2. 创建定时器
    mstimeStampTimerHandle = osTimerCreate(osTimer(mstimeStampTimer), osTimerPeriodic, NULL);
    // 3. 启动定时器，周期1ms（FreeRTOS tick需配置为1ms，即configTICK_RATE_HZ=1000）
    if(mstimeStampTimerHandle != NULL)
    {
        osTimerStart(mstimeStampTimerHandle, 1);
    }
}


static void TimerCallback(void)
{
    // if (update_flag == 1)
    // {
    Comm_ChassisInfo_t* chassis_info = ChassisInfo_Pointer();
        if (gimbal_handle.yaw_motor.mode == J4310_MIT_VEL_MODE)
        {
            motor[Motor1].ctrl.pos_set = 0;
            motor[Motor1].ctrl.kp_set = 0.0f;
            motor[Motor1].ctrl.kd_set = 3.0f;
            // gimbal_handle.yaw_motor.given_value = gimbal_handle.yaw_motor.sensor.gyro_angle;
            if(chassis_info->mode == CHASSIS_SPIN)
            {
                motor[Motor1].ctrl.vel_set = -j4310_pid_calc(&gimbal_handle.yaw_motor.j4310_pid, gimbal_handle.yaw_motor.sensor.gyro_angle, gimbal_handle.yaw_motor.given_value-37.f);
            }
            else
            {
                motor[Motor1].ctrl.vel_set = -j4310_pid_calc(&gimbal_handle.yaw_motor.j4310_pid, gimbal_handle.yaw_motor.sensor.gyro_angle, gimbal_handle.yaw_motor.given_value);
            }
            // motor[Motor1].ctrl.vel_set = -j4310_pid_calc(&gimbal_handle.yaw_motor.j4310_pid, gimbal_handle.yaw_motor.sensor.gyro_angle, gimbal_handle.yaw_motor.given_value);
        }
        // motor[Motor1].ctrl.pos_set = -motor[Motor1].ctrl.pos_set ;
        else if(gimbal_handle.yaw_motor.mode == J4310_MIT_ANGLE_MODE)
        {
            motor[Motor1].ctrl.kp_set = 0.0f;
            motor[Motor1].ctrl.kd_set = 3.0f;
            motor[Motor1].ctrl.pos_set = 0;
            // gimbal_handle.yaw_motor.given_value = gimbal_handle.yaw_motor.sensor.relative_angle;

            motor[Motor1].ctrl.vel_set = j4310_pid_calc(&gimbal_handle.yaw_motor.j4310_pid, gimbal_handle.yaw_motor.sensor.relative_angle, gimbal_handle.yaw_motor.given_value);
        
        }
        update_flag = 0;

    // }
    // else 
    // {
    //     motor[Motor1].ctrl.pos_set = 0.0;
    //     motor[Motor1].ctrl.kp_set = 0.0f;
    //     motor[Motor1].ctrl.vel_set = 0.0f;

    // }

    dm_motor_ctrl_send(gimbal_handle.gimbal_can, &motor[Motor1]);
    if (CheckDeviceIsOffline(OFFLINE_GIMBAL_YAW))
    {
        dm_motor_enable(gimbal_handle.gimbal_can,&motor[Motor1]);//电机掉线重新使能
    }

}

static void GimbalSensorUpdata(void)
{
    // gimbal_handle.yaw_motor.sensor.relative_angle =  gimbal_handle.yaw_motor.ecd_ratio * (fp32)Motor_RelativePosition(gimbal_handle.yaw_motor.j4310_info->ecd,
    //                                                                                                                   gimbal_handle.yaw_motor.offset_ecd);

    gimbal_handle.yaw_motor.sensor.relative_angle = DM_AngleTransform(gimbal_handle.yaw_motor.position);
    gimbal_handle.pitch_motor.sensor.relative_angle =  gimbal_handle.pitch_motor.ecd_ratio * (fp32)Motor_RelativePosition(gimbal_handle.pitch_motor.motor_info->ecd,
                                                                                                                          gimbal_handle.pitch_motor.offset_ecd);
    gimbal_handle.yaw_motor.sensor.gyro_angle = gimbal_handle.imu->attitude.yaw;
    gimbal_handle.pitch_motor.sensor.gyro_angle = gimbal_handle.imu->attitude.pitch;
    gimbal_handle.yaw_motor.sensor.palstance = gimbal_handle.imu->gyro[2] * RAD_TO_ANGLE;
    gimbal_handle.pitch_motor.sensor.palstance = gimbal_handle.imu->gyro[1] * RAD_TO_ANGLE;
}

static void GimbalCtrlModeSwitch(void)
{
    gimbal_handle.last_ctrl_mode = gimbal_handle.ctrl_mode;
    if (gimbal_handle.console->gimbal_cmd == GIMBAL_RELEASE_CMD)
    {
        gimbal_handle.ctrl_mode = GIMBAL_RELAX;
        // gimbal_handle.ctrl_mode = GIMBAL_INIT;
    }
    else if (gimbal_handle.console->gimbal_cmd == GIMBAL_INIT_CMD)
    {
        gimbal_handle.ctrl_mode = GIMBAL_INIT;
    }
    else if (gimbal_handle.console->gimbal_cmd == GIMBAL_GYRO_CMD)
    {
        gimbal_handle.ctrl_mode = GIMBAL_GYRO;
    }
    else if (gimbal_handle.console->gimbal_cmd == GIMBAL_RELATIVE_CMD)
    {
        gimbal_handle.ctrl_mode = GIMBAL_RELATIVE;
    }
    else if (gimbal_handle.console->gimbal_cmd == GIMBAL_NORMAL_CMD)
    {
        gimbal_handle.ctrl_mode = GIMBAL_NORMAL;
    }
    else if (gimbal_handle.console->gimbal_cmd == GIMBAL_VISION_CMD)
    {
        gimbal_handle.ctrl_mode = GIMBAL_VISION;
    }
}


static void GimbalMotorSendCurrent(int16_t yaw_cur, int16_t pitch_cur)
{
    Motor_SendMessage(gimbal_handle.gimbal_can, GIMBAL_MOTOR_CONTROL_STD_ID, pitch_cur, 0, 0, 0);
    // osDelay(5);
    // float a = 5.0f;
    // DM_Motor_SendMessage(gimbal_handle.gimbal_can, DM_1TO4_CONTROL_STD_ID, yaw_cur, 0, 0, 0);
    // Motor_SendMessage(gimbal_handle.gimbal_can, 0x3FE, 1000, 1000, 1000, 1000);

}

static void GimbalInitMode(void)
{
    if(gimbal_handle.last_ctrl_mode != GIMBAL_INIT)
    {
        ramp_v0_init(&yaw_ramp, BACK_CENTER_TIME/GIMBAL_TASK_PERIOD);
        ramp_v0_init(&pitch_ramp, BACK_CENTER_TIME/GIMBAL_TASK_PERIOD);
    }

    

    // gimbal_handle.yaw_motor.mode = ENCONDE_MODE;
    gimbal_handle.pitch_motor.mode = ENCONDE_MODE;
    gimbal_handle.yaw_motor.mode = J4310_MIT_ANGLE_MODE;
    dm_motor_enable(gimbal_handle.gimbal_can,&motor[Motor1]);
    
    gimbal_handle.yaw_motor.given_value = gimbal_handle.yaw_motor.sensor.relative_angle;
    gimbal_handle.pitch_motor.given_value = gimbal_handle.pitch_motor.sensor.relative_angle * (1 - ramp_v0_calculate(&pitch_ramp));
    if (fabsf(gimbal_handle.pitch_motor.sensor.relative_angle) <= 2.0f)
    {
        gimbal_handle.yaw_motor.given_value = gimbal_handle.yaw_motor.sensor.relative_angle * (1 - ramp_v0_calculate(&yaw_ramp));


        if (fabsf(gimbal_handle.yaw_motor.sensor.relative_angle) <= 3.0f )
        {
            gimbal_handle.ctrl_mode = GIMBAL_NORMAL;
        }
    }

}

static void GimbalGyroAngleMode(void)
{
    gimbal_handle.yaw_motor.mode = J4310_MIT_VEL_MODE;
    gimbal_handle.pitch_motor.mode = GYRO_MODE;

    fp32 yaw_target = 0, pitch_target = 0;

    yaw_target = gimbal_handle.yaw_motor.given_value + gimbal_handle.console->gimbal.yaw_v ;//云台电机角度设置
    pitch_target = gimbal_handle.pitch_motor.given_value + gimbal_handle.console->gimbal.pitch_v ;

    gimbal_handle.yaw_motor.given_value = AngleTransform(yaw_target, gimbal_handle.yaw_motor.sensor.gyro_angle);//单位为度
    gimbal_handle.pitch_motor.given_value = AngleTransform(pitch_target, gimbal_handle.pitch_motor.sensor.gyro_angle);//单位为度
    // motor[Motor1].ctrl.pos_set = gimbal_handle.yaw_motor.given_value * PI/180.f;

    VAL_LIMIT(gimbal_handle.pitch_motor.given_value, gimbal_handle.pitch_motor.min_relative_angle, gimbal_handle.pitch_motor.max_relative_angle);

}

static void GimbalRelativeAngleMode(void)
{
    gimbal_handle.yaw_motor.mode = J4310_MIT_VEL_MODE;
    gimbal_handle.pitch_motor.mode = ENCONDE_MODE;


    
    gimbal_handle.yaw_motor.given_value += gimbal_handle.console->gimbal.yaw_v ;
    gimbal_handle.pitch_motor.given_value += gimbal_handle.console->gimbal.pitch_v;

    VAL_LIMIT(gimbal_handle.yaw_motor.given_value, gimbal_handle.yaw_motor.min_relative_angle, gimbal_handle.yaw_motor.max_relative_angle);
    // motor[Motor1].ctrl.pos_set = gimbal_handle.yaw_motor.given_value * PI/180.f;

    VAL_LIMIT(gimbal_handle.pitch_motor.given_value, gimbal_handle.pitch_motor.min_relative_angle, gimbal_handle.pitch_motor.max_relative_angle);

}

static void GimbalNormalMode(void)
{
    fp32 yaw_target = 0, yaw = 0;
    
    

    gimbal_handle.yaw_motor.mode = J4310_MIT_VEL_MODE;
    gimbal_handle.pitch_motor.mode = ENCONDE_MODE;

    Comm_ChassisInfo_t* chassis_info = ChassisInfo_Pointer();

    yaw_target = gimbal_handle.console->gimbal.yaw_v + gimbal_handle.yaw_motor.given_value;
    // ANGLE_LIMIT_360(yaw,gimbal_handle.yaw_motor.sensor.gyro_angle);
    

    // gimbal_handle.yaw_motor.given_value =  AngleTransform(yaw_target, gimbal_handle.yaw_motor.sensor.gyro_angle);

    if(chassis_info->mode == CHASSIS_SPIN)
    {
        gimbal_handle.yaw_motor.given_value =  AngleTransform(yaw_target, gimbal_handle.yaw_motor.sensor.gyro_angle);
    }
    else
    {
        gimbal_handle.yaw_motor.given_value =  AngleTransform(yaw_target, gimbal_handle.yaw_motor.sensor.gyro_angle);
    }
    // gimbal_handle.yaw_motor.given_value = yaw_target + gimbal_handle.yaw_motor.sensor.gyro_angle;
    // gimbal_handle.yaw_motor.given_value = yaw_target - gimbal_handle.yaw_motor.sensor.gyro_angle + gimbal_handle.yaw_motor.sensor.gyro_angle;
    // target_err = gimbal_handle.yaw_motor.given_value - gimbal_handle.last_yaw_given_value;

    // if (gimbal_handle.yaw_motor.given_value - gimbal_handle.last_yaw_given_value >= 358)
    // {
    //     target_err = 360;
    // }
    // else if (gimbal_handle.yaw_motor.given_value - gimbal_handle.last_yaw_given_value <= -358)
    // {
    //     target_err += 360;
    // }
    // if(gimbal_handle.total_yaw_target >= 720)
    // {
    //     gimbal_handle.total_yaw_target -= 1440;
    // }
    // else if (gimbal_handle.total_yaw_target <= -720)
    // {
    //     gimbal_handle.total_yaw_target += 1440;
    // }
    // gimbal_handle.total_yaw_target -= target_err;
    
    // gimbal_handle.yaw_motor.given_value = yaw_target + gimbal_handle.yaw_motor.sensor.gyro_angle;
    gimbal_handle.pitch_motor.given_value += gimbal_handle.console->gimbal.pitch_v;
    // motor[Motor1].ctrl.pos_set = gimbal_handle.total_yaw_target * PI/180.f;

    // gimbal_handle.last_yaw_given_value = gimbal_handle.yaw_motor.given_value;

    VAL_LIMIT(gimbal_handle.pitch_motor.given_value, gimbal_handle.pitch_motor.min_relative_angle, gimbal_handle.pitch_motor.max_relative_angle);

}

static void GimbalVisionMode(void)
{

   
    gimbal_handle.yaw_motor.mode = J4310_MIT_VEL_MODE;
    gimbal_handle.pitch_motor.mode = ENCONDE_MODE;

    fp32 yaw_target = 0 , pitch_angle = 0;//2025.3.10 add
    
    Comm_VisionInfo_t* info = VisionInfo_Pointer();
       
    if (info->is_track == 1)
    {
        if (info->up_date == 1)
        {
        
            info->up_date = 0;


            // gimbal_handle.yaw_motor.given_value = gimbal_handle.yaw_motor.sensor.gyro_angle + info->yaw_angle + gimbal_handle.console->gimbal.yaw_v;//2025.3.10加入

            // gimbal_handle.yaw_motor.given_value =  (rv_gb_s.yaw_e * 57.324 + (RV_RXS.v_yaw * rv_gb_s.t) * 57.324) + gimbal_handle.console->gimbal.yaw_v;


  
            pitch_angle += gimbal_handle.console->gimbal.pitch_v;
            

            gimbal_handle.pitch_motor.vision_angle = gimbal_handle.pitch_angle;

            // gimbal_handle.pitch_motor.given_value = gimbal_handle.pitch_motor.sensor.relative_angle - info->pitch_angle   + pitch_angle*3 ;
            gimbal_handle.pitch_motor.given_value = info->pitch_angle + pitch_angle*3;
            gimbal_handle.yaw_motor.given_value = info->yaw_angle + gimbal_handle.console->gimbal.yaw_v;
            // motor[Motor1].ctrl.pos_set = gimbal_handle.yaw_motor.given_value * PI/180.f;
            if (info->is_shoot == 1)
            {
                gimbal_handle.is_fire = 1;
            }
            else
            {
                gimbal_handle.is_fire = 0;
            }
            // gimbal_handle.pitch_motor.given_value = rv_gb_s.pitch_e * 57.324  + RV_PITCH_OFFSET + pitch_angle*3;
            // gimbal_handle.pitch_motor.given_value =gimbal_handle.pitch_motor.sensor.relative_angle+ rv_gb_s.pitch_e* 57.324*0.1   + pitch_angle*3;

       
        }

        
        else
        {

          
            yaw_target = gimbal_handle.yaw_motor.given_value + gimbal_handle.console->gimbal.yaw_v;

            gimbal_handle.yaw_motor.given_value = AngleTransform(yaw_target, gimbal_handle.yaw_motor.sensor.gyro_angle);
            gimbal_handle.pitch_motor.given_value += gimbal_handle.console->gimbal.pitch_v;
            

            VAL_LIMIT(gimbal_handle.pitch_motor.given_value, gimbal_handle.pitch_motor.min_relative_angle, gimbal_handle.pitch_motor.max_relative_angle);
        }

    }    
    else
    {
        yaw_target = gimbal_handle.yaw_motor.given_value + gimbal_handle.console->gimbal.yaw_v;

        gimbal_handle.yaw_motor.given_value = AngleTransform(yaw_target, gimbal_handle.yaw_motor.sensor.gyro_angle);
        gimbal_handle.pitch_motor.given_value += gimbal_handle.console->gimbal.pitch_v;
        

        VAL_LIMIT(gimbal_handle.pitch_motor.given_value, gimbal_handle.pitch_motor.min_relative_angle, gimbal_handle.pitch_motor.max_relative_angle);
    }


    VAL_LIMIT(gimbal_handle.pitch_motor.given_value, gimbal_handle.pitch_motor.min_relative_angle, gimbal_handle.pitch_motor.max_relative_angle);

}
