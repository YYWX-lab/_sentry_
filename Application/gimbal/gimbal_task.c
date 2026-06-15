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

static uint64_t lost_tick = 0;//失去目标计数时间

/* control ramp parameter */
static ramp_v0_t yaw_ramp = RAMP_GEN_DAFAULT;
static ramp_v0_t pitch_ramp = RAMP_GEN_DAFAULT;
static ramp_v0_t vision_pitch_ramp = RAMP_GEN_DAFAULT;
static ramp_v0_t pitch_vision_ramp = RAMP_GEN_DAFAULT;
static uint8_t update_flag = 0;
static float pitch_add = 0;
static float yaw_add = 0;
static float pitch_target = 0;
static u8 last_track = 2;


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
static void InfantryVisionMode(void);
float traj_plan(float get, float set);
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

            case INFANTRY_VISION:
            {
                InfantryVisionMode();
            }
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


// 简易梯形轨迹规划（100Hz控制周期，阶跃角度转平滑运动）
float traj_plan(float get, float set)
{
    // 计算轨迹误差
    float err, traj_vel, traj_pos;
    // float err = dpid->outer_ref - dpid->traj_pos;
    err = set - traj_pos;
    // 360°环向修正
    if (err > 180)  err -= 360;
    if (err < -180) err += 360;
    
    // 简易速度规划（快速平滑，无冲击）
    if (err > 0) {
        traj_vel = 30*57.3;  // 正速度
        if (traj_vel > err) traj_vel = err;
    } else if (err < 0) {
        traj_vel = -30*57.3; // 负速度
        if (traj_vel < err) traj_vel = err;
    } else {
        traj_vel = 0; // 到达目标
    }
    
    // 更新轨迹位置
    traj_pos += traj_vel;
    return traj_vel;
}
static float tick = 0;
static void TimerCallback(void)
{
    // if (update_flag == 1)
    // {
    Comm_ChassisInfo_t* chassis_info = ChassisInfo_Pointer();
    Comm_VisionInfo_t* vision_info = VisionInfo_Pointer();
    float angle_err, traj_vel;
        if (gimbal_handle.yaw_motor.mode == J4310_MIT_VEL_MODE)
        {
            tick += 0.01;
            motor[Motor1].ctrl.pos_set = 0.0;
            motor[Motor1].ctrl.kp_set = 0.0f;
            motor[Motor1].ctrl.kd_set = 6.0f;
            
            // traj_vel = traj_plan(gimbal_handle.yaw_motor.sensor.gyro_angle, gimbal_handle.yaw_motor.given_value)/57.3f;
            // gimbal_handle.yaw_motor.given_value = gimbal_handle.yaw_motor.sensor.gyro_angle;

            // gimbal_handle.yaw_motor.given_value = 30*sinf(tick);
            
            if(chassis_info->mode == CHASSIS_SPIN)
            {
                motor[Motor1].ctrl.vel_set = -j4310_pid_calc(&gimbal_handle.yaw_motor.j4310_pid, gimbal_handle.yaw_motor.sensor.gyro_angle, gimbal_handle.yaw_motor.given_value-2) - vision_info->yaw_vel*1 - vision_info->yaw_acc*0.05 + chassis_info->yaw_speed/57.3f;//37
            }
            else
            {
                if(vision_info->distance !=0)
                {
                    motor[Motor1].ctrl.vel_set = -j4310_pid_calc(&gimbal_handle.yaw_motor.j4310_pid, gimbal_handle.yaw_motor.sensor.gyro_angle, gimbal_handle.yaw_motor.given_value) - vision_info->yaw_vel*1 - vision_info->yaw_acc*0.08 + chassis_info->yaw_speed/57.3f  ;// vision_info->yaw_acc*0.08
                }
                
                motor[Motor1].ctrl.vel_set = -j4310_pid_calc(&gimbal_handle.yaw_motor.j4310_pid, gimbal_handle.yaw_motor.sensor.gyro_angle, gimbal_handle.yaw_motor.given_value) - vision_info->yaw_vel*1 - vision_info->yaw_acc*0.08 + chassis_info->yaw_speed/57.3f   ;// vision_info->yaw_acc*0.08
                
                // motor[Motor1].ctrl.vel_set = 0;
                // motor[Motor1].ctrl.pos_set = gimbal_handle.yaw_motor.given_value;
                // motor[Motor1].ctrl.vel_set = 30*cosf(tick);
                // motor[Motor1].ctrl.vel_set = -vision_info->yaw_vel - traj_vel * 0.5;
                // angle_err = (gimbal_handle.yaw_motor.given_value - gimbal_handle.yaw_motor.sensor.gyro_angle);
                // if (angle_err > 180)
                // {
                //     angle_err -= 360;
                // }
                // else if (angle_err < -180)
                // {
                //     angle_err += 360;
                // }
                // motor[Motor1].ctrl.pos_set = -angle_err/57.3f;
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
    else if (gimbal_handle.console->gimbal_cmd == INFANTRY_VISION_CMD)
    {
        gimbal_handle.ctrl_mode = INFANTRY_VISION;
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
    gimbal_handle.pitch_motor.mode = GYRO_MODE;//


    
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
    gimbal_handle.pitch_motor.mode = GYRO_MODE;//ENCONDE_MODE

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


static void InfantryVisionMode(void)
{

   
    gimbal_handle.yaw_motor.mode = J4310_MIT_VEL_MODE;
    gimbal_handle.pitch_motor.mode = ENCONDE_MODE;
    
    fp32 yaw_target = 0 , pitch_angle = 0;//2025.3.10 add
    
    Comm_VisionInfo_t* info = VisionInfo_Pointer();
       
    if (info->is_track == 1)
    {
        if(info->is_track)
        lost_tick = 0;
        if (info->up_date == 1)
        {
        
            pitch_angle += gimbal_handle.console->gimbal.pitch_v;
            
            gimbal_handle.pitch_motor.vision_angle = gimbal_handle.pitch_angle;

            // gimbal_handle.pitch_motor.given_value = gimbal_handle.pitch_motor.sensor.relative_angle - info->pitch_angle   + pitch_angle*3 ;

            gimbal_handle.pitch_motor.given_value = info->pitch_angle + pitch_angle*3;
            gimbal_handle.yaw_motor.given_value = info->yaw_angle + gimbal_handle.console->gimbal.yaw_v ;

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
            last_track = 1;
       
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




static void GimbalVisionMode(void)
{

   
    gimbal_handle.yaw_motor.mode = J4310_MIT_VEL_MODE;
    gimbal_handle.pitch_motor.mode = GYRO_MODE;//ENCONDE_MODE
    
    fp32 yaw_target = 0 , pitch_angle = 0;//2025.3.10 add
    
    Comm_VisionInfo_t* info = VisionInfo_Pointer();
       
    if (info->is_track == 1)
    {
        if(info->is_track)
        lost_tick = 0;
        if (info->up_date == 1)
        {
            
            // if (last_track == 0 )
            // {
            // //     k = 5;
            // //     yaw_add = -45;
            //     for (u16 i; i<500; i++)
            //     {
            //         yaw_add -= 3;
            //         gimbal_handle.yaw_motor.given_value = AngleTransform(yaw_add, gimbal_handle.yaw_motor.sensor.gyro_angle);
            //     }

            // }
            // else
            // {
            //     yaw_add = 0;
            //     k = 0;
            // }
            // if (k == 5)
            // {
            //     for (u8 i; i<15; i++)
            //     {
            //         gimbal_handle.yaw_motor.given_value = info->yaw_angle + gimbal_handle.console->gimbal.yaw_v + yaw_add;
            //     }
            // }
            // else
            // {
            //     gimbal_handle.yaw_motor.given_value = info->yaw_angle + gimbal_handle.console->gimbal.yaw_v ;
            // }
            pitch_add = 0;
  
            pitch_angle += gimbal_handle.console->gimbal.pitch_v;
            

            gimbal_handle.pitch_motor.vision_angle = gimbal_handle.pitch_angle;
            gimbal_handle.pitch_motor.vision_speed = 0;
            // gimbal_handle.pitch_motor.given_value = gimbal_handle.pitch_motor.sensor.relative_angle - info->pitch_angle   + pitch_angle*3 ;

            gimbal_handle.pitch_motor.given_value = info->pitch_angle + pitch_angle*3 ;
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
            last_track = 1;
       
        }

        
        else
        {
            

            gimbal_handle.is_fire = 0;
            yaw_target = gimbal_handle.yaw_motor.given_value + gimbal_handle.console->gimbal.yaw_v;

            gimbal_handle.yaw_motor.given_value = AngleTransform(yaw_target, gimbal_handle.yaw_motor.sensor.gyro_angle);
            gimbal_handle.pitch_motor.given_value += gimbal_handle.console->gimbal.pitch_v;
            

            VAL_LIMIT(gimbal_handle.pitch_motor.given_value, gimbal_handle.pitch_motor.min_relative_angle, gimbal_handle.pitch_motor.max_relative_angle);
        }

    }    
    else
    {

        gimbal_handle.is_fire = 0;
        lost_tick ++;
        if (lost_tick >= 50)
        {
            last_track = 0;
            lost_tick = 50;
            yaw_add += 0.0;//0.5
            pitch_add += 0.08;//0.15
            pitch_target = 10*arm_sin_f32(pitch_add) - 6;
            yaw_target = gimbal_handle.yaw_motor.given_value + gimbal_handle.console->gimbal.yaw_v;
            yaw_target += 0.8;//2.5
            gimbal_handle.yaw_motor.given_value = AngleTransform(yaw_target, gimbal_handle.yaw_motor.sensor.gyro_angle);
            
        }
       
        
        // gimbal_handle.pitch_motor.given_value = pitch_target;
        gimbal_handle.pitch_motor.given_value += gimbal_handle.console->gimbal.pitch_v;
        gimbal_handle.pitch_motor.given_value = pitch_target;

        VAL_LIMIT(gimbal_handle.pitch_motor.given_value, gimbal_handle.pitch_motor.min_relative_angle, gimbal_handle.pitch_motor.max_relative_angle);
    }


    VAL_LIMIT(gimbal_handle.pitch_motor.given_value, gimbal_handle.pitch_motor.min_relative_angle, gimbal_handle.pitch_motor.max_relative_angle);

}
