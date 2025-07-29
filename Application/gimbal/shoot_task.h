#ifndef SHOOT_TASK_H
#define SHOOT_TASK_H

/* 包含头文件 ----------------------------------------------------------------*/
#include "RemoteControl/remote_control.h"
#include "Motor/motor.h"
#include "Motor/blocked.h"
#include "pid.h"
#include "infantry_console.h"

/* 类型定义 ------------------------------------------------------------------*/
typedef enum
{
    SHOOT_RELAX = 0,          //安全模式
    SHOOT_START,
    SHOOT_STOP,
} ShootCtrlMode_e;

typedef enum
{
    MAGAZINE_INIT_STATE = 0,
    MAGAZINE_OFF_STATE,
    MAGAZINE_ON_STATE,
} MagazineState_e;

typedef enum
{
    TRIGGER_END = 0,
    TRIGGER_BEGIN,
    TRIGGERING
} TriggerState_e;

typedef struct
{
    MotorInfo_t*    motor_info;
    int32_t         offset_ecd;
    fp32            ecd_ratio;

    Double_PID_t    pid;
    fp32            speed;
    fp32            angle;
    fp32            set_speed;
    fp32            set_angle;
    BlockedHandle_t blocked_handle;

    int16_t         current_set;
} ShootMotor_t;

typedef struct
{
    MotorInfo_t*    motor_info;

    pid_t           pid;
    fp32            set_speed;

    int16_t         current_set;
} FrictionWheelMotor_t;

typedef struct
{
    Console_t*      console;
    CAN_Object_t*   shoot_can;    //

    ShootCtrlMode_e ctrl_mode;
    ShootMotor_t    magazine_motor; //弹仓盖电机
    ShootMotor_t    trigger_motor[2]; //拨弹盘电机
    FrictionWheelMotor_t  fric_wheel_motor[4]; //摩擦轮电机

    MagazineState_e magazine_state; //弹仓盖
    TriggerState_e  trigger_state;
    uint16_t        fire_bullet_number;
    uint16_t        bullet_count;

    uint16_t        shooter_heat_cooling_rate;
    uint16_t        shooter_heat_cooling_limit;
    uint16_t        shooter_speed_limit;
    uint16_t        shooter_heat;
} ShootHandle_t;

/* 宏定义 --------------------------------------------------------------------*/

/* 扩展变量 ------------------------------------------------------------------*/

/* 函数声明 ------------------------------------------------------------------*/
void ShootTaskInit(void);

#endif  // SHOOT_TASK_H

