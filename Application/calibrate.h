#ifndef CALIBRATE_H
#define CALIBRATE_H

/* 包含头文件 ----------------------------------------------------------------*/
#include "struct_typedef.h"

/* 类型定义 ------------------------------------------------------------------*/
typedef uint8_t (*CalibrateHookFunc_t)(uint32_t *cali, uint8_t cmd);

typedef enum
{
    CALIBRATE_GIMBAL = 0,
    CALIBRATE_GYRO,
    CALIBRATE_ACC,
    CALIBRATE_MAG,
    //add more...
    CALI_LIST_LENGHT,
} Calibrate_ID_e;

typedef struct
{
    uint8_t cali_done;                                  //0x55 means has been calibrated
    uint8_t flash_len : 7;                              //buf lenght
    uint8_t cali_cmd : 1;                               //1 means to run cali hook function,
    uint32_t *flash_buffer;                             //link to device calibration data
    CalibrateHookFunc_t hook_func;                      //cali function
} CalibrateObject_t;

//gimbal device
typedef struct
{
    uint16_t yaw_offset;
    uint16_t pitch_offset;
    fp32 yaw_max_angle;
    fp32 yaw_min_angle;
    fp32 pitch_max_angle;
    fp32 pitch_min_angle;
} GimbalCalibrate_t;
//gyro, accel, mag device
typedef struct
{
    fp32 offset[3]; //x,y,z
} IMU_Calibrate_t;

/* 宏定义 --------------------------------------------------------------------*/

/* 扩展变量 ------------------------------------------------------------------*/

/* 函数声明 ------------------------------------------------------------------*/
void Calibrate_Init(void);

#endif  // CALIBRATE_H

