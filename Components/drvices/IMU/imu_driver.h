#ifndef IMU_DRIVER_H
#define IMU_DRIVER_H

/* 包含头文件 ----------------------------------------------------------------*/
#include "struct_typedef.h"
#include "BMI088driver.h"
#include "ist8310driver.h"
#include "AHRS.h"

/* 类型定义 ------------------------------------------------------------------*/
typedef struct
{
    fp32 gyro[3];       /* 角速度 */
    fp32 accel[3];      /* 加速度 */
    fp32 mag[3];        /* 磁力计 */
    fp32 temp;          /* 温度 */
    fp32 temp_tar;      /* 目标温度 */
    fp32 quat[4];       /* 四元数 */

    /* 欧拉角 单位 rad */
    struct
    {

        fp32 yaw_buff[64];
        fp32 pitch;
        fp32 roll;
        fp32 yaw;
        fp32 yaw_last;
        fp32 true_yaw;
        fp32 true_yaw_last;
        fp32 err;
        fp32 err_p;
        fp32 yaw_aver;//漂移平均值
        fp32 yaw_mid ;
        fp32 b;
    } euler;

    /* 姿态角 */
    struct
    {
        float pitch;
        float roll;
        float yaw;
        float my_yaw;
        float my_yaw_2;
    } attitude;
} IMU_Data_t;

/* 宏定义 --------------------------------------------------------------------*/

/* 扩展变量 ------------------------------------------------------------------*/

/* 函数声明 ------------------------------------------------------------------*/
void IMU_Init(void);
void IMU_Update(const fp32 period_time);
IMU_Data_t* IMU_GetDataPointer(void);
void IMU_CalibrateGyro(fp32 cali_offset[3]);
void IMU_SetGyroOffset(fp32 cali_offset[3]);

#endif  // IMU_DRIVER_H

