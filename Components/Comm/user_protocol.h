#ifndef USER_PROTOCOL_H
#define USER_PROTOCOL_H

/* 包含头文件 ----------------------------------------------------------------*/
#include "struct_typedef.h"
#include "comm_protocol.h"
#include "chassis/chassis_app.h"
#include "gimbal/gimbal_app.h"
/* 类型定义 ------------------------------------------------------------------*/
typedef enum
{
    RC_DATA_CMD_ID           = 0x0001,
    CHASSIS_INFO_CMD_ID      = 0x0002,
    GIMBAL_INFO_CMD_ID       = 0x0003,
} USER_CMD_ID_e;

#pragma pack(push,1)
typedef struct
{
    ChassisCtrlMode_e mode;
    fp32    x_speed;
    fp32    y_speed;
} Comm_ChassisInfo_t;

typedef struct
{
    GimbalCtrlMode_e mode;
    /* unit: degree */
    fp32 pitch_ecd_angle;
    fp32 yaw_ecd_angle;
    fp32 pitch_gyro_angle;
    fp32 yaw_gyro_angle;
    /* uint: degree/s */
    fp32 pitch_rate;
    fp32 yaw_rate;
    uint8_t vision_up_date;
    fp32 vx_pc;
    fp32 vy_pc;
    fp32 vw_pc;
} Comm_GimbalInfo_t;

typedef struct 
{
    fp32    yaw_angle;//传下来的yaw轴角度（度）
    fp32    pitch_angle;//传下来的pitch轴角度（度）
    fp32    distance;//距离（mm）
    uint8_t is_track;
    uint8_t vision_up_date;
    uint8_t up_date;
    uint8_t is_shoot;//射击为1
} Comm_VisionInfo_t;

#pragma pack(pop)
/* 宏定义 --------------------------------------------------------------------*/
#define USER_PROTOCOL_HEADER_SOF     0xAA
/* 扩展变量 ------------------------------------------------------------------*/

/* 函数声明 ------------------------------------------------------------------*/
void UserProtocol_ParseHandler(uint16_t cmd_id, uint8_t* data, uint16_t len);
Comm_ChassisInfo_t* ChassisInfo_Pointer(void);
Comm_GimbalInfo_t* GimbalInfo_Pointer(void);
Comm_VisionInfo_t* VisionInfo_Pointer(void);

#endif  // USER_PROTOCOL_H

