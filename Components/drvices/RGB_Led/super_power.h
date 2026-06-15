#ifndef SUPER_POWER_H
#define SUPER_POWER_H

/* 包含头文件 ----------------------------------------------------------------*/
#include "struct_typedef.h"
#include "bsp_can.h"

/* 类型定义 ------------------------------------------------------------------*/
typedef enum
{
    SUPER_POWER_OFF = 0,
    SUPER_POWER_ON,
} SuperPowerMode_e;

typedef struct
{
    SuperPowerMode_e ctrl_mode;
    float pwoer_value;
    float bat_current;
    float cap_v;
    float bat_v;
    float power_limit;
} SuperPowerInfo_t;

/* 宏定义 --------------------------------------------------------------------*/

/* 扩展变量 ------------------------------------------------------------------*/
#define SUPER_POWER_V1_CONTROL_STD_ID   0x210
#define SUPER_POWER_V1_FEEDBACK_STD_ID   0x211

/* 函数声明 ------------------------------------------------------------------*/
void SuperPowerV1_DataParse(uint8_t data[], uint8_t len);
void SuperPowerV1_SendMessage(CAN_Object_t* obj, u16 power_limit);
void SuperPower_Ctrl(SuperPowerMode_e mode);
SuperPowerInfo_t* SuperPower_Pointer(void);

#endif  // SUPER_POWER_H

