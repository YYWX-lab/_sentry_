/* 包含头文件 ----------------------------------------------------------------*/
#include "super_power.h"

/* 私有类型定义 --------------------------------------------------------------*/

/* 私有宏定义 ----------------------------------------------------------------*/

/* 私有变量 ------------------------------------------------------------------*/
SuperPowerInfo_t super_power;
float cap_v;
/* 扩展变量 ------------------------------------------------------------------*/

/* 私有函数原形 --------------------------------------------------------------*/

/* 函数体 --------------------------------------------------------------------*/
//V1代表19版超电
void SuperPowerV1_DataParse(uint8_t data[], uint8_t len)
{
    // memcpy(&super_power.pwoer_value, data, sizeof(float));
    super_power.bat_v = (data[1]<<8)|data[0];
    super_power.bat_v /= 100;
    // cap_v = (data[3]<<8)|data[2];
    super_power.cap_v = (data[3]<<8)|data[2];
    super_power.cap_v /= 100;
    super_power.bat_current = (data[5]<<8)|data[4];
    super_power.bat_current /= 100;
}

// void SuperPowerV1_SendMessage(CAN_Object_t* obj, int16_t super_power_cmd, int16_t chassis_power, int16_t chassis_power_buffer, int16_t chassis_power_limit)
// {
    // uint8_t TxData[8] = {0};
//     TxData[0] = (uint8_t)(super_power_cmd >> 8);
//     TxData[1] = (uint8_t)super_power_cmd;
//     TxData[2] = (uint8_t)(chassis_power >> 8);
//     TxData[3] = (uint8_t)chassis_power;
//     TxData[4] = (uint8_t)(chassis_power_buffer >> 8);
//     TxData[5] = (uint8_t)chassis_power_buffer;
//     TxData[6] = (uint8_t)(chassis_power_limit >> 8);
//     TxData[7] = (uint8_t)chassis_power_limit;
//     BSP_CAN_TransmitData(obj, 0x210, TxData, 8);
// }

void SuperPowerV1_SendMessage(CAN_Object_t* obj, u16 power_limit)
{
    uint8_t TxData[2] = {0};
    TxData[0] = (uint8_t)(power_limit >> 8);
    TxData[1] = (uint8_t)power_limit ;
    BSP_CAN_TransmitData(obj, 0x210, TxData, sizeof(TxData));
}

void SuperPower_Ctrl(SuperPowerMode_e mode)
{
    super_power.ctrl_mode = mode;
}

SuperPowerInfo_t* SuperPower_Pointer(void)
{
    return &super_power;
}
