//
// Created by E_LJF on 2024/11/18.
//

#ifndef F411_FLY_CTRL_MY_PROTOCOL_H
#define F411_FLY_CTRL_MY_PROTOCOL_H

#include "MY_protocol_configuration.h"
#include "main.h"

typedef struct PACK_ANALYSIS_T
{
    uint8_t header;
    uint8_t cmd;

    float f1;
    float f2;
    float f3;

    uint16_t d1;
    uint16_t d2;
    uint16_t d3;
    uint16_t d4;
    uint8_t  d5;

    uint8_t state;//视觉为1，导航为0

    uint8_t is_shoot;

}PACK_ANALYSIS_T;

uint8_t tx_pack_make(uint8_t *tx_pack,uint8_t header,uint8_t cmd,float f1,float f2,float f3 ,float f4, float f5 , float f6, float f7, float f8, uint16_t d1, uint8_t game_state,uint8_t robot_id);//,uint16_t d1,uint16_t d2,uint16_t d3,uint16_t d4);
void pack_analysis(uint8_t *rx_pack,PACK_ANALYSIS_T *pack_analysis);


#endif //F411_FLY_CTRL_MY_PROTOCOL_H
