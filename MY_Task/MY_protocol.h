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

}PACK_ANALYSIS_T;

uint8_t tx_pack_make(uint8_t *tx_pack,uint8_t header,uint8_t cmd,float f1,float f2,float f3 ,float f4, float f5 , uint16_t d1,uint8_t game_state);//,uint16_t d1,uint16_t d2,uint16_t d3,uint16_t d4);
void pack_analysis(uint8_t *rx_pack,PACK_ANALYSIS_T *pack_analysis);


#endif //F411_FLY_CTRL_MY_PROTOCOL_H
