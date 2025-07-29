//
// Created by E_LJF on 2024/11/18.
//

#include "MY_protocol.h"
#include "crc.h"


uint8_t tx_pack_make(uint8_t *tx_pack,uint8_t header,uint8_t cmd,float f1,float f2, float f3 ,float f4, float f5 , uint16_t d1, uint8_t game_state)//,uint16_t d1,uint16_t d2,uint16_t d3,uint16_t d4)
{
    uint8_t len = 0;
    uint8_t *p = (uint8_t *)(&f1);

    tx_pack[0] = header;
    tx_pack[1] = cmd;
    len += 3;

    tx_pack[len] = p[3] & 0xff;
    tx_pack[len+1] = p[2] & 0xff;
    tx_pack[len+2] = p[1] & 0xff;
    tx_pack[len+3] = p[0] & 0xff;
    len += 4;

    p = (uint8_t *)(&f2);

    tx_pack[len] = p[3] & 0xff;
    tx_pack[len+1] = p[2] & 0xff;
    tx_pack[len+2] = p[1] & 0xff;
    tx_pack[len+3] = p[0] & 0xff;
    len += 4;

    p = (uint8_t *)(&f3);

    tx_pack[len] = p[3] & 0xff;
    tx_pack[len+1] = p[2] & 0xff;
    tx_pack[len+2] = p[1] & 0xff;
    tx_pack[len+3] = p[0] & 0xff;
    len += 4;

    p = (uint8_t *)(&f4);

    tx_pack[len] = p[3] & 0xff;
    tx_pack[len+1] = p[2] & 0xff;
    tx_pack[len+2] = p[1] & 0xff;
    tx_pack[len+3] = p[0] & 0xff;
    len += 4;


    p = (uint8_t *)(&f5);

    tx_pack[len] = p[3] & 0xff;
    tx_pack[len+1] = p[2] & 0xff;
    tx_pack[len+2] = p[1] & 0xff;
    tx_pack[len+3] = p[0] & 0xff;
    len += 4;

    
    tx_pack[len] = d1 & 0xff;
    tx_pack[len+1] = (d1 >> 8) & 0xff;
    len += 2;

    tx_pack[len] = game_state;
    len += 1;
    // tx_pack[len] = d2 & 0xff;
    // tx_pack[len+1] = (d2 >> 8) & 0xff;
    // len += 2;

    //  tx_pack[len] = d3 & 0xff;
    // tx_pack[len+1] = (d3 >> 8) & 0xff;
    // len += 2;

    //  tx_pack[len] = d4 & 0xff;
    // tx_pack[len+1] = (d4 >> 8) & 0xff;
    // len += 2;

    tx_pack[2] = len + 2;

    append_crc16_check_sum(tx_pack,len + 2);
    len += 2;

    return len;
}


void pack_analysis(uint8_t *rx_pack,PACK_ANALYSIS_T *pack_analysis)
{
    uint8_t len = 0;
    if (rx_pack[0] == NAVIGATION_HEAD)
     {
            len = rx_pack[2];
            if(verify_crc16_check_sum(rx_pack,len))
            {
                pack_analysis->header = rx_pack[0];
                pack_analysis->cmd    = rx_pack[1];
                len = 3;

                uint8_t *fp = (uint8_t *)&pack_analysis->f1;

                // fp[3] = rx_pack[len];
                // fp[2] = rx_pack[len+1];
                // fp[1] = rx_pack[len+2];
                // fp[0] = rx_pack[len+3];
                fp[0] = rx_pack[len];
                fp[1] = rx_pack[len+1];
                fp[2] = rx_pack[len+2];
                fp[3] = rx_pack[len+3];
                


                len += 4;
                fp = (uint8_t *)&pack_analysis->f2;

                fp[0] = rx_pack[len];
                fp[1] = rx_pack[len+1];
                fp[2] = rx_pack[len+2];
                fp[3] = rx_pack[len+3];

                len += 4;
                fp = (uint8_t *)&pack_analysis->f3;

                fp[0] = rx_pack[len];
                fp[1] = rx_pack[len+1];
                fp[2] = rx_pack[len+2];
                fp[3] = rx_pack[len+3];

                len += 4;

                pack_analysis->d1 = (rx_pack[len+1] << 8) | rx_pack[len];
                len += 2;

                pack_analysis->d2 = (rx_pack[len+1] << 8) | rx_pack[len];
                len += 2;

                pack_analysis->d3 = (rx_pack[len+1] << 8) | rx_pack[len];
                len += 2;

                pack_analysis->d4 = (rx_pack[len+1] << 8) | rx_pack[len];
                len += 2;
            }

    }
}



