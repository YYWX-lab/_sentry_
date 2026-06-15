//
// Created by E_LJF on 2024/11/18.
//

#include "MY_protocol.h"
#include "crc.h"
#include "string.h"
#include "data_send_task.h"
#include "gimbal_app.h"
#include "bsp_uart.h"
#include "bsp_init.h"
#include <stdlib.h>

#define RX_BUF_LEN 16


uint8_t tx_pack_make(uint8_t *tx_pack,uint8_t header,uint8_t cmd,float f1,float f2, float f3 ,float f4, float f5, float f6, float f7, float f8, float f9, float f10, uint16_t d1 ,uint8_t game_state, uint8_t robot_id)//,uint16_t d1,uint16_t d2,uint16_t d3,uint16_t d4)
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

    p = (uint8_t *)(&f6);

    tx_pack[len] = p[3] & 0xff;
    tx_pack[len+1] = p[2] & 0xff;
    tx_pack[len+2] = p[1] & 0xff;
    tx_pack[len+3] = p[0] & 0xff;
    len += 4;

    p = (uint8_t *)(&f7);

    tx_pack[len] = p[3] & 0xff;
    tx_pack[len+1] = p[2] & 0xff;
    tx_pack[len+2] = p[1] & 0xff;
    tx_pack[len+3] = p[0] & 0xff;
    len += 4;

    p = (uint8_t *)(&f8);

    tx_pack[len] = p[3] & 0xff;
    tx_pack[len+1] = p[2] & 0xff;
    tx_pack[len+2] = p[1] & 0xff;
    tx_pack[len+3] = p[0] & 0xff;
    len += 4;

    p = (uint8_t *)(&f9);

    tx_pack[len] = p[3] & 0xff;
    tx_pack[len+1] = p[2] & 0xff;
    tx_pack[len+2] = p[1] & 0xff;
    tx_pack[len+3] = p[0] & 0xff;
    len += 4;

    p = (uint8_t *)(&f10);

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

    tx_pack[len] = robot_id;
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
            pack_analysis->state = 0;
            len = rx_pack[2];
            if(verify_crc16_check_sum(rx_pack,len))
            {
                pack_analysis->header = rx_pack[0];
                pack_analysis->cmd    = rx_pack[1];
                len = 3;

                uint8_t *fp = (uint8_t *)&pack_analysis->f1;
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
    else if (rx_pack[0] == VISION_HEAD)
    {
        pack_analysis->state = 1;
        len = rx_pack[2];
        if(verify_crc16_check_sum(rx_pack,len))
        {
            
            pack_analysis->header = rx_pack[0];
            pack_analysis->cmd    = rx_pack[1];
            len = 3;

            uint8_t *fp = (uint8_t *)&pack_analysis->f1;
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

            fp = (uint8_t *)&pack_analysis->f4;
            fp[0] = rx_pack[len];
            fp[1] = rx_pack[len+1];
            fp[2] = rx_pack[len+2];
            fp[3] = rx_pack[len+3];
            len += 4;

            fp = (uint8_t *)&pack_analysis->f5;
            fp[0] = rx_pack[len];
            fp[1] = rx_pack[len+1];
            fp[2] = rx_pack[len+2];
            fp[3] = rx_pack[len+3];
            len += 4;

            fp = (uint8_t *)&pack_analysis->f6;
            fp[0] = rx_pack[len];
            fp[1] = rx_pack[len+1];
            fp[2] = rx_pack[len+2];
            fp[3] = rx_pack[len+3];
            len += 4;

            fp = (uint8_t *)&pack_analysis->f7;
            fp[0] = rx_pack[len];
            fp[1] = rx_pack[len+1];
            fp[2] = rx_pack[len+2];
            fp[3] = rx_pack[len+3];
            len += 4;

            pack_analysis->d5 = rx_pack[len];
            len += 1;

            pack_analysis->is_shoot = rx_pack[len];



        }
    }
}

extern GimbalHandle_t gimbal_handle;
extern Vofa_data vofa_data;
// 解析后的 int16_t 数据，根据VOFA通道数调整
static uint16_t dat1 = 0, dat3 = 0;
static float dat2 = 0;

static uint8_t buff[30];


// /**
//  * @brief  普通字符串/指令解析
//  */
// void UART_String_Parse(uint8_t *buf)
// {
//     char temp[RX_BUF_LEN];
//     char *str = NULL;
//     // strcpy(temp, (char *)buf);
    
//     // 2. 解析纯数字（整数）
//     if(atoi((char*)buf) > 0)
//     {


//                 // 中断里必须用 taskENTER_CRITICAL_FROM_ISR / taskEXIT_CRITICAL_FROM_ISR
//         // UBaseType_t isr_mask = taskENTER_CRITICAL_FROM_ISR();

//         // vofa_data.num1 = (float)recv_num;

//         // taskEXIT_CRITICAL_FROM_ISR(isr_mask);
//         vofa_data.num1 = int16_t((char *)buf);
//     }

// }


// ASCII字符串转 int16_t
int16_t str_to_int16(uint8_t *str)
{
    int16_t res = 0;
    uint8_t sign = 0;  // 符号位 0正 1负

    // 处理负号
    // if(*str == '-')
    // {
    //     sign = 1;
    //     str++;
    // }

    // 逐字符转数字
    while( (*str >= '0') && (*str <= '9') )
    {
        res = res * 10 + (*str - '0');
        str++;
    }

    // if(sign) res = -res;
    return res;
}

// 分割逗号分隔的字符串，解析多组 int16_t
void vofa_parse(uint8_t *buf)
{
    uint8_t *p = buf;
    uint8_t temp_buf[50];
    uint8_t t_idx = 0;

    // 解析第一个数据
    while(*p != ',')
    {
        temp_buf[t_idx++] = *p++;
    }
    temp_buf[t_idx] = '\0';
    dat1 = str_to_int16(temp_buf);
    t_idx = 0;
    p++;

    // 解析第二个数据
    while(*p != ',')
    {
        temp_buf[t_idx++] = *p++;
    }
    temp_buf[t_idx] = '\0';
    dat2 = atof(temp_buf);
    t_idx = 0;
    p++;

    // 解析第三个数据（直到 \r）
    while(*p != '\r')
    {
        temp_buf[t_idx++] = *p++;
    }
    temp_buf[t_idx] = '\0';
    dat3 = str_to_int16(temp_buf);

    // BSP_UART_TransmitData(&com1_obj, buf, sizeof(buf));
    sprintf(buff,"samples: %.d, %.5f, %.d\r\n",dat1, dat2, dat3);
    BSP_UART_TransmitData(&com1_obj, buff, sizeof(buff)); 
    // }
    vofa_data.fnum1 = dat2;
    vofa_data.num1 = dat1;
}