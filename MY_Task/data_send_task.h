#ifndef DATA_SEND_TASK_H
#define DATA_SEND_TASK_H

#include <stdint.h>

typedef struct 
{
    uint16_t num1;
    uint16_t num2;
    float fnum1;
} Vofa_data;



void data_send_task(void *argument);
void vofa_send_task(void *argumen);
void timeStampTimerCallback(void const *argument);
void data_send_task_init();
void vofa_send_task_init();
void time_stamp_timer_init(void);

void PC_ReceiveCallback(uint8_t* data, uint16_t len);


#endif // DATA_SEND_TASK_H