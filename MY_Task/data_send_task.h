#ifndef DATA_SEND_TASK_H
#define DATA_SEND_TASK_H

#include <stdint.h>

void data_send_task(void *argument);
void data_send_task_init();
void PC_ReceiveCallback(uint8_t* data, uint16_t len);


#endif // !DATA_SEND_TASK_H