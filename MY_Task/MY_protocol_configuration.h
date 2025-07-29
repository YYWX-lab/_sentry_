//
// Created by E_LJF on 2024/11/18.
//

#ifndef F411_FLY_CTRL_MY_PROTOCOL__H
#define F411_FLY_CTRL_MY_PROTOCOL__H

/*-----------------------帧头定义----------------------------*/

#define FLOAT_HEAD          0xA1    //浮点数帧头
#define UINT16_HEAD         0xA2    //uint_16数据帧头
#define PID_HEAD            0xA3    //PID调参数据帧头
#define NAVIGATION_HEAD     0xDF    //导航数据帧头

/*----------------------------------------------------------*/


/*------------------------命令定义---------------------------*/

#define MOVE_CMD                0xC1    //运动模式命令
#define LOCK_CMD                0xC2    //锁定模式命令
#define MCU_TO_PC_SEND_CMD      0xC3    //单片机到PC数据发送指令
#define PC_TO_MCU_RECEIVE       0xC4    //PC到单片机数据接收指令

/*----------------------------------------------------------*/

#endif //F411_FLY_CTRL_MY_PROTOCOL__H
