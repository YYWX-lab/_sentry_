/* 包含头文件 ----------------------------------------------------------------*/
#include "calibrate.h"
#include "cmsis_os.h"
#include "bsp_flash.h"
#include "buzzer.h"
#include "rgb_led.h"
#include "motor.h"
#include "infantry_console.h"
#include "imu_driver.h"

/* 私有类型定义 --------------------------------------------------------------*/
enum
{
    LOCK_STATE          = 0,
    CALI_BEGIN_STATE,
    CALI_GIMBAL_STATE,
    CALI_GYRO_STATE,
    MOTOR_SET_ID_STATE,
    CALI_END_STATE
};

/* 私有宏定义 ----------------------------------------------------------------*/
#define FLASH_USER_ADDR         ADDR_FLASH_SECTOR_9

#define FLASH_WRITE_BUF_LENGHT  100

#define CALIED_FLAG             0x55                //means it has been calibrated
#define CALI_FUNC_CMD_ON        1                   //need calibrate
#define CALI_FUNC_CMD_INIT      0                   //has been calibrated, set value to init

#define RC_CALI_VALUE_HOLE          600     //remote control threshold, the max value of remote control channel is 660.
#define RC_CMD_LONG_TIME            200

/* 私有变量 ------------------------------------------------------------------*/
osThreadId CalibrateTaskHandle;

static GimbalCalibrate_t    gimbal_cali;     //gimbal cali data
static IMU_Calibrate_t      gyro_cali;       //gyro cali data
static IMU_Calibrate_t      accel_cali;      //accel cali data
static IMU_Calibrate_t      mag_cali;        //mag cali data

CalibrateObject_t cali_obj[CALI_LIST_LENGHT];

/* 扩展变量 ------------------------------------------------------------------*/
extern osThreadId ConsoleTaskHandle;
extern osThreadId DetectTaskHandle;
extern Console_t console;

/* 私有函数原形 --------------------------------------------------------------*/
void CalibrateObject_Init(Calibrate_ID_e id, uint32_t *buffer, uint8_t len, CalibrateHookFunc_t func);
static void Calibrate_ReadData(void);
static void Calibrate_WriteData(void);
static uint8_t RC_Action(RC_Info_t* rc);
static uint8_t GyroCalibrate_Hook(uint32_t *cali, uint8_t cmd);
static uint8_t GimbalCalibrate_Hook(uint32_t *cali, uint8_t cmd);

/* 函数体 --------------------------------------------------------------------*/
void Calibrate_Use_RC(RC_Info_t* rc)
{
    static uint8_t  i;
    static uint32_t rc_cmd_time = 0;
    static uint8_t cali_state = LOCK_STATE;

    for (i = 0; i < CALI_LIST_LENGHT; i++)
    {
        if (cali_obj[i].cali_cmd)
        {
            rc_cmd_time = 0;
            return;
        }
    }

    uint8_t rc_action = RC_Action(rc);

    switch (cali_state)
    {
        case LOCK_STATE:
        {
            if (rc_action == 1)
                rc_cmd_time++;
            else
                rc_cmd_time=0;

            if (rc_cmd_time > RC_CMD_LONG_TIME)
            {
                cali_state = CALI_BEGIN_STATE;
                rc_cmd_time = 0;
            }
            BeepTimeSet_ON_OFF(50, 50);
            BeepTimesSet(BEEP_PERIOD/100);
            aRGB_led_show(BLUE);
        }break;

        case CALI_BEGIN_STATE:
        {
            BeepTimeSet_ON_OFF(500, 500);
            BeepTimesSet(1);
            aRGB_led_show(BLUE);
            if (rc_action == 2 || rc_action == 3 || rc_action == 4)
                rc_cmd_time++;
            else
                rc_cmd_time=0;

            if (rc_cmd_time > RC_CMD_LONG_TIME)
            {
                if (rc_action == 2)
                {
                    cali_state = CALI_GIMBAL_STATE;
                }
                else if (rc_action == 3)
                {
                    cali_state = CALI_GYRO_STATE;
                }
                else if (rc_action == 4)
                {
                    cali_state = MOTOR_SET_ID_STATE;
                }
                rc_cmd_time=0;
            }
        }break;

        case CALI_GIMBAL_STATE:
        {
            BeepTimesSet(2);
            aRGB_led_show(RED);
            rc_cmd_time=0;
            cali_state = CALI_BEGIN_STATE;
        }break;

        case CALI_GYRO_STATE:
        {
            BeepTimesSet(3);
            aRGB_led_show(GREEN);
            rc_cmd_time=0;
            cali_state = CALI_BEGIN_STATE;
            cali_obj[CALIBRATE_GYRO].cali_cmd = 1;
        }break;

        case MOTOR_SET_ID_STATE:
        {
            rc_cmd_time=0;
            cali_state = CALI_BEGIN_STATE;
            Motor_QuicklySetID(&can2_obj);
            Motor_QuicklySetID(&can2_obj);
            Motor_QuicklySetID(&can2_obj);
        }break;
    }
}


void CalibrateTask(void)
{
    for (;;)
    {
        Calibrate_Use_RC(console.rc);

        for (uint8_t i = 0; i < CALI_LIST_LENGHT; i++)
        {
            if (cali_obj[i].cali_cmd)
            {
                if (cali_obj[i].hook_func != NULL)
                {
                    if (cali_obj[i].hook_func(cali_obj[i].flash_buffer, CALI_FUNC_CMD_ON))
                    {
                        //set 0x55
                        cali_obj[i].cali_done = CALIED_FLAG;
                        cali_obj[i].cali_cmd = 0;
                        //write
                        Calibrate_WriteData();
                    }
                }
            }
        }

        BeepHandler();
        osDelay(10);
    }
}


void Calibrate_Init(void)
{
    uint8_t i = 0;

//    CalibrateObject_Init(CALIBRATE_GIMBAL, (uint32_t*)&gimbal_cali, sizeof(GimbalCalibrate_t), GimbalCalibrate_Hook);
//    CalibrateObject_Init(CALIBRATE_GYRO, (uint32_t*)&gyro_cali, sizeof(IMU_Calibrate_t), GyroCalibrate_Hook);
//    CalibrateObject_Init(CALIBRATE_ACC, (uint32_t*)&accel_cali, sizeof(IMU_Calibrate_t), NULL);
//    CalibrateObject_Init(CALIBRATE_MAG, (uint32_t*)&mag_cali, sizeof(IMU_Calibrate_t), NULL);
//
//    Calibrate_ReadData();

//    for (i = 0; i < CALI_LIST_LENGHT; i++)
//    {
//        if (cali_obj[i].cali_done == CALIED_FLAG)
//        {
//            if (cali_obj[i].hook_func != NULL)
//            {
//                //if has been calibrated, set to init
//                cali_obj[i].hook_func(cali_obj[i].flash_buffer, CALI_FUNC_CMD_INIT);
//            }
//        }
//    }

    osDelay(1000);
    if (console.rc->sw1 == REMOTE_SWITCH_VALUE_DOWN && console.rc->sw2 == REMOTE_SWITCH_VALUE_DOWN)
    {
        osThreadTerminate(ConsoleTaskHandle);
        osThreadTerminate(DetectTaskHandle);
        console.gimbal_cmd  = GIMBAL_RELEASE_CMD;
        console.chassis_cmd  = CHASSIS_RELEASE_CMD;
    }
    else
    {
        return;
    }

    osThreadDef(calibrate_task, CalibrateTask, osPriorityNormal, 0, 256);
    CalibrateTaskHandle = osThreadCreate(osThread(calibrate_task), NULL);
}

void CalibrateObject_Init(Calibrate_ID_e id, uint32_t *buffer, uint8_t len, CalibrateHookFunc_t func)
{
    cali_obj[id].flash_len = len;
    cali_obj[id].flash_buffer = buffer;
    cali_obj[id].hook_func = func;
}

static void Calibrate_ReadData(void)
{
    uint8_t i = 0;
    uint16_t offset = 0;
    uint8_t flash_read_buf[4];
    for (i = 0; i < CALI_LIST_LENGHT; i++)
    {
        //read the data in flash,
        BSP_Flash_Read(FLASH_USER_ADDR + offset, cali_obj[i].flash_buffer, cali_obj[i].flash_len/4);
        offset += cali_obj[i].flash_len;

        //read the name and cali flag,
        BSP_Flash_Read(FLASH_USER_ADDR + offset, (uint32_t *)flash_read_buf, 1);
        cali_obj[i].cali_done = flash_read_buf[0];
        offset += 1;

        if (cali_obj[i].cali_done != CALIED_FLAG && cali_obj[i].hook_func != NULL)
        {
            cali_obj[i].cali_cmd = CALI_FUNC_CMD_ON;
        }
    }
}

static void Calibrate_WriteData(void)
{
    uint8_t i = 0;
    uint16_t offset = 0;
    static uint8_t flash_write_buffer[FLASH_WRITE_BUF_LENGHT] = {0};

    for (i = 0; i < CALI_LIST_LENGHT; i++)
    {
        //copy the data of device calibration data
        memcpy((void *)(flash_write_buffer + offset), (void *)cali_obj[i].flash_buffer, cali_obj[i].flash_len);
        offset += cali_obj[i].flash_len;

        //copy the name and "CALI_FLAG" of device
        memcpy((void *)(flash_write_buffer + offset), &cali_obj[i].cali_done, sizeof(uint8_t));
        offset += 1;
    }
    //erase the page
    BSP_Flash_EraseAddress(FLASH_USER_ADDR,1);
    //write data
    BSP_Flash_WriteSingleAddress(FLASH_USER_ADDR, (uint32_t*)flash_write_buffer, FLASH_WRITE_BUF_LENGHT/4);
}

static uint8_t RC_Action(RC_Info_t* rc)
{
    if (rc->ch1 < -RC_CALI_VALUE_HOLE && rc->ch2 < -RC_CALI_VALUE_HOLE && rc->ch3 > RC_CALI_VALUE_HOLE && rc->ch4 < -RC_CALI_VALUE_HOLE)
    {
        return 1;
    }
    else if (rc->ch1 > RC_CALI_VALUE_HOLE && rc->ch2 > RC_CALI_VALUE_HOLE && rc->ch3 < -RC_CALI_VALUE_HOLE && rc->ch4 > RC_CALI_VALUE_HOLE)
    {
        return 2;
    }
    else if (rc->ch1 > RC_CALI_VALUE_HOLE && rc->ch2 < -RC_CALI_VALUE_HOLE && rc->ch3 < -RC_CALI_VALUE_HOLE && rc->ch4 < -RC_CALI_VALUE_HOLE)
    {
        return 3;
    }
    else if (rc->ch1 < -RC_CALI_VALUE_HOLE && rc->ch2 > RC_CALI_VALUE_HOLE && rc->ch3 > RC_CALI_VALUE_HOLE && rc->ch4 > RC_CALI_VALUE_HOLE)
    {
        return 4;
    }
    else
    {
        return 0;
    }
}

static uint8_t GyroCalibrate_Hook(uint32_t *cali, uint8_t cmd)
{
    static uint16_t count_time = 0;
    IMU_Calibrate_t* local_cali = (IMU_Calibrate_t*)cali;
    if (cmd == CALI_FUNC_CMD_INIT)
    {
        IMU_SetGyroOffset(local_cali->offset);
        return 0;
    }
    else if (cmd == CALI_FUNC_CMD_ON)
    {
        IMU_CalibrateGyro(local_cali->offset);
        count_time++;

        if (count_time > 2000)
        {
            count_time = 0;
            return 1;
        }
        else
        {
            return 0;
        }
    }

    return 0;
}

static uint8_t GimbalCalibrate_Hook(uint32_t *cali, uint8_t cmd)
{
    return 0;
}
