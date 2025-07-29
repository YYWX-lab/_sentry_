/* 包含头文件 ----------------------------------------------------------------*/
#include "buzzer.h"

/* 私有类型定义 --------------------------------------------------------------*/

/* 私有宏定义 ----------------------------------------------------------------*/

/* 私有变量 ------------------------------------------------------------------*/
static uint8_t beep_times;
static uint16_t beep_on_time = BEEP_ON_TIME;
static uint16_t beep_off_time = BEEP_OFF_TIME;

/* 扩展变量 ------------------------------------------------------------------*/

/* 私有函数原形 --------------------------------------------------------------*/

/* 函数体 --------------------------------------------------------------------*/
/*************************************************
 * Function: Buzzer_SetBeep
 * Description: 设置蜂鸣器音调
 * Input: tune 曲调
 *        value pwm占空比
 * Return: 无
*************************************************/
void Buzzer_SetBeep(uint16_t tune, uint16_t value)
{
    ((TIM_HandleTypeDef *)buzzer_gpio.handle)->Instance->ARR = tune;
    BSP_GPIO_SetPwmValue(&buzzer_gpio, value);
}

/*************************************************
 * Function: BeepHandler
 * Description: 蜂鸣器控制器
 * Input: 无
 * Return: 无
*************************************************/
void BeepHandler(void)
{
    static uint32_t beep_tick;
    static uint32_t times_tick;
    static uint8_t times;

    uint32_t time_now = BSP_GetTime_ms();
    /* The beep works after the system starts 3s */
    if (time_now / 1000 < 3)
    {
        return;
    }

    if (time_now - beep_tick > BEEP_PERIOD)
    {
        times = beep_times;
        beep_tick = time_now;
        times_tick = time_now;
    }
    else if (times != 0)
    {
        if (time_now - times_tick < beep_on_time)
        {
            Buzzer_SetBeep(BEEP_TUNE_VALUE, BEEP_CTRL_VALUE);
        }
        else if (time_now - times_tick < beep_on_time + beep_off_time)
        {
            Buzzer_SetBeep(0, 0);
        }
        else
        {
            times--;
            times_tick = time_now;
        }
    }
}

/*************************************************
 * Function: BeepTimesSet
 * Description: 设置蜂鸣器鸣叫次数
 * Input: times 次数
 * Return: 无
*************************************************/
void BeepTimesSet(uint8_t times)
{
    beep_times = times;
}

/*************************************************
 * Function: BeepTimeSet_ON_OFF
 * Description: 蜂鸣器开关时间设置
 * Input: on_time 打开时间
 *        off_time 关闭时间
 * Return: 无
*************************************************/
void BeepTimeSet_ON_OFF(uint16_t on_time, uint16_t off_time)
{
    beep_on_time = on_time;
    beep_off_time = off_time;

}
