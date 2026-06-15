/*******************************************************************************
 * Robofuture RM Team
 * File name: pid.c
 * Author: Zhb        Version: 1.0        Date: 2021/4/6
 * Description: 实现pid算法
 * Function List:
 *   1. pid_init pid参数初始化
 *   2. pid_calc pid算法计算
 *   3. pid_clear 清除pid计算数据
 *   4. DoublePID_Calc 双环pid计算
 * History:
 *      <author> <time>  <version > <desc>
 *        Zhb   21/04/06  1.0       首次提交
*******************************************************************************/

/* 包含头文件 ----------------------------------------------------------------*/
#include "pid.h"
#include <math.h>
#include "user_lib.h"
#include "stddef.h"


/* 私有类型定义 --------------------------------------------------------------*/

/* 私有宏定义 ----------------------------------------------------------------*/

/* 私有变量 ------------------------------------------------------------------*/

/* 扩展变量 ------------------------------------------------------------------*/

/* 私有函数原形 --------------------------------------------------------------*/

/* 函数体 --------------------------------------------------------------------*/
//static void abs_limit(float *a, float ABS_MAX)
//{
//    if (*a > ABS_MAX)
//        *a = ABS_MAX;
//    if (*a < -ABS_MAX)
//        *a = -ABS_MAX;
//}

/*************************************************
 * Function: pid_param_init
 * Description: pid参数初始化
 * Input: pid pid指针
 *        mode pid算法类型，可选： POSITION_PID位置式，
 *                               DELTA_PID增量式
 *        maxout 最大输出
 *        intergral_limit 积分最大限幅
 *        kp 比例系数
 *        ki 积分系数
 *        kd 微分系数
 * Return: 无
*************************************************/
static void pid_param_init(
    pid_t* pid,
    int   mode,
    float maxout,
    float intergral_limit,
    float    kp,
    float    ki,
    float    kd,
    float    kf)
{

    pid->integral_limit = intergral_limit;
    pid->max_out        = maxout;
    pid->pid_mode       = mode;

    pid->p = kp;
    pid->i = ki;
    pid->d = kd;
    pid->k_f = kf;

}

/*************************************************
 * Function: pid_reset
 * Description: pid参数修改
 * Input: pid pid指针
 *        kp 比例系数
 *        ki 积分系数
 *        kd 微分系数
 * Return: 无
*************************************************/
static void pid_reset(pid_t* pid, float kp, float ki, float kd, float kf)
{
    pid->p = kp;
    pid->i = ki;
    pid->d = kd;
    pid->k_f = kf;

    pid->pout = 0;
    pid->iout = 0;
    pid->dout = 0;
    pid->fout = 0;
    pid->out  = 0;

}

/*************************************************
 * Function: pid_calc
 * Description: pid计算
 * Input: pid pid指针
 *        get 测量反馈值
 *        set 目标值
 * Return: pid计算输出值
*************************************************/
float pid_calc(pid_t* pid, float get, float set)
{
    pid->get = get;
    pid->set = set;
    pid->err[NOW] = set - get;

    if ((pid->input_max_err != 0) && (fabs(pid->err[NOW]) > pid->input_max_err))
        return 0;

    if (pid->pid_mode == POSITION_PID) //position PID
    {
        pid->pout = pid->p * pid->err[NOW];
        pid->iout += pid->i * pid->err[NOW];
        pid->dout = pid->d * (pid->err[NOW] - pid->err[LAST]);

        abs_limit(&(pid->iout), pid->integral_limit);
        pid->fout = pid->k_f * (pid->set - pid->last_set);
        pid->out = pid->pout + pid->iout + pid->dout;
        abs_limit(&(pid->out), pid->max_out);
    }
    else if (pid->pid_mode == DELTA_PID) //delta PID
    {
        pid->pout = pid->p * (pid->err[NOW] - pid->err[LAST]);
        pid->iout = pid->i * pid->err[NOW];
        pid->dout = pid->d * (pid->err[NOW] - 2 * pid->err[LAST] + pid->err[LLAST]);

        pid->out += pid->pout + pid->iout + pid->dout;
        abs_limit(&(pid->out), pid->max_out);
    }

    pid->err[LLAST] = pid->err[LAST];
    pid->err[LAST]  = pid->err[NOW];
    pid->last_set = pid->set;


    if ((pid->output_deadband != 0) && (fabs(pid->out) < pid->output_deadband))
        return 0;
    else
        return pid->out;
}

/*************************************************
 * Function: j4310_pid_calc
 * Description: pid计算
 * Input: pid pid指针
 *        get 测量反馈值
 *        set 目标值
 * Return: pid计算输出值
*************************************************/
uint16_t tick = 100;
float feedforward_out = 0;
float alpha = 0;
float j4310_pid_calc(pid_t* pid, float get, float set)
{
    
    pid->get = get;
    pid->set = set;
   
    pid->err[NOW] = set - get;

    if (pid->err[NOW] > 180.0f)
    {
        pid->err[NOW] -= 360.0f;
    }
    else if (pid->err[NOW] < -180.0f)
    {
        pid->err[NOW] += 360.0f;
    }

     // 1. 计算目标角度变化量（速度）
    float target_speed = pid->set - pid->last_set;
    // 2. 速度前馈（仅目标变化时输出）
    float instant_ff = pid->k_f * target_speed;
    // if(instant_ff != 0)
    // {
    //     feedforward_out = instant_ff;
    // }
    feedforward_out = instant_ff;
      // 3. 低通滤波：让前馈平滑输出，解决「一瞬间失效」问题
    // if (target_speed == 0 && abs(feedforward_out) < 0.5)
    // {
    //     alpha = 1;
    // }
    // else
    // {
    //     alpha = 0.00001;
    // }
    // // alpha = 0.05f; // 平滑系数(0~1)，越小越平滑
    // feedforward_out = alpha * instant_ff + (1 - alpha) * feedforward_out;
    
    if ((pid->input_max_err != 0) && (fabs(pid->err[NOW]) > pid->input_max_err))
        return 0;

    if (pid->pid_mode == POSITION_PID) //position PID
    {
        pid->pout = pid->p * pid->err[NOW];
        pid->iout += pid->i * pid->err[NOW];
        pid->dout = pid->d * (pid->err[NOW] - pid->err[LAST]);

        abs_limit(&(pid->iout), pid->integral_limit);
        // pid->fout = pid->k_f * (pid->set - pid->last_set);
        pid->fout = feedforward_out;
        pid->out = pid->pout + pid->iout + pid->dout + pid->fout;
        abs_limit(&(pid->out), pid->max_out);
    }
    else if (pid->pid_mode == DELTA_PID) //delta PID
    {
        pid->pout = pid->p * (pid->err[NOW] - pid->err[LAST]);
        pid->iout = pid->i * pid->err[NOW];
        pid->dout = pid->d * (pid->err[NOW] - 2 * pid->err[LAST] + pid->err[LLAST]);

        pid->out += pid->pout + pid->iout + pid->dout ;
        abs_limit(&(pid->out), pid->max_out);
    }
    


    pid->err[LLAST] = pid->err[LAST];
    pid->err[LAST]  = pid->err[NOW];
    pid->last_set = pid->set;
    


    if ((pid->output_deadband != 0) && (fabs(pid->out) < pid->output_deadband))
        return 0;
    else
        return pid->out ;
}

// // 简易梯形轨迹规划（100Hz控制周期，阶跃角度转平滑运动）
// float j4310_traj_plan(float get, float set)
// {
//     // 计算轨迹误差
//     float err = dpid->out - dpid->traj_pos;
//     // 360°环向修正
//     if (err > 180)  err -= 360;
//     if (err < -180) err += 360;
    
//     // 简易速度规划（快速平滑，无冲击）
//     if (err > 0) {
//         dpid->traj_vel = dpid->max_vel;  // 正速度
//         if (dpid->traj_vel > err) dpid->traj_vel = err;
//     } else if (err < 0) {
//         dpid->traj_vel = -dpid->max_vel; // 负速度
//         if (dpid->traj_vel < err) dpid->traj_vel = err;
//     } else {
//         dpid->traj_vel = 0; // 到达目标
//     }
    
//     // 更新轨迹位置
//     dpid->traj_pos += dpid->traj_vel;
// }

/*************************************************
 * Function: vision_pid
 * Description: 视觉pid算法
 * Input: pid pid指针
 *        err 上位机传下来的角度差
 * Return: pid计算输出值
*************************************************/
float vision_pid_calc(pid_t *pid, float err)
{
    pid->err[NOW] = err;
    if ((pid->input_max_err != 0) && (fabs(pid->err[NOW]) > pid->input_max_err))
        return 0;

    if (pid->pid_mode == POSITION_PID) //position PID
    {
        pid->pout = pid->p * pid->err[NOW];
        pid->iout += pid->i * pid->err[NOW];
        pid->dout = pid->d * (pid->err[NOW] - pid->err[LAST]);

        abs_limit(&(pid->iout), pid->integral_limit);
        pid->out = pid->pout + pid->iout + pid->dout;
        abs_limit(&(pid->out), pid->max_out);
    }
    else if (pid->pid_mode == DELTA_PID) //delta PID
    {
        pid->pout = pid->p * (pid->err[NOW] - pid->err[LAST]);
        pid->iout = pid->i * pid->err[NOW];
        pid->dout = pid->d * (pid->err[NOW] - 2 * pid->err[LAST] + pid->err[LLAST]);

        pid->out += pid->pout + pid->iout + pid->dout;
        abs_limit(&(pid->out), pid->max_out);
    }

    pid->err[LLAST] = pid->err[LAST];
    pid->err[LAST]  = pid->err[NOW];


    if ((pid->output_deadband != 0) && (fabs(pid->out) < pid->output_deadband))
        return 0;
    else
        return pid->out;
}

/*************************************************
 * Function: pid_clear
 * Description: 清除pid计算值
 * Input: pid pid指针
 * Return: 无
*************************************************/
void pid_clear(pid_t* pid)
{
    if (pid == NULL)
    {
        return;
    }

    pid->err[NOW] = pid->err[LAST] = pid->err[LLAST] = 0.0f;
    pid->out = pid->pout = pid->iout = pid->dout = 0.0f;
    pid->get = pid->set = 0.0f;


}

/*************************************************
 * Function: pid_init
 * Description: pid初始化
 * Input: pid pid指针
 *        mode pid算法类型，可选： POSITION_PID位置式，
 *                               DELTA_PID增量式
 *        maxout 最大输出
 *        intergral_limit 积分最大限幅
 *        kp 比例系数
 *        ki 积分系数
 *        kd 微分系数
 * Return: 无
*************************************************/
void pid_init(
    pid_t* pid,
    int    mode,
    float  maxout,
    float  intergral_limit,

    float kp,
    float ki,
    float kd,
    float kf)
{
    pid_param_init(pid, mode, maxout, intergral_limit, kp, ki, kd, kf);
    pid_reset(pid, kp, ki, kd, kf);
}

/*************************************************
 * Function: DoublePID_Calc
 * Description: 双环串级pid计算
 * Input: dpid 双环pid指针
 *        outer_ref 外环目标值
 *        outer_fdb 外环反馈值
 *        inter_fdb 内环反馈值
 * Return: 无
*************************************************/
float DoublePID_Calc(Double_PID_t* dpid, float outer_ref, float outer_fdb, float inter_fdb)
{
    dpid->outer_ref = outer_ref;
    dpid->outer_fdb = outer_fdb;
    pid_calc(&dpid->outer_pid, dpid->outer_fdb, dpid->outer_ref);
    dpid->inter_ref = dpid->outer_pid.out;
    dpid->inter_fdb = inter_fdb;
    pid_calc(&dpid->inter_pid, dpid->inter_fdb, dpid->inter_ref);
    return dpid->inter_pid.out;
}
