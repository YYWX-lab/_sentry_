#ifndef GIMBAL_FUNCTION_H
#define GIMBAL_FUNCTION_H

/* 包含头文件 ----------------------------------------------------------------*/
#include "gimbal_app.h"
#include "ramp.h"
#include "arm_math.h"
#include "user_lib.h"

/* 类型定义 ------------------------------------------------------------------*/

/* 宏定义 --------------------------------------------------------------------*/

/* 扩展变量 ------------------------------------------------------------------*/
#include <math.h>
#include <stdint.h>

// ========== 云台整定参数（现场自己微调）==========
// #define DT              0.01f    // 100Hz控制周期
// #define ALPHA_THETA     1.f     // 角度低通滤波系数 0.1~0.3
// #define I_MAX_GRAV      4000.f     // 水平时重力补偿最大电流(标定得到)
// #define K_VEL_FF        0.f    // 速度前馈系数，解决快拨下坠
// #define K_P             50.f     // 微弱比例闭环，不要开大
// #define ERR_THRESHOLD   2.f   // 2度，误差超过则跟随目标
// #define MAX_P_OUT       4000.f     // P输出限幅，避免手感僵硬抖动
// #define MAX_DELTA_I     4000.f     // 单帧电流最大变化，消除卡顿阶梯
// #define I_LIMIT         4000.0f     // 电机最大输出电流限幅

// 控制参数（静止调试专用，全部调静态悬停）
#define DT             0.01f    // 100Hz周期
#define ALPHA_ANGLE    0.18f    // 角度低通平滑系数
#define ALPHA_CURR     0.12f    // 输出电流平滑，根治段落卡顿
#define I_GRAV_MAX     4000.f     // 水平标定重力补偿电流
#define DEAD_ZONE_RAD  1.f   // 误差死区，小于该值不修正，消抖动
#define I_FIX_OFFSET   0.0f     // 摩擦固定偏置，按需微调
// ================================================

// 状态变量
// 状态缓存变量
static float angle_raw;
static float angle_filt;
static float curr_out_filt;
static float target_angle;
/* 函数声明 ------------------------------------------------------------------*/
fp32 Gimbal_PID_Calc(Gimbal_PID_t* pid, fp32 angle_ref, fp32 angle_fdb, fp32 speed_fdb);
fp32 Gimbal_PID_feedforward_Calc(Gimbal_PID_t* pid, fp32 angle_ref, fp32 angle_fdb, fp32 speed_fdb, fp32 vision_speed);
fp32 Gimbal_vision_PID_Calc(Gimbal_PID_t* pid, fp32 angle_err, fp32 speed_fdb);
void Gimbal_PID_Clear(Gimbal_PID_t* pid);
void GimbalMotorChangeProtect(GimbalMotor_t* motor);
void GimbalMotorControl(GimbalMotor_t* motor);
fp32 AngleTransform(fp32 target_angle, fp32 gyro_angle);
fp32 DM_AngleTransform(fp32 dm_angle);
float clamp(float val, float min, float max);
// float GravityCompensate(float enc_theta_rad);
float GravityStatic_Calc(float enc_ang_rad);

#endif  // GIMBAL_FUNCTION_H

