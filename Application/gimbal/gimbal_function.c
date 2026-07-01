/* 包含头文件 ----------------------------------------------------------------*/
#include "gimbal_function.h"

/* 私有类型定义 --------------------------------------------------------------*/

/* 私有宏定义 ----------------------------------------------------------------*/

/* 私有变量 ------------------------------------------------------------------*/

/* 扩展变量 ------------------------------------------------------------------*/
extern GimbalHandle_t gimbal_handle;
/* 私有函数原形 --------------------------------------------------------------*/

static void traj_plan(Gimbal_PID_t* dpid);

/* 函数体 --------------------------------------------------------------------*/
fp32 Gimbal_PID_Calc(Gimbal_PID_t* pid, fp32 angle_ref, fp32 angle_fdb, fp32 speed_fdb)
{
    pid->angle_ref = angle_ref;
    pid->angle_fdb = angle_fdb;
    pid_calc(&pid->outer_pid, pid->angle_fdb, pid->angle_ref);
    pid->speed_ref = pid->outer_pid.out;
    pid->speed_fdb = speed_fdb;
    pid_calc(&pid->inter_pid, pid->speed_fdb, pid->speed_ref);
    return pid->inter_pid.out;
}


fp32 Gimbal_PID_feedforward_Calc(Gimbal_PID_t* pid, fp32 angle_ref, fp32 angle_fdb, fp32 speed_fdb, fp32 vision_speed)
{
    pid->angle_ref = angle_ref;
    pid->angle_fdb = angle_fdb;

    traj_plan(pid);  
    pid->outer_pid.fout = pid->outer_pid.k_f * pid->traj_vel;

    pid_calc(&pid->outer_pid, pid->angle_fdb, pid->angle_ref);
    // float outer_total_out = dpid->outer_pid.out + d;
    pid->speed_ref = pid->outer_pid.out + pid->outer_pid.fout + vision_speed;

    // pid->speed_ref = pid->outer_pid.out + vision_speed;
    pid->speed_fdb = speed_fdb;
    pid_calc(&pid->inter_pid, pid->speed_fdb, pid->speed_ref);
    return pid->inter_pid.out;
}

void Gimbal_PID_Clear(Gimbal_PID_t* pid)
{
    pid_clear(&pid->outer_pid);
    pid_clear(&pid->inter_pid);
}

void GimbalMotorChangeProtect(GimbalMotor_t* motor)
{
    if (motor->last_mode != motor->mode)
    {
        if(motor->mode == RAW_VALUE_MODE)
        {
            motor->given_value = motor->current_set;
        }
        else if (motor->mode == GYRO_MODE || motor->mode == J4310_MIT_VEL_MODE)
        {
            motor->given_value = motor->sensor.gyro_angle;
        }
        else if (motor->mode == ENCONDE_MODE || motor->mode == J4310_MIT_ANGLE_MODE)
        {
            motor->given_value = motor->sensor.relative_angle;
        }
    }
    motor->last_mode = motor->mode;
}

// 简易梯形轨迹规划（100Hz控制周期，阶跃角度转平滑运动）
static void traj_plan(Gimbal_PID_t* dpid)
{
    // 计算轨迹误差
    float err = dpid->angle_ref - dpid->traj_pos;
    // 360°环向修正
    if (err > 180)  err -= 360;
    if (err < -180) err += 360;
    
    // 简易速度规划（快速平滑，无冲击）
    if (err > 0) {
        dpid->traj_vel = dpid->max_vel;  // 正速度
        if (dpid->traj_vel > err) dpid->traj_vel = err;
    } else if (err < 0) {
        dpid->traj_vel = -dpid->max_vel; // 负速度
        if (dpid->traj_vel < err) dpid->traj_vel = err;
    } else {
        dpid->traj_vel = 0; // 到达目标
    }
    
    // 更新轨迹位置
    dpid->traj_pos += dpid->traj_vel;
}

void GimbalMotorControl(GimbalMotor_t* motor)
{
    GimbalMotorChangeProtect(motor);
    if (motor->mode == RAW_VALUE_MODE)
    {
        motor->current_set = motor->given_value;
        Gimbal_PID_Clear(&motor->pid);
    }
    else if(motor->mode == GYRO_MODE)
    {
        // motor->current_set = Gimbal_PID_Calc(&motor->pid,
        //                                      motor->given_value,
        //                                      motor->sensor.gyro_angle,
        //                                      motor->sensor.palstance);
        motor->current_set = Gimbal_PID_feedforward_Calc(&motor->pid,
                                             motor->given_value,
                                             motor->sensor.gyro_angle,
                                             motor->sensor.palstance,
                                             motor->vision_speed) + cosf(motor->sensor.gyro_angle/57.3f) * 4200.f;    
        // motor->current_set = cosf(motor->sensor.gyro_angle/57.3f) * 4200.f;      
        // motor->current_set = Gimbal_PID_feedforward_Calc(&motor->pid,
        //                                      motor->given_value,
        //                                      motor->sensor.gyro_angle,
        //                                      motor->sensor.palstance,
        //                                      motor->vision_speed) ;                                   
    }
    else if(motor->mode == ENCONDE_MODE)
    {
        // motor->current_set = Gimbal_PID_Calc(&motor->pid,
        //                                      motor->given_value,
        //                                      motor->sensor.relative_angle,
        //                                      motor->sensor.palstance);
        motor->current_set = Gimbal_PID_feedforward_Calc(&motor->pid,
                                             motor->given_value,
                                             motor->sensor.relative_angle,
                                             motor->sensor.palstance,
                                             motor->vision_speed) + cosf(motor->sensor.gyro_angle/57.3f) * 4200.f;
    }
}

static float i_grav;
// 每10ms 100Hz调用，输入原始编码器角度(弧度)，输出电机电流指令
float GravityStatic_Calc(float enc_ang_rad)
{
    // 1. 角度弱平滑，消除编码器噪声带来cos跳变
    angle_raw = enc_ang_rad;
    angle_filt = ALPHA_ANGLE * angle_raw + (1 - ALPHA_ANGLE) * angle_filt;

    // 2. 基础重力前馈核心公式（简洁优美，无需查表）
    i_grav = I_GRAV_MAX * cosf(angle_filt) + I_FIX_OFFSET;

    // 3. 手拨跟随逻辑：外力拨动时同步目标角度
    float err = target_angle - angle_filt;
    if(fabsf(err) > DEAD_ZONE_RAD * 3.0f)
    {
        target_angle = angle_filt;
    }

    // 4. 死区设计：微小误差完全不做修正，彻底消除定点来回抖动
    float i_correct = 0.0f;
    if(fabsf(err) > DEAD_ZONE_RAD)
    {
        // 仅微弱一次性补偿，无连续闭环震荡
        i_correct = clamp(50.f * err, -I_GRAV_MAX, I_GRAV_MAX);
    }

    // 5. 原始总电流
    float curr_raw = i_grav + i_correct;

    // 6. 电流一阶平滑输出，解决100Hz离散刷新的段落卡顿
    curr_out_filt = ALPHA_CURR * curr_raw + (1 - ALPHA_CURR) * curr_out_filt;

    return curr_out_filt;
}

float clamp(float val, float min, float max)
{
    if(val < min) return min;
    if(val > max) return max;
    return val;
}

fp32 AngleTransform(fp32 target_angle, fp32 gyro_angle)
{
    float offset = 0, now = 0, target = 0;

    ANGLE_LIMIT_360(target, target_angle);
    ANGLE_LIMIT_360(now, gyro_angle);

    offset = target - now;
    if (offset > 180)
    {
        offset = offset - 360;
    }
    else if (offset < -180)
    {
        offset = offset + 360;
    }
    return gyro_angle + offset;
}

//输入弧度，变为角度，归到180度之间
fp32 DM_AngleTransform(fp32 dm_angle)
{
    float angle = 0;
    dm_angle *= (180.0f)/PI;

    // angle = dm_angle % 360;
    // ANGLE_LIMIT_180(angle , dm_angle);
    angle = dm_angle;
    // if (angle < 180.f)
    // {
    //     angle += 360;
    // }
    // else if (angle > 180.f)
    // {
    //     angle -= 360;
    // }



    return angle;//弧度转为角度
}