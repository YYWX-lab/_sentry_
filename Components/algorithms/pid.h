#ifndef __PID_H__
#define __PID_H__

/* 包含头文件 ----------------------------------------------------------------*/

/* 类型定义 ------------------------------------------------------------------*/
enum
{
    LLAST = 0,
    LAST,
    NOW,
    POSITION_PID,
    DELTA_PID,
};

typedef struct pid_t
{
    float p;
    float i;
    float d;
    float k_f;//前馈值

    float set;
    float last_set;
    float get;
    float err[3];

    float pout;
    float iout;
    float dout;
    float fout;
    float out;

    float input_max_err;    //input max err;
    float output_deadband;  //output deadband;

    int pid_mode;
    float max_out;
    float integral_limit;

        // 新增：轨迹规划 + 前馈专用
    float plan_set;       // 规划后的平滑目标位置
    float plan_speed;     // 规划后的平滑速度（前馈的依据）
    float max_speed;      // 允许的最大速度
    float max_accel;      // 允许的最大加速度
    float dt;             // 控制周期（例如 0.001s 对应 1kHz 控制频率）
    

} pid_t;

typedef struct
{
    pid_t           outer_pid;
    pid_t           inter_pid;
    pid_t           pid;
    float           outer_ref;
    float           outer_fdb;
    float           inter_ref;
    float           inter_fdb;

    
} Double_PID_t;

/* 宏定义 --------------------------------------------------------------------*/

/* 扩展变量 ------------------------------------------------------------------*/

/* 函数声明 ------------------------------------------------------------------*/
void pid_init(pid_t* pid, int mode, float maxout, float intergral_limit, float kp, float ki, float kd, float kf);
float pid_calc(pid_t* pid, float get, float set);
float j4310_pid_calc(pid_t* pid, float get, float set);
// void j4310_traj_plan(pid_t* dpid);
float vision_pid_calc(pid_t *pid, float err);
void pid_clear(pid_t* pid);
float DoublePID_Calc(Double_PID_t* dpid, float outer_ref, float outer_fdb, float inter_fdb);
void angle_ramp_plan(pid_t *pid, float target_set);

#endif  // __PID_H__

