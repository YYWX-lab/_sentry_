// #ifndef DM_MOTOR_H
// #define DM_MOTOR_H

// /* Includes ------------------------------------------------------------------*/
// #include <stdint.h>
// #include "can.h"

// /* 前向声明：补充缺失的类型定义（需确保实际工程中包含对应头文件） */
// struct Struct_CAN_Manage_Object;
// typedef struct
// {
//     /* 补充PID结构体的实际成员（根据工程需求定义） */
//     float Kp;
//     float Ki;
//     float Kd;
//     float target;
//     float feedback;
//     float output;
//     float max_out;
//     float integral_limit;
// } Struct_PID;

// /* Exported macros -----------------------------------------------------------*/
// /* 定义默认值宏 */
// #define MOTOR_DM_DEFAULT_CTRL_METHOD    Motor_DM_Control_Method_NORMAL_MIT //默认是mit模式
// #define MOTOR_DM_DEFAULT_ANGLE_MAX      0.0f
// #define MOTOR_DM_DEFAULT_OMEGA_MAX      12.5f
// #define MOTOR_DM_DEFAULT_TORQUE_MAX     10.0f
// #define MOTOR_DM_DEFAULT_CURRENT_MAX    10.261194f
// #define MOTOR_DM_DEFAULT_ENCODER_OFFSET 0
// #define DM_Motor_ID                     0x301

// /* Exported types ------------------------------------------------------------*/
// /**
//  * @brief 达妙电机状态
//  */
// typedef enum 
// {
//     Motor_DM_Status_DISABLE = 0,
//     Motor_DM_Status_ENABLE
// } Enum_Motor_DM_Status;

// /**
//  * @brief 达妙电机的ID枚举类型, 一拖四模式用
//  */
// typedef enum 
// {
//     Motor_DM_ID_0x301 = 1,
//     Motor_DM_ID_0x302,
//     Motor_DM_ID_0x303,
//     Motor_DM_ID_0x304,
//     Motor_DM_ID_0x305,
//     Motor_DM_ID_0x306,
//     Motor_DM_ID_0x307,
//     Motor_DM_ID_0x308
// } Enum_Motor_DM_Motor_ID_1_To_4;

// /**
//  * @brief 达妙电机控制状态, 传统模式有效
//  */
// typedef enum 
// {
//     Motor_DM_Control_Status_DISABLE = 0x0,
//     Motor_DM_Control_Status_ENABLE,
//     Motor_DM_Control_Status_OVERVOLTAGE = 0x8,
//     Motor_DM_Control_Status_UNDERVOLTAGE,
//     Motor_DM_Control_Status_OVERCURRENT,
//     Motor_DM_Control_Status_MOS_OVERTEMPERATURE,
//     Motor_DM_Control_Status_ROTOR_OVERTEMPERATURE,
//     Motor_DM_Control_Status_LOSE_CONNECTION,
//     Motor_DM_Control_Status_MOS_OVERLOAD
// } Enum_Motor_DM_Control_Status_Normal;

// /**
//  * @brief 达妙电机控制方式
//  */
// typedef enum 
// {
//     Motor_DM_Control_Method_NORMAL_MIT = 0,
//     Motor_DM_Control_Method_NORMAL_ANGLE_OMEGA,
//     Motor_DM_Control_Method_NORMAL_OMEGA,
//     Motor_DM_Control_Method_NORMAL_EMIT,
//     Motor_DM_Control_Method_1_TO_4_CURRENT,
//     Motor_DM_Control_Method_1_TO_4_OMEGA,
//     Motor_DM_Control_Method_1_TO_4_ANGLE
// } Enum_Motor_DM_Control_Method;

// // /**
// //  * @brief 达妙电机传统模式源数据
// //  */
// // struct Struct_Motor_DM_CAN_Rx_Data_Normal
// // {
// //     uint8_t CAN_ID : 4;
// //     uint8_t Control_Status_Enum : 4;
// //     uint16_t Angle_Reverse;
// //     uint8_t Omega_11_4;
// //     uint8_t Omega_3_0_Torque_11_8;
// //     uint8_t Torque_7_0;
// //     uint8_t MOS_Temperature;
// //     uint8_t Rotor_Temperature;
// // } __attribute__((packed));

// /**
//  * @brief 达妙电机源数据结构体
//  */
// typedef struct 
// {
//     uint8_t CAN_ID : 4;
//     uint8_t Control_Status_Enum : 4;
//     uint16_t Angle_Reverse;
//     uint8_t Omega_11_4;
//     uint8_t Omega_3_0_Torque_11_8;
//     uint8_t Torque_7_0;
//     uint8_t MOS_Temperature;
//     uint8_t Rotor_Temperature;
// } DM_Motor_Ori_Data;

// /**
//  * @brief 达妙电机数据结构体
//  */
// typedef struct
// {
//     uint8_t CAN_ID : 4;
//     uint8_t Control_Status_Enum : 4;
//     int16_t angle;
//     int16_t vel;
//     int16_t torque;
//     uint8_t MOS_Temperature;
//     uint8_t Rotor_Temperature;
// } dm_motor_data;


// /**
//  * @brief 达妙电机一拖四模式源数据
//  */
// struct Struct_Motor_DM_CAN_Rx_Data_1_To_4
// {
//     uint16_t Encoder_Reverse;
//     // 角速度100倍
//     int16_t Omega_Reverse;
//     // 电流值, mA
//     int16_t Current_Reverse;
//     uint8_t Rotor_Temperature;
//     uint8_t MOS_Temperature;
// } __attribute__((packed));

// /**
//  * @brief 达妙电机常规源数据, MIT控制报文
//  */
// struct Struct_Motor_DM_CAN_Tx_Data_Normal_MIT
// {
//     uint16_t Control_Angle_Reverse;
//     uint8_t Control_Omega_11_4;
//     uint8_t Control_Omega_3_0_K_P_11_8;
//     uint8_t K_P_7_0;
//     uint8_t K_D_11_4;
//     uint8_t K_D_3_0_Control_Torque_11_8;
//     uint8_t Control_Torque_7_0;
// } __attribute__((packed));

// /**
//  * @brief 达妙电机常规源数据, 位置速度控制报文
//  */
// struct Struct_Motor_DM_CAN_Tx_Data_Normal_Angle_Omega
// {
//     float Control_Angle;
//     float Control_Omega;
// } __attribute__((packed));

// /**
//  * @brief 达妙电机常规源数据, 速度控制报文
//  */
// struct Struct_Motor_DM_CAN_Tx_Data_Normal_Omega
// {
//     float Control_Omega;
// } __attribute__((packed));

// /**
//  * @brief 达妙电机常规源数据, EMIT控制报文
//  */
// struct Struct_Motor_DM_CAN_Tx_Data_Normal_EMIT
// {
//     float Control_Angle;
//     // 限定速度用, rad/s的100倍
//     uint16_t Control_Omega;
//     // 限定电流用, 电流最大值的10000倍
//     uint16_t Control_Current;
// } __attribute__((packed));

// /**
//  * @brief 达妙电机经过处理的数据, 传统模式有效
//  */
// struct Struct_Motor_DM_Rx_Data_Normal
// {
//     Enum_Motor_DM_Control_Status_Normal Control_Status;
//     float Now_Angle;
//     float Now_Omega;
//     float Now_Torque;
//     float Now_MOS_Temperature;
//     float Now_Rotor_Temperature;
//     uint32_t Pre_Encoder;
//     int32_t Total_Encoder;
//     int32_t Total_Round;
// };

// /**
//  * @brief 达妙电机经过处理的数据, 一拖四模式有效
//  */
// struct Struct_Motor_DM_Rx_Data_1_To_4
// {
//     float Now_Angle;
//     float Now_Omega;
//     float Now_Current;
//     float Now_MOS_Temperature;
//     float Now_Rotor_Temperature;
//     uint32_t Pre_Encoder;
//     int32_t Total_Encoder;
//     int32_t Total_Round;
// };

// /**
//  * @brief 模拟C++的Class_Motor_DM_Normal类：达妙电机传统模式结构体
//  */
// typedef struct
// {
//     // 原protected成员（C无访问限定符，通过命名规范区分）
//     struct Struct_CAN_Manage_Object *CAN_Manage_Object;
//     uint16_t CAN_Rx_ID;
//     uint16_t CAN_Tx_ID;
//     float Angle_Max;
//     float Omega_Max;
//     float Torque_Max;
//     float Current_Max;

//     // 内部变量
//     uint32_t Flag;
//     uint32_t Pre_Flag;
//     uint8_t Tx_Data[8];

//     // 读变量
//     Enum_Motor_DM_Status Motor_DM_Status;
//     struct Struct_Motor_DM_Rx_Data_Normal Rx_Data;

//     // 读写变量
//     Enum_Motor_DM_Control_Method Motor_DM_Control_Method;
//     float Control_Angle;
//     float Control_Omega;
//     float Control_Torque;
//     float Control_Current;
//     float K_P;
//     float K_D;
// } Struct_Motor_DM_Normal;

// /**
//  * @brief 模拟C++的Class_Motor_DM_1_To_4类：达妙电机一拖四模式结构体
//  */
// typedef struct
// {
//     // PID控制器（替代原Class_PID）
//     Struct_PID PID_Angle;
//     Struct_PID PID_Omega;

//     // 原protected成员
//     struct Struct_CAN_Manage_Object *CAN_Manage_Object;
//     Enum_Motor_DM_Motor_ID_1_To_4 CAN_Rx_ID;
//     int32_t Encoder_Offset;
//     uint8_t *Tx_Data;
//     float Current_Max;

//     // 常量（C中通过初始化函数赋值）
//     uint16_t Encoder_Num_Per_Round;
//     float Current_To_Out;
//     float Theoretical_Output_Current_Max;

//     // 内部变量
//     uint32_t Flag;
//     uint32_t Pre_Flag;
//     float Out;

//     // 读变量
//     Enum_Motor_DM_Status Motor_DM_Status;
//     struct Struct_Motor_DM_Rx_Data_1_To_4 Rx_Data;

//     // 读写变量
//     Enum_Motor_DM_Control_Method Motor_DM_Control_Method;
//     float Target_Angle;
//     float Target_Omega;
//     float Target_Current;
//     float Feedforward_Omega;
//     float Feedforward_Current;
// } Struct_Motor_DM_1_To_4;

// /* Exported function declarations --------------------------------------------*/
// /**
//  * @brief 达妙电机传统模式初始化（替代C++的默认参数：提供默认值宏）
//  * @param motor      电机结构体指针（必须初始化）
//  * @param hcan       CAN句柄
//  * @param __CAN_Rx_ID CAN接收ID
//  * @param __CAN_Tx_ID CAN发送ID
//  * @param __Motor_DM_Control_Method 控制方式
//  * @param __Angle_Max 最大角度
//  * @param __Omega_Max 最大角速度
//  * @param __Torque_Max 最大扭矩
//  * @param __Current_Max 最大电流
//  */
// void Motor_DM_Normal_Init(Struct_Motor_DM_Normal *motor,
//                           CAN_HandleTypeDef *hcan,
//                           uint8_t __CAN_Rx_ID,
//                           uint8_t __CAN_Tx_ID,
//                           Enum_Motor_DM_Control_Method __Motor_DM_Control_Method,
//                           float __Angle_Max,
//                           float __Omega_Max,
//                           float __Torque_Max,
//                           float __Current_Max);

// /**
//  * @brief 达妙电机传统模式初始化（默认参数版，简化调用）
//  */
// static inline void Motor_DM_Normal_Init_Default(Struct_Motor_DM_Normal *motor,
//                                                 CAN_HandleTypeDef *hcan,
//                                                 uint8_t __CAN_Rx_ID,
//                                                 uint8_t __CAN_Tx_ID)
// {
//     Motor_DM_Normal_Init(motor, hcan, __CAN_Rx_ID, __CAN_Tx_ID,
//                          MOTOR_DM_DEFAULT_CTRL_METHOD,
//                          MOTOR_DM_DEFAULT_ANGLE_MAX,
//                          MOTOR_DM_DEFAULT_OMEGA_MAX,
//                          MOTOR_DM_DEFAULT_TORQUE_MAX,
//                          MOTOR_DM_DEFAULT_CURRENT_MAX);
// }

// // ------------- 传统模式Get/Set接口（替代C++的inline成员函数）-------------
// inline float Motor_DM_Normal_Get_Angle_Max(Struct_Motor_DM_Normal *motor);
// inline float Motor_DM_Normal_Get_Omega_Max(Struct_Motor_DM_Normal *motor);
// inline float Motor_DM_Normal_Get_Torque_Max(Struct_Motor_DM_Normal *motor);
// inline float Motor_DM_Normal_Get_Current_Max(Struct_Motor_DM_Normal *motor);
// inline Enum_Motor_DM_Status Motor_DM_Normal_Get_Status(Struct_Motor_DM_Normal *motor);
// inline Enum_Motor_DM_Control_Status_Normal Motor_DM_Normal_Get_Control_Status(Struct_Motor_DM_Normal *motor);
// inline float Motor_DM_Normal_Get_Now_Angle(Struct_Motor_DM_Normal *motor);
// inline float Motor_DM_Normal_Get_Now_Omega(Struct_Motor_DM_Normal *motor);
// inline float Motor_DM_Normal_Get_Now_Torque(Struct_Motor_DM_Normal *motor);
// inline float Motor_DM_Normal_Get_Now_MOS_Temperature(Struct_Motor_DM_Normal *motor);
// inline float Motor_DM_Normal_Get_Now_Rotor_Temperature(Struct_Motor_DM_Normal *motor);
// inline Enum_Motor_DM_Control_Method Motor_DM_Normal_Get_Control_Method(Struct_Motor_DM_Normal *motor);
// inline float Motor_DM_Normal_Get_Control_Angle(Struct_Motor_DM_Normal *motor);
// inline float Motor_DM_Normal_Get_Control_Omega(Struct_Motor_DM_Normal *motor);
// inline float Motor_DM_Normal_Get_Control_Torque(Struct_Motor_DM_Normal *motor);
// inline float Motor_DM_Normal_Get_Control_Current(Struct_Motor_DM_Normal *motor);
// inline float Motor_DM_Normal_Get_K_P(Struct_Motor_DM_Normal *motor);
// inline float Motor_DM_Normal_Get_K_D(Struct_Motor_DM_Normal *motor);

// inline void Motor_DM_Normal_Set_Control_Angle(Struct_Motor_DM_Normal *motor, float __Control_Angle);
// inline void Motor_DM_Normal_Set_Control_Omega(Struct_Motor_DM_Normal *motor, float __Control_Omega);
// inline void Motor_DM_Normal_Set_Control_Torque(Struct_Motor_DM_Normal *motor, float __Control_Torque);
// inline void Motor_DM_Normal_Set_Control_Current(Struct_Motor_DM_Normal *motor, float __Control_Current);
// inline void Motor_DM_Normal_Set_K_P(Struct_Motor_DM_Normal *motor, float __K_P);
// inline void Motor_DM_Normal_Set_K_D(Struct_Motor_DM_Normal *motor, float __K_D);

// // ------------- 传统模式回调/操作函数 -------------
// void Motor_DM_Normal_CAN_RxCpltCallback(Struct_Motor_DM_Normal *motor, uint8_t *Rx_Data);
// void Motor_DM_Normal_CAN_Send_Clear_Error(Struct_Motor_DM_Normal *motor);
// void Motor_DM_Normal_CAN_Send_Enter(Struct_Motor_DM_Normal *motor);
// void Motor_DM_Normal_CAN_Send_Exit(Struct_Motor_DM_Normal *motor);
// void Motor_DM_Normal_CAN_Send_Save_Zero(Struct_Motor_DM_Normal *motor);
// void Motor_DM_Normal_TIM_Alive_PeriodElapsedCallback(Struct_Motor_DM_Normal *motor);
// void Motor_DM_Normal_TIM_Send_PeriodElapsedCallback(Struct_Motor_DM_Normal *motor);

// // ------------- 内部函数（需在.c文件中实现）-------------
// void Motor_DM_Normal_Data_Process(Struct_Motor_DM_Normal *motor);
// void Motor_DM_Normal_Output(Struct_Motor_DM_Normal *motor);

// /**
//  * @brief 达妙电机一拖四模式初始化
//  * @param motor      电机结构体指针
//  * @param hcan       CAN句柄
//  * @param __CAN_Rx_ID CAN接收ID（枚举类型）
//  * @param __Motor_DM_Control_Method 控制方式
//  * @param __Encoder_Offset 编码器偏移
//  * @param __Current_Max 最大电流
//  */
// void Motor_DM_1_To_4_Init(Struct_Motor_DM_1_To_4 *motor,
//                           CAN_HandleTypeDef *hcan,
//                           Enum_Motor_DM_Motor_ID_1_To_4 __CAN_Rx_ID,
//                           Enum_Motor_DM_Control_Method __Motor_DM_Control_Method,
//                           int32_t __Encoder_Offset,
//                           float __Current_Max);

// /**
//  * @brief 达妙电机一拖四模式初始化（默认参数版）
//  */
// static inline void Motor_DM_1_To_4_Init_Default(Struct_Motor_DM_1_To_4 *motor,
//                                                 CAN_HandleTypeDef *hcan,
//                                                 Enum_Motor_DM_Motor_ID_1_To_4 __CAN_Rx_ID)
// {
//     Motor_DM_1_To_4_Init(motor, hcan, __CAN_Rx_ID,
//                          Motor_DM_Control_Method_1_TO_4_ANGLE,
//                          MOTOR_DM_DEFAULT_ENCODER_OFFSET,
//                          MOTOR_DM_DEFAULT_CURRENT_MAX);
// }

// // ------------- 一拖四模式Get/Set接口 -------------
// inline float Motor_DM_1_To_4_Get_Current_Max(Struct_Motor_DM_1_To_4 *motor);
// inline float Motor_DM_1_To_4_Get_Theoretical_Output_Current_Max(Struct_Motor_DM_1_To_4 *motor);
// inline Enum_Motor_DM_Status Motor_DM_1_To_4_Get_Status(Struct_Motor_DM_1_To_4 *motor);
// inline float Motor_DM_1_To_4_Get_Now_Angle(Struct_Motor_DM_1_To_4 *motor);
// inline float Motor_DM_1_To_4_Get_Now_Omega(Struct_Motor_DM_1_To_4 *motor);
// inline float Motor_DM_1_To_4_Get_Now_Current(Struct_Motor_DM_1_To_4 *motor);
// inline float Motor_DM_1_To_4_Get_Now_MOS_Temperature(Struct_Motor_DM_1_To_4 *motor);
// inline float Motor_DM_1_To_4_Get_Now_Rotor_Temperature(Struct_Motor_DM_1_To_4 *motor);
// inline Enum_Motor_DM_Control_Method Motor_DM_1_To_4_Get_Control_Method(Struct_Motor_DM_1_To_4 *motor);
// inline float Motor_DM_1_To_4_Get_Target_Angle(Struct_Motor_DM_1_To_4 *motor);
// inline float Motor_DM_1_To_4_Get_Target_Omega(Struct_Motor_DM_1_To_4 *motor);
// inline float Motor_DM_1_To_4_Get_Target_Current(Struct_Motor_DM_1_To_4 *motor);
// inline float Motor_DM_1_To_4_Get_Feedforward_Omega(Struct_Motor_DM_1_To_4 *motor);
// inline float Motor_DM_1_To_4_Get_Feedforward_Current(Struct_Motor_DM_1_To_4 *motor);

// inline void Motor_DM_1_To_4_Set_Control_Method(Struct_Motor_DM_1_To_4 *motor, Enum_Motor_DM_Control_Method __DM_Motor_Control_Method);
// inline void Motor_DM_1_To_4_Set_Target_Angle(Struct_Motor_DM_1_To_4 *motor, float __Target_Angle);
// inline void Motor_DM_1_To_4_Set_Target_Omega(Struct_Motor_DM_1_To_4 *motor, float __Target_Omega);
// inline void Motor_DM_1_To_4_Set_Target_Current(Struct_Motor_DM_1_To_4 *motor, float __Target_Current);
// inline void Motor_DM_1_To_4_Set_Feedforward_Omega(Struct_Motor_DM_1_To_4 *motor, float __Feedforward_Omega);
// inline void Motor_DM_1_To_4_Set_Feedforward_Current(Struct_Motor_DM_1_To_4 *motor, float __Feedforward_Current);

// // ------------- 一拖四模式回调/操作函数 -------------
// void Motor_DM_1_To_4_CAN_RxCpltCallback(Struct_Motor_DM_1_To_4 *motor, uint8_t *Rx_Data);
// void Motor_DM_1_To_4_TIM_100ms_Alive_PeriodElapsedCallback(Struct_Motor_DM_1_To_4 *motor);
// void Motor_DM_1_To_4_TIM_1ms_Calculate_PeriodElapsedCallback(Struct_Motor_DM_1_To_4 *motor);

// // ------------- 一拖四模式内部函数 -------------
// void Motor_DM_1_To_4_Data_Process(Struct_Motor_DM_1_To_4 *motor);
// void Motor_DM_1_To_4_PID_Calculate(Struct_Motor_DM_1_To_4 *motor);
// void Motor_DM_1_To_4_Output(Struct_Motor_DM_1_To_4 *motor);

// /* Inline函数实现（替代C++的inline成员函数）---------------------------------- */
// inline float Motor_DM_Normal_Get_Angle_Max(Struct_Motor_DM_Normal *motor)
// {
//     return motor->Angle_Max;
// }

// inline float Motor_DM_Normal_Get_Omega_Max(Struct_Motor_DM_Normal *motor)
// {
//     return motor->Omega_Max;
// }

// inline float Motor_DM_Normal_Get_Torque_Max(Struct_Motor_DM_Normal *motor)
// {
//     return motor->Torque_Max;
// }

// inline float Motor_DM_Normal_Get_Current_Max(Struct_Motor_DM_Normal *motor)
// {
//     return motor->Current_Max;
// }

// inline Enum_Motor_DM_Status Motor_DM_Normal_Get_Status(Struct_Motor_DM_Normal *motor)
// {
//     return motor->Motor_DM_Status;
// }

// inline Enum_Motor_DM_Control_Status_Normal Motor_DM_Normal_Get_Control_Status(Struct_Motor_DM_Normal *motor)
// {
//     return motor->Rx_Data.Control_Status;
// }

// inline float Motor_DM_Normal_Get_Now_Angle(Struct_Motor_DM_Normal *motor)
// {
//     return motor->Rx_Data.Now_Angle;
// }

// inline float Motor_DM_Normal_Get_Now_Omega(Struct_Motor_DM_Normal *motor)
// {
//     return motor->Rx_Data.Now_Omega;
// }

// inline float Motor_DM_Normal_Get_Now_Torque(Struct_Motor_DM_Normal *motor)
// {
//     return motor->Rx_Data.Now_Torque;
// }

// inline float Motor_DM_Normal_Get_Now_MOS_Temperature(Struct_Motor_DM_Normal *motor)
// {
//     return motor->Rx_Data.Now_MOS_Temperature;
// }

// inline float Motor_DM_Normal_Get_Now_Rotor_Temperature(Struct_Motor_DM_Normal *motor)
// {
//     return motor->Rx_Data.Now_Rotor_Temperature;
// }

// inline Enum_Motor_DM_Control_Method Motor_DM_Normal_Get_Control_Method(Struct_Motor_DM_Normal *motor)
// {
//     return motor->Motor_DM_Control_Method;
// }

// inline float Motor_DM_Normal_Get_Control_Angle(Struct_Motor_DM_Normal *motor)
// {
//     return motor->Control_Angle;
// }

// inline float Motor_DM_Normal_Get_Control_Omega(Struct_Motor_DM_Normal *motor)
// {
//     return motor->Control_Omega;
// }

// inline float Motor_DM_Normal_Get_Control_Torque(Struct_Motor_DM_Normal *motor)
// {
//     return motor->Control_Torque;
// }

// inline float Motor_DM_Normal_Get_Control_Current(Struct_Motor_DM_Normal *motor)
// {
//     return motor->Control_Current;
// }

// inline float Motor_DM_Normal_Get_K_P(Struct_Motor_DM_Normal *motor)
// {
//     return motor->K_P;
// }

// inline float Motor_DM_Normal_Get_K_D(Struct_Motor_DM_Normal *motor)
// {
//     return motor->K_D;
// }

// inline void Motor_DM_Normal_Set_Control_Angle(Struct_Motor_DM_Normal *motor, float __Control_Angle)
// {
//     motor->Control_Angle = __Control_Angle;
// }

// inline void Motor_DM_Normal_Set_Control_Omega(Struct_Motor_DM_Normal *motor, float __Control_Omega)
// {
//     motor->Control_Omega = __Control_Omega;
// }

// inline void Motor_DM_Normal_Set_Control_Torque(Struct_Motor_DM_Normal *motor, float __Control_Torque)
// {
//     motor->Control_Torque = __Control_Torque;
// }

// inline void Motor_DM_Normal_Set_Control_Current(Struct_Motor_DM_Normal *motor, float __Control_Current)
// {
//     motor->Control_Current = __Control_Current;
// }

// inline void Motor_DM_Normal_Set_K_P(Struct_Motor_DM_Normal *motor, float __K_P)
// {
//     motor->K_P = __K_P;
// }

// inline void Motor_DM_Normal_Set_K_D(Struct_Motor_DM_Normal *motor, float __K_D)
// {
//     motor->K_D = __K_D;
// }

// // 一拖四模式inline函数实现
// inline float Motor_DM_1_To_4_Get_Current_Max(Struct_Motor_DM_1_To_4 *motor)
// {
//     return motor->Current_Max;
// }

// inline float Motor_DM_1_To_4_Get_Theoretical_Output_Current_Max(Struct_Motor_DM_1_To_4 *motor)
// {
//     return motor->Theoretical_Output_Current_Max;
// }

// inline Enum_Motor_DM_Status Motor_DM_1_To_4_Get_Status(Struct_Motor_DM_1_To_4 *motor)
// {
//     return motor->Motor_DM_Status;
// }

// inline float Motor_DM_1_To_4_Get_Now_Angle(Struct_Motor_DM_1_To_4 *motor)
// {
//     return motor->Rx_Data.Now_Angle;
// }

// inline float Motor_DM_1_To_4_Get_Now_Omega(Struct_Motor_DM_1_To_4 *motor)
// {
//     return motor->Rx_Data.Now_Omega;
// }

// inline float Motor_DM_1_To_4_Get_Now_Current(Struct_Motor_DM_1_To_4 *motor)
// {
//     return motor->Rx_Data.Now_Current;
// }

// inline float Motor_DM_1_To_4_Get_Now_MOS_Temperature(Struct_Motor_DM_1_To_4 *motor)
// {
//     return motor->Rx_Data.Now_MOS_Temperature;
// }

// inline float Motor_DM_1_To_4_Get_Now_Rotor_Temperature(Struct_Motor_DM_1_To_4 *motor)
// {
//     return motor->Rx_Data.Now_Rotor_Temperature;
// }

// inline Enum_Motor_DM_Control_Method Motor_DM_1_To_4_Get_Control_Method(Struct_Motor_DM_1_To_4 *motor)
// {
//     return motor->Motor_DM_Control_Method;
// }

// inline float Motor_DM_1_To_4_Get_Target_Angle(Struct_Motor_DM_1_To_4 *motor)
// {
//     return motor->Target_Angle;
// }

// inline float Motor_DM_1_To_4_Get_Target_Omega(Struct_Motor_DM_1_To_4 *motor)
// {
//     return motor->Target_Omega;
// }

// inline float Motor_DM_1_To_4_Get_Target_Current(Struct_Motor_DM_1_To_4 *motor)
// {
//     return motor->Target_Current;
// }

// inline float Motor_DM_1_To_4_Get_Feedforward_Omega(Struct_Motor_DM_1_To_4 *motor)
// {
//     return motor->Feedforward_Omega;
// }

// inline float Motor_DM_1_To_4_Get_Feedforward_Current(Struct_Motor_DM_1_To_4 *motor)
// {
//     return motor->Feedforward_Current;
// }

// inline void Motor_DM_1_To_4_Set_Control_Method(Struct_Motor_DM_1_To_4 *motor, Enum_Motor_DM_Control_Method __DM_Motor_Control_Method)
// {
//     motor->Motor_DM_Control_Method = __DM_Motor_Control_Method;
// }

// inline void Motor_DM_1_To_4_Set_Target_Angle(Struct_Motor_DM_1_To_4 *motor, float __Target_Angle)
// {
//     motor->Target_Angle = __Target_Angle;
// }

// inline void Motor_DM_1_To_4_Set_Target_Omega(Struct_Motor_DM_1_To_4 *motor, float __Target_Omega)
// {
//     motor->Target_Omega = __Target_Omega;
// }

// inline void Motor_DM_1_To_4_Set_Target_Current(Struct_Motor_DM_1_To_4 *motor, float __Target_Current)
// {
//     motor->Target_Current = __Target_Current;
// }

// inline void Motor_DM_1_To_4_Set_Feedforward_Omega(Struct_Motor_DM_1_To_4 *motor, float __Feedforward_Omega)
// {
//     motor->Feedforward_Omega = __Feedforward_Omega;
// }

// inline void Motor_DM_1_To_4_Set_Feedforward_Current(Struct_Motor_DM_1_To_4 *motor, float __Feedforward_Current)
// {
//     motor->Feedforward_Current = __Feedforward_Current;
// }

// #endif /* DM_MOTOR_H */