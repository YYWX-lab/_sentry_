/* 包含头文件 ----------------------------------------------------------------*/
#include "motor.h"
#include "dm_motor_ctrl.h"

/* 私有类型定义 --------------------------------------------------------------*/

/* 私有宏定义 ----------------------------------------------------------------*/
// rpm换算到rad/s
#define RPM_TO_RADPS (2.0f * pi / 60.0f)
// 圆周率PI
#define pi (3.14159265358979323846f)

/* 私有变量 ------------------------------------------------------------------*/

/* 扩展变量 ------------------------------------------------------------------*/
MotorInfo_t chassis_motor[4];
DM_1TO4_MotorInfo_t gimbal_motor_yaw;
MotorInfo_t gimbal_motor_pitch;
MotorInfo_t friction_wheel_motor[4];
MotorInfo_t trigger_motor[2];
MotorInfo_t Magazine_motor;
/* 私有函数原形 --------------------------------------------------------------*/

/* 函数体 --------------------------------------------------------------------*/
/*************************************************
 * Function: Motor_EncoderData
 * Description: 电机编码器数据解析
 * Input: ptr 电机信息指针
 *        data 数据指针
 * Return: 无
*************************************************/
static void Motor_EncoderData(MotorInfo_t* ptr, uint8_t data[])
{
    ptr->last_ecd = ptr->ecd;
    ptr->ecd = (uint16_t)(data[0] << 8 | data[1]);

    if (ptr->ecd - ptr->last_ecd > MOTOR_ENCODER_RANGE_HALF)
    {
        ptr->round_cnt--;
        ptr->ecd_raw_rate = ptr->ecd - ptr->last_ecd - MOTOR_ENCODER_RANGE;
    }
    else if (ptr->ecd - ptr->last_ecd < -MOTOR_ENCODER_RANGE_HALF)
    {
        ptr->round_cnt++;
        ptr->ecd_raw_rate = ptr->ecd - ptr->last_ecd + MOTOR_ENCODER_RANGE;
    }
    else
    {
        ptr->ecd_raw_rate = ptr->ecd - ptr->last_ecd;
    }

    ptr->total_ecd = ptr->round_cnt * MOTOR_ENCODER_RANGE + ptr->ecd - ptr->offset_ecd;
    /* total angle, unit is degree */
    ptr->total_angle = ptr->total_ecd / ENCODER_ANGLE_RATIO;

    ptr->speed_rpm = (int16_t)(data[2] << 8 | data[3]);
    ptr->given_current = (int16_t)(data[4] << 8 | data[5]);
    ptr->temperature = data[6];
}


/*************************************************
 * Function: DM_1TO4_Motor_EncoderData
 * Description: 达妙一拖四电机编码器数据解析
 * Input: ptr 电机信息指针
 *        data 数据指针
 * Return: 无
*************************************************/
static void DM_1TO4_Motor_EncoderData(DM_1TO4_MotorInfo_t* ptr, uint8_t data[])
{
    int16_t i_16_speed_rpm;
    ptr->last_ecd = ptr->ecd;
    ptr->ecd = (uint16_t)(data[0] << 8 | data[1]);

    if (ptr->ecd - ptr->last_ecd > MOTOR_ENCODER_RANGE_HALF)
    {
        ptr->round_cnt--;
        ptr->ecd_raw_rate = ptr->ecd - ptr->last_ecd - MOTOR_ENCODER_RANGE;
    }
    else if (ptr->ecd - ptr->last_ecd < -MOTOR_ENCODER_RANGE_HALF)
    {
        ptr->round_cnt++;
        ptr->ecd_raw_rate = ptr->ecd - ptr->last_ecd + MOTOR_ENCODER_RANGE;
    }
    else
    {
        ptr->ecd_raw_rate = ptr->ecd - ptr->last_ecd;
    }

    ptr->total_ecd = ptr->round_cnt * MOTOR_ENCODER_RANGE + ptr->ecd - ptr->offset_ecd;
    /* total angle, unit is degree */
    ptr->total_angle = ptr->total_ecd / ENCODER_ANGLE_RATIO;

    i_16_speed_rpm = (int16_t)(data[2] << 8 | data[3]);

    ptr->speed_rpm = i_16_speed_rpm;
    ptr->rad_s = i_16_speed_rpm / 100 * RPM_TO_RADPS ;

    ptr->given_current = (int16_t)(data[4] << 8 | data[5]);

    ptr->temperature = data[6];

    ptr->err_code = data[7];
}

/*************************************************
 * Function: Motor_EncoderOffset
 * Description: 电机编码器补偿
 * Input: ptr 电机信息指针
 *        data 数据指针
 * Return: 无
*************************************************/
static void Motor_EncoderOffset(MotorInfo_t* ptr, uint8_t data[])
{
    ptr->ecd        = (uint16_t)(data[0] << 8 | data[1]);
    ptr->offset_ecd = ptr->ecd;
}


/*************************************************
 * Function: DM_1TO4_Motor_EncoderOffset
 * Description: 达妙一拖四电机编码器补偿
 * Input: ptr 电机信息指针
 *        data 数据指针
 * Return: 无
*************************************************/
static void DM_1TO4_Motor_EncoderOffset(DM_1TO4_MotorInfo_t* ptr, uint8_t data[])
{
    ptr->ecd        = (uint16_t)(data[0] << 8 | data[1]);
    ptr->offset_ecd = ptr->ecd;
}



/*************************************************
 * Function: Motor_DataParse
 * Description: 电机编码器数据处理
 * Input: ptr 电机信息指针
 *        data 数据指针
 * Return: 无
*************************************************/
void Motor_DataParse(MotorInfo_t *ptr, uint8_t data[])
{
    if (ptr == NULL)
        return;
    ptr->msg_cnt++;

    if (ptr->msg_cnt < 50)
    {
        Motor_EncoderOffset(ptr, data);
        return;
    }

    Motor_EncoderData(ptr, data);
}


/*************************************************
 * Function: DM_1_TO_4_Motor_DataParse
 * Description: 达妙一拖四电机编码器数据处理
 * Input: ptr 电机信息指针
 *        data 数据指针
 * Return: 无
*************************************************/
void DM_1TO4_Motor_DataParse(DM_1TO4_MotorInfo_t *ptr, uint8_t data[])
{
    if (ptr == NULL)
        return;
    ptr->msg_cnt++;

    if (ptr->msg_cnt < 50)
    {
        DM_1TO4_Motor_EncoderOffset(ptr, data);
        return;
    }

    DM_1TO4_Motor_EncoderData(ptr, data);
}

/*************************************************
 * Function: Motor_RelativePosition
 * Description: 电机编码器绝对位置计算
 * Input: ecd 编码器值
 *        offset 补偿
 * Return: 无
*************************************************/
int16_t Motor_RelativePosition(int16_t ecd, int16_t center_offset)
{
    // int16_t tmp = 0;
    // if (offset >= MOTOR_ENCODER_RANGE_HALF)
    // {
    //     if (ecd > offset - MOTOR_ENCODER_RANGE_HALF)
    //         tmp = ecd - offset;
    //     else
    //         tmp = ecd + MOTOR_ENCODER_RANGE - offset;
    // }
    // else
    // {
    //     if (ecd > offset + MOTOR_ENCODER_RANGE_HALF)
    //         tmp = ecd - MOTOR_ENCODER_RANGE - offset;
    //     else
    //         tmp = ecd - offset;
    // }
    // return tmp;
        int16_t tmp = 0;
    if (center_offset >= MOTOR_ENCODER_RANGE_HALF)
    {
        if (ecd > center_offset - MOTOR_ENCODER_RANGE_HALF)
            tmp = ecd - center_offset;
        else
            tmp = ecd + MOTOR_ENCODER_RANGE - center_offset;
    }
    else
    {
        if (ecd > center_offset + MOTOR_ENCODER_RANGE_HALF)
            tmp = ecd - MOTOR_ENCODER_RANGE - center_offset;
        else
            tmp = ecd - center_offset;
    }
    return tmp;
}

/*************************************************
 * Function: Motor_SendMessage
 * Description: 电机控制数据发送
 * Input: obj CAN对象指针
 *        std_id CAN发送标识符
 *        cur1 电机1电流值
 *        cur2 电机2电流值
 *        cur3 电机3电流值
 *        cur4 电机4电流值
 * Return: 无
*************************************************/
void Motor_SendMessage(CAN_Object_t* obj, uint32_t std_id, int16_t cur1, int16_t cur2, int16_t cur3, int16_t cur4)
{
    uint8_t TxData[8] = {0};
    TxData[0] = (uint8_t)(cur1 >> 8);
    TxData[1] = (uint8_t)cur1;
    TxData[2] = (uint8_t)(cur2 >> 8);
    TxData[3] = (uint8_t)cur2;
    TxData[4] = (uint8_t)(cur3 >> 8);
    TxData[5] = (uint8_t)cur3;
    TxData[6] = (uint8_t)(cur4 >> 8);
    TxData[7] = (uint8_t)cur4;
    BSP_CAN_TransmitData(obj, std_id, TxData, 8);
}

/*************************************************
 * Function: DM_Motor_SendMessage
 * Description: 达妙电机控制数据发送
 * Input: obj CAN对象指针
 *        std_id CAN发送标识符
 *        cur1 电机1电流值
 *        cur2 电机2电流值
 *        cur3 电机3电流值
 *        cur4 电机4电流值
 * Return: 无
*************************************************/
void DM_Motor_SendMessage(CAN_Object_t* obj, uint32_t std_id, int16_t cur1, int16_t cur2, int16_t cur3, int16_t cur4)
{
    uint8_t TxData[8] = {0};
    TxData[0] = (uint8_t)(cur1 << 8);
    TxData[1] = (uint8_t)cur1;
    TxData[2] = (uint8_t)(cur2 << 8);
    TxData[3] = (uint8_t)cur2;
    TxData[4] = (uint8_t)(cur3 << 8);
    TxData[5] = (uint8_t)cur3;
    TxData[6] = (uint8_t)(cur4 << 8);
    TxData[7] = (uint8_t)cur4;
    BSP_CAN_TransmitData(obj, std_id, TxData, 8);
}

/*************************************************
 * Function: Motor_QuicklySetID
 * Description: 快速设置电机ID
 * Input: obj CAN对象指针
 * Return: 无
*************************************************/
void Motor_QuicklySetID(CAN_Object_t* obj)
{
    uint8_t TxData[8] = {0};
    BSP_CAN_TransmitData(obj, 0x700, TxData, 8);
}

MotorInfo_t* ChassisMotor_Pointer(uint8_t i)
{
    return &chassis_motor[i];
}

DM_1TO4_MotorInfo_t* GimbalMotorYaw_Pointer(void)
{
    return &gimbal_motor_yaw;
}

MotorInfo_t* GimbalMotorPitch_Pointer(void)
{
    return &gimbal_motor_pitch;
}

MotorInfo_t* FrictionWheelMotor_1_Pointer(void)
{
    return &friction_wheel_motor[0];
}

MotorInfo_t* FrictionWheelMotor_2_Pointer(void)
{
    return &friction_wheel_motor[1];
}


MotorInfo_t* FrictionWheelMotor_3_Pointer(void)
{
    return &friction_wheel_motor[2];
}


MotorInfo_t* FrictionWheelMotor_4_Pointer(void)
{
    return &friction_wheel_motor[3];
}


MotorInfo_t* TriggerMotor_1_Pointer(void)
{
    return &trigger_motor[0];
}


MotorInfo_t* TriggerMotor_2_Pointer(void)
{
    return &trigger_motor[1];
}


// MotorInfo_t* MagazineMotor_Pointer(void)
// {
//     return &Magazine_motor;
// }
