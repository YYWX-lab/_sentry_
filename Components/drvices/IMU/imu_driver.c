/* 包含头文件 ----------------------------------------------------------------*/
#include "imu_driver.h"
#include "cmsis_os.h"
#include "math.h"
#include <stdio.h>
#include <stdlib.h>


/* 私有类型定义 --------------------------------------------------------------*/

/* 私有宏定义 ----------------------------------------------------------------*/
#define BMI088_BOARD_INSTALL_SPIN_MATRIX     \
    { 0.0f, -1.0f, 0.0f},                     \
    {1.0f, 0.0f, 0.0f},                     \
    { 0.0f, 0.0f, 1.0f}                      \

#define IST8310_BOARD_INSTALL_SPIN_MATRIX   \
    {1.0f, 0.0f, 0.0f},                     \
    {0.0f, 1.0f, 0.0f},                     \
    {0.0f, 0.0f, 1.0f}                      \

/* 私有变量 ------------------------------------------------------------------*/
/* IMU数据结构 */
IMU_Data_t m_imu;
bmi088_real_data_t bmi088_data;
ist8310_real_data_t ist8310_data;

/* 校准使用数据 */
fp32 gyro_scale_factor[3][3] = {BMI088_BOARD_INSTALL_SPIN_MATRIX};
fp32 gyro_offset[3];
fp32 gyro_cali_offset[3];

fp32 accel_scale_factor[3][3] = {BMI088_BOARD_INSTALL_SPIN_MATRIX};
fp32 accel_offset[3];

fp32 mag_scale_factor[3][3] = {IST8310_BOARD_INSTALL_SPIN_MATRIX};
fp32 mag_offset[3];

fp32 yaw_mid;
fp32 mag_x_compensated;
fp32 mag_y_compensated;
int i = 0;



/* 加速度计使用的低通滤波 */
static fp32 accel_fliter_1[3] = {0.0f, 0.0f, 0.0f};
static fp32 accel_fliter_2[3] = {0.0f, 0.0f, 0.0f};
static fp32 accel_fliter_3[3] = {0.0f, 0.0f, 0.0f};
static const fp32 fliter_num[3] = {1.929454039488895f, -0.93178349823448126f, 0.002329458745586203f};

/* 扩展变量 ------------------------------------------------------------------*/

/* 私有函数原形 --------------------------------------------------------------*/
static void IMU_CaliSlove(fp32 gyro[3], fp32 accel[3], fp32 mag[3], bmi088_real_data_t *bmi088, ist8310_real_data_t *ist8310);
static void IMU_Accel_LowPassFilter(fp32 accel[3]);

/* 函数体 --------------------------------------------------------------------*/
/**
  * @brief  根据安装方向旋转陀螺仪，加速度计和磁力计，并计算零漂
  */
static void IMU_CaliSlove(fp32 gyro[3], fp32 accel[3], fp32 mag[3], bmi088_real_data_t *bmi088, ist8310_real_data_t *ist8310)
{
    for (uint8_t i = 0; i < 3; i++)
    {
        gyro[i] = bmi088->gyro[0] * gyro_scale_factor[i][0] + bmi088->gyro[1] * gyro_scale_factor[i][1] + bmi088->gyro[2] * gyro_scale_factor[i][2] + gyro_offset[i];
        accel[i] = bmi088->accel[0] * accel_scale_factor[i][0] + bmi088->accel[1] * accel_scale_factor[i][1] + bmi088->accel[2] * accel_scale_factor[i][2] + accel_offset[i];
        mag[i] = ist8310->mag[0] * mag_scale_factor[i][0] + ist8310->mag[1] * mag_scale_factor[i][1] + ist8310->mag[2] * mag_scale_factor[i][2] + mag_offset[i];
    }
}

/**
  * @brief  accel 低通滤波
  */
static void IMU_Accel_LowPassFilter(fp32 accel[3])
{
    accel_fliter_1[0] = accel_fliter_2[0];
    accel_fliter_2[0] = accel_fliter_3[0];

    accel_fliter_3[0] = accel_fliter_2[0] * fliter_num[0] + accel_fliter_1[0] * fliter_num[1] + accel[0] * fliter_num[2];

    accel_fliter_1[1] = accel_fliter_2[1];
    accel_fliter_2[1] = accel_fliter_3[1];

    accel_fliter_3[1] = accel_fliter_2[1] * fliter_num[0] + accel_fliter_1[1] * fliter_num[1] + accel[1] * fliter_num[2];

    accel_fliter_1[2] = accel_fliter_2[2];
    accel_fliter_2[2] = accel_fliter_3[2];

    accel_fliter_3[2] = accel_fliter_2[2] * fliter_num[0] + accel_fliter_1[2] * fliter_num[1] + accel[2] * fliter_num[2];
}

// double calculateHeadingInRadians(double mag_x, double mag_y) {
//     // 先计算航向角（度）
//     double headingInDegrees = atan2(mag_y, mag_x) * (180 / PI);
//     // 将角度转换为弧度
//     double headingInRadians = headingInDegrees * (PI / 180);
//     return headingInRadians;
// }

void IMU_Init(void)
{
    while(BMI088_init())
    {
        osDelay(100);
    }
    while(ist8310_init())
    {
        osDelay(100);
    }

    BMI088_read(bmi088_data.gyro, bmi088_data.accel, &bmi088_data.temp);
    ist8310_read_mag(ist8310_data.mag);
    
    IMU_CaliSlove(m_imu.gyro, m_imu.accel, m_imu.mag, &bmi088_data, &ist8310_data);

    AHRS_init(m_imu.quat, m_imu.accel, m_imu.mag);

    accel_fliter_1[0] = accel_fliter_2[0] = accel_fliter_3[0] = m_imu.accel[0];
    accel_fliter_1[1] = accel_fliter_2[1] = accel_fliter_3[1] = m_imu.accel[1];
    accel_fliter_1[2] = accel_fliter_2[2] = accel_fliter_3[2] = m_imu.accel[2];

    get_angle(m_imu.quat, &(m_imu.euler.yaw), &(m_imu.euler.pitch), &(m_imu.euler.roll));
    
}

void IMU_Update(const fp32 period_time)
{
    BMI088_read(bmi088_data.gyro, bmi088_data.accel, &bmi088_data.temp);
    ist8310_read_mag(ist8310_data.mag);
    IMU_CaliSlove(m_imu.gyro, m_imu.accel, m_imu.mag, &bmi088_data, &ist8310_data);
    m_imu.temp = bmi088_data.temp;

    IMU_Accel_LowPassFilter(m_imu.accel);

    AHRS_update(m_imu.quat, period_time, m_imu.gyro, accel_fliter_3, m_imu.mag);
    get_angle(m_imu.quat, &m_imu.euler.yaw, &m_imu.euler.pitch, &m_imu.euler.roll);

    
    //

    mag_x_compensated = m_imu.mag[0] * cos(m_imu.euler.pitch) + m_imu.mag[2] * sin(m_imu.euler.pitch);
    mag_y_compensated = m_imu.mag[0] * sin(m_imu.euler.pitch) * sin(m_imu.euler.roll) + m_imu.mag[1] * cos(m_imu.euler.roll) - m_imu.mag[2] * sin(m_imu.euler.roll) * cos(m_imu.euler.pitch);
     // 计算 yaw
    m_imu.euler.true_yaw = atan2(-mag_y_compensated, mag_x_compensated);

    // 将 yaw 转换为 0° 到 360°
    // if (m_imu.euler.true_yaw < 0) {
    //     m_imu.euler.true_yaw += 2 * PI;
    // }

    
    // m_imu.euler.true_yaw * 180 / PI; // 转换为角度




    // m_imu.euler.err = (m_imu.euler.yaw - m_imu.euler.yaw_last);



    // if (m_imu.euler.err < 0)
    // {
    //     m_imu.euler.err_p = -m_imu.euler.err;

    // }
    // else
    // {
    //     m_imu.euler.err_p = m_imu.euler.err;
    // }
    
    // if (m_imu.euler.err_p < 0.0001f)
    // {
    //     m_imu.euler.true_yaw = m_imu.euler.true_yaw_last;
    //     m_imu.euler.yaw_buff[i] = m_imu.euler.err;
    //     i ++;
    //     if (i >= 64)
    //     {
    //         i = 0;
            
    //     }
    //     m_imu.euler.yaw_mid = 0;
    //     for(int j = 0 ;j<64;j++)
    //     {
    //         m_imu.euler.yaw_mid += m_imu.euler.yaw_buff[j];

    //     }
    //     m_imu.euler.yaw_aver = m_imu.euler.yaw_mid/64;

        
    // }
    // else
    // {
    //     m_imu.euler.true_yaw += (m_imu.euler.err - m_imu.euler.yaw_aver);
    //     m_imu.euler.b += 1;//检测运行次数
    // }
    // m_imu.euler.true_yaw_last = m_imu.euler.true_yaw;
    // m_imu.euler.yaw_last = m_imu.euler.yaw;

    


    // float norm = sqrt(m_imu.mag[0]*m_imu.mag[0]+m_imu.mag[1]*m_imu.mag[1]+m_imu.mag[2]*m_imu.mag[2]);
    // m_imu.mag[0] = m_imu.mag[0]/norm;
    // m_imu.mag[1] = m_imu.mag[1]/norm;
    // m_imu.mag[2] = m_imu.mag[2]/norm;
    // float hx = 2.0f*m_imu.mag[0]*(0.5f - m_imu.quat[2]*m_imu.quat[2] - m_imu.quat[3]*m_imu.quat[3]) + 2.0f*m_imu.mag[1]*(m_imu.quat[1]*m_imu.quat[2] - m_imu.quat[0]*m_imu.quat[3]) + 2.0f*m_imu.mag[2]*(m_imu.quat[1]*m_imu.quat[3] + m_imu.quat[0]*m_imu.quat[2]);
    // float hy = 2.0f*m_imu.mag[0]*( m_imu.quat[1]*m_imu.quat[2] + m_imu.quat[0]*m_imu.quat[3]) + 2.0f*m_imu.mag[1]*(0.5f - m_imu.quat[1]*m_imu.quat[1] - m_imu.quat[3]*m_imu.quat[3]) + 2.0f*m_imu.mag[2]*(m_imu.quat[2]*m_imu.quat[3] - m_imu.quat[0]*m_imu.quat[1]);
    // float hz = 2.0f*m_imu.mag[0]*( m_imu.quat[1]*m_imu.quat[3] - m_imu.quat[0]*m_imu.quat[2]) + 2.0f*m_imu.mag[1]*(m_imu.quat[2]*m_imu.quat[3] + m_imu.quat[0]*m_imu.quat[1]) + 2.0f*m_imu.mag[2]*(0.5f - m_imu.quat[1]*m_imu.quat[1] - m_imu.quat[2]*m_imu.quat[2]);
    // float bz = hz;
    // float bx = sqrt((hx*hx+hy*hy));
    // m_imu.attitude.my_yaw = bx;
    // float wx = 2.0f * bx*(0.5f - m_imu.quat[2]*m_imu.quat[2] - m_imu.quat[3]*m_imu.quat[3]) + 2.0f *bz*(m_imu.quat[1]*m_imu.quat[3] - m_imu.quat[0]*m_imu.quat[2]);
    // m_imu.attitude.my_yaw_2 = wx * 90*1.021482758f;

//      以上为磁力计转角度，分四个90度
    
   
    m_imu.attitude.pitch = m_imu.euler.pitch * RAD_TO_ANGLE;
    m_imu.attitude.yaw = m_imu.euler.yaw * RAD_TO_ANGLE;
    m_imu.attitude.roll = m_imu.euler.roll * RAD_TO_ANGLE;
}

IMU_Data_t* IMU_GetDataPointer(void)
{
    return &m_imu;
}

void IMU_GyroOffsetCalc(fp32 gyro_offset[3], fp32 gyro[3])
{
    if (gyro_offset == NULL || gyro == NULL)
    {
        return;
    }

    gyro_offset[0] = gyro_offset[0] - 0.0003f * gyro[0];
    gyro_offset[1] = gyro_offset[1] - 0.0003f * gyro[1];
    gyro_offset[2] = gyro_offset[2] - 0.0003f * gyro[2];
}

void IMU_CalibrateGyro(fp32 cali_offset[3])
{
    IMU_GyroOffsetCalc(gyro_offset, m_imu.gyro);

    cali_offset[0] = gyro_offset[0];
    cali_offset[1] = gyro_offset[1];
    cali_offset[2] = gyro_offset[2];
}

void IMU_SetGyroOffset(fp32 cali_offset[3])
{
    gyro_cali_offset[0] = cali_offset[0];
    gyro_cali_offset[1] = cali_offset[1];
    gyro_cali_offset[2] = cali_offset[2];
    gyro_offset[0] = gyro_cali_offset[0];
    gyro_offset[1] = gyro_cali_offset[1];
    gyro_offset[2] = gyro_cali_offset[2];
}

