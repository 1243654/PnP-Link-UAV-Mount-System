#ifndef __MPU9250_H
#define __MPU9250_H

#ifdef __cplusplus
extern "C" {
#endif

#include "stm32f4xx_hal.h"   // 根据实际MCU系列调整头文件
#include <math.h>

#ifndef M_PI
#define M_PI 3.14159265358979323846f
#endif

/* MPU9250 I2C地址（AD0引脚接地时为0x68，接VCC时为0x69） */
#define MPU9250_ADDR         0x68 << 1   // 使用8位地址，HAL需要左移一位

/* 寄存器地址 */
#define MPU9250_SMPLRT_DIV    0x19
#define MPU9250_CONFIG        0x1A
#define MPU9250_GYRO_CONFIG   0x1B
#define MPU9250_ACCEL_CONFIG  0x1C
#define MPU9250_ACCEL_CONFIG2 0x1D
#define MPU9250_PWR_MGMT_1    0x6B
#define MPU9250_WHO_AM_I      0x75

/* 加速度和陀螺仪数据寄存器（连续块） */
#define MPU9250_ACCEL_XOUT_H  0x3B
#define MPU9250_GYRO_XOUT_H   0x43

/* 磁力计AK8963地址和寄存器（MPU9250内置，通过I2C主模式访问，此处简化直接读取） */
#define AK8963_ADDR           0x0C << 1
#define AK8963_WHO_AM_I       0x00
#define AK8963_CNTL           0x0A
#define AK8963_ST1            0x02
#define AK8963_HXL            0x03
#define AK8963_ST2            0x09

/* 数据结构体 */
typedef struct {
    I2C_HandleTypeDef *hi2c;      // I2C句柄
    float accel_resolution;       // 加速度计分辨率
    float gyro_resolution;        // 陀螺仪分辨率
    float mag_resolution;         // 磁力计分辨率
    float accel[3];               // 加速度 (g)
    float gyro[3];                // 角速度 (度/秒)
    float mag[3];                 // 磁力计 (uT)
    float temperature;            // 温度 (摄氏度)
} MPU9250_HandleTypeDef;

/* 卡尔曼滤波器结构体（用于单轴角度估计） */
typedef struct {
    float Q_angle;   // 角度过程噪声协方差
    float Q_bias;    // 角速度偏置过程噪声协方差
    float R_measure; // 测量噪声协方差
    float angle;     // 估计角度
    float bias;      // 估计角速度偏置
    float P[2][2];   // 误差协方差矩阵
    float dt;        // 采样时间 (s)
} Kalman_HandleTypeDef;

/* 函数声明 */
HAL_StatusTypeDef MPU9250_Init(MPU9250_HandleTypeDef *dev);
HAL_StatusTypeDef MPU9250_ReadAccel(MPU9250_HandleTypeDef *dev);
HAL_StatusTypeDef MPU9250_ReadGyro(MPU9250_HandleTypeDef *dev);
HAL_StatusTypeDef MPU9250_ReadMag(MPU9250_HandleTypeDef *dev);
HAL_StatusTypeDef MPU9250_ReadTemp(MPU9250_HandleTypeDef *dev);
void MPU9250_Kalman_Init(Kalman_HandleTypeDef *kf, float Q_angle, float Q_bias, float R_measure);
float MPU9250_Kalman_Update(Kalman_HandleTypeDef *kf, float new_angle, float new_rate, float dt);
float MPU9250_AccelToAngle(float ax, float ay, float az, char axis); // axis: 'p' for pitch, 'r' for roll

#ifdef __cplusplus
}
#endif

#endif /* __MPU9250_H */
