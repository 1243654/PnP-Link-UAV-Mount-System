#include "mpu9250.h"

#include <stdio.h>
#include <string.h>
extern UART_HandleTypeDef huart4;

/**
  * @brief  向MPU9250写一个字节
  * @param  dev: 设备句柄
  * @param  reg: 寄存器地址
  * @param  data: 要写入的数据
  * @retval HAL状态
  */
static HAL_StatusTypeDef MPU9250_WriteReg(MPU9250_HandleTypeDef *dev, uint8_t reg, uint8_t data)
{
    return HAL_I2C_Mem_Write(dev->hi2c, MPU9250_ADDR, reg, I2C_MEMADD_SIZE_8BIT, &data, 1, HAL_MAX_DELAY);
}

/**
  * @brief  从MPU9250读一个字节
  * @param  dev: 设备句柄
  * @param  reg: 寄存器地址
  * @param  data: 数据存储指针
  * @retval HAL状态
  */
static HAL_StatusTypeDef MPU9250_ReadReg(MPU9250_HandleTypeDef *dev, uint8_t reg, uint8_t *data)
{
    return HAL_I2C_Mem_Read(dev->hi2c, MPU9250_ADDR, reg, I2C_MEMADD_SIZE_8BIT, data, 1, HAL_MAX_DELAY);
}

/**
  * @brief  从AK8963读多个字节
  * @param  dev: 设备句柄
  * @param  reg: 起始寄存器
  * @param  data: 数据缓冲区
  * @param  len: 要读取的字节数
  * @retval HAL状态
  */
static HAL_StatusTypeDef AK8963_Read(MPU9250_HandleTypeDef *dev, uint8_t reg, uint8_t *data, uint16_t len)
{
    return HAL_I2C_Mem_Read(dev->hi2c, AK8963_ADDR, reg, I2C_MEMADD_SIZE_8BIT, data, len, HAL_MAX_DELAY);
}

/**
  * @brief  向AK8963写一个字节
  * @param  dev: 设备句柄
  * @param  reg: 寄存器地址
  * @param  data: 要写入的数据
  * @retval HAL状态
  */
static HAL_StatusTypeDef AK8963_WriteReg(MPU9250_HandleTypeDef *dev, uint8_t reg, uint8_t data)
{
    return HAL_I2C_Mem_Write(dev->hi2c, AK8963_ADDR, reg, I2C_MEMADD_SIZE_8BIT, &data, 1, HAL_MAX_DELAY);
}

/**
  * @brief  初始化MPU9250
  * @param  dev: 设备句柄
  * @retval HAL状态
  */
HAL_StatusTypeDef MPU9250_Init(MPU9250_HandleTypeDef *dev)
{
    uint8_t whoami = 0;
    HAL_StatusTypeDef status;
    char msg[64];

    // 1. 检查 WHO_AM_I
    status = MPU9250_ReadReg(dev, MPU9250_WHO_AM_I, &whoami);
    if (status != HAL_OK || whoami != 0x71) {
        HAL_UART_Transmit(&huart4, (uint8_t*)"WHO_AM_I check failed\r\n", 23, HAL_MAX_DELAY);
        return HAL_ERROR;
    }
    HAL_UART_Transmit(&huart4, (uint8_t*)"WHO_AM_I OK\r\n", 14, HAL_MAX_DELAY);

    // 2. 复位设备
    status = MPU9250_WriteReg(dev, MPU9250_PWR_MGMT_1, 0x80);
    if (status != HAL_OK) {
        HAL_UART_Transmit(&huart4, (uint8_t*)"Reset write failed\r\n", 21, HAL_MAX_DELAY);
        return HAL_ERROR;
    }
    HAL_UART_Transmit(&huart4, (uint8_t*)"Reset OK\r\n", 11, HAL_MAX_DELAY);
    HAL_Delay(100);

    // 3. 选择时钟源
    status = MPU9250_WriteReg(dev, MPU9250_PWR_MGMT_1, 0x01);
    if (status != HAL_OK) {
        HAL_UART_Transmit(&huart4, (uint8_t*)"Clock select write failed\r\n", 28, HAL_MAX_DELAY);
        return HAL_ERROR;
    }
    HAL_UART_Transmit(&huart4, (uint8_t*)"Clock select OK\r\n", 18, HAL_MAX_DELAY);
    HAL_Delay(10);

    // 4. 配置加速度计量程为±2g
    status = MPU9250_WriteReg(dev, MPU9250_ACCEL_CONFIG, 0x00);
    if (status != HAL_OK) {
        HAL_UART_Transmit(&huart4, (uint8_t*)"Accel config write failed\r\n", 28, HAL_MAX_DELAY);
        return HAL_ERROR;
    }
    HAL_UART_Transmit(&huart4, (uint8_t*)"Accel config OK\r\n", 18, HAL_MAX_DELAY);
    dev->accel_resolution = 16384.0f;

    // 5. 配置加速度计低通滤波
    status = MPU9250_WriteReg(dev, MPU9250_ACCEL_CONFIG2, 0x03);
    if (status != HAL_OK) {
        HAL_UART_Transmit(&huart4, (uint8_t*)"Accel LPF write failed\r\n", 25, HAL_MAX_DELAY);
        return HAL_ERROR;
    }
    HAL_UART_Transmit(&huart4, (uint8_t*)"Accel LPF OK\r\n", 15, HAL_MAX_DELAY);

    // 6. 配置陀螺仪量程为±250°/s
    status = MPU9250_WriteReg(dev, MPU9250_GYRO_CONFIG, 0x00);
    if (status != HAL_OK) {
        HAL_UART_Transmit(&huart4, (uint8_t*)"Gyro config write failed\r\n", 27, HAL_MAX_DELAY);
        return HAL_ERROR;
    }
    HAL_UART_Transmit(&huart4, (uint8_t*)"Gyro config OK\r\n", 17, HAL_MAX_DELAY);
    dev->gyro_resolution = 131.0f;

    // 7. 配置陀螺仪低通滤波
    status = MPU9250_WriteReg(dev, MPU9250_CONFIG, 0x03);
    if (status != HAL_OK) {
        HAL_UART_Transmit(&huart4, (uint8_t*)"Gyro LPF write failed\r\n", 24, HAL_MAX_DELAY);
        return HAL_ERROR;
    }
    HAL_UART_Transmit(&huart4, (uint8_t*)"Gyro LPF OK\r\n", 14, HAL_MAX_DELAY);

    // 8. 设置采样率分频器
    status = MPU9250_WriteReg(dev, MPU9250_SMPLRT_DIV, 0x04);
    if (status != HAL_OK) {
        HAL_UART_Transmit(&huart4, (uint8_t*)"Sample rate write failed\r\n", 27, HAL_MAX_DELAY);
        return HAL_ERROR;
    }
    HAL_UART_Transmit(&huart4, (uint8_t*)"Sample rate OK\r\n", 17, HAL_MAX_DELAY);

	// 9. 初始化磁力计AK8963,使能 I2C 旁路模式，允许外部主机直接访问磁力计
    status = MPU9250_WriteReg(dev, 0x37, 0x02);  // INT_PIN_CFG 寄存器，BYPASS_EN = 1
    if (status != HAL_OK) {
        HAL_UART_Transmit(&huart4, (uint8_t*)"Bypass enable failed\r\n", 23, HAL_MAX_DELAY);
        return HAL_ERROR;
    }
    HAL_UART_Transmit(&huart4, (uint8_t*)"Bypass enable OK\r\n", 19, HAL_MAX_DELAY);
    HAL_Delay(10);
    uint8_t mag_whoami = 0;
    status = AK8963_Read(dev, AK8963_WHO_AM_I, &mag_whoami, 1);
    if (status != HAL_OK) {
        HAL_UART_Transmit(&huart4, (uint8_t*)"Mag WHO_AM_I read failed\r\n", 27, HAL_MAX_DELAY);
        return HAL_ERROR;
    }
    sprintf(msg, "Mag WHO_AM_I = 0x%02X\r\n", mag_whoami);
    HAL_UART_Transmit(&huart4, (uint8_t*)msg, strlen(msg), HAL_MAX_DELAY);
    if (mag_whoami != 0x48) {
        HAL_UART_Transmit(&huart4, (uint8_t*)"Mag WHO_AM_I mismatch\r\n", 24, HAL_MAX_DELAY);
        return HAL_ERROR;
    }
    HAL_UART_Transmit(&huart4, (uint8_t*)"Mag WHO_AM_I OK\r\n", 18, HAL_MAX_DELAY);

    // 10. 复位磁力计
    status = AK8963_WriteReg(dev, AK8963_CNTL, 0x01);
    if (status != HAL_OK) {
        HAL_UART_Transmit(&huart4, (uint8_t*)"Mag reset write failed\r\n", 25, HAL_MAX_DELAY);
        return HAL_ERROR;
    }
    HAL_UART_Transmit(&huart4, (uint8_t*)"Mag reset OK\r\n", 15, HAL_MAX_DELAY);
    HAL_Delay(10);

    // 11. 设置磁力计模式
    status = AK8963_WriteReg(dev, AK8963_CNTL, 0x16);
    if (status != HAL_OK) {
        HAL_UART_Transmit(&huart4, (uint8_t*)"Mag mode write failed\r\n", 24, HAL_MAX_DELAY);
        return HAL_ERROR;
    }
    HAL_UART_Transmit(&huart4, (uint8_t*)"Mag mode OK\r\n", 14, HAL_MAX_DELAY);
    dev->mag_resolution = 0.15f;

    HAL_UART_Transmit(&huart4, (uint8_t*)"MPU9250 init success\r\n", 23, HAL_MAX_DELAY);
    return HAL_OK;
}
/**
  * @brief  读取加速度计原始数据并转换为g
  * @param  dev: 设备句柄
  * @retval HAL状态
  */
HAL_StatusTypeDef MPU9250_ReadAccel(MPU9250_HandleTypeDef *dev)
{
    uint8_t buf[6];
    HAL_StatusTypeDef status = HAL_I2C_Mem_Read(dev->hi2c, MPU9250_ADDR, MPU9250_ACCEL_XOUT_H, I2C_MEMADD_SIZE_8BIT, buf, 6, HAL_MAX_DELAY);
    if (status != HAL_OK) return status;

    int16_t ax = (int16_t)((buf[0] << 8) | buf[1]);
    int16_t ay = (int16_t)((buf[2] << 8) | buf[3]);
    int16_t az = (int16_t)((buf[4] << 8) | buf[5]);

    dev->accel[0] = (float)ax / dev->accel_resolution;
    dev->accel[1] = (float)ay / dev->accel_resolution;
    dev->accel[2] = (float)az / dev->accel_resolution;
    return HAL_OK;
}

/**
  * @brief  读取陀螺仪原始数据并转换为度/秒
  * @param  dev: 设备句柄
  * @retval HAL状态
  */
HAL_StatusTypeDef MPU9250_ReadGyro(MPU9250_HandleTypeDef *dev)
{
    uint8_t buf[6];
    HAL_StatusTypeDef status = HAL_I2C_Mem_Read(dev->hi2c, MPU9250_ADDR, MPU9250_GYRO_XOUT_H, I2C_MEMADD_SIZE_8BIT, buf, 6, HAL_MAX_DELAY);
    if (status != HAL_OK) return status;

    int16_t gx = (int16_t)((buf[0] << 8) | buf[1]);
    int16_t gy = (int16_t)((buf[2] << 8) | buf[3]);
    int16_t gz = (int16_t)((buf[4] << 8) | buf[5]);

    dev->gyro[0] = (float)gx / dev->gyro_resolution;
    dev->gyro[1] = (float)gy / dev->gyro_resolution;
    dev->gyro[2] = (float)gz / dev->gyro_resolution;
    return HAL_OK;
}

/**
  * @brief  读取磁力计数据并转换为uT
  * @param  dev: 设备句柄
  * @retval HAL状态
  */
HAL_StatusTypeDef MPU9250_ReadMag(MPU9250_HandleTypeDef *dev)
{
    uint8_t buf[7];
    HAL_StatusTypeDef status;

    // 检查数据就绪
    uint8_t st1;
    AK8963_Read(dev, AK8963_ST1, &st1, 1);
    if (!(st1 & 0x01)) return HAL_BUSY;

    // 读取6字节数据 + ST2
    status = AK8963_Read(dev, AK8963_HXL, buf, 7);
    if (status != HAL_OK) return status;

    // 检查溢出或错误
    if (buf[6] & 0x08) return HAL_ERROR;

    int16_t mx = (int16_t)(buf[1] << 8 | buf[0]);  // 注意AK8963输出顺序：X low, X high, Y low, Y high, Z low, Z high
    int16_t my = (int16_t)(buf[3] << 8 | buf[2]);
    int16_t mz = (int16_t)(buf[5] << 8 | buf[4]);

    dev->mag[0] = (float)mx * dev->mag_resolution;
    dev->mag[1] = (float)my * dev->mag_resolution;
    dev->mag[2] = (float)mz * dev->mag_resolution;
    return HAL_OK;
}

/**
  * @brief  读取温度传感器 (原始值转换为摄氏度)
  * @param  dev: 设备句柄
  * @retval HAL状态
  */
HAL_StatusTypeDef MPU9250_ReadTemp(MPU9250_HandleTypeDef *dev)
{
    uint8_t buf[2];
    HAL_StatusTypeDef status = HAL_I2C_Mem_Read(dev->hi2c, MPU9250_ADDR, 0x41, I2C_MEMADD_SIZE_8BIT, buf, 2, HAL_MAX_DELAY);
    if (status != HAL_OK) return status;

    int16_t temp = (int16_t)((buf[0] << 8) | buf[1]);
    dev->temperature = ((float)temp / 333.87f) + 21.0f; // MPU9250温度公式
    return HAL_OK;
}

/**
  * @brief  初始化卡尔曼滤波器
  * @param  kf: 卡尔曼滤波器句柄
  * @param  Q_angle: 角度过程噪声
  * @param  Q_bias: 角速度偏置过程噪声
  * @param  R_measure: 测量噪声
  */
void MPU9250_Kalman_Init(Kalman_HandleTypeDef *kf, float Q_angle, float Q_bias, float R_measure)
{
    kf->Q_angle = Q_angle;
    kf->Q_bias = Q_bias;
    kf->R_measure = R_measure;

    kf->angle = 0.0f;
    kf->bias = 0.0f;

    // 协方差矩阵初始值（高不确定性）
    kf->P[0][0] = 0.0f;
    kf->P[0][1] = 0.0f;
    kf->P[1][0] = 0.0f;
    kf->P[1][1] = 0.0f;
}

/**
  * @brief  卡尔曼滤波器更新（单轴）
  * @param  kf: 卡尔曼滤波器句柄
  * @param  new_angle: 由加速度计计算得到的角度测量值（单位：度）
  * @param  new_rate: 陀螺仪测得的角速度（单位：度/秒），需减去偏置前
  * @param  dt: 采样时间（秒）
  * @retval 滤波后的角度估计值（度）
  */
float MPU9250_Kalman_Update(Kalman_HandleTypeDef *kf, float new_angle, float new_rate, float dt)
{
    // 预测步骤
    // 状态：angle, bias
    // 模型：angle = angle + (rate - bias)*dt, bias不变
    kf->angle += dt * (new_rate - kf->bias);

    // 预测协方差
    // P = F * P * F^T + Q
    // 其中 F = [1 -dt; 0 1]
    kf->P[0][0] += dt * (dt * kf->P[1][1] - kf->P[0][1] - kf->P[1][0] + kf->Q_angle);
    kf->P[0][1] -= dt * kf->P[1][1];
    kf->P[1][0] -= dt * kf->P[1][1];
    kf->P[1][1] += kf->Q_bias * dt;

    // 测量更新（新息 = 测量 - 预测角度）
    float y = new_angle - kf->angle;

    // 计算卡尔曼增益
    // S = H * P * H^T + R, H = [1 0]
    float S = kf->P[0][0] + kf->R_measure;
    float K[2]; // K = P * H^T / S
    K[0] = kf->P[0][0] / S;
    K[1] = kf->P[1][0] / S;

    // 更新状态
    kf->angle += K[0] * y;
    kf->bias  += K[1] * y;

    // 更新协方差
    float P00_temp = kf->P[0][0];
    float P01_temp = kf->P[0][1];

    kf->P[0][0] -= K[0] * P00_temp;
    kf->P[0][1] -= K[0] * P01_temp;
    kf->P[1][0] -= K[1] * P00_temp;
    kf->P[1][1] -= K[1] * P01_temp;

    return kf->angle;
}

/**
  * @brief  从加速度计数据计算角度（横滚或俯仰）
  * @param  ax, ay, az: 加速度值（单位g）
  * @param  axis: 'p' 计算俯仰角，'r' 计算横滚角
  * @retval 角度（度）
  */
float MPU9250_AccelToAngle(float ax, float ay, float az, char axis)
{
    float angle;
    if (axis == 'p') {
        // 俯仰角 (绕Y轴) pitch = atan2(-ax, sqrt(ay*ay + az*az))
        angle = atan2f(-ax, sqrtf(ay*ay + az*az)) * 180.0f / M_PI;
    } else if (axis == 'r') {
        // 横滚角 (绕X轴) roll = atan2(ay, az)
        angle = atan2f(ay, az) * 180.0f / M_PI;
    } else {
        angle = 0.0f;
    }
    return angle;
}
