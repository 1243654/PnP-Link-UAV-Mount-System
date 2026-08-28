#ifndef __SERVO_UART_H
#define __SERVO_UART_H

#include "stm32f4xx_hal.h"   // 根据您的MCU系列调整头文件

/**
 * @brief 计算飞特舵机校验和
 * @param packet 指向ID字节的指针
 * @param len    从ID到数据末尾的字节数（通常为9或10）
 * @return 校验和字节
 */
uint8_t Servo_CalcChecksum(uint8_t *packet, uint8_t len);

/**
 * @brief 写舵机位置指令（地址0x2A，6数据字节：位置、时间、速度）
 * @param huart   串口句柄（如&huart2）
 * @param id      舵机ID (0~253)
 * @param position 目标位置（0~4095，负值表示反向）
 * @param speed   速度值（0~255，超过255舵机可能无响应，请根据手册限制）
 * @return HAL_StatusTypeDef HAL_OK 表示发送成功
 */
HAL_StatusTypeDef Servo_WritePosEx(UART_HandleTypeDef *huart, uint8_t id, int16_t position, uint16_t speed);

/**
 * @brief 接收舵机应答（写指令应答固定6字节）
 * @param huart   串口句柄
 * @param resp    接收缓冲区（至少6字节）
 * @param timeout 超时时间（ms）
 * @return HAL_StatusTypeDef HAL_OK 表示接收成功
 */
HAL_StatusTypeDef Servo_Receive(UART_HandleTypeDef *huart, uint8_t *resp, uint32_t timeout);

#endif
