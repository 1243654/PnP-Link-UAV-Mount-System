#include "servo_uart.h"

uint8_t Servo_CalcChecksum(uint8_t *packet, uint8_t len)
{
    uint16_t sum = 0;
    for (uint8_t i = 0; i < len; i++) {
        sum += packet[i];
    }
    return (uint8_t)(~sum);
}

HAL_StatusTypeDef Servo_WritePosEx(UART_HandleTypeDef *huart, uint8_t id, int16_t position, uint16_t speed)
{
    uint8_t frame[13];          // 总帧长：2头 + ID + 长度 + 指令 + 地址 + 6数据 + 校验 = 13
    uint8_t *pData;
    uint16_t pos_abs;

    // 处理方向位
    if (position < 0) {
        pos_abs = (uint16_t)(-position) | 0x8000;
    } else {
        pos_abs = (uint16_t)position;
    }

    // 构建帧头
    frame[0] = 0xFF;
    frame[1] = 0xFF;
    frame[2] = id;
    frame[3] = 0x09;            // 数据长度：ID+指令+地址+6数据 = 1+1+1+6=9
    frame[4] = 0x03;            // 写指令
    frame[5] = 0x2A;            // 位置寄存器起始地址（官方指定）

    // 数据段
    pData = &frame[6];
    pData[0] = pos_abs & 0xFF;          // 位置低
    pData[1] = (pos_abs >> 8) & 0xFF;   // 位置高
    pData[2] = 0x00;                    // 时间低
    pData[3] = 0x00;                    // 时间高
    pData[4] = speed & 0xFF;            // 速度低
    pData[5] = (speed >> 8) & 0xFF;     // 速度高

    // 校验和
    frame[12] = Servo_CalcChecksum(&frame[2], 9);

    return HAL_UART_Transmit(huart, frame, sizeof(frame), HAL_MAX_DELAY);
}

HAL_StatusTypeDef Servo_Receive(UART_HandleTypeDef *huart, uint8_t *resp, uint32_t timeout)
{
    // 写指令的应答固定为6字节
    return HAL_UART_Receive(huart, resp, 6, timeout);
}
