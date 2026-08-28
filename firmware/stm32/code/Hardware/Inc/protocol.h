#ifndef __PROTOCOL_H
#define __PROTOCOL_H

#include <stdint.h>
#include "stm32f4xx_hal.h"

/* 协议帧格式 */
#define FRAME_HEADER        0xAA
#define FRAME_TAIL          0x55
#define MAX_DATA_LEN        255
#define MAX_FRAME_LEN       (MAX_DATA_LEN + 7)

/* 通配符（用于注册回调时匹配任意主ID或子ID） */
#define MAIN_ID_ANY         0xFF
#define SUB_ID_ANY          0xFF

/* 回调函数原型：收到完整帧时调用 */
typedef void (*ProtocolCallback)(uint8_t main_id, uint8_t sub_id, uint8_t type,
                                 uint8_t *data, uint8_t len);

/* API 函数 */
void PROTOCOL_Init(UART_HandleTypeDef *huart, uint8_t *rx_dma_buf, uint16_t rx_buf_size);
void PROTOCOL_RegisterCallback(uint8_t main_id, uint8_t sub_id, ProtocolCallback cb);
void PROTOCOL_SendFrame(uint8_t main_id, uint8_t sub_id, uint8_t type,
                        uint8_t *data, uint8_t len);
void PROTOCOL_ProcessRxData(uint8_t *buffer, uint16_t length);
void PROTOCOL_TxCpltCallback(UART_HandleTypeDef *huart);   // 在发送完成中断中调用
uint8_t PROTOCOL_IsTxBusy(void);                           // 查询发送是否空闲

#endif
