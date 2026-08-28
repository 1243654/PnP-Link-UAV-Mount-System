#include "protocol.h"
#include <string.h>

/* ========== 静态变量 ========== */
static UART_HandleTypeDef *huart_proto = NULL;   // 串口句柄
static uint8_t *rx_buffer = NULL;                 // DMA 接收缓冲区
static uint16_t rx_buf_size = 0;                  // 缓冲区大小
static uint8_t tx_buffer[MAX_FRAME_LEN];          // 发送缓冲区
static volatile uint8_t tx_busy = 0;              // 发送忙标志

/* 回调注册表 */
#define MAX_CALLBACKS 8
typedef struct {
    uint8_t main_id;
    uint8_t sub_id;
    ProtocolCallback cb;
} CallbackEntry;
static CallbackEntry callbacks[MAX_CALLBACKS];
static uint8_t callback_count = 0;

/* ========== 内部函数 ========== */
static uint8_t calc_checksum(uint8_t *buf, uint16_t len)
{
    uint8_t sum = 0;
    for (uint16_t i = 0; i < len; i++) {
        sum += buf[i];
    }
    return sum;
}

static ProtocolCallback find_callback(uint8_t main_id, uint8_t sub_id)
{
    for (int i = 0; i < callback_count; i++) {
        uint8_t reg_main = callbacks[i].main_id;
        uint8_t reg_sub  = callbacks[i].sub_id;
        if ((reg_main == MAIN_ID_ANY || reg_main == main_id) &&
            (reg_sub  == SUB_ID_ANY  || reg_sub  == sub_id)) {
            return callbacks[i].cb;
        }
    }
    return NULL;
}

static uint16_t parse_one_frame(uint8_t *buffer, uint16_t length, uint16_t offset)
{
    if (offset + 7 > length) return 0;   // 最小帧长度7

    if (buffer[offset] != FRAME_HEADER) return 0;

    uint8_t data_len = buffer[offset + 4];
    uint16_t frame_len = 7 + data_len;   // 头+主+子+类型+长度+数据+校验+尾
    if (offset + frame_len > length) return 0;

    if (buffer[offset + frame_len - 1] != FRAME_TAIL) return 0;

    uint8_t calc_cs = calc_checksum(&buffer[offset], 5 + data_len);
    uint8_t recv_cs = buffer[offset + 5 + data_len];
    if (calc_cs != recv_cs) return 0;

    uint8_t main_id = buffer[offset + 1];
    uint8_t sub_id  = buffer[offset + 2];
    uint8_t type    = buffer[offset + 3];
    uint8_t *data   = &buffer[offset + 5];

    ProtocolCallback cb = find_callback(main_id, sub_id);
    if (cb != NULL) {
        cb(main_id, sub_id, type, data, data_len);
    }

    return frame_len;
}

/* ========== 公共函数实现 ========== */
void PROTOCOL_Init(UART_HandleTypeDef *huart, uint8_t *rx_dma_buf, uint16_t rx_buf_size)
{
    huart_proto = huart;
    rx_buffer = rx_dma_buf;
    rx_buf_size = rx_buf_size;
    tx_busy = 0;
    callback_count = 0;
    memset(callbacks, 0, sizeof(callbacks));

    // 启动 DMA 空闲中断接收
    HAL_UARTEx_ReceiveToIdle_DMA(huart_proto, rx_buffer, rx_buf_size);
}

void PROTOCOL_RegisterCallback(uint8_t main_id, uint8_t sub_id, ProtocolCallback cb)
{
    if (callback_count >= MAX_CALLBACKS) return;
    callbacks[callback_count].main_id = main_id;
    callbacks[callback_count].sub_id  = sub_id;
    callbacks[callback_count].cb      = cb;
    callback_count++;
}

void PROTOCOL_SendFrame(uint8_t main_id, uint8_t sub_id, uint8_t type,
                        uint8_t *data, uint8_t len)
{
    if (len > MAX_DATA_LEN) return;
    if (tx_busy) return;   // 等待上一次发送完成

    // 构建帧
    uint8_t *p = tx_buffer;
    *p++ = FRAME_HEADER;
    *p++ = main_id;
    *p++ = sub_id;
    *p++ = type;
    *p++ = len;
    if (len > 0 && data != NULL) {
        memcpy(p, data, len);
        p += len;
    }
    uint8_t cs = calc_checksum(tx_buffer, p - tx_buffer);
    *p++ = cs;
    *p++ = FRAME_TAIL;
    uint16_t frame_len = p - tx_buffer;

    tx_busy = 1;
    HAL_UART_Transmit_DMA(huart_proto, tx_buffer, frame_len);
}

void PROTOCOL_ProcessRxData(uint8_t *buffer, uint16_t length)
{
    uint16_t offset = 0;
    while (offset < length) {
        uint16_t frame_len = parse_one_frame(buffer, length, offset);
        if (frame_len == 0) {
            offset++;   // 未找到有效帧，跳过当前字节
        } else {
            offset += frame_len;
        }
    }
}

void PROTOCOL_TxCpltCallback(UART_HandleTypeDef *huart)
{
    if (huart == huart_proto) {
        tx_busy = 0;   // 发送完成，清除忙标志
    }
}

uint8_t PROTOCOL_IsTxBusy(void)
{
    return tx_busy;
}
