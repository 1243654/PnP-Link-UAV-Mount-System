#include "my_protocol.h"
#include "config.h"
#include <Arduino.h>

static parse_state_t state = STATE_HEAD;
static uint8_t frame_buffer[256];
static uint8_t data_len = 0;
static uint8_t data_index = 0;
static uint8_t calculated_checksum = 0;
static protocol_callback_t user_callback = NULL;

static uint8_t calculate_checksum(const uint8_t *buffer, uint8_t len) {
    uint8_t sum = 0;
    for (uint8_t i = 0; i < len; i++) sum += buffer[i];
    return sum;
}

void protocol_init(protocol_callback_t callback) {
    user_callback = callback;
    state = STATE_HEAD;
}

void protocol_parse_byte(uint8_t byte) {
        // 添加调试打印（可选，验证是否收到数据）
    Serial.printf("Rx: 0x%02X, state=%d\n", byte, state);
    
    switch (state) {
        case STATE_HEAD:
            if (byte == FRAME_HEAD) {
                state = STATE_MASTER_ID;
                data_index = 0;
                calculated_checksum = byte;
                frame_buffer[0] = byte;
            }
            break;
        case STATE_MASTER_ID:
            frame_buffer[1] = byte;
            calculated_checksum += byte;
            state = STATE_SUB_ID;
            break;
        case STATE_SUB_ID:
            frame_buffer[2] = byte;
            calculated_checksum += byte;
            state = STATE_TYPE;
            break;
        case STATE_TYPE:
            frame_buffer[3] = byte;
            calculated_checksum += byte;
            state = STATE_LEN;
            break;
        case STATE_LEN:
            frame_buffer[4] = byte;
            calculated_checksum += byte;
            data_len = byte;
            if (data_len > 0) {
                state = STATE_DATA;
                data_index = 0;
            } else {
                state = STATE_CHECKSUM;
            }
            break;
        case STATE_DATA:
            frame_buffer[5 + data_index] = byte;
            calculated_checksum += byte;
            if (++data_index >= data_len) state = STATE_CHECKSUM;
            break;
        case STATE_CHECKSUM:
            if (byte == calculated_checksum) state = STATE_TAIL;
            else state = STATE_HEAD;
            break;
        case STATE_TAIL:
            if (byte == FRAME_TAIL) {
                if (user_callback) {
                    user_callback(frame_buffer[1], frame_buffer[2], &frame_buffer[5], frame_buffer[4]);
                }
            }
            state = STATE_HEAD;
            break;
        default:
            state = STATE_HEAD;
            break;
    }
}

void protocol_send_frame(uint8_t master_id, uint8_t sub_id, uint8_t type, const uint8_t *data, uint8_t len) {
    uint8_t buffer[256];
    uint8_t idx = 0;
    buffer[idx++] = FRAME_HEAD;
    buffer[idx++] = master_id;
    buffer[idx++] = sub_id;
    buffer[idx++] = type;
    buffer[idx++] = len;
    for (uint8_t i = 0; i < len; i++) buffer[idx++] = data[i];
    uint8_t cs = calculate_checksum(buffer, idx);
    buffer[idx++] = cs;
    buffer[idx++] = FRAME_TAIL;
    SERIAL_PORT.write(buffer, idx);
}

void protocol_send_heartbeat(uint8_t custom_data) {
    protocol_send_frame(0x02, 0, TYPE_HEARTBEAT, &custom_data, 1);
}