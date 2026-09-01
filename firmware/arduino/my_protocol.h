#ifndef MY_PROTOCOL_H
#define MY_PROTOCOL_H

#include <stdint.h>

typedef enum {
    STATE_HEAD,
    STATE_MASTER_ID,
    STATE_SUB_ID,
    STATE_TYPE,
    STATE_LEN,
    STATE_DATA,
    STATE_CHECKSUM,
    STATE_TAIL
} parse_state_t;

typedef void (*protocol_callback_t)(uint8_t master_id, uint8_t sub_id, const uint8_t *data, uint8_t len);

void protocol_init(protocol_callback_t callback);
void protocol_parse_byte(uint8_t byte);
void protocol_send_frame(uint8_t master_id, uint8_t sub_id, uint8_t type, const uint8_t *data, uint8_t len);
void protocol_send_heartbeat(uint8_t custom_data);

#endif