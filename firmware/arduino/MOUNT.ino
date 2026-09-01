#include "config.h"
#include "my_protocol.h"
#include "servo_control.h"

unsigned long last_heartbeat_time = 0;
uint8_t heartbeat_counter = 0;

void on_protocol_frame(uint8_t master_id, uint8_t sub_id, const uint8_t *data, uint8_t len) {
    if (master_id == 1 && len == 2) {
        uint16_t pwm = data[0] | (data[1] << 8);
        uint8_t angle = 0;
        
        switch (sub_id) {
            case 1:
                angle = (pwm > SERVO1_PWM_THRESHOLD) ? SERVO1_ANGLE_HIGH : SERVO1_ANGLE_LOW;
                servo_set_angle(1, angle);
                Serial.printf("Servo1 PWM=%d -> %d deg (th=%d)\n", pwm, angle, SERVO1_PWM_THRESHOLD);
                break;
            case 2:
                angle = (pwm > SERVO2_PWM_THRESHOLD) ? SERVO2_ANGLE_HIGH : SERVO2_ANGLE_LOW;
                servo_set_angle(2, angle);
                Serial.printf("Servo2 PWM=%d -> %d deg (th=%d)\n", pwm, angle, SERVO2_PWM_THRESHOLD);
                break;
            default:
                Serial.printf("Unknown sub_id %d\n", sub_id);
                break;
        }
    }
}

void setup() {
    Serial.begin(115200);
    Serial.println("ESP32 Dual Servo Start");
    SERIAL_PORT.begin(BAUD_RATE, SERIAL_8N1, RX2_PIN, TX2_PIN);
    servo_init();
    protocol_init(on_protocol_frame);
    last_heartbeat_time = millis();
    Serial.println("Setup done");
}

void loop() {
    while (SERIAL_PORT.available()) {
        uint8_t b = SERIAL_PORT.read();
        protocol_parse_byte(b);
    }
    if (millis() - last_heartbeat_time >= HEARTBEAT_INTERVAL) {
        last_heartbeat_time = millis();
        protocol_send_heartbeat(heartbeat_counter++);
    }
}
