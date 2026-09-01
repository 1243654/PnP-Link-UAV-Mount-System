#include "servo_control.h"
#include "config.h"

Servo servo1;
Servo servo2;

void servo_init() {
    servo1.attach(SERVO1_PIN);
    servo2.attach(SERVO2_PIN);
    servo1.write(0);
    servo2.write(0);
}

void servo_set_angle(uint8_t servo_id, uint8_t angle) {
    if (angle > 180) angle = 180;
    switch (servo_id) {
        case 1: servo1.write(angle); break;
        case 2: servo2.write(angle); break;
    }
}
