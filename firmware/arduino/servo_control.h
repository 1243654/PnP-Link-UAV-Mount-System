#ifndef SERVO_CONTROL_H
#define SERVO_CONTROL_H

#include <ESP32Servo.h>

void servo_init();
void servo_set_angle(uint8_t servo_id, uint8_t angle);

#endif
