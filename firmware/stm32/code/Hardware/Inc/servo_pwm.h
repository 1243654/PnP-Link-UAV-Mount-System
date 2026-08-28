#ifndef __SERVO_PWM_H
#define __SERVO_PWM_H

#include "main.h"

typedef struct {
    TIM_HandleTypeDef *htim;
    uint32_t channel;
    uint16_t min_pulse;      // 最小脉宽（对应 min_angle）
    uint16_t max_pulse;      // 最大脉宽（对应 max_angle）
    int16_t min_angle;       // 最小角度（默认 0）
    int16_t max_angle;       // 最大角度（默认 180）
} Servo_Handle;

void Servo_Init(Servo_Handle *servo, TIM_HandleTypeDef *htim, uint32_t channel,
                uint16_t min_pulse, uint16_t max_pulse);
void Servo_SetAngle(Servo_Handle *servo, int16_t angle);
void Servo_SetPulse(Servo_Handle *servo, uint16_t pulse_us);
void Servo_Start(Servo_Handle *servo);
void Servo_Stop(Servo_Handle *servo);

#endif
