#include "servo_pwm.h"

void Servo_Init(Servo_Handle *servo, TIM_HandleTypeDef *htim, uint32_t channel,
                uint16_t min_pulse, uint16_t max_pulse)
{
    servo->htim = htim;
    servo->channel = channel;
    servo->min_pulse = min_pulse;
    servo->max_pulse = max_pulse;
    servo->min_angle = 0;
    servo->max_angle = 180;
}

void Servo_SetAngle(Servo_Handle *servo, int16_t angle)
{
    if (angle < servo->min_angle) angle = servo->min_angle;
    if (angle > servo->max_angle) angle = servo->max_angle;

    uint16_t pulse = servo->min_pulse +
                     (uint32_t)(angle - servo->min_angle) *
                     (servo->max_pulse - servo->min_pulse) /
                     (servo->max_angle - servo->min_angle);

    Servo_SetPulse(servo, pulse);
}

void Servo_SetPulse(Servo_Handle *servo, uint16_t pulse_us)
{
    __HAL_TIM_SET_COMPARE(servo->htim, servo->channel, pulse_us);
}

void Servo_Start(Servo_Handle *servo)
{
    HAL_TIM_PWM_Start(servo->htim, servo->channel);
}

void Servo_Stop(Servo_Handle *servo)
{
    HAL_TIM_PWM_Stop(servo->htim, servo->channel);
}
